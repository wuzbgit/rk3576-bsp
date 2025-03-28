/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * Authors	:
 * Dong Xiang <dong.xiang@spreadtrum.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "sprdwl.h"
#include "work.h"
#include "rx_msg_sc2355.h"
#include "tx_msg_sc2355.h"
#include "if_sc2355.h"
#ifdef DFS_MASTER
#include "11h.h"
#endif

struct sprdwl_work *sprdwl_alloc_work(int len)
{
	struct sprdwl_work *sprdwl_work;
	int size = sizeof(*sprdwl_work) + len;

	sprdwl_work = kzalloc(size, GFP_ATOMIC);
	if (sprdwl_work) {
		INIT_LIST_HEAD(&sprdwl_work->list);
		sprdwl_work->len = len;
	}

	return sprdwl_work;
}

static struct sprdwl_work *sprdwl_get_work(struct sprdwl_priv *priv)
{
	struct sprdwl_work *sprdwl_work = NULL;

	spin_lock_bh(&priv->work_lock);
	if (!list_empty(&priv->work_list)) {
		sprdwl_work = list_first_entry(&priv->work_list,
					       struct sprdwl_work, list);
		list_del(&sprdwl_work->list);
	}
	spin_unlock_bh(&priv->work_lock);

	return sprdwl_work;
}

static void sprdwl_do_work(struct work_struct *work)
{
	struct sprdwl_work *sprdwl_work;
	struct sprdwl_reg_mgmt *reg_mgmt;
	struct sprdwl_tdls_work *tdls;
	struct sprdwl_assert_info *assert_info;
	struct sprdwl_vif *vif;
	struct sprdwl_priv *priv = container_of(work, struct sprdwl_priv, work);
	unsigned char *data = NULL;
	u8 mac_addr[ETH_ALEN];
	u16 reason_code;

	while (1) {
		sprdwl_work = sprdwl_get_work(priv);
		if (!sprdwl_work)
			return;

		vif = sprdwl_work->vif;
		netdev_dbg(vif->ndev, "process delayed work: %d\n",
			   sprdwl_work->id);

		switch (sprdwl_work->id) {
		case SPRDWL_WORK_REG_MGMT:
			reg_mgmt = (struct sprdwl_reg_mgmt *)sprdwl_work->data;
			sprdwl_register_frame(priv, vif->ctx_id,
					      reg_mgmt->type,
					      reg_mgmt->reg ? 1 : 0);
			break;
		case SPRDWL_WORK_DEAUTH:
		case SPRDWL_WORK_DISASSOC:
			cfg80211_rx_unprot_mlme_mgmt(vif->ndev,
						     sprdwl_work->data,
						     sprdwl_work->len);
			break;
		case SPRDWL_WORK_MC_FILTER:
			if (vif->mc_filter->mc_change)
				sprdwl_set_mc_filter(priv, vif->ctx_id,
						     vif->mc_filter->subtype,
						     vif->mc_filter->mac_num,
						     vif->mc_filter->mac_addr);
			break;
		case SPRDWL_WORK_NOTIFY_IP:
			sprdwl_notify_ip(priv, vif->ctx_id, SPRDWL_IPV6,
					 sprdwl_work->data);
			break;
		case SPRDWL_WORK_BA_MGMT:
			sprdwl_send_ba_mgmt(priv, vif->ctx_id,
					    sprdwl_work->data,
					    sprdwl_work->len);
			break;
		case SPRDWL_WORK_ADDBA:
			sprdwl_tx_send_addba(vif, sprdwl_work->data,
				sprdwl_work->len);
			break;
		case SPRDWL_WORK_DELBA:
			sprdwl_tx_send_delba(vif, sprdwl_work->data,
				sprdwl_work->len);
			break;
		case SPRDWL_ASSERT:
			assert_info = (struct sprdwl_assert_info *)(sprdwl_work->data);
			wlan_set_assert(vif->priv, vif->ctx_id, assert_info->cmd_id, assert_info->reason);
			break;
		case SPRDWL_HANG_RECEIVED:
			sprdwl_send_hang_received_cmd(vif->priv, vif->ctx_id);
			break;
		case SPRDWL_POP_MBUF:
			sprdwl_handle_pop_list(sprdwl_work->data);
			break;
		case SPRDWL_TDLS_CMD:
			tdls = (struct sprdwl_tdls_work *)sprdwl_work->data;
			sprdwl_tdls_oper(vif->priv, tdls->vif_ctx_id,
					 tdls->peer, tdls->oper);
			break;
		case SPRDWL_SEND_CLOSE:
			sprdwl_close_fw(vif->priv, vif->ctx_id, *(sprdwl_work->data));
			break;
#ifdef DFS_MASTER
		case SPRDWL_WORK_DFS:
			sprdwl_send_dfs_cmd(vif, sprdwl_work->data,
							sprdwl_work->len);
			break;
#endif
		case SPRDWL_PCIE_RX_ALLOC_BUF:
			sprdwl_mm_fill_buffer(priv->hw_priv);
			break;
		case SPRDWL_PCIE_RX_FLUSH_BUF:
			sprdwl_rx_flush_buffer(priv->hw_priv);
			break;
		case SPRDWL_PCIE_TX_MOVE_BUF:
			sprdwl_add_to_free_list(priv,
						(struct list_head *)sprdwl_work->data,
						sprdwl_work->len);
			break;
		case SPRDWL_PCIE_TX_FREE_BUF:
			memcpy((unsigned char *)&data, sprdwl_work->data,
				sizeof(unsigned char *));
			sprdwl_tx_free_pcie_data(priv, data);
			sprdwl_free_data(data, sprdwl_work->len);
			break;
		case SPRDWL_CMD_TX_DATA:
			sprdwl_send_data2cmd(vif->priv, vif->ctx_id,
					sprdwl_work->data, sprdwl_work->len);
			break;
		case SPRDWL_WORK_FW_PWR_DOWN:
			sprdwl_fw_power_down_ack(vif->priv, vif->ctx_id);
			break;
		case SPRDWL_WORK_HOST_WAKEUP_FW:
			sprdwl_cmd_host_wakeup_fw(vif->priv, vif->ctx_id);
			break;
		case SPRDWL_WORK_VOWIFI_DATA_PROTECTION:
			sprdwl_send_vowifi_data_prot(vif->priv, vif->ctx_id,
						     sprdwl_work->data,
						     sprdwl_work->len);
			break;
#ifdef ENABLE_PAM_WIFI
		case SPRDWL_PAM_WIFI_MISS_NODE_WORK:
			sprdwl_pam_wifi_miss_node_send(sprdwl_work->data);
			break;
#endif
		case SPRDWL_REFRESH_BO:
			sprdwl_fcc_fresh_bo_work(vif->priv, sprdwl_work->data,
						 sprdwl_work->len);
			break;
		case SPRDWL_P2P_GO_DEL_STATION:
			memcpy(mac_addr, (u8 *)sprdwl_work->data, ETH_ALEN);
			memcpy(&reason_code, (u16 *)(sprdwl_work->data + ETH_ALEN), sizeof(u16));
			sprdwl_del_station(vif->priv, vif->ctx_id, mac_addr, reason_code);
			break;
		default:
			netdev_dbg(vif->ndev, "Unknown delayed work: %d\n",
				   sprdwl_work->id);
			break;
		}

		kfree(sprdwl_work);
	}
}

void sprdwl_queue_work(struct sprdwl_priv *priv,
		       struct sprdwl_work *sprdwl_work)
{
	spin_lock_bh(&priv->work_lock);
	list_add_tail(&sprdwl_work->list, &priv->work_list);
	spin_unlock_bh(&priv->work_lock);

	if (!work_pending(&priv->work))
		queue_work(priv->common_workq, &priv->work);
}

void sprdwl_cancle_work(struct sprdwl_priv *priv, struct sprdwl_vif *vif)
{
	struct sprdwl_work *sprdwl_work, *pos;

	spin_lock_bh(&priv->work_lock);
	list_for_each_entry_safe(sprdwl_work, pos, &priv->work_list, list) {
		if (vif == sprdwl_work->vif) {
			list_del(&sprdwl_work->list);
			kfree(sprdwl_work);
		}
	}
	spin_unlock_bh(&priv->work_lock);

	flush_work(&priv->work);
}

int sprdwl_init_work(struct sprdwl_priv *priv)
{
	spin_lock_init(&priv->work_lock);
	INIT_LIST_HEAD(&priv->work_list);
	INIT_WORK(&priv->work, sprdwl_do_work);

	priv->common_workq = alloc_ordered_workqueue("sprdwl_work",
				WQ_HIGHPRI | WQ_CPU_INTENSIVE |
				WQ_MEM_RECLAIM);
	if (!priv->common_workq) {
		wl_err("%s sprdwl_work create failed\n", __func__);
		return -ENOMEM;
	}
	return 0;
}

void sprdwl_deinit_work(struct sprdwl_priv *priv)
{
	struct sprdwl_work *sprdwl_work, *pos;

	cancel_work_sync(&priv->work);

	list_for_each_entry_safe(sprdwl_work, pos, &priv->work_list, list) {
		list_del(&sprdwl_work->list);
		kfree(sprdwl_work);
	}

	flush_workqueue(priv->common_workq);
	destroy_workqueue(priv->common_workq);
}

#ifdef DRV_POWER_DOWN
void sprdwl_clean_work(struct sprdwl_priv *priv)
{
	struct sprdwl_work *sprdwl_work, *pos;

	cancel_work_sync(&priv->work);

	spin_lock_bh(&priv->work_lock);
	list_for_each_entry_safe(sprdwl_work, pos, &priv->work_list, list) {
		list_del(&sprdwl_work->list);
		kfree(sprdwl_work);
	}
	spin_unlock_bh(&priv->work_lock);

	flush_workqueue(priv->common_workq);
}
#endif

#ifdef DRV_RESET_SELF
static void sprdwl_do_reset_work(struct work_struct *work)
{
	int ret;
	struct sprdwl_priv *priv = container_of(work, struct sprdwl_priv, reset_work);
	ret = sprdwl_host_reset_self(priv);
	if(!ret)
		wl_err("%s host reset self success!\n", __func__);
}

void sprdwl_cancle_reset_work(struct sprdwl_priv *priv)
{
	flush_work(&priv->reset_work);
}

int sprdwl_init_reset_work(struct sprdwl_priv *priv)
{
	INIT_WORK(&priv->reset_work, sprdwl_do_reset_work);

	priv->reset_workq = alloc_ordered_workqueue("sprdwl_reset_work",
				WQ_HIGHPRI | WQ_CPU_INTENSIVE |
				WQ_MEM_RECLAIM);
	if (!priv->reset_workq) {
		wl_err("%s sprdwl_reset_work create failed\n", __func__);
		return -ENOMEM;
	}
	return 0;
}

void sprdwl_deinit_reset_work(struct sprdwl_priv *priv)
{
	cancel_work_sync(&priv->reset_work);
	flush_workqueue(priv->reset_workq);
	destroy_workqueue(priv->reset_workq);
}
#endif

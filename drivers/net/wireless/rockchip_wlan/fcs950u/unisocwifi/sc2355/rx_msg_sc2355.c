/*
 * Copyright (C) 2016 Spreadtrum Communications Inc.
 *
 * Authors	:
 * star.liu <star.liu@spreadtrum.com>
 * yifei.li <yifei.li@spreadtrum.com>
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

#include "rx_msg_sc2355.h"
#include "core_sc2355.h"
#include "msg.h"
#include "sprdwl.h"
#include "txrx.h"
#include "intf.h"
#include "if_sc2355.h"
#include "tx_msg_sc2355.h"
#include <net/ip6_checksum.h>
#include "work.h"
#include "debug.h"
#include "tcp_ack.h"
#include <linux/kthread.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
#include <uapi/linux/sched/types.h>
#include <linux/file.h>
#endif

#ifdef SC2355_HW_CSUM
bool mh_ipv6_ext_hdr(unsigned char nexthdr)
{
	return (nexthdr == NEXTHDR_HOP) ||
	       (nexthdr == NEXTHDR_ROUTING) ||
	       (nexthdr == NEXTHDR_DEST);
}

int ipv6_csum(void *data, __wsum csum)
{
	int ret = 0;
	struct rx_msdu_desc *msdu_desc = (struct rx_msdu_desc *)data;
	struct ethhdr *eth = (struct ethhdr *)(data + msdu_desc->msdu_offset);
	struct ipv6hdr *ip6h = NULL;
	struct ipv6_opt_hdr *hp = NULL;
	unsigned short dataoff = ETH_HLEN;
	unsigned short nexthdr = 0;

	wl_debug("%s: eth_type: 0x%x\n", __func__, eth->h_proto);

	if (eth->h_proto == cpu_to_be16(ETH_P_IPV6)) {
		data += msdu_desc->msdu_offset;
		ip6h = data + dataoff;
		nexthdr = ip6h->nexthdr;
		dataoff += sizeof(*ip6h);

		while (mh_ipv6_ext_hdr(nexthdr))  {
			wl_debug("%s: nexthdr: %d\n", __func__, nexthdr);
			hp = (struct ipv6_opt_hdr *)(data + dataoff);
			dataoff += ipv6_optlen(hp);
			nexthdr = hp->nexthdr;
		}

		wl_debug("%s: nexthdr: %d, dataoff: %d, len: %d\n",
			 __func__, nexthdr, dataoff,
			 (msdu_desc->msdu_len - dataoff));

		if (!csum_ipv6_magic(&ip6h->saddr, &ip6h->daddr,
				     (msdu_desc->msdu_len - dataoff),
				     nexthdr, csum)) {
			ret = 1;
		} else {
			ret = -1;
		}

		wl_debug("%s: ret: %d\n", __func__, ret);
	}

	return ret;
}

unsigned short get_sdio_data_csum(void *entry, void *data)
{
	unsigned short csum = 0;
	struct sdiohal_puh *puh = (struct sdiohal_puh *)data;
	struct rx_msdu_desc *msdu_desc =
			(struct rx_msdu_desc *)(data + sizeof(*puh));
	unsigned int csum_offset = msdu_total_len(msdu_desc) + sizeof(*puh);
	struct sprdwl_intf *intf = (struct sprdwl_intf *)entry;

	wl_debug("%s: check_sum: %d\n", __func__, puh->check_sum);
	if ((intf->priv->hw_type == SPRDWL_HW_SC2355_SDIO) && puh->check_sum) {
		memcpy(&csum, (void *)(data + csum_offset), sizeof(csum));
		wl_debug("%s: csum: 0x%x\n", __func__, csum);
	}

	return csum;
}

unsigned short get_pcie_data_csum(void *entry, void *data)
{
	unsigned short csum = 0;
	struct rx_mh_desc *mh_desc = (struct rx_mh_desc *)data;
	struct sprdwl_intf *intf = (struct sprdwl_intf *)entry;

	if (intf->priv->hw_type == SPRDWL_HW_SC2355_PCIE) {
		if (mh_desc->tcp_checksum_en)
			csum = mh_desc->tcp_hw_checksum;
	}

	return csum;
}

inline int fill_skb_csum(struct sk_buff *skb, unsigned short csum)
{
	int ret = 0;

	if (csum) {
		ret = ipv6_csum(skb->data, (__force __wsum)csum);
		if (!ret) {
			skb->ip_summed = CHECKSUM_COMPLETE;
			skb->csum = (__force __wsum)csum;
		} else if (ret > 0) {
			skb->ip_summed = CHECKSUM_UNNECESSARY;
		}
	} else {
		skb->ip_summed = CHECKSUM_NONE;
	}

	return ret;
}
#endif

void sprdwl_rx_send_cmd(struct sprdwl_intf *intf, void *data, int len,
			unsigned char id, unsigned char ctx_id)
{
	struct sprdwl_priv *priv = intf->priv;

	sprdwl_rx_send_cmd_process(priv, data, len, id, ctx_id);
}

void sprdwl_queue_rx_buff_work(struct sprdwl_priv *priv, unsigned char id)
{
	struct sprdwl_work *misc_work;

	misc_work = sprdwl_alloc_work(0);
	switch (id) {
	case SPRDWL_PCIE_RX_ALLOC_BUF:
	case SPRDWL_PCIE_RX_FLUSH_BUF:
		misc_work->id = id;
		sprdwl_queue_work(priv, misc_work);
		break;
	default:
		wl_err("%s: err id: %d\n", __func__, id);
		kfree(misc_work);
		break;
	}

}

void rx_up(struct sprdwl_rx_if *rx_if)
{
	complete(&rx_if->rx_completed);
}

void sprdwl_rx_process(struct sprdwl_rx_if *rx_if, struct sk_buff *pskb)
{
#ifndef SPLIT_STACK
	struct sprdwl_priv *priv = rx_if->intf->priv;
	struct sk_buff *reorder_skb = NULL, *skb = NULL;
#endif

	/* TODO: Add rx mh data process */
#ifdef SPLIT_STACK
	reorder_data_process(&rx_if->ba_entry, pskb);

	if (!work_pending(&rx_if->rx_net_work))
		queue_work(rx_if->rx_net_workq, &rx_if->rx_net_work);
#else
	reorder_skb = reorder_data_process(&rx_if->ba_entry, pskb);
	while (reorder_skb) {
		SPRDWL_GET_FIRST_SKB(skb, reorder_skb);
		skb = defrag_data_process(&rx_if->defrag_entry, skb);
		if (skb)
			sprdwl_rx_skb_process(priv, skb);
	}
#endif
}

static inline void
sprdwl_rx_mh_data_process(struct sprdwl_rx_if *rx_if, void *data,
			  int len, int buffer_type)
{
	mm_mh_data_process(&rx_if->mm_entry, data, len, buffer_type);
}

static void
sprdwl_rx_mh_addr_process(struct sprdwl_rx_if *rx_if, void *data,
			  int len, int buffer_type)
{
	struct sprdwl_intf *intf = rx_if->intf;
	struct sprdwl_common_hdr *hdr =
		(struct sprdwl_common_hdr *)(data + intf->hif_offset);
	struct sprdwl_work *misc_work = NULL;
	static unsigned long time = 0;

	wl_debug("%s: rx_data_addr=0x%lx\n", __func__, (unsigned long int)data);

	if (hdr->reserv) {
		wl_debug("%s: Add RX code here\n", __func__);
		mm_mh_data_event_process(&rx_if->mm_entry, data,
					 len, buffer_type);
	} else {
		/* TODO: Add TX complete code here */
		wl_debug("%s: Add TX complete code here\n", __func__);

		if ((time != 0) && ((jiffies - time) >= msecs_to_jiffies(1000))) {
			wl_err("%s: out of time %d\n",
				__func__, jiffies_to_msecs(jiffies - time));
		}

		time = jiffies;

		sprdwl_tx_free_pcie_data_num(intf, (unsigned char *)data);
		misc_work = sprdwl_alloc_work(sizeof(void *));

		if (misc_work) {
			misc_work->id = SPRDWL_PCIE_TX_FREE_BUF;
			memcpy(misc_work->data, &data, sizeof(void *));
			misc_work->len = buffer_type;

			sprdwl_queue_work(intf->priv, misc_work);
		} else {
			wl_err("%s fail\n", __func__);
		}
	}
}

#ifdef SPLIT_STACK
void sprdwl_rx_net_work_queue(struct work_struct *work)
{
	struct sprdwl_rx_if *rx_if;
	struct sprdwl_priv *priv;
	struct sk_buff *reorder_skb = NULL, *skb = NULL;

	rx_if = container_of(work, struct sprdwl_rx_if, rx_net_work);
	priv = rx_if->intf->priv;

	reorder_skb = reorder_get_skb_list(&rx_if->ba_entry);
	while (reorder_skb) {
		SPRDWL_GET_FIRST_SKB(skb, reorder_skb);
		skb = defrag_data_process(&rx_if->defrag_entry, skb);
		if (skb)
			sprdwl_rx_skb_process(priv, skb);
	}
}
#endif

static void sprdwl_rx_work_queue(struct work_struct *work)
{
	struct sprdwl_msg_buf *msg;
	struct sprdwl_priv *priv;
	struct sprdwl_rx_if *rx_if;
	struct sprdwl_intf *intf;
	void *pos = NULL, *data = NULL, *tran_data = NULL;
	int len = 0, num = 0;
/*
	struct sprdwl_vif *vif;
	struct sprdwl_cmd_hdr *hdr;
*/

	rx_if = container_of(work, struct sprdwl_rx_if, rx_work);
	intf = rx_if->intf;
	priv = intf->priv;

#ifndef SC2355_RX_NAPI
	if (!intf->exit && !sprdwl_peek_msg_buf(&rx_if->rx_list))
		sprdwl_rx_process(rx_if, NULL);
#endif

	while ((msg = sprdwl_peek_msg_buf(&rx_if->rx_list))) {
		if (intf->exit)
			goto next;

		pos = msg->tran_data;
		for (num = msg->len; num > 0; num--) {
			pos = sprdwl_get_rx_data(intf, pos, &data, &tran_data,
						 &len, intf->hif_offset);

			wl_debug("%s: rx type:%d, num = %d\n",
				 __func__, SPRDWL_HEAD_GET_TYPE(data), num);

			/* len in mbuf_t just means buffer len in ADMA,
			 * so need to get data len in sdiohal_puh
			 */
			if (sprdwl_debug_level >= L_DBG) {
				int print_len =
					((struct sdiohal_puh *)tran_data)->len;

				if (print_len > 100)
					print_len = 100;
				sprdwl_hex_dump("rx data",
						(unsigned char *)data,
						print_len);
			}
#if 0
			/* to check is the rsp_cnt from CP2
			* eqaul to rsp_cnt count on driver side.
			* if not equal, must be lost on SDIOHAL/PCIE.
			* assert to warn CP2
			*/
			hdr = (struct sprdwl_cmd_hdr *)data;
			vif = ctx_id_to_vif(priv, hdr->common.ctx_id);
			if ((SPRDWL_HEAD_GET_TYPE(data) == SPRDWL_TYPE_CMD ||
				SPRDWL_HEAD_GET_TYPE(data) == SPRDWL_TYPE_EVENT)) {
				if (rx_if->rsp_event_cnt != hdr->rsp_cnt) {
					wl_info("%s, %d, rsp_event_cnt=%d, hdr->cnt=%d\n",
						__func__, __LINE__,
						rx_if->rsp_event_cnt, hdr->rsp_cnt);

					if (hdr->rsp_cnt == 0) {
						rx_if->rsp_event_cnt = 0;
						wl_info("%s reset rsp_event_cnt", __func__);
					}
					/* hdr->rsp_cnt=0 means it's a old version CP2,
					* so do not assert.
					* vif=NULL means driver not init ok,
					* send cmd may cause crash
					*/
					if (vif != NULL && hdr->rsp_cnt != 0)
						sprdwl_send_assert_cmd(vif, hdr->cmd_id, RSP_CNT_ERROR);
				}
				rx_if->rsp_event_cnt++;
			}
			sprdwl_put_vif(vif);
#endif
			if (unlikely(priv->wakeup_tracer.resume_flag))
				trace_rx_wakeup(&priv->wakeup_tracer, data,
						tran_data + intf->hif_offset);

			switch (SPRDWL_HEAD_GET_TYPE(data)) {
			case SPRDWL_TYPE_DATA:
#if defined FPGA_LOOPBACK_TEST
				if (intf->loopback_n < 500) {
					unsigned char *r_buf;

					r_buf = (unsigned char *)data;
					sprdwl_intf_tx_data_fpga_test(intf,
								      r_buf,
								      len);
				}
#else
				if (msg->len > SPRDWL_MAX_DATA_RXLEN)
					wl_err("err rx data too long:%d > %d\n",
					       len, SPRDWL_MAX_DATA_RXLEN);
				sprdwl_rx_data_process(priv, data);
#endif /* FPGA_LOOPBACK_TEST */
				break;
			case SPRDWL_TYPE_CMD:
				if (msg->len > SPRDWL_MAX_CMD_RXLEN)
					wl_err("err rx cmd too long:%d > %d\n",
					       len, SPRDWL_MAX_CMD_RXLEN);
				sprdwl_rx_rsp_process(priv, data);
				break;
			case SPRDWL_TYPE_PKT_LOG:
				if (sprdwl_pkt_log_save(intf, data) == 1)
					wl_err("%s: pkt log file open or create	failed!\n",
					       __func__);
				break;
			case SPRDWL_TYPE_EVENT:
				if (msg->len > SPRDWL_MAX_CMD_RXLEN)
					wl_err("err rx event too long:%d > %d\n",
					       len, SPRDWL_MAX_CMD_RXLEN);
				sprdwl_rx_event_process(priv, data);
				break;
			case SPRDWL_TYPE_DATA_SPECIAL:
				debug_ts_leave(RX_SDIO_PORT);
				debug_ts_enter(RX_SDIO_PORT);

				if (msg->len > SPRDWL_MAX_DATA_RXLEN)
					wl_err("err data trans too long:%d > %d\n",
					       len, SPRDWL_MAX_CMD_RXLEN);
				sprdwl_rx_mh_data_process(rx_if, tran_data, len,
							  msg->buffer_type);
				tran_data = NULL;
				data = NULL;
				break;
			case SPRDWL_TYPE_DATA_PCIE_ADDR:
				if (msg->len > SPRDWL_MAX_CMD_RXLEN)
					wl_err("err rx mh data too long:%d > %d\n",
					       len, SPRDWL_MAX_DATA_RXLEN);
				sprdwl_rx_mh_addr_process(rx_if, tran_data, len,
							  msg->buffer_type);
				tran_data = NULL;
				data = NULL;
				break;
			default:
				wl_err("rx unknown type:%d\n",
				       SPRDWL_HEAD_GET_TYPE(data));
				break;
			}

			/* Marlin3 should release buffer by ourself */
			if (tran_data)
				sprdwl_free_data(tran_data, msg->buffer_type);

			if (!pos) {
				wl_debug("%s no mbuf\n", __func__);
				break;
			}
		}
next:
		/* TODO: Should we free mbuf one by one? */
		sprdwl_free_rx_data(intf, msg->fifo_id, msg->tran_data,
				    msg->data, msg->len);
		sprdwl_dequeue_msg_buf(msg, &rx_if->rx_list);
	}
}

/*
 * data_len:length of data recv by driver once time
 * pkt_len:length of one type of packet in data
 * pkt_line_num:line num in pkt log file
 */
int sprdwl_pkt_log_save(struct sprdwl_intf *intf, void *data)
{
	int i, j, temp, data_len, pkt_line_num,
		temp_pkt_line_num, pkt_len, m = 0;
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 14, 0)
	mm_segment_t fs;
#endif
	/*for pkt log space key and enter key*/
	char temp_space, temp_enter;
	/*for pkt log txt line number and write pkt log into file*/
	char temphdr[6], tempdata[2];

	intf->pfile = filp_open(
					"storage/sdcard0/Download/sprdwl_pkt_log.txt",
					O_CREAT | O_RDWR, 0);
	if (IS_ERR(intf->pfile)) {
		wl_err("file create/open fail %s, %d\n", __func__, __LINE__);
		return 1;
	}
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 14, 0)
	fs = get_fs();
	set_fs(KERNEL_DS);
#endif
	pkt_len = ((struct sprdwl_pktlog_hdr *)(data))->plen;
	if (pkt_len > SPRDWL_MAX_DATA_RXLEN) {
		wl_err("%s pkt len is invalid!\n", __func__);
		return 1;
	}
	data += sizeof(struct sprdwl_pktlog_hdr);
	while (m < pkt_len) {
		data_len = *((unsigned char *)(data+2)) + 4;
		m += data_len;
		temp_space = ' ';
		temp_enter = '\n';
		temp_pkt_line_num = 0;
		pkt_line_num = 0;
		for (j = 0; j < 6; j++) {
		     temphdr[j] = '0';
		}
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 14, 0)
		vfs_write(intf->pfile, temphdr, 6, &intf->lp);
		vfs_write(intf->pfile, &temp_space, 1, &intf->lp);
#else
		wl_err("%s go here to kernel write! [DEBUG_haolin.li1]\n", __func__);
		kernel_write(intf->pfile, temphdr, 6, &intf->lp);
		kernel_write(intf->pfile, &temp_space, 1, &intf->lp);
#endif
		memset(tempdata, 0x00, 2);
		for (i = 0; i < data_len; i++) {
				snprintf(tempdata, sizeof(tempdata), "%02x",
					*(unsigned char *)data);
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 14, 0)
				vfs_write(intf->pfile, tempdata,
						  2, &intf->lp);
#else
				wl_err("%s go here to kernel write! [DEBUG_haolin.li1]\n", __func__);
				kernel_write(intf->pfile, tempdata,
						  2, &intf->lp);
#endif
				memset(tempdata, 0x00, 2);
				if ((i != 0) && ((i+1)%16 == 0)) {
					if (i < (data_len - 1)) {
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 14, 0)
						vfs_write(intf->pfile, &temp_enter,
								  sizeof(temp_enter), &intf->lp);
#else
						wl_err("%s go here to kernel write! [DEBUG_haolin.li1]\n", __func__);
						kernel_write(intf->pfile, &temp_enter,
								  sizeof(temp_enter), &intf->lp);
#endif	  
						pkt_line_num += 16;
						temp_pkt_line_num = pkt_line_num;
						for (j = 0; j < 6; j++) {
							temp = (temp_pkt_line_num >> (j*4)) & 0xf;
							temphdr[5-j] = (temp < 10) ? (temp+'0') : (temp-10+'a');
						}
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 14, 0)
						vfs_write(intf->pfile, temphdr,
								  6, &intf->lp);
						vfs_write(intf->pfile, &temp_space,
								  1, &intf->lp);
#else
						wl_err("%s go here to kernel write! [DEBUG_haolin.li1]\n", __func__);
						kernel_write(intf->pfile, temphdr,
								  6, &intf->lp);
						kernel_write(intf->pfile, &temp_space,
								  1, &intf->lp);
#endif
					}
				} else {
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 14, 0)
					vfs_write(intf->pfile, &temp_space,
							  sizeof(temp_space), &intf->lp);
#else
					wl_err("%s go here to kernel write! [DEBUG_haolin.li1]\n", __func__);
					kernel_write(intf->pfile, &temp_space,
							  sizeof(temp_space), &intf->lp);
#endif
				}
				data++;
		}
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 14, 0)
		vfs_write(intf->pfile, &temp_enter, sizeof(temp_enter), &intf->lp);
#else
		wl_err("%s go here to kernel write! [DEBUG_haolin.li1]\n", __func__);
		kernel_write(intf->pfile, &temp_enter, sizeof(temp_enter), &intf->lp);
#endif
		memset(temphdr, 0x00, 6);
	}
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 14, 0)
	filp_close(intf->pfile, NULL);
	set_fs(fs);
#else
	fput(intf->pfile);
#endif
	return 0;
}

int sprdwl_mm_fill_buffer(void *intf)
{
	struct sprdwl_rx_if *rx_if =
		(struct sprdwl_rx_if *)((struct sprdwl_intf *)intf)->sprdwl_rx;
	struct sprdwl_mm *mm_entry = &rx_if->mm_entry;
	unsigned int num = 0, alloc_num = atomic_xchg(&mm_entry->alloc_num, 0);

	num = mm_buffer_alloc(&rx_if->mm_entry, alloc_num);
	if_tx_addr_trans_pcie(intf, NULL, 0, true);

	if (num)
		num = atomic_add_return(num, &mm_entry->alloc_num);

	if (num > SPRDWL_MAX_ADD_MH_BUF_ONCE || rx_if->addr_trans_head)
		sprdwl_queue_rx_buff_work(rx_if->intf->priv,
					  SPRDWL_PCIE_RX_ALLOC_BUF);

	return num;
}

void sprdwl_mm_fill_all_buffer(void *intf)
{
	struct sprdwl_rx_if *rx_if =
		(struct sprdwl_rx_if *)((struct sprdwl_intf *)intf)->sprdwl_rx;
	struct sprdwl_mm *mm_entry = &rx_if->mm_entry;
	int num = SPRDWL_MAX_MH_BUF - skb_queue_len(&mm_entry->buffer_list);

	if (num >= 0) {
		atomic_add(num, &mm_entry->alloc_num);
		sprdwl_mm_fill_buffer(intf);
	}
}

void sprdwl_rx_flush_buffer(void *intf)
{
	struct sprdwl_rx_if *rx_if =
		(struct sprdwl_rx_if *)((struct sprdwl_intf *)intf)->sprdwl_rx;
	struct sprdwl_mm *mm_entry = &rx_if->mm_entry;

	if (rx_if->addr_trans_head != NULL)
		if_tx_addr_trans_free(intf);

	mm_flush_buffer(mm_entry);
}

#ifdef SC2355_RX_NAPI
static int sprdwl_netdev_poll_rx(struct napi_struct *napi, int budget)
{
	struct sprdwl_msg_buf *msg;
	struct sprdwl_priv *priv;
	struct sprdwl_rx_if *rx_if;
	struct sprdwl_intf *intf;
	void *pos = NULL, *data = NULL, *tran_data = NULL;
	int len = 0, num = 0;
	int print_len;

	int quota = budget;
	int done;

	rx_if = container_of(napi, struct sprdwl_rx_if, napi_rx);
	intf = rx_if->intf;
	priv = intf->priv;

	if (!intf->exit && !sprdwl_peek_msg_buf(&rx_if->rx_data_list))
		sprdwl_rx_process(rx_if, NULL);

	while (quota && (msg = sprdwl_peek_msg_buf(&rx_if->rx_data_list))) {
		if (intf->exit)
			goto next;

		pos = msg->tran_data;
		for (num = msg->len; num > 0; num--) {
			pos = sprdwl_get_rx_data(intf, pos, &data, &tran_data,
						 &len, intf->hif_offset);

			wl_info("%s: rx type:%d\n",
				__func__, SPRDWL_HEAD_GET_TYPE(data));

			/* len in mbuf_t just means buffer len in ADMA,
			 * so need to get data len in sdiohal_puh
			 */
			if (((struct sdiohal_puh *)tran_data)->len > 100)
				print_len = 100;
			else
				print_len = ((struct sdiohal_puh *)
					     tran_data)->len;
			sprdwl_hex_dump("rx data",
					(unsigned char *)data, print_len);

			if (sprdwl_sdio_process_credit(intf, data))
				goto free;

			switch (SPRDWL_HEAD_GET_TYPE(data)) {
			case SPRDWL_TYPE_DATA_SPECIAL:
				if (msg->len > SPRDWL_MAX_DATA_RXLEN)
					wl_err("err data trans too long:%d > %d\n",
					       len, SPRDWL_MAX_CMD_RXLEN);
				sprdwl_rx_mh_data_process(rx_if, tran_data, len,
							  msg->buffer_type);
				tran_data = NULL;
				data = NULL;
				break;
			case SPRDWL_TYPE_DATA_PCIE_ADDR:
				if (msg->len > SPRDWL_MAX_CMD_RXLEN)
					wl_err("err rx mh data too long:%d > %d\n",
					       len, SPRDWL_MAX_DATA_RXLEN);
				sprdwl_rx_mh_addr_process(rx_if, tran_data, len,
							  msg->buffer_type);
				tran_data = NULL;
				data = NULL;
				break;
			default:
				wl_err("rx unknown type:%d\n",
				       SPRDWL_HEAD_GET_TYPE(data));
				break;
			}
free:
			/* Marlin3 should release buffer by ourself */
			if (tran_data)
				sprdwl_free_data(tran_data, msg->buffer_type);

			if (!pos) {
				wl_debug("%s no mbuf\n", __func__);
				break;
			}
		}
next:
		/* TODO: Should we free mbuf one by one? */
		sprdwl_free_rx_data(intf, msg->fifo_id, msg->tran_data,
				    msg->data, msg->len);
		sprdwl_dequeue_msg_buf(msg, &rx_if->rx_data_list);
		quota--;
	}

	done = budget - quota;
	if (done <= 1)
		napi_complete(napi);

	return done;
}

void sprdwl_rx_napi_init(struct net_device *ndev, struct sprdwl_intf *intf)
{
	struct sprdwl_rx_if *rx_if = (struct sprdwl_rx_if *)intf->sprdwl_rx;

	netif_napi_add(ndev, &rx_if->napi_rx, sprdwl_netdev_poll_rx, 16);
	napi_enable(&rx_if->napi_rx);
}
#endif

int sprdwl_rx_init(struct sprdwl_intf *intf)
{
	int ret = 0;
	struct sprdwl_rx_if *rx_if = NULL;

	rx_if = kzalloc(sizeof(*rx_if), GFP_KERNEL);
	if (!rx_if) {
		ret = -ENOMEM;
		goto err_rx_if;
	}

	/* init rx_list */
	ret = sprdwl_msg_init(SPRDWL_RX_MSG_NUM, &rx_if->rx_list);
	if (ret) {
		wl_err("%s tx_buf create failed: %d\n",
		       __func__, ret);
		goto err_rx_list;
	}

#ifdef SC2355_RX_NAPI
	ret = sprdwl_msg_init(SPRDWL_RX_MSG_NUM, &rx_if->rx_data_list);
	if (ret) {
		wl_err("%s tx_buf create failed: %d\n",
		       __func__, ret);
		goto err_rx_data_list;
	}
#endif

	/* init rx_work */
	rx_if->rx_queue =
		alloc_ordered_workqueue("SPRDWL_RX_QUEUE", WQ_MEM_RECLAIM |
					WQ_HIGHPRI | WQ_CPU_INTENSIVE);
	if (!rx_if->rx_queue) {
		wl_err("%s SPRDWL_RX_QUEUE create failed\n", __func__);
		ret = -ENOMEM;
		goto err_rx_work;
	}

	/*init rx_queue*/
	INIT_WORK(&rx_if->rx_work, sprdwl_rx_work_queue);

#ifdef SPLIT_STACK
	rx_if->rx_net_workq = alloc_ordered_workqueue("SPRDWL_RX_NET_QUEUE",
					WQ_HIGHPRI | WQ_CPU_INTENSIVE |
					WQ_MEM_RECLAIM);
	if (!rx_if->rx_net_workq) {
		wl_err("%s SPRDWL_RX_NET_QUEUE create failed\n", __func__);
		ret = -ENOMEM;
		goto err_rx_net_work;
	}

	/*init rx_queue*/
	INIT_WORK(&rx_if->rx_net_work, sprdwl_rx_net_work_queue);
#endif

	ret = sprdwl_defrag_init(&rx_if->defrag_entry);
	if (ret) {
		wl_err("%s init defrag fail: %d\n", __func__, ret);
		goto err_rx_defrag;
	}

	ret = sprdwl_mm_init(&rx_if->mm_entry, (void *)intf);
	if (ret) {
		wl_err("%s init mm fail: %d\n", __func__, ret);
		goto err_rx_mm;
	}

	sprdwl_reorder_init(&rx_if->ba_entry);

	intf->lp = 0;
	intf->sprdwl_rx = (void *)rx_if;
	rx_if->intf = intf;

	return ret;

err_rx_mm:
	sprdwl_mm_deinit(&rx_if->mm_entry, intf);
err_rx_defrag:
#ifdef SPLIT_STACK
	destroy_workqueue(rx_if->rx_net_workq);
err_rx_net_work:
#endif
	destroy_workqueue(rx_if->rx_queue);
err_rx_work:
#ifdef SC2355_RX_NAPI
	sprdwl_msg_deinit(&rx_if->rx_data_list);
err_rx_data_list:
#endif
	sprdwl_msg_deinit(&rx_if->rx_list);
err_rx_list:
	kfree(rx_if);
err_rx_if:
	return ret;
}

int sprdwl_rx_deinit(struct sprdwl_intf *intf)
{
	struct sprdwl_rx_if *rx_if = (struct sprdwl_rx_if *)intf->sprdwl_rx;

	flush_workqueue(rx_if->rx_queue);
	destroy_workqueue(rx_if->rx_queue);

#ifdef SPLIT_STACK
	flush_workqueue(rx_if->rx_net_workq);
	destroy_workqueue(rx_if->rx_net_workq);
#endif

	sprdwl_msg_deinit(&rx_if->rx_list);
#ifdef SC2355_RX_NAPI
	sprdwl_msg_deinit(&rx_if->rx_data_list);
	napi_disable(&rx_if->napi_rx);
#endif

	sprdwl_defrag_deinit(&rx_if->defrag_entry);
	sprdwl_mm_deinit(&rx_if->mm_entry, intf);
	sprdwl_reorder_deinit(&rx_if->ba_entry);

	kfree(rx_if);
	intf->sprdwl_rx = NULL;

	return 0;
}

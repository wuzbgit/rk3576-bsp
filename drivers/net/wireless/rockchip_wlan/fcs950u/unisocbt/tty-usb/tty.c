/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/tty.h>
#include <linux/vt_kern.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/semaphore.h>
#include <linux/version.h>
#ifdef CONFIG_OF
#include <linux/of_device.h>
#endif
#include <linux/compat.h>
#include <linux/tty_flip.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>

#include <marlin_platform.h>
#ifndef MTTY_TEST
//#include <sdiom_rx_api.h>
//#include <sdiom_tx_api.h>
#endif
#include "tty.h"
#include "lpm.h"
#include "rfkill.h"

#include "alignment/sitm.h"

#define MTTY_DEV_MAX_NR     1

struct semaphore sem_id[3];//cmd,acl,sco

struct rx_data {
    unsigned int channel;
    struct mbuf_t *head;
    struct mbuf_t *tail;
    unsigned int num;
    struct list_head entry;
};

struct mtty_device {
    struct mtty_init_data   *pdata;
    struct tty_port *port0;
    struct tty_port *port1;
    struct tty_struct   *tty;
    struct tty_driver   *driver;

    /* mtty state */
    uint32_t    state;
    /*spinlock_t    stat_lock;*/
    /*spinlock_t    rw_lock;*/
    struct mutex    stat_mutex;
    struct mutex    rw_mutex;
    struct list_head        rx_head;
    /*struct tasklet_struct rx_task;*/
    struct work_struct bt_rx_work;
    struct workqueue_struct *bt_rx_workqueue;
};

static struct mtty_device *mtty_dev;
static unsigned int que_task = 1;
static int que_sche = 1;

static int send_dummy_data(void);

static int find_chn_by_type(int pkt_type, int inout)
{
    if (BT_TX_INOUT == inout) {
        switch (pkt_type) {
            case HCI_PACKET_TYPE_COMMAND:
                return BT_CMD_TX_CHANNEL;
            case HCI_PACKET_TYPE_ACL_DATA:
                return BT_ACL_TX_CHANNEL;
            case HCI_PACKET_TYPE_SCO_DATA:
                return BT_SCO_TX_CHANNEL;
            default:
                pr_err("find_chn_by_type TX: Unknown pkt_type %d\n", pkt_type);
                return -1;
        }
    } else if (BT_RX_INOUT == inout) {
        switch (pkt_type) {
            case HCI_PACKET_TYPE_EVENT:
                return BT_EVENT_RX_CHANNEL;
            case HCI_PACKET_TYPE_ACL_DATA:
                return BT_ACL_RX_CHANNEL;
            case HCI_PACKET_TYPE_SCO_DATA:
                return BT_SCO_RX_CHANNEL;
            default:
                pr_err("find_chn_by_type RX: Unknown pkt_type %d\n", pkt_type);
                return -1;
        }
    } else {
        pr_err("find_chn_by_type : Unknown inout %d\n", inout);
        return -1;
    }
}

static int find_type_by_chn(int chn)
{
    switch (chn) {
        case BT_CMD_TX_CHANNEL:
            return HCI_PACKET_TYPE_COMMAND;
        case BT_ACL_TX_CHANNEL:
        case BT_ACL_RX_CHANNEL:
            return HCI_PACKET_TYPE_ACL_DATA;
        case BT_SCO_TX_CHANNEL:
        case BT_SCO_RX_CHANNEL:
            return HCI_PACKET_TYPE_SCO_DATA;
        case BT_EVENT_RX_CHANNEL:
            return HCI_PACKET_TYPE_EVENT;
        default:
            pr_err("find_type_by_chn Unknown chn %d\n", chn);
            return -1;
    }
}

/* static void mtty_rx_task(unsigned long data) */
static void mtty_rx_work_queue(struct work_struct *work)

{
    int i, ret = 0;
    /*struct mtty_device *mtty = (struct mtty_device *)data;*/
    struct mtty_device *mtty;
    struct rx_data *rx = NULL;

    que_task = que_task + 1;
    if (que_task > 65530)
        que_task = 0;
    pr_info("mtty que_task= %d\n", que_task);
    que_sche = que_sche - 1;

    mtty = container_of(work, struct mtty_device, bt_rx_work);
    if (unlikely(!mtty)) {
        pr_err("mtty_rx_task mtty is NULL\n");
        return;
    }

    mutex_lock(&mtty->stat_mutex);
    if (mtty->state == MTTY_STATE_OPEN) {
        mutex_unlock(&mtty->stat_mutex);

        do {
            mutex_lock(&mtty->rw_mutex);
            if (list_empty_careful(&mtty->rx_head)) {
                pr_err("mtty over load queue done\n");
                mutex_unlock(&mtty->rw_mutex);
                break;
            }
            rx = list_first_entry_or_null(&mtty->rx_head,
                        struct rx_data, entry);
            if (!rx) {
                pr_err("mtty over load queue abort\n");
                mutex_unlock(&mtty->rw_mutex);
                break;
            }
            list_del(&rx->entry);
            mutex_unlock(&mtty->rw_mutex);

            pr_err("mtty over load working at channel: %d, len: %d\n",
                        rx->channel, rx->head->len);
            for (i = 0; i < rx->head->len; i++) {
                ret = tty_insert_flip_char(mtty->port0,
                            *(rx->head->buf+i), TTY_NORMAL);
                if (ret != 1) {
                    i--;
                    continue;
                } else {
                    tty_flip_buffer_push(mtty->port0);
                }
            }
            pr_err("mtty over load cut channel: %d\n", rx->channel);
            kfree(rx->head->buf);
            kfree(rx);

        } while (1);
    } else {
        pr_info("mtty status isn't open, status:%d\n", mtty->state);
        mutex_unlock(&mtty->stat_mutex);
    }
}

static int mtty_rx_cb(int chn, struct mbuf_t *head, struct mbuf_t *tail, int num)
{
    int ret = 0, len_send, pkt_type;
    unsigned char *sdio_buf = NULL;
    unsigned char *sdio_buf_ptr = NULL;
    unsigned short sdio_buf_len = 0;
    unsigned char loop_count = 0;
    struct mbuf_t *mbuf_ptr = head;
    pr_err("mtty_rx_cb() num = %d\n", num);
    for (; loop_count < num; loop_count++, mbuf_ptr = mbuf_ptr->next) {
        sdio_buf_ptr = mbuf_ptr->buf;
        sdio_buf_len = mbuf_ptr->len;

#if (defined(HCI_USB_WITH_TYPE) && HCI_USB_WITH_TYPE == TRUE)
        sdio_buf = kmalloc(sdio_buf_len, GFP_KERNEL);
#else
        sdio_buf = kmalloc(sdio_buf_len + 1, GFP_KERNEL);
#endif
        pr_err("mtty_rx_cb() sdio_buf len = %d\n", sdio_buf_len);
        if (sdio_buf == NULL) {
            pr_err("mtty_rx_cb() sdio_buf = null\n");
            sprdwcn_bus_push_list(chn, head, tail, num);
            return -ENOMEM;
        }

#if (defined(HCI_USB_WITH_TYPE) && HCI_USB_WITH_TYPE == TRUE)
        memcpy(sdio_buf, sdio_buf_ptr, sdio_buf_len);
#else
        memcpy(sdio_buf, sdio_buf_ptr, BT_SDIO_HEAD_LEN);
        memcpy(&sdio_buf[BT_SDIO_HEAD_LEN + 1], sdio_buf_ptr + BT_SDIO_HEAD_LEN,
               sdio_buf_len - BT_SDIO_HEAD_LEN);
#endif

        //bt_wakeup_host();

        pkt_type = find_type_by_chn(chn);
        switch (pkt_type) {
            case HCI_PACKET_TYPE_EVENT:
                len_send = sdio_buf[2] + 3;
                break;
            case HCI_PACKET_TYPE_ACL_DATA:
                len_send = (sdio_buf[4] << 8) + sdio_buf[3] + 5;
                break;
            case HCI_PACKET_TYPE_SCO_DATA:
                len_send = sdio_buf[3] + 4;
                break;
            default:
                pr_err("%s() error pkt_type\n", __func__);
                kfree(sdio_buf);
                sprdwcn_bus_push_list(chn, head, tail, num);
                return -1;
        }
#if (defined(HCI_USB_WITH_TYPE) && HCI_USB_WITH_TYPE == TRUE)
        pr_debug("%s() usb with type, no need add!\n", __func__);
#else
        sdio_buf[BT_SDIO_HEAD_LEN] = pkt_type;
#endif

        printk("%s() channel: %d, num: %d", __func__, chn, loop_count);
        pr_info("%s() ---mtty receive channel= %d, len_send = %d\n", __func__, chn, len_send);

        if (mtty_dev->state != MTTY_STATE_OPEN) {
            pr_err("%s() mtty bt is closed abnormally\n", __func__);
            kfree(sdio_buf);
            sprdwcn_bus_push_list(chn, head, tail, num);
            return -1;
        }

        if (mtty_dev != NULL) {
            if (!work_pending(&mtty_dev->bt_rx_work)) {
                pr_info("%s() tty_insert_flip_string", __func__);
                ret = tty_insert_flip_string(mtty_dev->port0,
                                (unsigned char *)sdio_buf + BT_SDIO_HEAD_LEN,
                                len_send);   // -BT_SDIO_HEAD_LEN
                pr_info("%s() ret=%d, len=%d\n", __func__, ret, len_send);
                if (ret)
                    tty_flip_buffer_push(mtty_dev->port0);
                if (ret == (len_send)) {
                    pr_info("%s() send success", __func__);
                    kfree(sdio_buf);
                    continue;
               }
            }
        }
        pr_err("mtty_rx_cb mtty_dev is NULL!!!\n");
        sprdwcn_bus_push_list(chn, head, tail, num);
        kfree(sdio_buf);
        return -1;
	}
    sprdwcn_bus_push_list(chn, head, tail, num);
    return 0;
        /*rx = kmalloc(sizeof(struct rx_data), GFP_KERNEL);
        if (rx == NULL) {
            pr_err("%s() rx == NULL\n", __func__);
            sprdwcn_bus_push_list(chn, head, tail, num);
            return -ENOMEM;
        }

        rx->head = head;
        rx->tail = tail;
        rx->channel = chn;
        rx->num = num;
        rx->head->len = (len_send) - ret;
        rx->head->buf = kmalloc(rx->head->len, GFP_KERNEL);
        if (rx->head->buf == NULL) {
            pr_err("mtty low memory!\n");
            kfree(rx);
            kfree(sdio_buf);
            sprdwcn_bus_push_list(chn, head, tail, num);
            return -ENOMEM;
        }

        memcpy(rx->head->buf, sdio_buf + BT_SDIO_HEAD_LEN + ret, rx->head->len);
        kfree(sdio_buf);
        sprdwcn_bus_push_list(chn, head, tail, num);
        mutex_lock(&mtty_dev->rw_mutex);
        pr_err("mtty over load push %d -> %d, channel: %d len: %d\n",
                len_send, ret, rx->channel, rx->head->len);
        list_add_tail(&rx->entry, &mtty_dev->rx_head);
        mutex_unlock(&mtty_dev->rw_mutex);
        if (!work_pending(&mtty_dev->bt_rx_work)) {
        pr_err("work_pending\n");
            queue_work(mtty_dev->bt_rx_workqueue,
                        &mtty_dev->bt_rx_work);
        }
        return 0;*/
}

static int mtty_tx_cb(int chn, struct mbuf_t *head, struct mbuf_t *tail, int num)
{
    int i;
    struct mbuf_t *pos = NULL;
    int type;
    pr_info("%s channel: %d, head: %p, tail: %p num: %d\n", __func__, chn, head, tail, num);
    pos = head;
    for (i = 0; i < num; i++, pos = pos->next) {
        kfree(pos->buf);
        pos->buf = NULL;
    }
    if ((sprdwcn_bus_list_free(chn, head, tail, num)) == 0) {
        pr_info("%s sprdwcn_bus_list_free() success\n", __func__);
        type = find_type_by_chn(chn);
        up(&sem_id[type - 1]);
    }
    else
        pr_err("%s sprdwcn_bus_list_free() fail\n", __func__);

    return 0;
}

struct mchn_ops_t bt_acl_tx_ops = {
    .channel = BT_ACL_TX_CHANNEL,
    .hif_type = HW_TYPE_USB,
    .inout = BT_TX_INOUT,
    .pool_size = BT_TX_POOL_SIZE,
    .pop_link = mtty_tx_cb,
};

struct mchn_ops_t bt_acl_rx_ops = {
    .channel = BT_ACL_RX_CHANNEL,
    .hif_type = HW_TYPE_USB,
    .inout = BT_RX_INOUT,
    .pool_size = BT_RX_POOL_SIZE,
    .pop_link = mtty_rx_cb,
};

struct mchn_ops_t bt_cmd_tx_ops = {
    .channel = BT_CMD_TX_CHANNEL,
    .hif_type = HW_TYPE_USB,
    .inout = BT_TX_INOUT,
    .pool_size = BT_TX_POOL_SIZE,
    .pop_link = mtty_tx_cb,
};

struct mchn_ops_t bt_event_rx_ops = {
    .channel = BT_EVENT_RX_CHANNEL,
    .hif_type = HW_TYPE_USB,
    .inout = BT_RX_INOUT,
    .pool_size = BT_RX_POOL_SIZE,
    .pop_link = mtty_rx_cb,
};

struct mchn_ops_t bt_sco_tx_ops = {
    .channel = BT_SCO_TX_CHANNEL,
    .hif_type = HW_TYPE_USB,
    .inout = BT_TX_INOUT,
    .pool_size = BT_TX_POOL_SIZE,
    .pop_link = mtty_tx_cb,
};

struct mchn_ops_t bt_sco_rx_ops = {
    .channel = BT_SCO_RX_CHANNEL,
    .hif_type = HW_TYPE_USB,
    .inout = BT_RX_INOUT,
    .pool_size = BT_RX_POOL_SIZE,
    .pop_link = mtty_rx_cb,
};

static int mtty_open(struct tty_struct *tty, struct file *filp)
{
    struct mtty_device *mtty = NULL;
    struct tty_driver *driver = NULL;
	
	bluetooth_set_power(0, 0);

    if (tty == NULL) {
        pr_err("mtty open input tty is NULL!\n");
        return -ENOMEM;
    }
    driver = tty->driver;
    mtty = (struct mtty_device *)driver->driver_state;

    if (mtty == NULL) {
        pr_err("mtty open input mtty NULL!\n");
        return -ENOMEM;
    }

    mtty->tty = tty;
    tty->driver_data = (void *)mtty;

    mutex_lock(&mtty->stat_mutex);
    mtty->state = MTTY_STATE_OPEN;
    mutex_unlock(&mtty->stat_mutex);
    que_task = 0;
    que_sche = 0;
    sitm_ini();
    sprdwcn_bus_chn_init(&bt_acl_tx_ops);
    sprdwcn_bus_chn_init(&bt_acl_rx_ops);
    sprdwcn_bus_chn_init(&bt_cmd_tx_ops);
    sprdwcn_bus_chn_init(&bt_event_rx_ops);
    /* not init for no use*/
    //sprdwcn_bus_chn_init(&bt_sco_tx_ops);
    //sprdwcn_bus_chn_init(&bt_sco_rx_ops);
    pr_info("mtty_open device success!\n");
	/* TODO: When using USB Bus, BT needs to send a dummy data after power on. */
	send_dummy_data();

    return 0;
}

static void mtty_close(struct tty_struct *tty, struct file *filp)
{
    struct mtty_device *mtty = NULL;

    if (tty == NULL) {
        pr_err("mtty close input tty is NULL!\n");
        return;
    }
    mtty = (struct mtty_device *) tty->driver_data;
    if (mtty == NULL) {
        pr_err("mtty close s tty is NULL!\n");
        return;
    }

    mutex_lock(&mtty->stat_mutex);
    mtty->state = MTTY_STATE_CLOSE;
    mutex_unlock(&mtty->stat_mutex);
    sprdwcn_bus_chn_deinit(&bt_acl_tx_ops);
    sprdwcn_bus_chn_deinit(&bt_acl_rx_ops);
    sprdwcn_bus_chn_deinit(&bt_cmd_tx_ops);
    sprdwcn_bus_chn_deinit(&bt_event_rx_ops);
    /* not deinit for no use*/
    //sprdwcn_bus_chn_deinit(&bt_sco_tx_ops);
    //sprdwcn_bus_chn_deinit(&bt_sco_rx_ops);
    sitm_cleanup();
    pr_info("mtty_close device success !\n");
	bluetooth_set_power(0, 1);
}

#if  0
static void send_sco_data(void)
{
    int num = 1, i;
    struct mbuf_t *tx_head = NULL;
    struct mbuf_t *tx_tail = NULL;
    int tx_chn = BT_CMD_TX_CHANNEL;
    int pkt_type = HCI_PACKET_TYPE_SCO_DATA;
    unsigned char *sdio_buf = NULL;

    int count = 1 + 2 + 1 + 0xF0;

    pr_info("%s() \n", __func__);

    tx_chn = find_chn_by_type(pkt_type, BT_TX_INOUT);
    if (tx_chn < 0) {
        pr_err("mtty_write() error pkt_type = %d\n", pkt_type);
        //return -EINVAL;
    }
    pr_info("%s channel : %d,pkt_type : %d\n", __func__, tx_chn, pkt_type);

    sdio_buf = kmalloc(count + BT_SDIO_HEAD_LEN, GFP_KERNEL);

    if (sdio_buf == NULL)
    {
        pr_err("mtty_write() sdio_buf = null\n");
        //return -ENOMEM;
    }

    memset(sdio_buf, 0, count + BT_SDIO_HEAD_LEN);

#if (defined(HCI_USB_WITH_TYPE) && HCI_USB_WITH_TYPE == TRUE)
    sdio_buf[0] = HCI_PACKET_TYPE_SCO_DATA;
    sdio_buf[1] = 0x00;
    sdio_buf[2] = 0x0e;
    sdio_buf[3] = 0xF0;
    for (i =0 ; i<0xF0;i++)
    {
        sdio_buf[4+i]=i;
    }
#else
    sdio_buf[0] = 0x00;
    sdio_buf[1] = 0x0e;
    sdio_buf[2] = 0xF0;
    for (i =0 ; i<0xF0;i++)
    {
        sdio_buf[3+i]=i;
    }
#endif

    down(&sem_id[pkt_type - 1]);
    if (!sprdwcn_bus_list_alloc(tx_chn, &tx_head, &tx_tail, &num)) {
        pr_info("%s() sprdwcn_bus_list_alloc() success\n", __func__);
        tx_head->buf = sdio_buf;
#if (defined(HCI_USB_WITH_TYPE) && HCI_USB_WITH_TYPE == TRUE)
        tx_head->len = count;
#else
        tx_head->len = count - 1;
#endif
        tx_head->next = NULL;
        /*packer type 0, subtype 0*/
        pr_info("%s() sprdwcn_bus_push_list() num=%d\n", __func__, num);
        if (sprdwcn_bus_push_list(tx_chn, tx_head, tx_tail, num))
        {
            pr_err("mtty write PT data to sdiom fail Error, free buf\n");
            kfree(tx_head->buf);
            tx_head->buf = NULL;
            sprdwcn_bus_list_free(tx_chn, tx_head, tx_tail, num);
        }
        else
        {
            pr_info("%s() sprdwcn_bus_push_list() success\n", __func__);
        }
    } else {
        pr_err("%s:%d sprdwcn_bus_list_alloc fail\n", __func__, __LINE__);
        up(&sem_id[pkt_type - 1]);
        kfree(sdio_buf);
        sdio_buf = NULL;
    }
}
#endif

static int mtty_write(struct tty_struct *tty,
            const unsigned char *buf, int count)
{
    int num = 1;
    struct mbuf_t *tx_head = NULL;
    struct mbuf_t *tx_tail = NULL;
    int tx_chn = BT_CMD_TX_CHANNEL;
    int pkt_type = buf[0];
    unsigned char *sdio_buf = NULL;

    tx_chn = find_chn_by_type(pkt_type, BT_TX_INOUT);
    if (tx_chn < 0) {
        pr_err("mtty_write() error pkt_type = %d\n", pkt_type);
        return -EINVAL;
    }
    pr_info("%s channel : %d,pkt_type : %d\n", __func__, tx_chn, pkt_type);
    /*if (unlikely(marlin_get_download_status() != true))
    {
        pr_err("grandlog mtty_write()1 status = false\n");
        return -EIO;
    }*/

    sdio_buf = kmalloc(count + BT_SDIO_HEAD_LEN, GFP_KERNEL);

    if (sdio_buf == NULL)
    {
        pr_err("mtty_write() sdio_buf = null\n");
        return -ENOMEM;
    }

    memset(sdio_buf, 0, count + BT_SDIO_HEAD_LEN);
#if (defined(HCI_USB_WITH_TYPE) && HCI_USB_WITH_TYPE == TRUE)
    memcpy(sdio_buf + BT_SDIO_HEAD_LEN, buf, count);
#else
    memcpy(sdio_buf + BT_SDIO_HEAD_LEN, buf + 1, count - 1);
#endif

    down(&sem_id[pkt_type - 1]);
    if (!sprdwcn_bus_list_alloc(tx_chn, &tx_head, &tx_tail, &num)) {
        pr_info("%s() sprdwcn_bus_list_alloc() success\n", __func__);
        tx_head->buf = sdio_buf;
#if (defined(HCI_USB_WITH_TYPE) && HCI_USB_WITH_TYPE == TRUE)
        tx_head->len = count;
#else
        tx_head->len = count - 1;
#endif
        tx_head->next = NULL;
        /*packer type 0, subtype 0*/
        pr_info("%s() sprdwcn_bus_push_list() num=%d\n", __func__, num);
        if (sprdwcn_bus_push_list(tx_chn, tx_head, tx_tail, num))
        {
            pr_err("mtty write PT data to sdiom fail Error, free buf\n");
            kfree(tx_head->buf);
            tx_head->buf = NULL;
            sprdwcn_bus_list_free(tx_chn, tx_head, tx_tail, num);
            return -EBUSY;
        } else {
            pr_info("%s() sprdwcn_bus_push_list() success\n", __func__);
            return count;
        }
    } else {
        pr_err("%s:%d sprdwcn_bus_list_alloc fail\n", __func__, __LINE__);
        up(&sem_id[pkt_type - 1]);
        kfree(sdio_buf);
        sdio_buf = NULL;
        return -ENOMEM;
    }
}


static  int sdio_data_transmit(uint8_t *data, size_t count)
{
	return mtty_write(NULL, data, count);
}


static int mtty_write_plus(struct tty_struct *tty,
	      const unsigned char *buf, int count)
{
	return sitm_write(buf, count, sdio_data_transmit);
}

static int send_dummy_data(void)
{
	char data[5] = {0x02, 0x32, 0x10, 0x00, 0x00};
	pr_err("%s entry!\n", __func__);
	return sdio_data_transmit(data, 5);
}

static void mtty_flush_chars(struct tty_struct *tty)
{
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
static unsigned int mtty_write_room(struct tty_struct *tty)
#else
static int mtty_write_room(struct tty_struct *tty)
#endif
{
	return INT_MAX;
}

static const struct tty_operations mtty_ops = {
    .open  = mtty_open,
    .close = mtty_close,
    .write = mtty_write_plus,
    .flush_chars = mtty_flush_chars,
    .write_room  = mtty_write_room,
};

static struct tty_port *mtty_port_init(void)
{
    struct tty_port *port = NULL;

    port = kzalloc(sizeof(struct tty_port), GFP_KERNEL);
    if (port == NULL)
        return NULL;
    tty_port_init(port);

    return port;
}

static int mtty_tty_driver_init(struct mtty_device *device)
{
    struct tty_driver *driver;
    int ret = 0;

    device->port0 = mtty_port_init();
    if (!device->port0)
        return -ENOMEM;

    device->port1 = mtty_port_init();
    if (!device->port1)
        return -ENOMEM;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
    driver = tty_alloc_driver(MTTY_DEV_MAX_NR * 2, 0);
#else
    driver = alloc_tty_driver(MTTY_DEV_MAX_NR * 2);
#endif
    if (!driver)
        return -ENOMEM;

    /*
    * Initialize the tty_driver structure
    * Entries in mtty_driver that are NOT initialized:
    * proc_entry, set_termios, flush_buffer, set_ldisc, write_proc
    */
    driver->owner = THIS_MODULE;
    driver->driver_name = device->pdata->name;
    driver->name = device->pdata->name;
    driver->major = 0;
    driver->type = TTY_DRIVER_TYPE_SYSTEM;
    driver->subtype = SYSTEM_TYPE_TTY;
    driver->init_termios = tty_std_termios;
    driver->driver_state = (void *)device;
    device->driver = driver;
    device->driver->flags = TTY_DRIVER_REAL_RAW;
    /* initialize the tty driver */
    tty_set_operations(driver, &mtty_ops);
    tty_port_link_device(device->port0, driver, 0);
    tty_port_link_device(device->port1, driver, 1);
    ret = tty_register_driver(driver);
    if (ret) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
        tty_driver_kref_put(driver);
#else
        put_tty_driver(driver);
#endif
        tty_port_destroy(device->port0);
        tty_port_destroy(device->port1);
        return ret;
    }
    return ret;
}

static void mtty_tty_driver_exit(struct mtty_device *device)
{
    struct tty_driver *driver = device->driver;

    tty_unregister_driver(driver);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
    tty_driver_kref_put(driver);
#else
    put_tty_driver(driver);
#endif
    tty_port_destroy(device->port0);
    tty_port_destroy(device->port1);
}

/*static int mtty_parse_dt(struct mtty_init_data **init, struct device *dev)
{
#ifdef CONFIG_OF
    struct device_node *np = dev->of_node;
    struct mtty_init_data *pdata = NULL;
    int ret;

    pdata = kzalloc(sizeof(struct mtty_init_data), GFP_KERNEL);
    if (!pdata)
        return -ENOMEM;

    ret = of_property_read_string(np,
                    "sprd,name",
                    (const char **)&pdata->name);
    if (ret)
        goto error;
    *init = pdata;

    return 0;
error:
    kfree(pdata);
    *init = NULL;
    return ret;
#else
    return -ENODEV;
#endif
}*/

static inline void mtty_destroy_pdata(struct mtty_init_data **init)
{
#ifdef CONFIG_OF
    struct mtty_init_data *pdata = *init;

    kfree(pdata);

    *init = NULL;
#else
    return;
#endif
}


static int bluetooth_reset(struct notifier_block *this, unsigned long ev, void *ptr)
{
#define RESET_BUFSIZE 5

    int ret = 0;
    int block_size = RESET_BUFSIZE;
	unsigned char reset_buf[RESET_BUFSIZE]= {0x04, 0xff, 0x02, 0x57, 0xa5};

	pr_info("%s: reset callback coming\n", __func__);
	if (mtty_dev != NULL) {
		if (!work_pending(&mtty_dev->bt_rx_work)) {

			pr_info("%s tty_insert_flip_string", __func__);

			while(ret < block_size){
				pr_info("%s before tty_insert_flip_string ret: %d, len: %d\n",
						__func__, ret, RESET_BUFSIZE);
				ret = tty_insert_flip_string(mtty_dev->port0,
									(unsigned char *)reset_buf,
									RESET_BUFSIZE);   // -BT_SDIO_HEAD_LEN
				pr_info("%s ret: %d, len: %d\n", __func__, ret, RESET_BUFSIZE);
				if (ret)
					tty_flip_buffer_push(mtty_dev->port0);
				block_size = block_size - ret;
				ret = 0;
			}
		}
	}
	return NOTIFY_DONE;
}

static struct notifier_block bluetooth_reset_block = {
	.notifier_call = bluetooth_reset,
};

static int  mtty_probe(struct platform_device *pdev)
{
    struct mtty_init_data *pdata = NULL;
    struct mtty_device *mtty;
    int rval = 0;

    /*if (pdev->dev.of_node && !pdata) {
        rval = mtty_parse_dt(&pdata, &pdev->dev);
        if (rval) {
            pr_err("failed to parse mtty device tree, ret=%d\n",
                    rval);
            return rval;
        }
    }*/

    pdata = kzalloc(sizeof(struct mtty_init_data), GFP_KERNEL);
    if (!pdata)
        return -ENOMEM;
    pdata->name = "ttyBT";
	pr_err("bt mtty_probe no dts\n");

    mtty = kzalloc(sizeof(struct mtty_device), GFP_KERNEL);
    if (mtty == NULL) {
        mtty_destroy_pdata(&pdata);
        pr_err("mtty Failed to allocate device!\n");
        return -ENOMEM;
    }

    mtty->pdata = pdata;
    rval = mtty_tty_driver_init(mtty);
    if (rval) {
        mtty_tty_driver_exit(mtty);
        kfree(mtty->port0);
        kfree(mtty->port1);
        kfree(mtty);
        mtty_destroy_pdata(&pdata);
        pr_err("regitster notifier failed (%d)\n", rval);
        return rval;
    }

    pr_info("mtty_probe init device addr: 0x%p\n", mtty);
    platform_set_drvdata(pdev, mtty);

    /*spin_lock_init(&mtty->stat_lock);*/
    /*spin_lock_init(&mtty->rw_lock);*/
    mutex_init(&mtty->stat_mutex);
    mutex_init(&mtty->rw_mutex);
    INIT_LIST_HEAD(&mtty->rx_head);
    /*tasklet_init(&mtty->rx_task, mtty_rx_task, (unsigned long)mtty);*/
    mtty->bt_rx_workqueue =
        create_singlethread_workqueue("SPRDBT_RX_QUEUE");
    if (!mtty->bt_rx_workqueue) {
        pr_err("%s SPRDBT_RX_QUEUE create failed", __func__);
        return -ENOMEM;
    }
    INIT_WORK(&mtty->bt_rx_work, mtty_rx_work_queue);

    mtty_dev = mtty;

    rfkill_bluetooth_init(pdev);
    atomic_notifier_chain_register(&wcn_reset_notifier_list,&bluetooth_reset_block);
    bluesleep_init();

    sema_init(&sem_id[HCI_PACKET_TYPE_COMMAND - 1], BT_TX_POOL_SIZE - 1);
    sema_init(&sem_id[HCI_PACKET_TYPE_ACL_DATA - 1], BT_TX_POOL_SIZE - 1);
    sema_init(&sem_id[HCI_PACKET_TYPE_SCO_DATA - 1], BT_TX_POOL_SIZE - 1);

    return 0;
}

static int  mtty_remove(struct platform_device *pdev)
{
    struct mtty_device *mtty = platform_get_drvdata(pdev);
    atomic_notifier_chain_unregister(&wcn_reset_notifier_list,
					 &bluetooth_reset_block);
    rfkill_bluetooth_remove(pdev);
    mtty_tty_driver_exit(mtty);
    kfree(mtty->port0);
    kfree(mtty->port1);
    mtty_destroy_pdata(&mtty->pdata);
    flush_workqueue(mtty->bt_rx_workqueue);
    destroy_workqueue(mtty->bt_rx_workqueue);
    /*tasklet_kill(&mtty->rx_task);*/
    kfree(mtty);
    platform_set_drvdata(pdev, NULL);
    bluesleep_exit();

    return 0;
}

static struct platform_device mtty_pdevice = {
    .name = "unisoc_mtty",
};

static const struct of_device_id mtty_match_table[] = {
    { .compatible = "sprd,mtty", },
    { },
};

static struct platform_driver mtty_driver = {
    .driver = {
        .owner = THIS_MODULE,
        .name = "unisoc_mtty",
        //.of_match_table = mtty_match_table,
    },
    .probe = mtty_probe,
    .remove = mtty_remove,
};

static int __init mtty_init(void)
{
       int ret;
       ret = platform_device_register(&mtty_pdevice);
	   if (ret) {
		   pr_err("platform_device_register failed\n"); 
	   }
       return platform_driver_register(&mtty_driver);
}

static void __exit mtty_exit(void)
{
       platform_driver_unregister(&mtty_driver);	
       platform_device_unregister(&mtty_pdevice);
}

late_initcall(mtty_init);
module_exit(mtty_exit);
MODULE_AUTHOR("Spreadtrum BSP");
MODULE_DESCRIPTION("SPRD marlin tty driver");

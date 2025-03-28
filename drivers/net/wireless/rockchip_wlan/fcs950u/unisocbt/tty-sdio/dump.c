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

#include <linux/vmalloc.h>
#include "dump.h"
#include "unisoc_bt_log.h"
extern struct device *ttyBT_dev;

extern bt_host_data_dump *data_dump;

static void data_left_shift(unsigned char data_inout)
{
    unsigned char loop_count_i = 0;
    switch (data_inout) {
        case BT_DATA_OUT:
            for(; loop_count_i < BT_MAX_DUMP_FRAME_LEN - 1; loop_count_i++) {
                memcpy(data_dump->txtime_t + loop_count_i, data_dump->txtime_t + loop_count_i + 1, sizeof(bt_host_time));
                memcpy(data_dump->tx[loop_count_i], data_dump->tx[loop_count_i + 1], BT_MAX_DUMP_DATA_LEN);
            }
        break;
        case BT_DATA_IN:
            for(; loop_count_i < BT_MAX_DUMP_FRAME_LEN - 1; loop_count_i++) {
                memcpy(data_dump->rxtime_t + loop_count_i, data_dump->rxtime_t + loop_count_i + 1, sizeof(bt_host_time));
                memcpy(data_dump->rx[loop_count_i], data_dump->rx[loop_count_i + 1], BT_MAX_DUMP_DATA_LEN);
            }
        break;
        default:
            break;
    }
}

static void get_time(unsigned char data_inout)
{
    switch (data_inout) {
        case BT_DATA_OUT:
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 0, 0)
            do_gettimeofday(&(data_dump->txtime_t[BT_MAX_DUMP_FRAME_LEN - 1].tv));
			rtc_time_to_tm(data_dump->txtime_t[BT_MAX_DUMP_FRAME_LEN - 1].tv.tv_sec,
            &(data_dump->txtime_t[BT_MAX_DUMP_FRAME_LEN - 1].rtc_t));
#else
	        ktime_get_real_ts64(&(data_dump->txtime_t[BT_MAX_DUMP_FRAME_LEN - 1].ts64));
			data_dump->txtime_t[BT_MAX_DUMP_FRAME_LEN - 1].rtc_t = rtc_ktime_to_tm(data_dump->txtime_t[BT_MAX_DUMP_FRAME_LEN - 1].ts64.tv_sec);
#endif
            break;
        case BT_DATA_IN:
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 0, 0)
            do_gettimeofday(&(data_dump->rxtime_t[BT_MAX_DUMP_FRAME_LEN - 1].tv));
            rtc_time_to_tm(data_dump->rxtime_t[BT_MAX_DUMP_FRAME_LEN - 1].tv.tv_sec,
            &(data_dump->rxtime_t[BT_MAX_DUMP_FRAME_LEN - 1].rtc_t));
#else
		    ktime_get_real_ts64(&(data_dump->rxtime_t[BT_MAX_DUMP_FRAME_LEN - 1].ts64));
			data_dump->rxtime_t[BT_MAX_DUMP_FRAME_LEN - 1].rtc_t = rtc_ktime_to_tm(data_dump->rxtime_t[BT_MAX_DUMP_FRAME_LEN - 1].ts64.tv_sec);
#endif
			break;
        default:
            break;
    }
}

void bt_host_data_save(const unsigned char *buf, int count, unsigned char data_inout) {
    if (data_dump == NULL)
        return;
    if ((buf[0] == HCI_COMMAND) ||
        ((buf[0] == HCI_EVENT) && (buf[1] == HCI_COMMAND_STATUS)) ||
        ((buf[0] == HCI_EVENT) && (buf[1] == HCI_COMMAND_COMPELET))) {
        //dev_unisoc_bt_debug(ttyBT_dev,"bt_host_data_save: data %d \n", data_inout);
        data_left_shift(data_inout);
        get_time(data_inout);
    } else {
        return;
    }
    if (count <= BT_MAX_DUMP_DATA_LEN) {
        switch (data_inout) {
            case BT_DATA_OUT:
                memset(data_dump->tx[BT_MAX_DUMP_FRAME_LEN - 1], 0, BT_MAX_DUMP_DATA_LEN);
                memcpy(data_dump->tx[BT_MAX_DUMP_FRAME_LEN - 1], buf, count);
                break;
            case BT_DATA_IN:
                memset(data_dump->rx[BT_MAX_DUMP_FRAME_LEN - 1], 0, BT_MAX_DUMP_DATA_LEN);
                memcpy(data_dump->rx[BT_MAX_DUMP_FRAME_LEN - 1], buf, count);
                break;
            default:
                break;
        }
    } else {
            switch (data_inout) {
            case BT_DATA_OUT:
                memcpy(data_dump->tx[BT_MAX_DUMP_FRAME_LEN - 1], buf, BT_MAX_DUMP_DATA_LEN);
                break;
            case BT_DATA_IN:
                memcpy(data_dump->rx[BT_MAX_DUMP_FRAME_LEN - 1], buf, BT_MAX_DUMP_DATA_LEN);
                break;
            default:
                break;
        }
    }
}

void bt_host_data_printf(void) {
    unsigned char loop_count_i = 0, loop_count_j = 0;
    if (data_dump == NULL)
        return;
    for(; loop_count_j < BT_MAX_DUMP_FRAME_LEN; loop_count_j++) {
        pr_info("bt_host_data_printf txdata[%d]: ",loop_count_j + 1);
        pr_info("%d-%d-%d %d:%d:%d.%06ld ",
                 1900 + data_dump->txtime_t[loop_count_j].rtc_t.tm_year,
        1 + data_dump->txtime_t[loop_count_j].rtc_t.tm_mon, data_dump->txtime_t[loop_count_j].rtc_t.tm_mday,
        data_dump->txtime_t[loop_count_j].rtc_t.tm_hour, data_dump->txtime_t[loop_count_j].rtc_t.tm_min,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 0, 0)
        data_dump->txtime_t[loop_count_j].rtc_t.tm_sec, data_dump->txtime_t[loop_count_j].tv.tv_usec);
#else
		data_dump->txtime_t[loop_count_j].rtc_t.tm_sec, (data_dump->txtime_t[loop_count_j].ts64.tv_nsec/1000));
#endif
        while(loop_count_i < BT_MAX_DUMP_DATA_LEN) {
            pr_info("%02X ",data_dump->tx[loop_count_j][loop_count_i++]);
        }
        pr_info("\n");
        loop_count_i = 0;
    }
    loop_count_j = 0;
    for(; loop_count_j < BT_MAX_DUMP_FRAME_LEN; loop_count_j++) {
        pr_info( "bt_host_data_printf rxdata[%d]: ",loop_count_j + 1);
        pr_info("%d-%d-%d %d:%d:%d.%06ld ",
        		1900 + data_dump->rxtime_t[loop_count_j].rtc_t.tm_year,
        1 + data_dump->rxtime_t[loop_count_j].rtc_t.tm_mon, data_dump->rxtime_t[loop_count_j].rtc_t.tm_mday,
        data_dump->rxtime_t[loop_count_j].rtc_t.tm_hour, data_dump->rxtime_t[loop_count_j].rtc_t.tm_min,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 0, 0)
        data_dump->rxtime_t[loop_count_j].rtc_t.tm_sec, data_dump->rxtime_t[loop_count_j].tv.tv_usec);
#else
		data_dump->rxtime_t[loop_count_j].rtc_t.tm_sec, (data_dump->txtime_t[loop_count_j].ts64.tv_nsec/1000));
#endif
        while(loop_count_i < BT_MAX_DUMP_DATA_LEN) {
            pr_info("%02X ",data_dump->rx[loop_count_j][loop_count_i++]);
        }
        pr_info("\n");
        loop_count_i = 0;
    }
}

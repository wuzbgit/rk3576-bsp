/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Defines for the sdio device driver
 */
#ifndef __SDIOHAL_H__
#define __SDIOHAL_H__

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include "wcn_bus.h"
#include <linux/version.h>
#if KERNEL_VERSION(4, 11, 0) <= LINUX_VERSION_CODE
#include <uapi/linux/sched/types.h>
#else
#include <linux/sched.h>
#endif

#include "../include/bus_common.h"

#ifdef CONFIG_WCN_SLP
#include "../sleep/sdio_int.h"
#include "../sleep/slp_mgr.h"
#endif

#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(fmt) "WCN SDIO: " fmt

#define PERFORMANCE_COUNT	100
#define SDIOHAL_PRINTF_LEN	10
#define SDIOHAL_NORMAL_LEVEL	0x01
#define SDIOHAL_DEBUG_LEVEL	0x02
#define SDIOHAL_LIST_LEVEL	0x04
#define SDIOHAL_DATA_LEVEL	0x08
#define SDIOHAL_PERF_LEVEL	0x10

#if 0
#ifdef CONFIG_DEBUG_FS
extern long int sdiohal_log_level;

#define sdiohal_normal(fmt, args...) \
	do { if (sdiohal_log_level & SDIOHAL_NORMAL_LEVEL) \
		pr_info(fmt, ## args); \
	} while (0)
#define sdiohal_debug(fmt, args...) \
	do { if (sdiohal_log_level & SDIOHAL_DEBUG_LEVEL) \
		pr_info(fmt, ## args); \
	} while (0)
#define sdiohal_pr_list(loglevel, fmt, args...) \
	do { if (sdiohal_log_level & loglevel) \
		pr_info(fmt, ## args); \
	} while (0)
#define sdiohal_pr_data(level, prefix_str, prefix_type, \
			rowsize, groupsize, buf, len, ascii, loglevel) \
	do { if (sdiohal_log_level & loglevel) \
		print_hex_dump(level, prefix_str, prefix_type, \
			       rowsize, groupsize, buf, len, ascii); \
	} while (0)
#define sdiohal_pr_perf(fmt, args...) \
	do { if (sdiohal_log_level & SDIOHAL_PERF_LEVEL) \
		trace_printk(fmt, ## args); \
	} while (0)
#else
#define sdiohal_normal(fmt, args...)
#define sdiohal_debug(fmt, args...)
#define sdiohal_pr_list(loglevel, fmt, args...)
#define	sdiohal_pr_data(level, prefix_str, prefix_type, \
			rowsize, groupsize, buf, len, ascii, loglevel)
#define sdiohal_pr_perf(fmt, args...)
#endif
#endif

#define sdiohal_normal(fmt, args...)
#define sdiohal_debug(fmt, args...)
#define sdiohal_pr_list(loglevel, fmt, args...)
#define	sdiohal_pr_data(level, prefix_str, prefix_type, \
			rowsize, groupsize, buf, len, ascii, loglevel)
#define sdiohal_pr_perf(fmt, args...)

#define SDIOHAL_SDMA	0
#define SDIOHAL_ADMA	1

#define SDIOHAL_RX_EXTERNAL_IRQ	0
#define SDIOHAL_RX_INBAND_IRQ	1
#define SDIOHAL_RX_POLLING	2

/* channel numbers */
#define SDIO_CHN_TX_NUM		12
#define SDIO_CHN_RX_NUM		14
#define SDIO_CHANNEL_NUM	(SDIO_CHN_TX_NUM + SDIO_CHN_RX_NUM)

/* list bumber */
#define SDIO_TX_LIST_NUM	SDIO_CHN_TX_NUM
#define SDIO_RX_LIST_NUM	SDIO_CHN_RX_NUM
#define MAX_CHAIN_NODE_NUM	100

/* task prio */
#define SDIO_TX_TASK_PRIO	89
#define SDIO_RX_TASK_PRIO	90

#ifdef CONFIG_THIRD_PARTY_BOARD

#ifdef CONFIG_SDIO_BLKSIZE_512
/* cp blk size */
#define SDIOHAL_BLK_SIZE 512
/* each pac data max size,cp align size */
#define MAX_PAC_SIZE (SDIOHAL_BLK_SIZE * 4)
#elif defined(CONFIG_WCN_PARSE_DTS)
/* cp blk size */
#define SDIOHAL_BLK_SIZE (sprdwcn_bus_get_blk_size())
/* each pac data max size,cp align size */
#define MAX_PAC_SIZE ((SDIOHAL_BLK_SIZE == 512) ? \
	(SDIOHAL_BLK_SIZE * 4) : (SDIOHAL_BLK_SIZE * 2))
#endif

#else

#define SDIOHAL_BLK_SIZE	840
#define MAX_PAC_SIZE		(SDIOHAL_BLK_SIZE * 2)
#endif
/* cp blk size */
//#define SDIOHAL_BLK_SIZE	840
//#define SDIOHAL_BLK_SIZE	512

/* mbuf max size */
#define MAX_MBUF_SIZE		(2 << 10)
/* each pac data max size,cp align size */
//#define MAX_PAC_SIZE		(SDIOHAL_BLK_SIZE * 2)
//#define MAX_PAC_SIZE		(SDIOHAL_BLK_SIZE * 4)

/* pub header size */
#define SDIO_PUB_HEADER_SIZE	(4)
#define SDIOHAL_DTBS_BUF_SIZE	SDIOHAL_BLK_SIZE

/* for rx buf */
#ifdef CONFIG_RK_BOARD
#define SDIOHAL_RX_NODE_NUM	(128 << 10)
#else
#define SDIOHAL_RX_NODE_NUM	(12 << 10)
#endif	/*CONFIG_RK_BOARD*/

/* for 64 bit sys */
#define SDIOHAL_RX_RECVBUF_LEN	(MAX_CHAIN_NODE_NUM * MAX_MBUF_SIZE)
#define SDIOHAL_FRAG_PAGE_MAX_ORDER \
		get_order(SDIOHAL_RX_RECVBUF_LEN)

/* for 32 bit sys */
#define SDIOHAL_32_BIT_RX_RECVBUF_LEN	(12 << 10)
#define SDIOHAL_FRAG_PAGE_MAX_ORDER_32_BIT \
		get_order(SDIOHAL_32_BIT_RX_RECVBUF_LEN)

#define SDIOHAL_FRAG_PAGE_MAX_SIZE \
		(PAGE_SIZE << SDIOHAL_FRAG_PAGE_MAX_ORDER)
#define SDIOHAL_PAGECNT_MAX_BIAS	SDIOHAL_FRAG_PAGE_MAX_SIZE

/* tx buf size for normal dma mode */
#define SDIOHAL_TX_SENDBUF_LEN		(MAX_CHAIN_NODE_NUM * MAX_MBUF_SIZE)

/* temp for marlin2 */
#define WIFI_MIN_RX		8
#define WIFI_MAX_RX		9

/* for adma */
#define SDIOHAL_READ		0	/* Read request */
#define SDIOHAL_WRITE		1	/* Write request */
#define SDIOHAL_DATA_FIX	0	/* Fixed addressing */
#define SDIOHAL_DATA_INC	1	/* Incremental addressing */
#define MAX_IO_RW_BLK		511

/* fun num */
#define FUNC_0			0
#define FUNC_1			1
#define SDIOHAL_MAX_FUNCS	2

/* cp sdio reg addr */
#define SDIOHAL_DT_MODE_ADDR	0x0f
#define SDIOHAL_PK_MODE_ADDR	0x20
#define SDIOHAL_CCCR_ABORT	0x06
#define VAL_ABORT_TRANS		0x01
#define SDIOHAL_FBR_SYSADDR0	0x15c
#define SDIOHAL_FBR_SYSADDR1	0x15d
#define SDIOHAL_FBR_SYSADDR2	0x15e
#define SDIOHAL_FBR_SYSADDR3	0x15f
#define SDIOHAL_FBR_APBRW0	0x180
#define SDIOHAL_FBR_APBRW1	0x181
#define SDIOHAL_FBR_APBRW2	0x182
#define SDIOHAL_FBR_APBRW3	0x183
#define SDIOHAL_FBR_STBBA0	0x1bc
#define SDIOHAL_FBR_STBBA1	0x1bd
#define SDIOHAL_FBR_STBBA2	0x1be
#define SDIOHAL_FBR_STBBA3	0x1bf
#define SDIOHAL_FBR_DEINT_EN	0x1ca
#define VAL_DEINT_ENABLE	0x3
#define SDIOHAL_FBR_PUBINT_RAW4	0x1e8

#define SDIOHAL_ALIGN_4BYTE(a)	(((a)+3)&(~3))

#ifdef CONFIG_THIRD_PARTY_BOARD
#ifdef CONFIG_SDIO_BLKSIZE_512
#define SDIOHAL_ALIGN_BLK(a)  (((a)+511)&(~511))
#elif defined(CONFIG_WCN_PARSE_DTS)
#define SDIOHAL_ALIGN_BLK(a) ((SDIOHAL_BLK_SIZE == 512) ? \
		(((a)+511)&(~511)) : (((a)%SDIOHAL_BLK_SIZE) ? \
		(((a)/SDIOHAL_BLK_SIZE + 1)*SDIOHAL_BLK_SIZE) : (a)))
#else
#define SDIOHAL_ALIGN_BLK(a) (((a)%SDIOHAL_BLK_SIZE) ? \
		(((a)/SDIOHAL_BLK_SIZE + 1)*SDIOHAL_BLK_SIZE) : (a))
#endif
#else
#define SDIOHAL_ALIGN_BLK(a) (((a)%SDIOHAL_BLK_SIZE) ? \
		(((a)/SDIOHAL_BLK_SIZE + 1)*SDIOHAL_BLK_SIZE) : (a))
#endif


#define CP_PMU_STATUS		(0x140)
#define CP_128BIT_SIZE		(0xf)
#define CP_SWITCH_SGINAL	(0x1a4)
#define CP_RESET_SLAVE		(0x1a8)
#define CP_BUS_HREADY		(0x144)
#define CP_HREADY_SIZE		(0x4)
#define SDIO_VER_CCCR		(0X0)

#define SDIOHAL_REMOVE_CARD_VAL	0x8000
#define WCN_CARD_EXIST(xmit)	(atomic_read(xmit) < SDIOHAL_REMOVE_CARD_VAL)

struct sdiohal_frag_mg {
	struct page_frag frag;
	unsigned int pagecnt_bias;
};

struct sdiohal_list_t {
	struct list_head head;
	struct mbuf_t *mbuf_head;
	struct mbuf_t *mbuf_tail;
	unsigned int type;
	unsigned int subtype;
	unsigned int node_num;
};

struct buf_pool_t {
	int size;
	int free;
	int payload;
	void *head;
	char *mem;
	spinlock_t lock;
};

struct sdiohal_sendbuf_t {
	unsigned int used_len;
	unsigned char *buf;
	unsigned int retry_len;
	unsigned char *retry_buf;
};

struct sdiohal_data_t {
	struct task_struct *tx_thread;
	struct task_struct *rx_thread;
	struct completion tx_completed;
	struct completion rx_completed;
	struct wakeup_source *tx_ws;
	struct wakeup_source *rx_ws;
	atomic_t tx_wake_flag;
	atomic_t rx_wake_flag;
#ifdef CONFIG_WCN_SLP
	atomic_t tx_wake_cp_count[SUBSYS_MAX];
	atomic_t rx_wake_cp_count[SUBSYS_MAX];
#endif
	struct mutex xmit_lock;
	struct mutex xmit_sdma;
	spinlock_t tx_spinlock;
	spinlock_t rx_spinlock;
	atomic_t flag_resume;
	atomic_t tx_mbuf_num;
	atomic_t xmit_cnt;
	bool exit_flag;
	/* 1:Enable the sdio adma function 0:disable the sdio adma function*/
	bool adma_tx_enable;
	bool adma_rx_enable;
	bool pwrseq_enable;
	
	/* blk_size: 0 840, 1 512 */
	bool blk_size;
	
	/* dedicated int1 is reusable with wifi analog iq monitor */
	bool debug_iq;
	
	/* EXTERNAL_IRQ 0, INBAND_IRQ 1, POLLING 2. */
	unsigned char irq_type;

	struct sdiohal_list_t tx_list_head;
	/* for pop after used list */
	struct sdiohal_list_t *list_tx[SDIO_CHN_TX_NUM];
	/* for dispatch received rx data list */
	struct sdiohal_list_t *list_rx[SDIO_CHN_RX_NUM];
	struct kmem_cache *rx_mbuf_cache;
	/* mbuf list */
	struct sdiohal_list_t list_rx_buf;
	/* frag ctl data buf */
	struct sdiohal_frag_mg frag_ctl;
	struct mchn_ops_t *ops[SDIO_CHANNEL_NUM];
	struct mutex callback_lock[SDIO_CHANNEL_NUM];
	struct buf_pool_t pool[SDIO_CHANNEL_NUM];

	bool flag_init;
	atomic_t flag_suspending;
	int gpio_num;
	unsigned int irq_num;
	atomic_t irq_cnt;
	unsigned int card_dump_flag;
	struct sdio_func *sdio_func[SDIOHAL_MAX_FUNCS];
	struct mmc_host *sdio_dev_host;
	struct scatterlist sg_list[MAX_CHAIN_NODE_NUM + 1];

	unsigned int success_pac_num;
	struct sdiohal_sendbuf_t send_buf;
	unsigned char *eof_buf;

	unsigned int dtbs;
	unsigned int remain_pac_num;
	unsigned long long rx_packer_cnt;
	char *dtbs_buf;

	/* for performance statistics */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
	struct timespec64 tm_begin_sch;
	struct timespec64 tm_end_sch;
	struct timespec64 tm_begin_irq;
	struct timespec64 tm_end_irq;
#else
	struct timespec tm_begin_sch;
	struct timespec tm_end_sch;
	struct timespec tm_begin_irq;
	struct timespec tm_end_irq;
#endif

	struct wakeup_source *scan_ws;
	struct completion scan_done;
	struct completion remove_done;
};

struct sdiohal_data_t *sdiohal_get_data(void);

unsigned char sdiohal_get_wl_wake_host_en(void);
unsigned char sdiohal_get_tx_mode(void);
unsigned char sdiohal_get_rx_mode(void);
unsigned char sdiohal_get_irq_type(void);
unsigned int sdiohal_get_blk_size(void);

/* for list manger */
void sdiohal_atomic_add(int count, atomic_t *value);
void sdiohal_atomic_sub(int count, atomic_t *value);

/* seam for thread */
void sdiohal_tx_down(void);
void sdiohal_tx_up(void);
void sdiohal_rx_down(void);
void sdiohal_rx_up(void);
int sdiohal_tx_thread(void *data);
int sdiohal_rx_thread(void *data);

/* for wakup event */
void sdiohal_lock_tx_ws(void);
void sdiohal_unlock_tx_ws(void);
void sdiohal_lock_rx_ws(void);
void sdiohal_unlock_rx_ws(void);
void sdiohal_lock_scan_ws(void);
void sdiohal_unlock_scan_ws(void);

/* for api mutex */
void sdiohal_callback_lock(struct mutex *mutex);
void sdiohal_callback_unlock(struct mutex *mutex);

/* for sleep */
#ifdef CONFIG_WCN_SLP
void sdiohal_cp_tx_sleep(enum slp_subsys subsys);
void sdiohal_cp_tx_wakeup(enum slp_subsys subsys);
void sdiohal_cp_rx_sleep(enum slp_subsys subsys);
void sdiohal_cp_rx_wakeup(enum slp_subsys subsys);
#else
#define sdiohal_cp_tx_sleep(args...) do {} while (0)
#define sdiohal_cp_tx_wakeup(args...) do {} while (0)
#define sdiohal_cp_rx_sleep(args...) do {} while (0)
#define sdiohal_cp_rx_wakeup(args...) do {} while (0)
#endif

void sdiohal_resume_check(void);
void sdiohal_resume_wait(void);
void sdiohal_op_enter(void);
void sdiohal_op_leave(void);
void sdiohal_sdma_enter(void);
void sdiohal_sdma_leave(void);
void sdiohal_channel_to_hwtype(int inout, int chn,
			       unsigned int *type, unsigned int *subtype);
int sdiohal_hwtype_to_channel(int inout, unsigned int type,
			      unsigned int subtype);

/* for list manger */
bool sdiohal_is_tx_list_empty(void);
int sdiohal_tx_packer(struct sdiohal_sendbuf_t *send_buf,
		      struct sdiohal_list_t *data_list,
		      struct mbuf_t *mbuf_node);
int sdiohal_tx_set_eof(struct sdiohal_sendbuf_t *send_buf,
		       unsigned char *eof_buf);
void sdiohal_tx_list_enq(int channel, struct mbuf_t *head,
			 struct mbuf_t *tail, int num);
void sdiohal_tx_find_data_list(struct sdiohal_list_t *data_list);
int sdiohal_tx_list_denq(struct sdiohal_list_t *data_list);
int sdiohal_rx_list_free(struct mbuf_t *mbuf_head,
			    struct mbuf_t *mbuf_tail, int num);
struct sdiohal_list_t *sdiohal_get_rx_mbuf_list(int num);
struct sdiohal_list_t *sdiohal_get_rx_mbuf_node(int num);
int sdiohal_rx_list_dispatch(void);
struct sdiohal_list_t *sdiohal_get_rx_channel_list(int channel);
void *sdiohal_get_rx_free_buf(unsigned int *alloc_size, unsigned int read_len);
void sdiohal_tx_init_retrybuf(void);
int sdiohal_misc_init(void);
void sdiohal_misc_deinit(void);

/* sdiohal main.c */
unsigned int sdiohal_get_trans_pac_num(void);
int sdiohal_sdio_pt_write(unsigned char *src, unsigned int datalen);
int sdiohal_sdio_pt_read(unsigned char *src, unsigned int datalen);
int sdiohal_adma_pt_write(struct sdiohal_list_t *data_list);
int sdiohal_adma_pt_read(struct sdiohal_list_t *data_list);
int sdiohal_tx_data_list_send(struct sdiohal_list_t *data_list,
			      bool pop_flag);
void sdiohal_enable_rx_irq(void);

/* for debugfs */
#if 0
#ifdef CONFIG_DEBUG_FS
void sdiohal_debug_init(void);
void sdiohal_debug_deinit(void);
#endif
#endif
void sdiohal_print_list_data(struct sdiohal_list_t *data_list,
			     const char *func, int loglevel);
void sdiohal_print_mbuf_data(int channel, struct mbuf_t *head,
			     struct mbuf_t *tail, int num,
			     const char *func, int loglevel);

void sdiohal_list_check(struct sdiohal_list_t *data_list,
			const char *func, bool dir);
void sdiohal_mbuf_list_check(int channel, struct mbuf_t *head,
			     struct mbuf_t *tail, int num,
			     const char *func, bool dir, int loglevel);
int sdiohal_list_push(int chn, struct mbuf_t *head,
		      struct mbuf_t *tail, int num);
int sdiohal_list_direct_write(int channel, struct mbuf_t *head,
			      struct mbuf_t *tail, int num);

int sdiohal_init(void);
void sdiohal_exit(void *wcn_dev);

/* driect mode,reg access.etc */
int sdiohal_dt_read(unsigned int addr, void *buf, unsigned int len);
int sdiohal_dt_write(unsigned int addr, void *buf, unsigned int len);
int sdiohal_aon_readb(unsigned int addr, unsigned char *val);
int sdiohal_aon_writeb(unsigned int addr, unsigned char val);
int sdiohal_writel(unsigned int system_addr, void *buf);
int sdiohal_readl(unsigned int system_addr, void *buf);
void sdiohal_dump_aon_reg(void);

/* for dumpmem */
unsigned int sdiohal_get_carddump_status(void);
void sdiohal_set_carddump_status(unsigned int flag);

/* for loopcheck */
unsigned long long sdiohal_get_rx_total_cnt(void);

/* for power on/off */
int sdiohal_runtime_get(void);
int sdiohal_runtime_put(void);

void sdiohal_register_scan_notify(void *func);
int sdiohal_scan_card(void *wcn_dev);
void sdiohal_remove_card(void *wcn_dev);

extern unsigned long long tm_enter_tx_thread;
extern unsigned long long tm_exit_tx_thread;

#endif

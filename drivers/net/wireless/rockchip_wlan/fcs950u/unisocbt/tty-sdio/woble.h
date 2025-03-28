/*
 * btif_woble.h
 *
 *  Created on: 2020
 *      Author: unisoc
 */

#ifndef __WOBLE_H
#define __WOBLE_H

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "WOBLE : " fmt

#define HCI_CMD			0x01
#define HCI_ACL			0x02
#define HCI_SCO			0x03
#define HCI_EVT			0x04

#define PACKET_TYPE         0
#define EVT_HEADER_TYPE     0
#define EVT_HEADER_EVENT    1
#define EVT_HEADER_SIZE     2
#define EVT_VENDOR_CODE_LSB 3
#define EVT_VENDOR_CODE_MSB 4
#define EVT_LE_META_SUBEVT  3
#define EVT_ADV_LENGTH      13

#define BT_HCI_EVT_CMD_COMPLETE    0x0e
#define BT_HCI_EVT_CMD_STATUS      0x0f

#define ACL_HEADER_SIZE_LB  3
#define ACL_HEADER_SIZE_HB  4
#define EVT_HEADER_STATUS   4

#define HCI_CMD_MAX_LEN     258
#define BD_ADDR_LEN         6

#define UINT8_TO_STREAM(p, u8)     {*(p)++ = (uint8_t)(u8); }
#define UINT16_TO_STREAM(p, u16)   {*(p)++ = (uint8_t)(u16); *(p)++ = (uint8_t)((u16) >> 8); }
#define ARRAY_TO_STREAM(p, a, len) {register int ijk; for (ijk = 0; ijk < len; ijk++) *(p)++ = (uint8_t) a[ijk]; }
#define STREAM_TO_UINT8(u8, p)     {u8 = (uint8_t)(*(p)); (p) += 1; }
#define STREAM_TO_UINT16(u16, p)   {u16 = ((uint16_t)(*(p)) + (((uint16_t)(*((p) + 1))) << 8)); (p) += 2; }
#define BDADDR_TO_STREAM(p, a)     {register int ijk; for (ijk = 0; ijk < BD_ADDR_LEN;  ijk++) *(p)++ = (uint8_t) a[BD_ADDR_LEN - 1 - ijk]; }
#define STREAM_TO_ARRAY(a, p, len) {register int ijk; for (ijk = 0; ijk < len; ijk++) ((uint8_t *) a)[ijk] = *p++; }
#define UINT16_TO_STREAM_SAME(p, u16) {*(p)++ = (uint8_t)((u16) >> 8); *(p)++ = (uint8_t)(u16); }

/*
**  Defentions for HCI OP CMD Codes.
*/
#define BT_HCI_OP_RESET            		0x0c03
#define BT_HCI_OP_ENABLE           		0xfca1
#define BT_HCI_OP_WOBLE            		0xfd08
#define BT_HCI_OP_READ_BDADR	    		0x1009
#define BT_HCI_OP_DISCONNECT       		0x0406
#define BT_HCI_OP_SET_SLEEPMODE    		0xfd09
#define BT_HCI_OP_ADD_WAKEUPLIST   		0xfd0a
#define BT_HCI_OP_SET_STARTSLEEP   		0xfd0d
#define BT_HCI_OP_CLEANUP_WAKEUPLIST    	0xfd0b
#define BT_HCI_OP_LE_SET_SCAN_PARAMETERS     	0x200b
#define BT_HCI_OP_LE_SET_SCAN_ENABLE         	0x200c
#define BT_HCI_OP_LE_SET_EX_SCAN_PARAMETERS  	0x2041
#define BT_HCI_OP_LE_SET_EX_SCAN_ENABLE      	0x2042

/*
**  Defentions for HCI OP EVT Codes.
*/
#define LE_HCI_EVT_CMD_META			0x3e
#define LE_HCI_EVT_CMD_CONN_COMPLETE		0x01
#define LE_HCI_EVT_CMD_ENHANCED_CONN_COMPLETE	0x0a
#define LE_HCI_EVT_CMD_DISCONNECTION_COMPLETE  	0x05

/*
**  Defentions for Others.
*/
#define UNISOC_WOBLE_UUID           		0xfd01
#define WOBLE_DEVICES_SIZE         		10

/*
**  Defentions for HCI Error Codes that are past in the events
*/
#define STATUS_SUCCESS				0x00
#define HCI_ERR_PEER_USER			0x13


struct HC_BT_HDR {
	unsigned short event;
	unsigned short len;
	unsigned short offset;
	unsigned short layer_specific;
	unsigned char data[];
};

struct hci_cmd_t {
	unsigned short opcode;
	struct semaphore wait;
	struct HC_BT_HDR response;
};

typedef enum {
	WOBLE_MOD_DISABLE = 0,
	WOBLE_MOD_ENABLE,
	WOBLE_MOD_UNDEFINE = 0xff,
} WOBLE_MOD;

typedef enum {
	WOBLE_SLEEP_MOD_COULD_KNOW = 0,
	WOBLE_SLEEP_MOD_COULD_NOT_KNOW
} WOBLE_SLEEP_MOD;

typedef enum {
	WOBLE_SLEEP_MOD_NOT_NEED_NOTITY = 0,
	WOBLE_SLEEP_MOD_NEED_NOTITY,
} WOBLE_NOFITY_MOD;

typedef enum {
	WOBLE_WAKE_MOD_ALL_ADV_DATA = (1 << 0),
	WOBLE_WAKE_MOD_ALL_ACL_DATA = (1 << 1),
	WOBLE_WAKE_MOD_SPECIAL_ADV_DATA = (1 << 2),
	WOBLE_WAKE_MOD_SPECIAL_ACL_DATA = (1 << 3),
	WOBLE_WAKE_MOD_SUBADV_FILTER_DATA = (1 << 4),
} WOBLE_WAKE_MOD;

typedef enum {
    ADV_PUBLIC_ADDRESS = 0,
    ADV_RANDOM_ADDRESS,
    ADV_ADDRESS_FROM_WHITTLIST = 0XFF,
} ADV_ADDRESS_TYPE;

typedef enum {
	WOBLE_DISCONNECT_MOD_NOT = 0,
	WOBLE_DISCONNECT_MOD_WILL
} WOBLE_DISCONNECT_MOD;

typedef enum {
	WOBLE_ADV_WAKE_MOD_RAW_DATA = (1 << 0),
	WOBLE_ADV_WAKE_MOD_ADTYE = (1 << 1),
} WOBLE_ADV_WAKE_MOD;

typedef enum {
	WOBLE_IS_NOT_SHUTDOWN = 0,
	WOBLE_IS_SHUTDOWN,
} WOBLE_SHUTDOWN_MOD;

typedef enum {
	WOBLE_IS_NOT_RESUME = 0,
	WOBLE_IS_RESUME,
} WOBLE_RESUME_MOD;

typedef enum {
	SUSPEND_IS_NOT_BT = 0,
	SUSPNED_IS_BT,
}WOBLE_SWITCH_ROLE;

typedef struct {
	uint8_t woble_mod;
	uint8_t sleep_mod;
	uint16_t timeout;
	uint8_t notify;
} woble_config_t;

typedef struct mtty_bt_wake_t {
	uint8_t addr[6];
	char *addr_str;
	uint8_t dev_tp;
	char *dev_tp_str;
	uint8_t addr_tp;
	char *addr_tp_str;
} mtty_bt_wake_t;

typedef struct {
	uint8_t suspend_role; /* 0:infrared, 1:bt */
	uint16_t conn_handle;
	uint8_t driver_diconn; /* 1:disconnect from driver */
}woble_switch_t;

extern woble_switch_t woble_switch;

int woble_init(void);
int mtty_bt_str_hex(char *str, uint8_t count, char *hex);
int mtty_bt_conf_prase(char *conf_str);
int mtty_bt_read_conf(void);
int woble_data_recv(const unsigned char *buf, int count);
int hci_cmd_send_sync(unsigned short opcode, struct HC_BT_HDR *py, struct HC_BT_HDR *rsp);
void hci_disconnect(void);
void hci_set_ap_sleep_mode(int is_shutdown, int is_resume);
void hci_cleanup_wakeup_list(void);
void hci_add_device_to_wakeup_list(void);
int bt_tx_powerchange(int channel, int is_resume);
void hci_set_ap_start_sleep(void);
void hci_set_scan_parameters(void);
void hci_set_scan_enable(int enable);
void hci_evt_disconnect_complete(void);

#endif


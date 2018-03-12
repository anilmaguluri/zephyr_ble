/*
 * Copyright (c) 2016-2018 Nordic Semiconductor ASA
 * Copyright (c) 2016 Vinayak Kariappa Chettimada
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _CTRL_H_
#define _CTRL_H_

/*****************************************************************************
 * Zephyr Kconfig defined
 ****************************************************************************/
#ifdef CONFIG_BT_MAX_CONN
#define RADIO_CONNECTION_CONTEXT_MAX CONFIG_BT_MAX_CONN
#else
#define RADIO_CONNECTION_CONTEXT_MAX 0
#endif

#ifdef CONFIG_BT_CTLR_RX_BUFFERS
#define RADIO_PACKET_COUNT_RX_MAX CONFIG_BT_CTLR_RX_BUFFERS
#endif

#ifdef CONFIG_BT_CTLR_TX_BUFFERS
#define RADIO_PACKET_COUNT_TX_MAX CONFIG_BT_CTLR_TX_BUFFERS
#endif

#ifdef CONFIG_BT_CTLR_TX_BUFFER_SIZE
#define RADIO_PACKET_TX_DATA_SIZE CONFIG_BT_CTLR_TX_BUFFER_SIZE
#endif

/*****************************************************************************
 * Timer Resources (Controller defined)
 ****************************************************************************/
#define RADIO_TICKER_ID_EVENT		 0
#define RADIO_TICKER_ID_MARKER_0	 1
#define RADIO_TICKER_ID_PRE_EMPT	 2
#define RADIO_TICKER_ID_ADV_STOP	 3
#define RADIO_TICKER_ID_SCAN_STOP	 4
#define RADIO_TICKER_ID_ADV		 5
#define RADIO_TICKER_ID_SCAN		 6
#define RADIO_TICKER_ID_FIRST_CONNECTION 7

#define RADIO_TICKER_INSTANCE_ID_RADIO	 0
#define RADIO_TICKER_INSTANCE_ID_APP	 1

#define RADIO_TICKER_USERS		 3

#define RADIO_TICKER_USER_ID_WORKER	 MAYFLY_CALL_ID_0
#define RADIO_TICKER_USER_ID_JOB	 MAYFLY_CALL_ID_1
#define RADIO_TICKER_USER_ID_APP	 MAYFLY_CALL_ID_PROGRAM

#define RADIO_TICKER_USER_WORKER_OPS	(7 + 1)
#define RADIO_TICKER_USER_JOB_OPS	(2 + 1)
#define RADIO_TICKER_USER_APP_OPS	(1 + 1)
#define RADIO_TICKER_USER_OPS		(RADIO_TICKER_USER_WORKER_OPS \
					+ RADIO_TICKER_USER_JOB_OPS \
					+ RADIO_TICKER_USER_APP_OPS \
					)

#define RADIO_TICKER_NODES		(RADIO_TICKER_ID_FIRST_CONNECTION \
					+ RADIO_CONNECTION_CONTEXT_MAX \
					)

/*****************************************************************************
 * Controller Reference Defines (compile time override-able)
 ****************************************************************************/
/* Minimum LL Payload support (Dont change). */
#define RADIO_LL_LENGTH_OCTETS_RX_MIN	27

/* Maximum LL Payload support (27 to 251). */
#ifndef RADIO_LL_LENGTH_OCTETS_RX_MAX
#define RADIO_LL_LENGTH_OCTETS_RX_MAX	251
#endif

/* Implementation default L2CAP MTU */
#ifndef RADIO_L2CAP_MTU_MAX
#define RADIO_L2CAP_MTU_MAX		(RADIO_LL_LENGTH_OCTETS_RX_MAX - 4)
#endif

/* Maximise L2CAP MTU to LL data PDU size */
#if (RADIO_L2CAP_MTU_MAX < (RADIO_LL_LENGTH_OCTETS_RX_MAX - 4))
#undef RADIO_L2CAP_MTU_MAX
#define RADIO_L2CAP_MTU_MAX		(RADIO_LL_LENGTH_OCTETS_RX_MAX - 4)
#endif

/* Maximum LL PDU Receive pool size. */
#ifndef RADIO_PACKET_COUNT_RX_MAX
#define RADIO_PACKET_COUNT_RX		((RADIO_L2CAP_MTU_MAX + \
						RADIO_LL_LENGTH_OCTETS_RX_MAX \
						+ 3) \
						/ \
						RADIO_LL_LENGTH_OCTETS_RX_MAX \
					)
#define RADIO_PACKET_COUNT_RX_MAX	(RADIO_PACKET_COUNT_RX + \
					((RADIO_CONNECTION_CONTEXT_MAX - 1) * \
					(RADIO_PACKET_COUNT_RX - 1)) \
					)
#endif /* RADIO_PACKET_COUNT_RX_MAX */

/* Maximum LL PDU Transmit pool size and application tx count. */
#ifndef RADIO_PACKET_COUNT_TX_MAX
#define RADIO_PACKET_COUNT_APP_TX_MAX	(RADIO_CONNECTION_CONTEXT_MAX)
#define RADIO_PACKET_COUNT_TX_MAX	(RADIO_PACKET_COUNT_RX_MAX + \
						RADIO_PACKET_COUNT_APP_TX_MAX \
					)
#else
#define RADIO_PACKET_COUNT_APP_TX_MAX	(RADIO_PACKET_COUNT_TX_MAX)
#endif

/* Tx Data Size */
#if !defined(RADIO_PACKET_TX_DATA_SIZE) || \
	(RADIO_PACKET_TX_DATA_SIZE < RADIO_LL_LENGTH_OCTETS_RX_MIN)
#define RADIO_PACKET_TX_DATA_SIZE RADIO_LL_LENGTH_OCTETS_RX_MIN
#endif

/*****************************************************************************
 * Controller Interface Structures
 ****************************************************************************/
struct radio_pdu_node_tx {
	void *next;
	u8_t pdu_data[1];
};

enum radio_pdu_node_rx_type {
	NODE_RX_TYPE_NONE,
	NODE_RX_TYPE_DC_PDU,
	NODE_RX_TYPE_REPORT,

#if defined(CONFIG_BT_CTLR_ADV_EXT)
	NODE_RX_TYPE_EXT_1M_REPORT,
	NODE_RX_TYPE_EXT_CODED_REPORT,
#endif /* CONFIG_BT_CTLR_ADV_EXT */

#if defined(CONFIG_BT_CTLR_SCAN_REQ_NOTIFY)
	NODE_RX_TYPE_SCAN_REQ,
#endif /* CONFIG_BT_CTLR_SCAN_REQ_NOTIFY */

	NODE_RX_TYPE_CONNECTION,
	NODE_RX_TYPE_TERMINATE,
	NODE_RX_TYPE_CONN_UPDATE,
	NODE_RX_TYPE_ENC_REFRESH,

#if defined(CONFIG_BT_CTLR_LE_PING)
	NODE_RX_TYPE_APTO,
#endif /* CONFIG_BT_CTLR_LE_PING */

	NODE_RX_TYPE_CHAN_SEL_ALGO,

#if defined(CONFIG_BT_CTLR_PHY)
	NODE_RX_TYPE_PHY_UPDATE,
#endif /* CONFIG_BT_CTLR_PHY */

#if defined(CONFIG_BT_CTLR_CONN_RSSI)
	NODE_RX_TYPE_RSSI,
#endif /* CONFIG_BT_CTLR_CONN_RSSI */

#if defined(CONFIG_BT_CTLR_PROFILE_ISR)
	NODE_RX_TYPE_PROFILE,
#endif /* CONFIG_BT_CTLR_PROFILE_ISR */

#if defined(CONFIG_BT_CTLR_ADV_INDICATION)
	NODE_RX_TYPE_ADV_INDICATION,
#endif /* CONFIG_BT_CTLR_ADV_INDICATION */

#if defined(CONFIG_BT_HCI_MESH_EXT)
	NODE_RX_TYPE_MESH_ADV_CPLT,
	NODE_RX_TYPE_MESH_REPORT,
#endif /* CONFIG_BT_HCI_MESH_EXT */
};

struct radio_le_conn_cmplt {
	u8_t  status;
	u8_t  role;
	u8_t  peer_addr_type;
	u8_t  peer_addr[BDADDR_SIZE];
#if defined(CONFIG_BT_CTLR_PRIVACY)
	u8_t  peer_rpa[BDADDR_SIZE];
	u8_t  own_addr_type;
	u8_t  own_addr[BDADDR_SIZE];
#endif /* CONFIG_BT_CTLR_PRIVACY */
	u16_t interval;
	u16_t latency;
	u16_t timeout;
	u8_t  mca;
} __packed;

struct radio_le_conn_update_cmplt {
	u8_t  status;
	u16_t interval;
	u16_t latency;
	u16_t timeout;
} __packed;

struct radio_le_chan_sel_algo {
	u8_t chan_sel_algo;
} __packed;

struct radio_le_phy_upd_cmplt {
	u8_t status;
	u8_t tx;
	u8_t rx;
} __packed;

struct radio_pdu_node_rx_hdr {
	union {
		sys_snode_t node; /* used by slist */
		void *next; /* used also by k_fifo once pulled */
		void *link;
		u8_t packet_release_last;
	} onion;

	enum radio_pdu_node_rx_type type;
	u16_t handle;
};

struct radio_pdu_node_rx {
	struct radio_pdu_node_rx_hdr hdr;
	u8_t   pdu_data[1];
};

/*****************************************************************************
 * Controller Interface Functions
 ****************************************************************************/
/* Downstream */
u32_t radio_init(void *hf_clock, u8_t sca, void *entropy,
		 u8_t connection_count_max,
		 u8_t rx_count_max, u8_t tx_count_max,
		 u16_t packet_data_octets_max,
		 u16_t packet_tx_data_size, u8_t *mem_radio,
		 u16_t mem_size);
struct device *radio_hf_clock_get(void);
void radio_ticks_active_to_start_set(u32_t ticks_active_to_start);
/* Downstream - Advertiser */
struct radio_adv_data *radio_adv_data_get(void);
struct radio_adv_data *radio_scan_data_get(void);

#if defined(CONFIG_BT_HCI_MESH_EXT)
#if defined(CONFIG_BT_CTLR_ADV_EXT)
u32_t radio_adv_enable(u8_t phy_p, u16_t interval, u8_t chan_map,
		       u8_t filter_policy, u8_t rl_idx,
#else /* !CONFIG_BT_CTLR_ADV_EXT */
u32_t radio_adv_enable(u16_t interval, u8_t chan_map, u8_t filter_policy,
		       u8_t rl_idx,
#endif /* !CONFIG_BT_CTLR_ADV_EXT */
		       u8_t at_anchor, u32_t ticks_anchor, u8_t retry,
		       u8_t scan_window, u8_t scan_delay);
#else /* !CONFIG_BT_HCI_MESH_EXT */
#if defined(CONFIG_BT_CTLR_ADV_EXT)
u32_t radio_adv_enable(u8_t phy_p, u16_t interval, u8_t chan_map,
		       u8_t filter_policy, u8_t rl_idx);
#else /* !CONFIG_BT_CTLR_ADV_EXT */
u32_t radio_adv_enable(u16_t interval, u8_t chan_map, u8_t filter_policy,
		       u8_t rl_idx);
#endif /* !CONFIG_BT_CTLR_ADV_EXT */
#endif /* !CONFIG_BT_HCI_MESH_EXT */

u32_t radio_adv_disable(void);
u32_t radio_adv_filter_pol_get(void);
/* Downstream - Scanner */
u32_t radio_scan_enable(u8_t type, u8_t init_addr_type, u8_t *init_addr,
			u16_t interval, u16_t window, u8_t filter_policy,
			u8_t rpa_gen, u8_t rl_idx);
u32_t radio_scan_disable(void);
u32_t ll_scan_is_enabled(void);
u32_t radio_scan_filter_pol_get(void);

u32_t radio_connect_enable(u8_t adv_addr_type, u8_t *adv_addr,
			   u16_t interval, u16_t latency,
			   u16_t timeout);
/* Upstream */
u8_t radio_rx_fc_set(u16_t handle, u8_t fc);
u8_t radio_rx_fc_get(u16_t *handle);

/* Callbacks */
extern void radio_active_callback(u8_t active);
extern void radio_event_callback(void);
extern void ll_adv_scan_state_cb(u8_t bm);

#endif

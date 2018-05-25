/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define SYS_LOG_LEVEL 4
#define SYS_LOG_DOMAIN "main"
#include <logging/sys_log.h>

#include <zephyr.h>

#include <usb/usb_device.h>
#include <usb/class/usb_hid.h>

#define REPORT_ID_1	0x01
#define REPORT_ID_2	0x02

/* size of stack area used by each thread */
#define STACKSIZE               1024

/* scheduling priority used by each thread */
#define PRIORITY                7

K_SEM_DEFINE(usb_sem, 0, 1);

u8_t rdata[CONFIG_HID_INTERRUPT_EP_MPS];
u32_t glen;

/* Some HID sample Report Descriptor */
static const u8_t hid_report_desc[] = {
	/* 0x05, 0x01,		USAGE_PAGE (Generic Desktop)		*/
	HID_GI_USAGE_PAGE, USAGE_GEN_DESKTOP,
	/* 0x09, 0x00,		USAGE (Undefined)			*/
	HID_LI_USAGE, USAGE_GEN_DESKTOP_UNDEFINED,
	/* 0xa1, 0x01,		COLLECTION (Application)		*/
	HID_MI_COLLECTION, COLLECTION_APPLICATION,
	/* 0x15, 0x00,			LOGICAL_MINIMUM one-byte (0)	*/
	HID_GI_LOGICAL_MIN(1), 0x00,
	/* 0x26, 0xff, 0x00,		LOGICAL_MAXIMUM two-bytes (255)	*/
	HID_GI_LOGICAL_MAX(2), 0xFF, 0x00,
	/* 0x85, 0x01,			REPORT_ID (1)			*/
	HID_GI_REPORT_ID, REPORT_ID_1,
	/* 0x75, 0x08,			REPORT_SIZE (8) in bits		*/
	HID_GI_REPORT_SIZE, 0x08,
	/* 0x95, 0x01,			REPORT_COUNT (1)		*/
	HID_GI_REPORT_COUNT, 0x01,
	/* 0x09, 0x00,			USAGE (Undefined)		*/
	HID_LI_USAGE, USAGE_GEN_DESKTOP_UNDEFINED,
	/* v0x81, 0x82,			INPUT (Data,Var,Abs,Vol)	*/
	HID_MI_INPUT, 0x82,
	/* 0x85, 0x02,			REPORT_ID (2)			*/
	HID_GI_REPORT_ID, REPORT_ID_2,
	/* 0x75, 0x08,			REPORT_SIZE (8) in bits		*/
	HID_GI_REPORT_SIZE, 0x08,
	/* 0x95, 0x01,			REPORT_COUNT (1)		*/
	HID_GI_REPORT_COUNT, 0x01,
	/* 0x09, 0x00,			USAGE (Undefined)		*/
	HID_LI_USAGE, USAGE_GEN_DESKTOP_UNDEFINED,
	/* 0x91, 0x82,			OUTPUT (Data,Var,Abs,Vol)	*/
	HID_MI_OUTPUT, 0x82,
	/* 0xc0			END_COLLECTION			*/
	HID_MI_COLLECTION_END,
};

int debug_cb(struct usb_setup_packet *setup, s32_t *len,
	     u8_t **data)
{
	SYS_LOG_DBG("Debug callback");

	return -ENOTSUP;
}

int set_idle_cb(struct usb_setup_packet *setup, s32_t *len,
		u8_t **data)
{
	SYS_LOG_DBG("Set Idle callback");

	/* TODO: Do something */

	return 0;
}

int get_report_cb(struct usb_setup_packet *setup, s32_t *len,
		  u8_t **data)
{
	SYS_LOG_DBG("Get report callback");

	/* TODO: Do something */

	return 0;
}

void read_cb(void *data, u32_t len)
{
	memcpy(rdata, data, len);
	glen = len;
	k_sem_give(&usb_sem);
}

static struct hid_ops ops = {
	.get_report = get_report_cb,
	.get_idle = debug_cb,
	.get_protocol = debug_cb,
	.set_report = debug_cb,
	.set_idle = set_idle_cb,
	.set_protocol = debug_cb,
#ifdef CONFIG_ENABLE_HID_INT_OUT_EP
	.int_out_read = read_cb,
#endif
};

void hid_thread(void)
{
	u8_t report_1[2] = { REPORT_ID_1, 0x00 };
	int ret, wrote, i;

	SYS_LOG_DBG("Starting application");

	usb_hid_register_device(hid_report_desc, sizeof(hid_report_desc), &ops);

	usb_hid_init();

	while (true) {
		k_sem_take(&usb_sem, K_FOREVER);

		for (i = 0; i < glen; i++) {
			printk("data read from host = %x\n", rdata[i]);
		}

		k_sleep(K_SECONDS(1));

		report_1[1]++;

		ret = usb_write(CONFIG_HID_INT_IN_EP_ADDR, report_1,
				sizeof(report_1), &wrote);
		SYS_LOG_DBG("Wrote %d bytes with ret %d", wrote, ret);
	}
}

K_THREAD_DEFINE(hid_thread_id, STACKSIZE, hid_thread, NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);

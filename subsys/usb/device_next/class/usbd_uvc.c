/*
 * Copyright (c) 2024 tinyVision.ai Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_uvc_device

#include <zephyr/init.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/video-controls.h>
#include <zephyr/logging/log.h>

#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usb_ch9.h>

LOG_MODULE_REGISTER(usbd_uvc, CONFIG_USBD_UVC_LOG_LEVEL);

/* Video Class-Specific Request Codes */
#define RC_UNDEFINED 0x00
#define SET_CUR      0x01
#define GET_CUR      0x81
#define GET_MIN      0x82
#define GET_MAX      0x83
#define GET_RES      0x84
#define GET_LEN      0x85
#define GET_INFO     0x86
#define GET_DEF      0x87

/* Flags announcing which controls are supported */
#define INFO_SUPPORTS_GET BIT(0)
#define INFO_SUPPORTS_SET BIT(1)

/* 4.2.1.2 Request Error Code Control */
#define ERR_NOT_READY                  0x01
#define ERR_WRONG_STATE                0x02
#define ERR_OUT_OF_RANGE               0x04
#define ERR_INVALID_UNIT               0x05
#define ERR_INVALID_CONTROL            0x06
#define ERR_INVALID_REQUEST            0x07
#define ERR_INVALID_VALUE_WITHIN_RANGE 0x08
#define ERR_UNKNOWN                    0xff

/* Video and Still Image Payload Headers */
#define UVC_BMHEADERINFO_FRAMEID              BIT(0)
#define UVC_BMHEADERINFO_END_OF_FRAME         BIT(1)
#define UVC_BMHEADERINFO_HAS_PRESENTATIONTIME BIT(2)
#define UVC_BMHEADERINFO_HAS_SOURCECLOCK      BIT(3)
#define UVC_BMHEADERINFO_PAYLOAD_SPECIFIC_BIT BIT(4)
#define UVC_BMHEADERINFO_STILL_IMAGE          BIT(5)
#define UVC_BMHEADERINFO_ERROR                BIT(6)
#define UVC_BMHEADERINFO_END_OF_HEADER        BIT(7)

/* Video Interface Subclass Codes */
#define SC_UNDEFINED				0x00
#define SC_VIDEOCONTROL				0x01
#define SC_VIDEOSTREAMING			0x02
#define SC_VIDEO_INTERFACE_COLLECITON		0x03

/* Video Interface Protocol Codes */
#define PC_PROTOCOL_UNDEFINED			0x00
#define PC_PROTOCOL_VERSION_1_5			0x01

/* Video Class-Specific Video Control Interface Descriptor Subtypes */
#define VC_DESCRIPTOR_UNDEFINED			0x00
#define VC_HEADER				0x01
#define VC_INPUT_TERMINAL			0x02
#define VC_OUTPUT_TERMINAL			0x03
#define VC_SELECTOR_UNIT			0x04
#define VC_PROCESSING_UNIT			0x05
#define VC_EXTENSION_UNIT			0x06
#define VC_ENCODING_UNIT			0x07

/* Video Class-Specific Video Stream Interface Descriptor Subtypes */
#define VS_UNDEFINED				0x00
#define VS_INPUT_HEADER				0x01
#define VS_OUTPUT_HEADER			0x02
#define VS_STILL_IMAGE_FRAME			0x03
#define VS_FORMAT_UNCOMPRESSED			0x04
#define VS_FRAME_UNCOMPRESSED			0x05
#define VS_FORMAT_MJPEG				0x06
#define VS_FRAME_MJPEG				0x07
#define VS_FORMAT_MPEG2TS			0x0A
#define VS_FORMAT_DV				0x0C
#define VS_COLORFORMAT				0x0D
#define VS_FORMAT_FRAME_BASED			0x10
#define VS_FRAME_FRAME_BASED			0x11
#define VS_FORMAT_STREAM_BASED			0x12
#define VS_FORMAT_H264				0x13
#define VS_FRAME_H264				0x14
#define VS_FORMAT_H264_SIMULCAST		0x15
#define VS_FORMAT_VP8				0x16
#define VS_FRAME_VP8				0x17
#define VS_FORMAT_VP8_SIMULCAST			0x18

/* Video Class-Specific Endpoint Descriptor Subtypes */
#define EP_UNDEFINED				0x00
#define EP_GENERAL				0x01
#define EP_ENDPOINT				0x02
#define EP_INTERRUPT				0x03

/* VideoControl Interface Selectors */
#define VC_CONTROL_UNDEFINED			0x00
#define VC_VIDEO_POWER_MODE_CONTROL		0x01
#define VC_REQUEST_ERROR_CODE_CONTROL		0x02

/* USB Terminal Types */
#define TT_VENDOR_SPECIFIC			0x0100
#define TT_STREAMING				0x0101

/* Input Terminal Types */
#define ITT_VENDOR_SPECIFIC			0x0200
#define ITT_CAMERA				0x0201
#define ITT_MEDIA_TRANSPORT_INPUT		0x0202

/* Output Terminal Types */
#define OTT_VENDOR_SPECIFIC			0x0300
#define OTT_DISPLAY				0x0301
#define OTT_MEDIA_TRANSPORT_OUTPUT		0x0302

/* External Terminal Types */
#define EXT_EXTERNAL_VENDOR_SPECIFIC		0x0400
#define EXT_COMPOSITE_CONNECTOR			0x0401
#define EXT_SVIDEO_CONNECTOR			0x0402
#define EXT_COMPONENT_CONNECTOR			0x0403

/* UVC controls for "zephyr,uvc-control-su" */
#define SU_INPUT_SELECT_CONTROL			0x01

/* UVC controls for "zephyr,uvc-control-ct" */
#define CT_SCANNING_MODE_CONTROL		0x01
#define CT_SCANNING_MODE_BIT			0
#define CT_AE_MODE_CONTROL			0x02
#define CT_AE_MODE_BIT				1
#define CT_AE_PRIORITY_CONTROL			0x03
#define CT_AE_PRIORITY_BIT			2
#define CT_EXPOSURE_TIME_ABSOLUTE_CONTROL	0x04
#define CT_EXPOSURE_TIME_ABSOLUTE_BIT		3
#define CT_EXPOSURE_TIME_RELATIVE_CONTROL	0x05
#define CT_EXPOSURE_TIME_RELATIVE_BIT		4
#define CT_FOCUS_ABSOLUTE_CONTROL		0x06
#define CT_FOCUS_ABSOLUTE_BIT			5
#define CT_FOCUS_RELATIVE_CONTROL		0x07
#define CT_FOCUS_RELATIVE_BIT			6
#define CT_FOCUS_AUTO_CONTROL			0x08
#define CT_FOCUS_AUTO_BIT			17
#define CT_IRIS_ABSOLUTE_CONTROL		0x09
#define CT_IRIS_ABSOLUTE_BIT			7
#define CT_IRIS_RELATIVE_CONTROL		0x0A
#define CT_IRIS_RELATIVE_BIT			8
#define CT_ZOOM_ABSOLUTE_CONTROL		0x0B
#define CT_ZOOM_ABSOLUTE_BIT			9
#define CT_ZOOM_RELATIVE_CONTROL		0x0C
#define CT_ZOOM_RELATIVE_BIT			10
#define CT_PANTILT_ABSOLUTE_CONTROL		0x0D
#define CT_PANTILT_ABSOLUTE_BIT			11
#define CT_PANTILT_RELATIVE_CONTROL		0x0E
#define CT_PANTILT_RELATIVE_BIT			12
#define CT_ROLL_ABSOLUTE_CONTROL		0x0F
#define CT_ROLL_ABSOLUTE_BIT			13
#define CT_ROLL_RELATIVE_CONTROL		0x10
#define CT_ROLL_RELATIVE_BIT			14
#define CT_PRIVACY_CONTROL			0x11
#define CT_PRIVACY_BIT				18
#define CT_FOCUS_SIMPLE_CONTROL			0x12
#define CT_FOCUS_SIMPLE_BIT			19
#define CT_WINDOW_CONTROL			0x13
#define CT_WINDOW_BIT				20
#define CT_REGION_OF_INTEREST_CONTROL		0x14
#define CT_REGION_OF_INTEREST_BIT		21

/* UVC controls for "zephyr,uvc-control-pu" */
#define PU_BACKLIGHT_COMPENSATION_CONTROL	0x01
#define PU_BACKLIGHT_COMPENSATION_BIT		8
#define PU_BRIGHTNESS_CONTROL			0x02
#define PU_BRIGHTNESS_BIT			0
#define PU_CONTRAST_CONTROL			0x03
#define PU_CONTRAST_BIT				1
#define PU_GAIN_CONTROL				0x04
#define PU_GAIN_BIT				9
#define PU_POWER_LINE_FREQUENCY_CONTROL		0x05
#define PU_POWER_LINE_FREQUENCY_BIT		10
#define PU_HUE_CONTROL				0x06
#define PU_HUE_BIT				2
#define PU_SATURATION_CONTROL			0x07
#define PU_SATURATION_BIT			3
#define PU_SHARPNESS_CONTROL			0x08
#define PU_SHARPNESS_BIT			4
#define PU_GAMMA_CONTROL			0x09
#define PU_GAMMA_BIT				5
#define PU_WHITE_BALANCE_TEMPERATURE_CONTROL	0x0A
#define PU_WHITE_BALANCE_TEMPERATURE_BIT	6
#define PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL 0x0B
#define PU_WHITE_BALANCE_TEMPERATURE_AUTO_BIT	12
#define PU_WHITE_BALANCE_COMPONENT_CONTROL	0x0C
#define PU_WHITE_BALANCE_COMPONENT_BIT		7
#define PU_WHITE_BALANCE_COMPONENT_AUTO_CONTROL	0x0D
#define PU_WHITE_BALANCE_COMPONENT_AUTO_BIT	13
#define PU_DIGITAL_MULTIPLIER_CONTROL		0x0E
#define PU_DIGITAL_MULTIPLIER_BIT		14
#define PU_DIGITAL_MULTIPLIER_LIMIT_CONTROL	0x0F
#define PU_DIGITAL_MULTIPLIER_LIMIT_BIT		15
#define PU_HUE_AUTO_CONTROL			0x10
#define PU_HUE_AUTO_BIT				11
#define PU_ANALOG_VIDEO_STANDARD_CONTROL	0x11
#define PU_ANALOG_VIDEO_STANDARD_BIT		16
#define PU_ANALOG_LOCK_STATUS_CONTROL		0x12
#define PU_ANALOG_LOCK_STATUS_BIT		17
#define PU_CONTRAST_AUTO_CONTROL		0x13
#define PU_CONTRAST_AUTO_BIT			18

/* UVC controls for "zephyr,uvc-control-eu" */
#define EU_SELECT_LAYER_CONTROL			0x01
#define EU_SELECT_LAYER_BIT			0
#define EU_PROFILE_TOOLSET_CONTROL		0x02
#define EU_PROFILE_TOOLSET_BIT			1
#define EU_VIDEO_RESOLUTION_CONTROL		0x03
#define EU_VIDEO_RESOLUTION_BIT			2
#define EU_MIN_FRAME_INTERVAL_CONTROL		0x04
#define EU_MIN_FRAME_INTERVAL_BIT		3
#define EU_SLICE_MODE_CONTROL			0x05
#define EU_SLICE_MODE_BIT			4
#define EU_RATE_CONTROL_MODE_CONTROL		0x06
#define EU_RATE_CONTROL_MODE_BIT		5
#define EU_AVERAGE_BITRATE_CONTROL		0x07
#define EU_AVERAGE_BITRATE_BIT			6
#define EU_CPB_SIZE_CONTROL			0x08
#define EU_CPB_SIZE_BIT				7
#define EU_PEAK_BIT_RATE_CONTROL		0x09
#define EU_PEAK_BIT_RATE_BIT			8
#define EU_QUANTIZATION_PARAMS_CONTROL		0x0A
#define EU_QUANTIZATION_PARAMS_BIT		9
#define EU_SYNC_REF_FRAME_CONTROL		0x0B
#define EU_SYNC_REF_FRAME_BIT			10
#define EU_LTR_BUFFER_CONTROL			0x0C
#define EU_LTR_BUFFER_BIT			11
#define EU_LTR_PICTURE_CONTROL			0x0D
#define EU_LTR_PICTURE_BIT			12
#define EU_LTR_VALIDATION_CONTROL		0x0E
#define EU_LTR_VALIDATION_BIT			13
#define EU_LEVEL_IDC_LIMIT_CONTROL		0x0F
#define EU_LEVEL_IDC_LIMIT_BIT			14
#define EU_SEI_PAYLOADTYPE_CONTROL		0x10
#define EU_SEI_PAYLOADTYPE_BIT			15
#define EU_QP_RANGE_CONTROL			0x11
#define EU_QP_RANGE_BIT				16
#define EU_PRIORITY_CONTROL			0x12
#define EU_PRIORITY_BIT				17
#define EU_START_OR_STOP_LAYER_CONTROL		0x13
#define EU_START_OR_STOP_LAYER_BIT		18
#define EU_ERROR_RESILIENCY_CONTROL		0x14
#define EU_ERROR_RESILIENCY_BIT			19

/* UVC controls for "zephyr,uvc-control-xu" */
#define XU_BASE_CONTROL				0x00
#define XU_BASE_BIT				0

/* UVC controls for Video Streaming Interface */
#define VS_PROBE_CONTROL			0x01
#define VS_COMMIT_CONTROL			0x02
#define VS_STILL_PROBE_CONTROL			0x03
#define VS_STILL_COMMIT_CONTROL			0x04
#define VS_STILL_IMAGE_TRIGGER_CONTROL		0x05
#define VS_STREAM_ERROR_CODE_CONTROL		0x06
#define VS_GENERATE_KEY_FRAME_CONTROL		0x07
#define VS_UPDATE_FRAME_SEGMENT_CONTROL		0x08
#define VS_SYNCH_DELAY_CONTROL			0x09

/* Turn larger types into list of bytes */
#define U(value, shift) (((value) >> shift) & 0xFF)
#define U16_LE(n) U(n, 0), U(n, 8)
#define U24_LE(n) U(n, 0), U(n, 8), U(n, 16)
#define U32_LE(n) U(n, 0), U(n, 8), U(n, 16), U(n, 24)
#define U40_LE(n) U(n, 0), U(n, 8), U(n, 16), U(n, 24), U(n, 32)
#define U48_LE(n) U(n, 0), U(n, 8), U(n, 16), U(n, 24), U(n, 32), U(n, 40)
#define U56_LE(n) U(n, 0), U(n, 8), U(n, 16), U(n, 24), U(n, 32), U(n, 40), U(n, 48)
#define U64_LE(n) U(n, 0), U(n, 8), U(n, 16), U(n, 24), U(n, 32), U(n, 40), U(n, 48), U(n, 56)
#define MEDIA_FOUNDATION 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71
#define GUID(fourcc) U32_LE(fourcc), MEDIA_FOUNDATION

/* Use the devicetree ordinal IDs as a source for the USB descriptor IDs */
#define NODE_ID(node) DT_DEP_ORD(node)

/* Expand fn(node) whenever the node has the mentioned property */
#define IF_HAS_PROP(node, prop, fn)						\
	IF_ENABLED(DT_NODE_HAS_PROP(node, prop), (fn(node)))

/* Map DT_STRING_UPPER_TOKEN(node, compatible) of a node to a bitmap */
#define VC_CONTROL_EU(node, t) 0
#define VC_CONTROL_PU(node, t) 0
#define VC_CONTROL_CT(node, t) 0
#define VC_CONTROL_XU(node, t) 0
#define VC_CONTROL_OT(node, t) 0
#define VC_CONTROL_IT(node, t) 0

/* Estimate the frame buffer size out of other fields */
#define FRAME_BUFFER_SIZE(node, prop, id)					\
	(DT_PHA_BY_IDX(node, prop, id, width) *					\
	 DT_PHA_BY_IDX(node, prop, id, height) *				\
	 DT_PHA_BY_IDX(node, prop, id, bits_per_pixel) / 8 +			\
	 CONFIG_USBD_VIDEO_HEADER_SIZE)

/* Helper to turn a frame rate in an UVC frame interval value */
#define FRMIVAL(fps) U32_LE(10000000 / fps)

/* Connect the entities to their source(s) */
#define VC_SOURCE_ID(ep)							\
	IF_ENABLED(DT_ENUM_HAS_VALUE(ep, direction, in),			\
		   (NODE_ID(DT_REMOTE_DEVICE(ep)),))
#define VC_SOURCE_IDS(node)							\
	DT_FOREACH_CHILD(DT_CHILD(node, port), VC_SOURCE_ID)

/* Count the number of sources or sink for this device */
#define X(node, dir) IF_ENABLED(DT_ENUM_HAS_VALUE(node, direction, dir), (x,))
#define VC_SOURCE_NUM(node)							\
	NUM_VA_ARGS_LESS_1(DT_FOREACH_CHILD_VARGS(DT_CHILD(node, port), X, in) x)
#define VC_SINK_NUM(node)							\
	NUM_VA_ARGS_LESS_1(DT_FOREACH_CHILD_VARGS(DT_CHILD(node, port), X, out) x)

/* Helpers for IF_ENABLED() conditions */
#define GT1(n) COND_CODE_0(n, (0), (COND_CODE_1(n, (0), (1))))
#define EQ1(n) COND_CODE_1(n, (1), (0))

/* Detection of  the type of unit according to then number of inputs/outputs */
#define VC_IS_IT(node) UTIL_NOT(VC_SOURCE_NUM(node))
#define VC_IS_OT(node) UTIL_NOT(VC_SINK_NUM(node))
#define VC_IS_PU(node) UTIL_AND(VC_SINK_NUM(node), EQ1(VC_SOURCE_NUM(node)))
#define VC_IS_SU(node) UTIL_AND(VC_SINK_NUM(node), GT1(VC_SOURCE_NUM(node)))

/* VideoControl descriptor content of a node according to its type */
#define VC_DESCRIPTOR(node)							\
	IF_ENABLED(VC_IS_IT(node), (VC_CT_DESCRIPTOR(node)))			\
	IF_ENABLED(VC_IS_OT(node), (VC_OT_DESCRIPTOR(node)))			\
	IF_ENABLED(VC_IS_PU(node), (VC_PU_DESCRIPTOR(node)))			\
	IF_ENABLED(VC_IS_SU(node), (VC_SU_DESCRIPTOR(node)))

/* VideoControl handler function according to its type */
#define VC_HANDLER(node)							\
	IF_ENABLED(VC_IS_IT(node), (uvc_control_it))				\
	IF_ENABLED(VC_IS_OT(node), (uvc_control_ot))				\
	IF_ENABLED(VC_IS_PU(node), (uvc_control_pu))				\
	IF_ENABLED(VC_IS_SU(node), (uvc_control_su))

/* Iterators over VideoControl and VideoStreaming elements */
#define VC_FOREACH_ENTITY(fn)							\
	DT_FOREACH_STATUS_OKAY_NODE_VARGS(IF_HAS_PROP, video_controls, fn)
#define VS_FOREACH_STREAM(fn)							\
	DT_FOREACH_CHILD(DT_CHILD(DT_DRV_INST(0), port), fn)
#define VS_FOREACH_FORMAT(node, fn)						\
	DT_FOREACH_PROP_ELEM(node, formats, fn)

/* 3.6 Interface Association Descriptor */
#define INTERFACE_ASSOCIATION_DESCRIPTOR(node)					\
	8,						/* bLength */		\
	USB_DESC_INTERFACE_ASSOC,			/* bDescriptorType */	\
	0x00,						/* bFirstInterface */	\
	0x02,						/* bInterfaceCount */	\
	USB_BCC_VIDEO,					/* bFunctionClass */	\
	SC_VIDEO_INTERFACE_COLLECITON,			/* bFunctionSubClass */	\
	PC_PROTOCOL_UNDEFINED,				/* bFunctionProtocol */	\
	0x00,						/* iFunction */

/* 3.7 VideoControl Interface Descriptors */
#define VC_INTERFACE_DESCRIPTOR(node)						\
	9,						/* bLength */		\
	USB_DESC_INTERFACE,				/* bDescriptorType */	\
	0x99,						/* bInterfaceNumber */	\
	0x00,						/* bAlternateSetting */	\
	0x00,						/* bNumEndpoints */	\
	USB_BCC_VIDEO,					/* bInterfaceClass */	\
	SC_VIDEOCONTROL,				/* bInterfaceSubClass */\
	0x00,						/* bInterfaceProtocol */\
	0x00,						/* iInterface */

/* 3.7.2 Interface Header Descriptor */
#define VC_DESCRIPTORS(node) VC_FOREACH_ENTITY(VC_DESCRIPTOR)
#define VC_IFSIZE(node) sizeof((char[]){VC_DESCRIPTORS(node)})
#define VS_IFNUM(node) 0x99,
#define VC_NUMVS(node) DT_CHILD_NUM(DT_CHILD(node, port))
#define VC_INTERFACE_HEADER_DESCRIPTOR(node)					\
	12 + VC_NUMVS(node),				/* bLength */		\
	USB_DESC_CS_INTERFACE,				/* bDescriptorType */	\
	VC_HEADER,					/* bDescriptorSubtype */\
	U16_LE(0x0150),					/* bcdUVC */		\
	U16_LE(12 + VC_NUMVS(node) + VC_IFSIZE(node)),	/* wTotalLength */	\
	U32_LE(30000000),				/* dwClockFrequency */	\
	VC_NUMVS(node),					/* bInCollection */	\
	VS_FOREACH_STREAM(VS_IFNUM)			/* baInterfaceNr */

/* 3.7.2.1 Input Terminal Descriptor */
#define VC_IT_STRING_OFFSET(n) ((n) - 1)
#define VC_IT_DESCRIPTOR(node)							\
	8,						/* bLength */		\
	USB_DESC_CS_INTERFACE,				/* bDescriptorType */	\
	VC_INPUT_TERMINAL,				/* bDescriptorSubtype */\
	NODE_ID(node),					/* bTerminalID */	\
	U16_LE(ITT_VENDOR_SPECIFIC),			/* wTerminalType */	\
	0x00,						/* bAssocTerminal */	\
	0x00,						/* iTerminal */

/* 3.7.2.2 Output Terminal Descriptor */
#define VC_OT_STRING_OFFSET(n) ((n) - 1)
#define VC_OT_DESCRIPTOR(node)							\
	VC_OT_DESCRIPTOR_EP(node, DT_CHILD(DT_CHILD(node, port), endpoint))
#define VC_OT_DESCRIPTOR_EP(node, endpoint)					\
	9,						/* bLength */		\
	USB_DESC_CS_INTERFACE,				/* bDescriptorType */	\
	VC_OUTPUT_TERMINAL,				/* bDescriptorSubtype */\
	NODE_ID(node),					/* bTerminalID */	\
	U16_LE(TT_STREAMING),				/* wTerminalType */	\
	0x00,						/* bAssocTerminal */	\
	NODE_ID(DT_REMOTE_DEVICE(endpoint)),		/* bSourceID */		\
	0x00,						/* iTerminal */

/* 3.7.2.3 Camera Terminal Descriptor */
#define VC_CT_STRING_OFFSET(n) 7
#define VC_CT_DESCRIPTOR(node)							\
	18,						/* bLength */		\
	USB_DESC_CS_INTERFACE,				/* bDescriptorType */	\
	VC_INPUT_TERMINAL,				/* bDescriptorSubtype */\
	NODE_ID(node),					/* bTerminalID */	\
	U16_LE(ITT_CAMERA),				/* wTerminalType */	\
	0x00,						/* bAssocTerminal */	\
	0x00,						/* iTerminal */		\
	U16_LE(0),					/* wObjectiveFocalLengthMin */\
	U16_LE(0),					/* wObjectiveFocalLengthMax */\
	U16_LE(0),					/* wOcularFocalLength */\
	0x03,						/* bControlSize */	\
	U24_LE(VC_CONTROL_CT(entity, BIT)),		/* bmControls */

/* 3.7.2.4 Selector Unit Descriptor */
#define VC_SU_STRING_OFFSET(n) ((n) - 1)
#define VC_SU_DESCRIPTOR(node)							\
	6 + VC_SOURCE_NUM(node),			/* bLength */		\
	USB_DESC_CS_INTERFACE,				/* bDescriptorType */	\
	VC_SELECTOR_UNIT,				/* bDescriptorSubtype */\
	NODE_ID(node),					/* bUnitID */		\
	VC_SOURCE_NUM(node),				/* bNrInPins */		\
	VC_SOURCE_IDS(node)				/* baSourceID */	\
	0x00,						/* iSelector */

/* 3.7.2.5 Processing Unit Descriptor */
#define VC_PU_STRING_OFFSET(n) ((n) - 2)
#define VC_PU_DESCRIPTOR(node)							\
	13,						/* bLength */		\
	USB_DESC_CS_INTERFACE,				/* bDescriptorType */	\
	VC_PROCESSING_UNIT,				/* bDescriptorSubtype */\
	NODE_ID(node),					/* bUnitID */		\
	VC_SOURCE_IDS(node)				/* bSourceID */		\
	U16_LE(0),					/* wMaxMultiplier */	\
	0x03,						/* bControlSize */	\
	U24_LE(VC_CONTROL_PU(entity, BIT)),		/* bmControls */	\
	0x00,						/* iProcessing */	\
	0x00,						/* bmVideoStandards */

/* 3.7.2.6 Encoding Unit Descriptor */
#define VC_EU_STRING_OFFSET(n) 5
#define VC_EU_DESCRIPTOR(node)							\
	13,						/* bLength */		\
	USB_DESC_CS_INTERFACE,				/* bDescriptorType */	\
	VC_ENCODING_UNIT,				/* bDescriptorSubtype */\
	NODE_ID(node),					/* bUnitID */		\
	VC_SOURCE_IDS(node)				/* bSourceID */		\
	0x00,						/* iEncoding */		\
	0x03,						/* bControlSize */	\
	U48_LE(VC_CONTROL_EU(entity, BIT)),		/* bmControls+Runtime */

/* 3.7.2.7 Extension Unit Descriptor */
#define VC_XU_STRING_OFFSET(n) ((n) - 1)
#define VC_XU_DESCRIPTOR(node)							\
	24 + 8 + VC_SOURCE_NUM(node),			/* bLength */		\
	USB_DESC_CS_INTERFACE,				/* bDescriptorType */	\
	VC_EXTENSION_UNIT,				/* bDescriptorSubtype */\
	NODE_ID(node),					/* bUnitID */		\
	GUID(node),					/* guidExtensionCode */	\
	DT_PROP(entity, control_num),			/* bNumControls */	\
	VC_SOURCE_NUM(node),				/* bNrInPins */		\
	VC_SOURCE_IDS(node)				/* baSourceID */	\
	0x08,						/* bControlSize */	\
	U64_LE(VC_CONTROL_XU(entity, BIT)),		/* bmControls */	\
	0x00,						/* iExtension */

/* 3.9 VideoStreaming Interface Descriptors */
#define VS_INTERFACE_DESCRIPTOR(node)						\
	9,						/* bLength */		\
	USB_DESC_INTERFACE,				/* bDescriptorType */	\
	0x99,						/* bInterfaceNumber */	\
	0x00,						/* bAlternateSetting */	\
	0x01,						/* bNumEndpoints */	\
	USB_BCC_VIDEO,					/* bInterfaceClass */	\
	SC_VIDEOSTREAMING,				/* bInterfaceSubClass */\
	0x00,						/* bInterfaceProtocol */\
	0x00,						/* iInterface */

/* 3.9.2.1 Input Header Descriptor */
#define VS_NUM_FORMATS(node) DT_PROP_LEN(node, formats)
#define VS_DESCRIPTORS(node)							\
	VS_FOREACH_FORMAT(node, VS_UNCOMPRESSED_FORMAT_DESCRIPTOR)		\
	VS_FOREACH_FORMAT(node, VS_UNCOMPRESSED_FRAME_DESCRIPTOR)		\
	VS_COLOR_MATCHING_DESCRIPTOR(node)
#define VS_IFSIZE(node) sizeof((uint8_t []){VS_DESCRIPTORS(node)})
#define VS_INPUT_HEADER_DESCRIPTOR(node)					\
	13,						/* bLength */		\
	USB_DESC_CS_INTERFACE,				/* bDescriptorType */	\
	VS_INPUT_HEADER,				/* bDescriptorSubtype */\
	VS_NUM_FORMATS(node),				/* bNumFormats */	\
	U16_LE(13 + VS_IFSIZE(node)),			/* wTotalLength */	\
	0x81,						/* bEndpointAddress */	\
	0x00,						/* bmInfo */		\
	NODE_ID(node),					/* bTerminalLink */	\
	0x00,						/* bStillCaptureMethod */\
	0x00,						/* bTriggerSupport */	\
	0x00,						/* bTriggerUsage */	\
	0x00,						/* bControlSize */	\

/* 3.9.2.6 VideoStreaming Color Matching Descriptor */
#define VS_COLOR_MATCHING_DESCRIPTOR(node)					\
	6,						/* bLength */		\
	USB_DESC_CS_INTERFACE,				/* bDescriptorType */	\
	VS_COLORFORMAT,					/* bDescriptorSubtype */\
	0x01,	/* BT.709, sRGB (default) */		/* bColorPrimaries */	\
	0x01,	/* BT.709 (default) */			/* bTransferCharacteristics */\
	0x04,	/* SMPTE 170M, BT.601 (default) */	/* bMatrixCoefficients */

/* 3.10 VideoStreaming Bulk FullSpeed Endpoint Descriptors */
#define VS_FULLSPEED_BULK_ENDPOINT_DESCRIPTOR(node)				\
	7,						/* bLength */		\
	USB_DESC_ENDPOINT,				/* bDescriptorType */	\
	0x81,						/* bEndpointAddress */	\
	USB_EP_TYPE_BULK,				/* bmAttributes */	\
	U16_LE(64),					/* wMaxPacketSize */	\
	0x00,						/* bInterval */

/* 3.10 VideoStreaming Bulk HighSpeed Endpoint Descriptors */
#define VS_HIGHSPEED_BULK_ENDPOINT_DESCRIPTOR(node)				\
	7,						/* bLength */		\
	USB_DESC_ENDPOINT,				/* bDescriptorType */	\
	0x81,						/* bEndpointAddress */	\
	USB_EP_TYPE_BULK,				/* bmAttributes */	\
	U16_LE(512),					/* wMaxPacketSize */	\
	0x00,						/* bInterval */

/* 3.1.1 Uncompressed Video Format Descriptor */
#define VS_UNCOMPRESSED_FORMAT_DESCRIPTOR(node, prop, id)			\
	27,						/* bLength */		\
	USB_DESC_CS_INTERFACE,				/* bDescriptorType */	\
	VS_FORMAT_UNCOMPRESSED,				/* bDescriptorSubtype */\
	UTIL_INC(id),					/* bFormatIndex */	\
	0x01,						/* bNumFrameDescriptors */\
	GUID(DT_PHA_BY_IDX(node, prop, id, fourcc)),	/* guidFormat */	\
	DT_PHA_BY_IDX(node, prop, id, bits_per_pixel),	/* bBitsPerPixel */	\
	0x01,						/* bDefaultFrameIndex */\
	0x00,						/* bAspectRatioX */	\
	0x00,						/* bAspectRatioY */	\
	0x00,						/* bmInterlaceFlags */	\
	0x00,						/* bCopyProtect */

/* 3.2.1 Uncompressed Video Frame Descriptors (discrete) */
#define VS_UNCOMPRESSED_FRAME_DESCRIPTOR(node, prop, id)			\
	26 + 4,						/* bLength */		\
	USB_DESC_CS_INTERFACE,				/* bDescriptorType */	\
	VS_FRAME_UNCOMPRESSED,				/* bDescriptorSubtype */\
	0x01,						/* bFrameIndex */	\
	0x00,						/* bmCapabilities */	\
	U16_LE(DT_PHA_BY_IDX(node, prop, id, width)),	/* wWidth */		\
	U16_LE(DT_PHA_BY_IDX(node, prop, id, height)),	/* wHeight */		\
	U32_LE(15360000),				/* dwMinBitRate */	\
	U32_LE(15360000),				/* dwMaxBitRate */	\
	U32_LE(FRAME_BUFFER_SIZE(node, prop, id)),	/* dwMaxVideoFrameBufferSize */\
	FRMIVAL(DT_PHA_BY_IDX(node, prop, id, max_fps)),/* dwDefaultFrameInterval */\
	0x01,						/* bFrameIntervalType */\
	FRMIVAL(DT_PHA_BY_IDX(node, prop, id, max_fps)),/* dwFrameInterval */

/* 3.1.1 Motion-JPEG Video Format Descriptor */
#define VS_MJPEG_FORMAT_DESCRIPTOR(node, prop, id)				\
	11,						/* bLength */		\
	USB_DESC_CS_INTERFACE,				/* bDescriptorType */	\
	VS_FORMAT_MJPEG,				/* bDescriptorSubtype */\
	id,						/* bFormatIndex */	\
	0x01,						/* bNumFrameDescriptors */\
	BIT(0),						/* bmFlags */		\
	0x01,						/* bDefaultFrameIndex */\
	0x00,						/* bAspectRatioX */	\
	0x00,						/* bAspectRatioY */	\
	0x00,						/* bmInterlaceFlags */	\
	0x00,						/* bCopyProtect */

/* 3.2.1 Motion-JPEG Video Frame Descriptors (discrete) */
#define VS_MJPEG_FRAME_DESCRIPTOR(node, prop, id)				\
	29,						/* bLength */		\
	USB_DESC_CS_INTERFACE,				/* bDescriptorType */	\
	VS_FRAME_MJPEG,					/* bDescriptorSubtype */\
	0x01,						/* bFrameIndex */	\
	0x00,						/* bmCapabilities */	\
	U16_LE(DT_PHA_BY_IDX(node, prop, id, width)),	/* wWidth */		\
	U16_LE(DT_PHA_BY_IDX(node, prop, id, height)),	/* wHeight */		\
	U32_LE(15360000),				/* dwMinBitRate */	\
	U32_LE(15360000),				/* dwMaxBitRate */	\
	U32_LE(FRAME_BUFFER_SIZE(node, prop, id)),	/* dwMaxVideoFrameBufferSize */\
	FRMIVAL(DT_PHA_BY_IDX(node, prop, id, max_fps)),/* dwDefaultFrameInterval */\
	0x01,						/* bFrameIntervalType */\
	FRMIVAL(DT_PHA_BY_IDX(node, prop, id, max_fps)),/* dwFrameInterval */

#define UVC_CLASS_ENABLED 0
#define UVC_CLASS_READY   1

/* Entity Descriptor */
struct uvc_desc_entity {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bEntityID;
};

/* Video Probe and Commit Controls */
struct uvc_probe {
	uint16_t bmHint;
	uint8_t bFormatIndex;
	uint8_t bFrameIndex;
	uint32_t dwFrameInterval;
	uint16_t wKeyFrameRate;
	uint16_t wPFrameRate;
	uint16_t wCompQuality;
	uint16_t wCompWindowSize;
	uint16_t wDelay;
	uint32_t dwMaxVideoFrameSize;
	uint32_t dwMaxPayloadTransferSize;
	uint32_t dwClockFrequency;
	uint8_t bmFramingInfo;
#define UVC_BMFRAMING_INFO_FID BIT(0)
#define UVC_BMFRAMING_INFO_EOF BIT(1)
#define UVC_BMFRAMING_INFO_EOS BIT(2)
	uint8_t bPreferedVersion;
	uint8_t bMinVersion;
	uint8_t bMaxVersion;
	uint8_t bUsage;
	uint8_t bBitDepthLuma;
	uint8_t bmSettings;
	uint8_t bMaxNumberOfRefFramesPlus1;
	uint16_t bmRateControlModes;
	uint64_t bmLayoutPerStream;
} __packed;

/* Video and Still Image Payload Headers */
struct uvc_payload_header {
	uint8_t bHeaderLength;
	uint8_t bmHeaderInfo;
	uint32_t dwPresentationTime; /* optional */
	uint32_t scrSourceClockSTC;  /* optional */
	uint16_t scrSourceClockSOF;  /* optional */
} __packed;

/* Information specific to each VideoControl interface */
struct uvc_control {
	uint8_t entity_id;
	const struct device *dev;
	/* Stream interface affected by this control */
	struct uvc_stream *stream;
	/* Handler function  */
	int (*fn)(const struct usb_setup_packet *setup, struct net_buf *buf,
		  const struct device *dev);
	/* Bitmask of enabled controls for this interface */
	uint64_t mask;
	/* USB descriptors */
	struct usbd_desc_node *desc_str;
	struct usb_desc_header *desc_ctl;
};

/* Information specific to each VideoStreaming interface */
struct uvc_stream {

	/* USB protocol state */

	/* USBD class state */
	atomic_t state;
	/* Descriptor fields that need to be accessed */
	uint8_t *desc_vs_ifnum;
	uint8_t *desc_vs_epaddr;

	/* Video API state */

	/* Device where the data is enqueued/dequeued */
	const struct device *dev;
	/* Self reference for use in work queues, loosing the context */
	const struct device *uvc_dev;
	/* Video capabilities selected for this device */
	const struct video_format_cap *caps;
	/* UVC worker to process the queue */
	struct k_work work;
	/* Video FIFOs for submission (in) and completion (out) queue */
	struct k_fifo fifo_in;
	struct k_fifo fifo_out;
	/* Offset in bytes within the fragment including the header */
	uint32_t xfer_offset;

	/* UVC protocol state */

	/* UVC probe-commit control default values */
	struct uvc_probe default_probe;
	/* UVC payload header, passed just before the image data */
	struct uvc_payload_header payload_header;
	/* UVC format currently selected */
	int format_id;
};

/* Global configuration */
struct uvc_data {
	/* UVC error from latest request */
	int err;
};

/* Compile-time constant configuration */
struct uvc_conf {
	/* USBD class structure */
	struct usbd_class_data *c_data;
	/* UVC lookup tables */
	const struct uvc_control *controls;
	struct uvc_stream *streams;
	/* UVC Descriptors */
	struct usb_desc_header *const *fs_desc;
	struct usb_desc_header *const *hs_desc;
	/* UVC Fields that need to be accessed */
	uint8_t *desc_iad_ifnum;
	uint8_t *desc_vc_ifnum;
	uint8_t *desc_vs_ifnum;
	uint8_t *desc_fs_epaddr;
	uint8_t *desc_hs_epaddr;
};

/* Specialized version of UDC net_buf metadata with extra fields */
struct uvc_buf_info {
	/* Regular UDC buf info so that it can be passed to USBD directly */
	struct udc_buf_info udc;
	/* Extra field at the end */
	struct video_buffer *vbuf;
	/* UVC stream this buffer belongs to */
	struct uvc_stream *stream;
} __packed;

NET_BUF_POOL_FIXED_DEFINE(uvc_pool, DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) * 100, 0,
			  sizeof(struct uvc_buf_info), NULL);

static int uvc_get_format(const struct device *dev, enum video_endpoint_id ep,
			  struct video_format *fmt);

int uvc_get_stream(const struct device *dev, enum video_endpoint_id ep, struct uvc_stream **result)
{
	const struct uvc_conf *conf = dev->config;

	if (ep == VIDEO_EP_OUT) {
		return -EINVAL;
	}

	if (ep == VIDEO_EP_IN || ep == VIDEO_EP_ALL) {
		*result = &conf->streams[0];
		return 0;
	}

	/* Iterate instead of dereference to prevent overflow */
	for (struct uvc_stream *strm = conf->streams; strm->dev != NULL; strm++, ep--) {
		if (ep == 0) {
			*result = strm;
			return 0;
		}
	}
	return -ENODEV;
}

static int uvc_get_errno(int err)
{
	switch (err) {
	case EBUSY:       /* Busy and not ready */
	case EAGAIN:      /* Try again when the device becomes ready */
	case EINPROGRESS: /* Will be ready after this ongoing operation */
	case EALREADY:    /* Already enqueued, will be ready when done */
		return ERR_NOT_READY;
	case EOVERFLOW: /* Values overflowed the range */
	case ERANGE:    /* Value not in range */
	case E2BIG:     /* Value too big for the range */
		return ERR_OUT_OF_RANGE;
	case EDOM:   /* Invalid but still in the range */
	case EINVAL: /* Invalid argument but not ERANGE */
		return ERR_INVALID_VALUE_WITHIN_RANGE;
	case ENODEV:  /* No device supporting this request */
	case ENOTSUP: /* Request not supported */
	case ENOSYS:  /* Request not implemented */
		return ERR_INVALID_REQUEST;
	default:
		return ERR_UNKNOWN;
	}
}

static uint32_t uvc_get_video_cid(const struct usb_setup_packet *setup, uint32_t cid)
{
	switch (setup->bRequest) {
	case GET_DEF:
		return VIDEO_CTRL_GET_DEF | cid;
	case GET_CUR:
		return VIDEO_CTRL_GET_CUR | cid;
	case GET_MIN:
		return VIDEO_CTRL_GET_MIN | cid;
	case GET_MAX:
		return VIDEO_CTRL_GET_MAX | cid;
	default:
		__ASSERT_NO_MSG(false);
		return 0;
	}
}

static int uvc_buf_add(struct net_buf *buf, uint16_t size, uint32_t value)
{
	if (buf->size != size) {
		LOG_ERR("invalid wLength %u, expected %u", buf->size, size);
		return -EINVAL;
	}

	switch (size) {
	case 4:
		net_buf_add_le32(buf, value);
		return 0;
	case 2:
		net_buf_add_le16(buf, value);
		return 0;
	case 1:
		net_buf_add_u8(buf, value);
		return 0;
	default:
		LOG_WRN("control: invalid size %u", size);
		return -ENOTSUP;
	}
}

static int uvc_buf_remove(struct net_buf *buf, uint16_t size, uint32_t *value)
{
	switch (size) {
	case 4:
		*value = net_buf_remove_le32(buf);
		return 0;
	case 2:
		*value = net_buf_remove_le16(buf);
		return 0;
	case 1:
		*value = net_buf_remove_u8(buf);
		return 0;
	default:
		LOG_WRN("control: invalid size %u", size);
		return -ENOTSUP;
	}
}

static uint8_t uvc_get_bulk_in(const struct device *const dev)
{
	const struct uvc_conf *conf = dev->config;

	switch (usbd_bus_speed(usbd_class_get_ctx(conf->c_data))) {
	case USBD_SPEED_FS:
		return *conf->desc_fs_epaddr;
	case USBD_SPEED_HS:
		return *conf->desc_hs_epaddr;
	default:
		__ASSERT_NO_MSG(false);
		return 0;
	}
}

static size_t uvc_get_bulk_mps(struct usbd_class_data *const c_data)
{
	struct usbd_context *uds_ctx = usbd_class_get_ctx(c_data);

	switch (usbd_bus_speed(uds_ctx)) {
	case USBD_SPEED_FS:
		return 64;
	case USBD_SPEED_HS:
		return 512;
	default:
		__ASSERT_NO_MSG(false);
		return 0;
	}
}

static uint32_t uvc_get_max_frame_size(struct uvc_stream *strm)
{
	const struct video_format_cap *cap = &strm->caps[strm->format_id];

	return cap->width_max * cap->height_max * video_bits_per_pixel(cap->pixelformat);
}

static int uvc_control_probe_format_index(const struct device *dev, struct uvc_stream *strm,
					  uint8_t request, struct uvc_probe *probe)
{
	switch (request) {
	case GET_MIN:
		probe->bFormatIndex = 1;
		break;
	case GET_MAX:
		for (size_t i = 0; strm->caps[i].pixelformat != 0; i++) {
			probe->bFormatIndex = i + 1;
		}
		break;
	case GET_RES:
		probe->bFormatIndex = 1;
		break;
	case GET_CUR:
		probe->bFormatIndex = strm->format_id + 1;
		break;
	case SET_CUR:
		if (probe->bFormatIndex == 0) {
			return 0;
		}
		for (size_t i = 0; strm->caps[i].pixelformat != 0; i++) {
			if (probe->bFormatIndex == i + 1) {
				strm->format_id = i;
				return 0;
			}
		}
		LOG_WRN("probe: format index %u not found", probe->bFormatIndex);
		return -ENOTSUP;
	}
	return 0;
}

static int uvc_control_probe_frame_index(const struct device *dev, struct uvc_stream *strm,
					 uint8_t request, struct uvc_probe *probe)
{
	switch (request) {
	case GET_MIN:
	case GET_MAX:
	case GET_RES:
	case GET_CUR:
		probe->bFrameIndex = 1;
		break;
	case SET_CUR:
		if (probe->bFrameIndex == 0 || probe->bFrameIndex == 1) {
			return 0;
		}
		LOG_WRN("probe: frame index %u not found", probe->bFrameIndex);
		return -ENOTSUP;
	}
	return 0;
}

static int uvc_control_probe_frame_interval(const struct device *dev, struct uvc_stream *strm,
					    uint8_t request, struct uvc_probe *probe)
{
	switch (request) {
	case GET_MIN:
	case GET_MAX:
		/* TODO call the frame interval API on the video source once supported */
		probe->dwFrameInterval = sys_cpu_to_le32(10000000);
		break;
	case GET_RES:
		probe->dwFrameInterval = sys_cpu_to_le32(1);
		break;
	case SET_CUR:
		/* TODO call the frame interval API on the video source once supported */
		break;
	}
	return 0;
}

static int uvc_control_probe_max_video_frame_size(const struct device *dev, struct uvc_stream *strm,
						  uint8_t request, struct uvc_probe *probe)
{
	uint32_t max_frame_size = uvc_get_max_frame_size(strm);

	switch (request) {
	case GET_MIN:
	case GET_MAX:
	case GET_CUR:
		probe->dwMaxVideoFrameSize = sys_cpu_to_le32(max_frame_size);
		break;
	case GET_RES:
		probe->dwMaxVideoFrameSize = sys_cpu_to_le32(1);
		break;
	case SET_CUR:
		if (sys_le32_to_cpu(probe->dwMaxVideoFrameSize) > 0 &&
		    sys_le32_to_cpu(probe->dwMaxVideoFrameSize) != max_frame_size) {
			LOG_WRN("probe: dwMaxVideoFrameSize is read-only");
		}
		break;
	}
	return 0;
}

static int uvc_control_probe_max_payload_size(const struct device *dev, struct uvc_stream *strm,
					      uint8_t request, struct uvc_probe *probe)
{
	uint32_t max_payload_size = uvc_get_max_frame_size(strm) + CONFIG_USBD_VIDEO_HEADER_SIZE;

	switch (request) {
	case GET_MIN:
	case GET_MAX:
	case GET_CUR:
		probe->dwMaxPayloadTransferSize = sys_cpu_to_le32(max_payload_size);
		break;
	case GET_RES:
		probe->dwMaxPayloadTransferSize = sys_cpu_to_le32(1);
		break;
	case SET_CUR:
		if (sys_le32_to_cpu(probe->dwMaxPayloadTransferSize) > 0 &&
		    sys_le32_to_cpu(probe->dwMaxPayloadTransferSize) != max_payload_size) {
			LOG_WRN("probe: dwPayloadTransferSize is read-only");
		}
		break;
	}
	return 0;
}

#if CONFIG_USBD_UVC_LOG_LEVEL >= LOG_LEVEL_DBG
static void uvc_log_probe(const char *name, uint32_t probe)
{
	if (probe > 0) {
		LOG_DBG(" %s %u", name, probe);
	}
}
#endif

static char const *uvc_get_request_str(const struct usb_setup_packet *setup)
{
	switch (setup->bRequest) {
	case SET_CUR:
		return "SET_CUR";
	case GET_CUR:
		return "GET_CUR";
	case GET_MIN:
		return "GET_MIN";
	case GET_MAX:
		return "GET_MAX";
	case GET_RES:
		return "GET_RES";
	case GET_LEN:
		return "GET_LEN";
	case GET_DEF:
		return "GET_DEF";
	case GET_INFO:
		return "GET_INFO";
	default:
		return "(unknown)";
	}
}

static int uvc_control_probe(const struct device *dev, struct uvc_stream *strm, uint8_t request,
			     struct uvc_probe *probe)
{
	int ret;

	if (request != GET_MIN && request != GET_MAX && request != GET_RES &&
	    request != GET_CUR && request != SET_CUR) {
		LOG_WRN("control: invalid bRequest %u", request);
		return -EINVAL;
	}

	/* Dynamic fields */
	ret = uvc_control_probe_format_index(dev, strm, request, probe);
	if (ret < 0) {
		return ret;
	}
	ret = uvc_control_probe_frame_index(dev, strm, request, probe);
	if (ret < 0) {
		return ret;
	}
	ret = uvc_control_probe_frame_interval(dev, strm, request, probe);
	if (ret < 0) {
		return ret;
	}
	ret = uvc_control_probe_max_video_frame_size(dev, strm, request, probe);
	if (ret < 0) {
		return ret;
	}
	ret = uvc_control_probe_max_payload_size(dev, strm, request, probe);
	if (ret < 0) {
		return ret;
	}

	/* Static fields */
	probe->dwClockFrequency = sys_cpu_to_le32(1);
	/* Include Frame ID and EOF fields in the payload header */
	probe->bmFramingInfo = BIT(0) | BIT(1);
	probe->bPreferedVersion = 1;
	probe->bMinVersion = 1;
	probe->bMaxVersion = 1;
	probe->bUsage = 0;
	probe->bBitDepthLuma = 0;
	probe->bmSettings = 0;
	probe->bMaxNumberOfRefFramesPlus1 = 1;
	probe->bmRateControlModes = 0;
	probe->bmLayoutPerStream = 0;
	probe->wKeyFrameRate = sys_cpu_to_le16(0);
	probe->wPFrameRate = sys_cpu_to_le16(0);
	probe->wCompQuality = sys_cpu_to_le16(0);
	probe->wCompWindowSize = sys_cpu_to_le16(0);
	/* TODO devicetree */
	probe->wDelay = sys_cpu_to_le16(1);

#if CONFIG_USBD_UVC_LOG_LEVEL >= LOG_LEVEL_DBG
	uvc_log_probe("bmHint", sys_le16_to_cpu(probe->bmHint));
	uvc_log_probe("bFormatIndex", probe->bFormatIndex);
	uvc_log_probe("bFrameIndex", probe->bFrameIndex);
	uvc_log_probe("dwFrameInterval (us)", sys_le32_to_cpu(probe->dwFrameInterval) / 10);
	uvc_log_probe("wKeyFrameRate", sys_le16_to_cpu(probe->wKeyFrameRate));
	uvc_log_probe("wPFrameRate", sys_le16_to_cpu(probe->wPFrameRate));
	uvc_log_probe("wCompQuality", sys_le16_to_cpu(probe->wCompQuality));
	uvc_log_probe("wCompWindowSize", sys_le16_to_cpu(probe->wCompWindowSize));
	uvc_log_probe("wDelay (ms)", sys_le16_to_cpu(probe->wDelay));
	uvc_log_probe("dwMaxVideoFrameSize", sys_le32_to_cpu(probe->dwMaxVideoFrameSize));
	uvc_log_probe("dwMaxPayloadTransferSize", sys_le32_to_cpu(probe->dwMaxPayloadTransferSize));
	uvc_log_probe("dwClockFrequency (Hz)", sys_le32_to_cpu(probe->dwClockFrequency));
	uvc_log_probe("bmFramingInfo", probe->bmFramingInfo);
	uvc_log_probe("bPreferedVersion", probe->bPreferedVersion);
	uvc_log_probe("bMinVersion", probe->bMinVersion);
	uvc_log_probe("bMaxVersion", probe->bMaxVersion);
	uvc_log_probe("bUsage", probe->bUsage);
	uvc_log_probe("bBitDepthLuma", probe->bBitDepthLuma + 8);
	uvc_log_probe("bmSettings", probe->bmSettings);
	uvc_log_probe("bMaxNumberOfRefFramesPlus1", probe->bMaxNumberOfRefFramesPlus1);
	uvc_log_probe("bmRateControlModes", probe->bmRateControlModes);
	uvc_log_probe("bmLayoutPerStream", probe->bmLayoutPerStream);
#endif

	return 0;
}

static int uvc_control_commit(const struct device *dev, struct uvc_stream *strm, uint8_t request,
			      struct uvc_probe *probe)
{
	struct video_format fmt = {0};
	int ret;

	switch (request) {
	case GET_CUR:
		return uvc_control_probe(dev, strm, request, probe);
	case SET_CUR:
		ret = uvc_control_probe(dev, strm, request, probe);
		if (ret < 0) {
			return ret;
		}

		atomic_set_bit(&strm->state, UVC_CLASS_READY);
		k_work_submit(&strm->work);

		ret = uvc_get_format(dev, VIDEO_EP_IN, &fmt);
		if (ret < 0) {
			LOG_ERR("Failed to inquire the current UVC format");
			return ret;
		}

		LOG_INF("control: ready to transfer %ux%u frames of %u bytes", fmt.width,
			fmt.height, fmt.height * fmt.pitch);

		if (strm->dev != NULL) {
			LOG_DBG("control: setting source format to %ux%u", fmt.width, fmt.height);

			ret = video_set_format(strm->dev, VIDEO_EP_OUT, &fmt);
			if (ret < 0) {
				LOG_ERR("Could not set the format of the video source");
				return ret;
			}

			ret = video_stream_start(strm->dev);
			if (ret < 0) {
				LOG_ERR("Could not start the video source");
				return ret;
			}
		}

		break;
	default:
		LOG_WRN("commit: invalid bRequest %u", request);
		return -EINVAL;
	}
	return 0;
}

static int uvc_control_vs(const struct device *dev, struct uvc_stream *strm,
			  const struct usb_setup_packet *setup, struct net_buf *buf)
{
	uint8_t control_selector = setup->wValue >> 8;
	struct uvc_probe *probe = (void *)buf->data;

	switch (setup->bRequest) {
	case GET_INFO:
		return uvc_buf_add(buf, 1, INFO_SUPPORTS_GET | INFO_SUPPORTS_SET);
	case GET_LEN:
		return uvc_buf_add(buf, 2, sizeof(probe));
	case GET_DEF:
		if (setup->wLength != sizeof(*probe) || buf->size < sizeof(*probe)) {
			LOG_ERR("control: bad wLength %u or bufsize %u", setup->wLength, buf->size);
			return -EINVAL;
		}
		net_buf_add_mem(buf, &strm->default_probe, sizeof(*probe));
		return 0;
	case GET_MIN:
	case GET_MAX:
	case GET_RES:
	case GET_CUR:
		if (setup->wLength != sizeof(*probe) || buf->size < sizeof(*probe)) {
			LOG_ERR("control: bad wLength %u or bufsize %u", setup->wLength, buf->size);
			return -EINVAL;
		}
		net_buf_add(buf, sizeof(*probe));
		break;
	case SET_CUR:
		if (setup->wLength != sizeof(*probe) || buf->len < sizeof(*probe)) {
			LOG_ERR("control: bad wLength %u or buflen %u", setup->wLength, buf->len);
			return -EINVAL;
		}
		break;
	}

	switch (control_selector) {
	case VS_PROBE_CONTROL:
		LOG_DBG("VS_PROBE_CONTROL");
		return uvc_control_probe(dev, strm, setup->bRequest, probe);
	case VS_COMMIT_CONTROL:
		LOG_DBG("VS_COMMIT_CONTROL");
		return uvc_control_commit(dev, strm, setup->bRequest, probe);
	default:
		LOG_WRN("control: unknown selector %u for streaming interface", control_selector);
		return -ENOTSUP;
	}
}

static int uvc_control_default(const struct usb_setup_packet *setup, struct net_buf *buf,
			       uint8_t size)
{
	switch (setup->bRequest) {
	case GET_INFO:
		return uvc_buf_add(buf, 1, INFO_SUPPORTS_GET | INFO_SUPPORTS_SET);
	case GET_LEN:
		return uvc_buf_add(buf, setup->wLength, size);
	case GET_RES:
		return uvc_buf_add(buf, size, 1);
	default:
		LOG_WRN("control: unsupported request type %u", setup->bRequest);
		return -ENOTSUP;
	}
}

static int uvc_control_fix(const struct usb_setup_packet *setup, struct net_buf *buf, uint8_t size,
			   uint32_t value)
{
	LOG_DBG("control: fixed type control, size %u", size);

	switch (setup->bRequest) {
	case GET_DEF:
	case GET_CUR:
	case GET_MIN:
	case GET_MAX:
		return uvc_buf_add(buf, size, value);
	case SET_CUR:
		return 0;
	default:
		return uvc_control_default(setup, buf, size);
	}
}

static int uvc_control_uint(const struct usb_setup_packet *setup, struct net_buf *buf, uint8_t size,
			    const struct device *dev, uint32_t cid)
{
	uint32_t value;
	int ret;

	LOG_DBG("control: integer type control, size %u", size);

	switch (setup->bRequest) {
	case GET_DEF:
	case GET_CUR:
	case GET_MIN:
	case GET_MAX:
		ret = video_get_ctrl(dev, uvc_get_video_cid(setup, cid), &value);
		if (ret < 0) {
			LOG_ERR("control: failed to query target video device");
			return ret;
		}
		LOG_DBG("control: value for CID 0x08%x is %u", cid, value);
		return uvc_buf_add(buf, size, value);
	case SET_CUR:
		ret = uvc_buf_remove(buf, size, &value);
		if (ret < 0) {
			return ret;
		}
		LOG_DBG("control: setting CID 0x08%x to %u", cid, value);
		ret = video_set_ctrl(dev, cid, (void *)value);
		if (ret < 0) {
			LOG_ERR("control: failed to configure target video device");
			return ret;
		}
		return 0;
	default:
		return uvc_control_default(setup, buf, size);
	}
}

__unused static int uvc_control_ct(const struct usb_setup_packet *setup, struct net_buf *buf,
				   const struct device *dev)
{
	uint8_t control_selector = setup->wValue >> 8;

	/* See also zephyr,uvc-control-ct.yaml */
	switch (control_selector) {
	case CT_AE_MODE_CONTROL:
		LOG_DBG("CT_AE_MODE_CONTROL -> (none)");
		return uvc_control_fix(setup, buf, 1, BIT(0));
	case CT_AE_PRIORITY_CONTROL:
		LOG_DBG("CT_AE_PRIORITY_CONTROL -> (none)");
		return uvc_control_fix(setup, buf, 1, 0);
	case CT_EXPOSURE_TIME_ABSOLUTE_CONTROL:
		LOG_DBG("CT_EXPOSURE_TIME_ABSOLUTE_CONTROL -> VIDEO_CID_CAMERA_EXPOSURE");
		return uvc_control_uint(setup, buf, 4, dev, VIDEO_CID_CAMERA_EXPOSURE);
	case CT_ZOOM_ABSOLUTE_CONTROL:
		LOG_DBG("CT_ZOOM_ABSOLUTE_CONTROL -> VIDEO_CID_CAMERA_ZOOM");
		return uvc_control_uint(setup, buf, 2, dev, VIDEO_CID_CAMERA_ZOOM);
	default:
		LOG_WRN("control: unsupported selector %u for camera terminal ", control_selector);
		return -ENOTSUP;
	}
}

__unused static int uvc_control_pu(const struct usb_setup_packet *setup, struct net_buf *buf,
				   const struct device *dev)
{
	uint8_t control_selector = setup->wValue >> 8;

	/* See also zephyr,uvc-control-pu.yaml */
	switch (control_selector) {
	case PU_BRIGHTNESS_CONTROL:
		LOG_DBG("PU_BRIGHTNESS_CONTROL -> VIDEO_CID_CAMERA_BRIGHTNESS");
		return uvc_control_uint(setup, buf, 2, dev, VIDEO_CID_CAMERA_BRIGHTNESS);
	case PU_CONTRAST_CONTROL:
		LOG_DBG("PU_CONTRAST_CONTROL -> VIDEO_CID_CAMERA_CONTRAST");
		return uvc_control_uint(setup, buf, 1, dev, VIDEO_CID_CAMERA_CONTRAST);
	case PU_GAIN_CONTROL:
		LOG_DBG("PU_GAIN_CONTROL -> VIDEO_CID_CAMERA_GAIN");
		return uvc_control_uint(setup, buf, 2, dev, VIDEO_CID_CAMERA_GAIN);
	case PU_SATURATION_CONTROL:
		LOG_DBG("PU_SATURATION_CONTROL -> VIDEO_CID_CAMERA_SATURATION");
		return uvc_control_uint(setup, buf, 2, dev, VIDEO_CID_CAMERA_SATURATION);
	case PU_WHITE_BALANCE_TEMPERATURE_CONTROL:
		LOG_DBG("PU_WHITE_BALANCE_TEMPERATURE_CONTROL -> VIDEO_CID_CAMERA_WHITE_BAL");
		return uvc_control_uint(setup, buf, 2, dev, VIDEO_CID_CAMERA_WHITE_BAL);
	default:
		LOG_WRN("control: unsupported selector %u for processing unit", control_selector);
		return -ENOTSUP;
	}
}

__unused static int uvc_control_xu(const struct usb_setup_packet *setup, struct net_buf *buf,
				   const struct device *dev)
{
	LOG_WRN("control: nothing supported for extension unit");
	return -ENOTSUP;
};

__unused static int uvc_control_it(const struct usb_setup_packet *setup, struct net_buf *buf,
				   const struct device *dev)
{
	LOG_WRN("control: nothing supported for input terminal");
	return -ENOTSUP;
};

__unused static int uvc_control_ot(const struct usb_setup_packet *setup, struct net_buf *buf,
			  const struct device *dev)
{
	LOG_WRN("control: nothing supported for output terminal");
	return -ENOTSUP;
};

static int uvc_control_vc(const struct device *dev, const struct uvc_control *ctrl,
			  const struct usb_setup_packet *setup, struct net_buf *buf)
{
	struct uvc_data *data = dev->data;
	uint8_t control_selector = setup->wValue >> 8;
	uint8_t entity_id = (setup->wIndex >> 8) & 0xff;
	int ret;

	if ((ctrl->mask & BIT(control_selector)) == 0) {
		LOG_WRN("control selector %u not enabled for bEntityID %u",
		control_selector, entity_id);
		data->err = ERR_INVALID_CONTROL;
		return -ENOTSUP;
	}

	/* Control set as supported by the devicetree, call the handler */
	ret = ctrl->fn(setup, buf, ctrl->dev);
	data->err = uvc_get_errno(-ret);
	return ret;
}

static int uvc_control_errno(const struct usb_setup_packet *setup, struct net_buf *buf, int err)
{
	switch (setup->bRequest) {
	case GET_INFO:
		return uvc_buf_add(buf, 1, INFO_SUPPORTS_GET);
	case GET_CUR:
		return uvc_buf_add(buf, 1, err);
	default:
		LOG_WRN("control: unsupported request type %u", setup->bRequest);
		return -ENOTSUP;
	}
}

static int uvc_control(const struct device *dev, const struct usb_setup_packet *const setup,
		       struct net_buf *buf)
{
	const struct uvc_conf *conf = dev->config;
	struct uvc_data *data = dev->data;
	uint8_t ifnum = (setup->wIndex >> 0) & 0xff;
	uint8_t entity_id = (setup->wIndex >> 8) & 0xff;

	LOG_DBG("Host send a %s control command", uvc_get_request_str(setup));

	/* VideoStreaming requests */

	for (struct uvc_stream *strm = conf->streams; strm->dev != NULL; strm++) {
		if (*strm->desc_vs_ifnum == ifnum) {
			return uvc_control_vs(dev, strm, setup, buf);
		}
	}

	if (ifnum == *conf->desc_vc_ifnum) {
		LOG_WRN("control: interface %u not found", ifnum);
		data->err = ERR_INVALID_UNIT;
		return -ENOTSUP;
	}

	/* VideoControl requests */

	if (entity_id == 0) {
		return uvc_control_errno(setup, buf, data->err);
	}

	for (const struct uvc_control *ctrl = conf->controls; ctrl->dev != NULL; ctrl++) {
		if (ctrl->entity_id == entity_id) {
			return uvc_control_vc(dev, ctrl, setup, buf);
		}
	}

	LOG_WRN("control: no unit %u found", entity_id);
	data->err = ERR_INVALID_UNIT;
	return -ENOTSUP;
}

static int uvc_control_to_host(struct usbd_class_data *const c_data,
			       const struct usb_setup_packet *const setup,
			       struct net_buf *const buf)
{
	errno = uvc_control(usbd_class_get_private(c_data), setup, buf);
	return 0;
}

static int uvc_control_to_dev(struct usbd_class_data *const c_data,
			      const struct usb_setup_packet *const setup,
			      const struct net_buf *const buf)
{
	errno = uvc_control(usbd_class_get_private(c_data), setup, (struct net_buf *)buf);
	return 0;
}

static int uvc_request(struct usbd_class_data *const c_data, struct net_buf *buf, int err)
{
	const struct device *dev = usbd_class_get_private(c_data);
	struct uvc_buf_info bi = *(struct uvc_buf_info *)udc_get_buf_info(buf);

	net_buf_unref(buf);

	if (bi.udc.ep == uvc_get_bulk_in(dev)) {
		if (bi.vbuf != NULL) {
			/* Upon completion, move the buffer from submission to completion queue */
			LOG_DBG("Request completed, vbuf %p transferred", bi.vbuf);
			k_fifo_put(&bi.stream->fifo_out, bi.vbuf);
		}
	} else {
		LOG_WRN("Request on unknown endpoint 0x%02x", bi.udc.ep);
	}

	return 0;
}

static void uvc_update(struct usbd_class_data *const c_data, uint8_t iface, uint8_t alternate)
{
	LOG_DBG("update");
}

static int uvc_init(struct usbd_class_data *const c_data)
{
	struct usbd_context *uds_ctx = usbd_class_get_ctx(c_data);
	const struct device *dev = usbd_class_get_private(c_data);
	const struct uvc_conf *conf = dev->config;
	int ret;

	for (struct uvc_stream *strm = conf->streams; strm->dev != NULL; strm++) {
		/* Get the default probe by querying the current probe at startup */
		ret = uvc_control_probe(dev, strm, GET_CUR, &strm->default_probe);
		if (ret < 0) {
			LOG_ERR("init: failed to query the default probe");
			return ret;
		}
	}

	for (const struct uvc_control *ctrl = conf->controls; ctrl->dev != NULL; ctrl++) {
		struct usbd_desc_node *desc_nd = ctrl->desc_str;
		struct uvc_desc_entity *desc = (void *)ctrl->desc_ctl;

		LOG_DBG("Adding string descriptor '%s'", (char *)desc_nd->ptr);

		ret = usbd_add_descriptor(uds_ctx, desc_nd);
		if (ret < 0) {
			LOG_WRN("Failed to add string descriptor %s to %s",
				(char *)desc_nd->ptr, dev->name);
			continue;
		}

		switch (desc->bDescriptorSubtype) {
		case VC_INPUT_TERMINAL:
			((uint8_t *)desc)[VC_CT_STRING_OFFSET(desc->bLength)] = desc_nd->str.idx;
			break;
		case VC_OUTPUT_TERMINAL:
			((uint8_t *)desc)[VC_OT_STRING_OFFSET(desc->bLength)] = desc_nd->str.idx;
			break;
		case VC_SELECTOR_UNIT:
			((uint8_t *)desc)[VC_SU_STRING_OFFSET(desc->bLength)] = desc_nd->str.idx;
			break;
		case VC_PROCESSING_UNIT:
			((uint8_t *)desc)[VC_PU_STRING_OFFSET(desc->bLength)] = desc_nd->str.idx;
			break;
		case VC_ENCODING_UNIT:
			((uint8_t *)desc)[VC_EU_STRING_OFFSET(desc->bLength)] = desc_nd->str.idx;
			break;
		case VC_EXTENSION_UNIT:
			((uint8_t *)desc)[VC_XU_STRING_OFFSET(desc->bLength)] = desc_nd->str.idx;
			break;
		default:
			LOG_WRN("Not adding '%s' to unknown subtype %u",
				(char *)desc_nd->ptr, desc->bEntityID);
			break;
		}
	}

	return 0;
}

static void uvc_update_desc(const struct device *dev)
{
	const struct uvc_conf *conf = dev->config;

	*conf->desc_iad_ifnum = *conf->desc_vc_ifnum;

	for (size_t i = 0; conf->streams[i].dev != NULL; i++) {
		struct uvc_stream *strm = &conf->streams[i];

		*strm->desc_vs_epaddr = uvc_get_bulk_in(dev);
		conf->desc_vs_ifnum[i] = *strm->desc_vs_ifnum;
	}
}

static void *uvc_get_desc(struct usbd_class_data *const c_data, const enum usbd_speed speed)
{
	const struct device *dev = usbd_class_get_private(c_data);
	const struct uvc_conf *conf = dev->config;

	uvc_update_desc(dev);

	switch (speed) {
	case USBD_SPEED_FS:
		return (void *)conf->fs_desc;
	case USBD_SPEED_HS:
		return (void *)conf->hs_desc;
	default:
		__ASSERT_NO_MSG(false);
		return NULL;
	}
}

static int uvc_enqueue_usb(const struct device *dev, struct uvc_stream *strm, struct net_buf *buf, struct video_buffer *vbuf)
{
	const struct uvc_conf *conf = dev->config;
	size_t mps = uvc_get_bulk_mps(conf->c_data);
	struct uvc_buf_info *bi = (void *)udc_get_buf_info(buf);
	size_t len = buf->len;
	int ret;

	memset(bi, 0, sizeof(struct udc_buf_info));
	bi->udc.zlp = (vbuf->flags & VIDEO_BUF_EOF) && buf->len == mps;
	bi->udc.ep = uvc_get_bulk_in(dev);
	/* If this is the last buffer, attach vbuf so we can free it from uvc_request() */
	bi->vbuf = (buf->len <= mps) ? vbuf : NULL;
	/* Reference to the stream to be able to find the correct FIFO */
	bi->stream = strm;

	/* Apply USB limit */
	buf->len = len = MIN(buf->len, mps);

	LOG_DBG("Queue USB buffer %p, data %p, size %u, len %u", buf, buf->data, buf->size,
		buf->len);

	ret = usbd_ep_enqueue(conf->c_data, buf);
	if (ret < 0) {
		return ret;
	}

	return len;
}

/* Here, the queue of video frame fragments is processed, each
 * fragment is prepended by the UVC header, and the result is cut into
 * USB packets submitted to the hardware:
 *
 *	frame: [vbuf: [header+payload, payload, payload, payload...],
 *		vbuf: [header+payload, payload, payload, payload...], ...],
 *	frame: [vbuf: [header+payload, payload, payload, payload...],
 *		vbuf: [header+payload, payload, payload, payload...], ...],
 *	frame: [vbuf: [header+payload, payload, payload, payload...],
 *		vbuf: [header+payload, payload, payload, payload...], ...],
 *	...
 */
static int uvc_queue_vbuf(const struct device *dev, struct uvc_stream *strm, struct video_buffer *vbuf)
{
	size_t xfer_len = CONFIG_USBD_VIDEO_HEADER_SIZE + vbuf->bytesused;
	struct net_buf *buf;
	int ret;

	if (strm->xfer_offset >= xfer_len) {
		strm->xfer_offset = 0;
		return 1;
	}

	LOG_DBG("Queue vbuf %p, offset %u/%u, flags 0x%02x", vbuf, strm->xfer_offset, xfer_len,
		vbuf->flags);

	/* Add another video frame an USB packet */
	buf = net_buf_alloc_with_data(&uvc_pool, vbuf->header + strm->xfer_offset,
				      xfer_len - strm->xfer_offset, K_NO_WAIT);
	if (buf == NULL) {
		LOG_ERR("Queue failed: cannot allocate USB buffer");
		return -ENOMEM;
	}

	if (strm->xfer_offset == 0) {
		LOG_INF("Queue start of frame, bmHeaderInfo 0x%02x",
			strm->payload_header.bmHeaderInfo);

		/* Only the 2 first 8-bit fields supported for now, the rest is padded with 0x00 */
		memcpy(vbuf->header, &strm->payload_header, 2);
		memset(vbuf->header + 2, 0, CONFIG_USBD_VIDEO_HEADER_SIZE - 2);

		if (vbuf->flags & VIDEO_BUF_EOF) {
			((struct uvc_payload_header *)vbuf->header)->bmHeaderInfo |=
				UVC_BMHEADERINFO_END_OF_FRAME;
		}

		if (vbuf->flags & VIDEO_BUF_EOF) {
			/* Toggle the Frame ID bit every new frame */
			strm->payload_header.bmHeaderInfo ^= UVC_BMHEADERINFO_FRAMEID;
		}
	}

	ret = uvc_enqueue_usb(dev, strm, buf, vbuf);
	if (ret < 0) {
		LOG_ERR("Queue to USB failed");
		strm->xfer_offset = 0;
		net_buf_unref(buf);
		return ret;
	}

	strm->xfer_offset += ret;
	return 0;
}

static void uvc_worker(struct k_work *work)
{
	struct uvc_stream *strm = CONTAINER_OF(work, struct uvc_stream, work);
	const struct device *dev = strm->uvc_dev;
	struct video_buffer *vbuf;
	int ret;

	if (!atomic_test_bit(&strm->state, UVC_CLASS_ENABLED) ||
	    !atomic_test_bit(&strm->state, UVC_CLASS_READY)) {
		LOG_DBG("Queue not ready");
		return;
	}

	/* Only remove the buffer from the queue after it is submitted to USB */
	vbuf = k_fifo_peek_head(&strm->fifo_in);
	if (vbuf == NULL) {
		return;
	}

	if (vbuf->buffer - vbuf->header != CONFIG_USBD_VIDEO_HEADER_SIZE) {
		LOG_ERR("Queue expecting header of size %u", CONFIG_USBD_VIDEO_HEADER_SIZE);
		/* TODO: Submit a k_poll event mentioning the error */
		return;
	}

	ret = uvc_queue_vbuf(dev, strm, vbuf);
	if (ret < 0) {
		LOG_ERR("Queue vbuf %p failed", vbuf);
		/* TODO: Submit a k_poll event mentioning the error */
		return;
	}
	if (ret == 1) {
		/* Remove the buffer from the queue now that USB driver received it */
		k_fifo_get(&strm->fifo_in, K_NO_WAIT);
	}

	/* Work on the next buffer */
	k_work_submit(&strm->work);
}

static void uvc_enable(struct usbd_class_data *const c_data)
{
	const struct device *dev = usbd_class_get_private(c_data);
	const struct uvc_conf *conf = dev->config;

	for (struct uvc_stream *strm = conf->streams; strm->dev != NULL; strm++) {
		/* Catch-up with buffers that might have been delayed */
		atomic_set_bit(&strm->state, UVC_CLASS_ENABLED);
		k_work_submit(&strm->work);
	}
}

static void uvc_disable(struct usbd_class_data *const c_data)
{
	const struct device *dev = usbd_class_get_private(c_data);
	const struct uvc_conf *conf = dev->config;

	for (struct uvc_stream *strm = conf->streams; strm->dev != NULL; strm++) {
		atomic_clear_bit(&strm->state, UVC_CLASS_ENABLED);
	}
}

struct usbd_class_api uvc_class_api = {
	.enable = uvc_enable,
	.disable = uvc_disable,
	.request = uvc_request,
	.update = uvc_update,
	.control_to_host = uvc_control_to_host,
	.control_to_dev = uvc_control_to_dev,
	.init = uvc_init,
	.get_desc = uvc_get_desc,
};

USBD_DEFINE_CLASS(uvc_c_data, &uvc_class_api, (void *)DEVICE_DT_GET(DT_DRV_INST(0)), NULL);

static int uvc_enqueue(const struct device *dev, enum video_endpoint_id ep,
		       struct video_buffer *vbuf)
{
	struct uvc_stream *strm;
	int ret;

	ret = uvc_get_stream(dev, ep, &strm);
	if (ret < 0) {
		return ret;
	}

	k_fifo_put(&strm->fifo_in, vbuf);
	k_work_submit(&strm->work);
	return 0;
}

static int uvc_dequeue(const struct device *dev, enum video_endpoint_id ep,
		       struct video_buffer **vbuf, k_timeout_t timeout)
{
	struct uvc_stream *strm;
	int ret;

	ret = uvc_get_stream(dev, ep, &strm);
	if (ret < 0) {
		return ret;
	}

	*vbuf = k_fifo_get(&strm->fifo_out, timeout);
	if (*vbuf == NULL) {
		return -EAGAIN;
	}

	return 0;
}

static int uvc_get_format(const struct device *dev, enum video_endpoint_id ep,
			  struct video_format *fmt)
{
	const struct video_format_cap *cap;
	struct uvc_stream *strm;
	int ret;

	ret = uvc_get_stream(dev, ep, &strm);
	if (ret < 0) {
		return ret;
	}

	if (!atomic_test_bit(&strm->state, UVC_CLASS_ENABLED) ||
	    !atomic_test_bit(&strm->state, UVC_CLASS_READY)) {
		return -EAGAIN;
	}

	cap = &strm->caps[strm->format_id];

	memset(fmt, 0, sizeof(*fmt));
	fmt->pixelformat = cap->pixelformat;
	fmt->width = cap->width_max;
	fmt->height = cap->height_max;
	fmt->pitch = fmt->width * video_bits_per_pixel(cap->pixelformat) / 8;
	return 0;
}

static int uvc_stream_start(const struct device *dev)
{
	/* TODO: resume the stream after it was interrupted if needed */
	return 0;
}

static int uvc_stream_stop(const struct device *dev)
{
	/* TODO: cancel the ongoing USB request and stop the stream */
	return 0;
}

static int uvc_get_caps(const struct device *dev, enum video_endpoint_id ep,
			struct video_caps *caps)
{
	struct uvc_stream *strm;
	int ret;

	ret = uvc_get_stream(dev, ep, &strm);
	if (ret < 0) {
		return ret;
	}

	caps->format_caps = strm->caps;
	return 0;
}

struct video_driver_api uvc_video_api = {
	.get_format = uvc_get_format,
	.stream_start = uvc_stream_start,
	.stream_stop = uvc_stream_stop,
	.get_caps = uvc_get_caps,
	.enqueue = uvc_enqueue,
	.dequeue = uvc_dequeue,
};

static int uvc_preinit(const struct device *dev)
{
	const struct uvc_conf *conf = dev->config;
	int ret;

	/* VideoStreaming initialization */

	for (size_t i = 0; conf->streams[i].dev != NULL; i++) {
		struct uvc_stream *strm = &conf->streams[i];

		k_fifo_init(&strm->fifo_in);
		k_fifo_init(&strm->fifo_out);
		k_work_init(&strm->work, &uvc_worker);
	}

	return 0;
}

#define INTERFACE_ASSOCIATION_DESCRIPTOR_ARRAYS(node)				\
	static uint8_t uvc_desc_iad[] = {					\
		INTERFACE_ASSOCIATION_DESCRIPTOR(node)				\
	};

#define VC_INTERFACE_DESCRIPTOR_ARRAYS(node)					\
	static uint8_t uvc_desc_vc_if[] = {					\
		VC_INTERFACE_DESCRIPTOR(node)					\
	};

#define VC_INTERFACE_HEADER_DESCRIPTOR_ARRAYS(node)				\
	static uint8_t uvc_desc_vc_header[] = {					\
		VC_INTERFACE_HEADER_DESCRIPTOR(node)				\
	};

#define VC_DESCRIPTOR_ARRAYS(node)						\
	static uint8_t DT_CAT(node, _desc)[] = {				\
		VC_DESCRIPTOR(node)						\
	};

#define VC_OT_DESCRIPTOR_ARRAYS(node)						\
	static uint8_t DT_CAT(node, _desc)[] = {				\
		VC_OT_DESCRIPTOR_EP(node, node)					\
	};

#define VS_DESCRIPTOR_ARRAYS(node)						\
	VS_INTERFACE_DESCRIPTOR_ARRAYS(node)					\
	VS_INTERFACE_HEADER_DESCRIPTOR_ARRAYS(node)				\
	VS_FOREACH_FORMAT(node, VS_UNCOMP_DESCRIPTOR_ARRAYS)

#define VS_INTERFACE_DESCRIPTOR_ARRAYS(node)					\
	static uint8_t DT_CAT(node, _desc_if)[] = {				\
		VS_INTERFACE_DESCRIPTOR(node)					\
	};

#define VS_INTERFACE_HEADER_DESCRIPTOR_ARRAYS(node)				\
	static uint8_t DT_CAT(node, _desc_header)[] = {				\
		VS_INPUT_HEADER_DESCRIPTOR(node)				\
	};									\

#define VS_UNCOMP_DESCRIPTOR_ARRAYS(node, prop, id)				\
	static const uint8_t DT_CAT4(node, _desc_uncomp_, id, _format)[] = {	\
		VS_UNCOMPRESSED_FORMAT_DESCRIPTOR(node, prop, id)		\
	};									\
	static const uint8_t DT_CAT4(node, _desc_uncomp_, id, _frame)[] = {	\
		VS_UNCOMPRESSED_FRAME_DESCRIPTOR(node, prop, id)		\
	};

#define VS_MJPEG_DESCRIPTOR_ARRAYS(node, prop, id)				\
	static const uint8_t DT_CAT4(node, _desc_mjpeg_, id, _format)[] = {	\
		VS_MJPEG_FORMAT_DESCRIPTOR(node, prop, id)			\
	};									\
	static const uint8_t DT_CAT4(node, _desc_mjpeg_, id, _frame)[] = {	\
		VS_MJPEG_FRAME_DESCRIPTOR(node, prop, id)			\
	};

#define VS_FULLSPEED_BULK_ENDPOINT_DESCRIPTOR_ARRAYS(node)			\
	static uint8_t uvc_desc_ep_fs[] = {					\
		VS_FULLSPEED_BULK_ENDPOINT_DESCRIPTOR(node)			\
	};

#define VS_HIGHSPEED_BULK_ENDPOINT_DESCRIPTOR_ARRAYS(node)			\
	static uint8_t uvc_desc_ep_hs[] = {					\
		VS_HIGHSPEED_BULK_ENDPOINT_DESCRIPTOR(node)			\
	};

#define VS_COLOR_MATCHING_DESCRIPTOR_ARRAYS(node)				\
	static uint8_t uvc_desc_color[] = {					\
		VS_COLOR_MATCHING_DESCRIPTOR(node)				\
	};

#define UVC_DESCRIPTOR_ARRAYS(node)						\
	INTERFACE_ASSOCIATION_DESCRIPTOR_ARRAYS(node)				\
	VC_INTERFACE_DESCRIPTOR_ARRAYS(node)					\
	VC_INTERFACE_HEADER_DESCRIPTOR_ARRAYS(node)				\
	VC_FOREACH_ENTITY(VC_DESCRIPTOR_ARRAYS)					\
	VS_FOREACH_STREAM(VC_OT_DESCRIPTOR_ARRAYS)				\
	VS_FOREACH_STREAM(VS_DESCRIPTOR_ARRAYS)					\
	VS_FULLSPEED_BULK_ENDPOINT_DESCRIPTOR_ARRAYS(node)			\
	VS_HIGHSPEED_BULK_ENDPOINT_DESCRIPTOR_ARRAYS(node)			\
	VS_COLOR_MATCHING_DESCRIPTOR_ARRAYS(node)

UVC_DESCRIPTOR_ARRAYS(DT_DRV_INST(0))

#define UVC_DESCRIPTOR_PTRS(node, type)						\
	(struct usb_desc_header *)uvc_desc_iad,					\
	(struct usb_desc_header *)uvc_desc_vc_if,				\
	(struct usb_desc_header *)uvc_desc_vc_header,				\
	VC_FOREACH_ENTITY(VC_DESCRIPTOR_PTRS)					\
	VS_FOREACH_STREAM(VC_DESCRIPTOR_PTRS)					\
	VS_FOREACH_STREAM(VS_DESCRIPTOR_PTRS)					\
	VS_##type##_ENDPOINT_DESCRIPTOR_PTRS(node)

#define VC_DESCRIPTOR_PTRS(node)						\
	(struct usb_desc_header *)DT_CAT(node, _desc),

#define VS_DESCRIPTOR_PTRS(node)						\
	(struct usb_desc_header *)DT_CAT(node, _desc_if),			\
	(struct usb_desc_header *)DT_CAT(node, _desc_header),			\
	VS_FOREACH_FORMAT(node, VS_UNCOMP_DESCRIPTOR_PTRS)

#define VS_UNCOMP_DESCRIPTOR_PTRS(node, prop, id)				\
	(struct usb_desc_header *)DT_CAT4(node, _desc_uncomp_, id, _format),	\
	(struct usb_desc_header *)DT_CAT4(node, _desc_uncomp_, id, _frame),	\
	(struct usb_desc_header *)uvc_desc_color,

#define VS_MJPEG_DESCRIPTOR_PTRS(node, prop, id)				\
	(struct usb_desc_header *)DT_CAT4(node, _desc_mjpeg_, id, _format),	\
	(struct usb_desc_header *)DT_CAT4(node, _desc_mjpeg_, id, _frame),	\
	(struct usb_desc_header *)uvc_desc_color,

#define VS_FULLSPEED_BULK_ENDPOINT_DESCRIPTOR_PTRS(node)			\
	(struct usb_desc_header *)uvc_desc_ep_fs,

#define VS_HIGHSPEED_BULK_ENDPOINT_DESCRIPTOR_PTRS(node)			\
	(struct usb_desc_header *)uvc_desc_ep_hs,

static struct usb_desc_header *uvc_fs_desc[] = {
	UVC_DESCRIPTOR_PTRS(DT_DRV_INST(0), FULLSPEED_BULK)
	NULL,
};
static struct usb_desc_header *uvc_hs_desc[] = {
	UVC_DESCRIPTOR_PTRS(DT_DRV_INST(0), HIGHSPEED_BULK)
	NULL,
};

#define VC_DESCRIPTOR_STRING(node)						\
	USBD_DESC_STRING_DEFINE(node ## _desc_str, DT_NODE_FULL_NAME(node),	\
				USBD_DUT_STRING_INTERFACE);
VC_FOREACH_ENTITY(VC_DESCRIPTOR_STRING)

#define UVC_CAPABILITY(node, prop, id)						\
	{.pixelformat = DT_PHA_BY_IDX(node, prop, id, fourcc),			\
	 .width_min = DT_PHA_BY_IDX(node, prop, id, width),			\
	 .width_max = DT_PHA_BY_IDX(node, prop, id, width),			\
	 .width_step = 1,							\
	 .height_min = DT_PHA_BY_IDX(node, prop, id, height),			\
	 .height_max = DT_PHA_BY_IDX(node, prop, id, height),			\
	 .height_step = 1},
#define UVC_STREAM_CAPS(node)							\
	static const struct video_format_cap DT_CAT(node, _caps)[] = {		\
		VS_FOREACH_FORMAT(node, UVC_CAPABILITY)				\
		{0},								\
	};
VS_FOREACH_STREAM(UVC_STREAM_CAPS)

#define UVC_STREAM(node)							\
	{.dev = DEVICE_DT_GET(DT_REMOTE_DEVICE(node)),				\
	 .uvc_dev = DEVICE_DT_GET(DT_GPARENT(node)),				\
	 .desc_vs_ifnum = DT_CAT(node, _desc_if) + 2,				\
	 .desc_vs_epaddr = DT_CAT(node, _desc_header) + 6,			\
	 .payload_header.bHeaderLength = CONFIG_USBD_VIDEO_HEADER_SIZE,		\
	 .caps = DT_CAT(node, _caps)},
static struct uvc_stream uvc_streams[] = {
	VS_FOREACH_STREAM(UVC_STREAM)
	{0},
};

#define UVC_CONTROL(node)							\
	{.dev = DEVICE_DT_GET(node),						\
	 .entity_id = NODE_ID(node),						\
	 .fn = VC_HANDLER(node),						\
	 .desc_str = &DT_CAT(node, _desc_str),					\
	 .desc_ctl = (struct usb_desc_header *)DT_CAT(node, _desc)},
static const struct uvc_control uvc_controls[] = {
	VC_FOREACH_ENTITY(UVC_CONTROL)
	{0},
};

static const struct uvc_conf uvc_conf = {
	.c_data = &uvc_c_data,
	.controls = uvc_controls,
	.streams = uvc_streams,
	.fs_desc = uvc_fs_desc,
	.hs_desc = uvc_hs_desc,
	.desc_iad_ifnum = uvc_desc_iad + 2,
	.desc_vc_ifnum = uvc_desc_vc_if + 2,
	.desc_vs_ifnum = uvc_desc_vc_header + 12,
	.desc_fs_epaddr = uvc_desc_ep_fs + 2,
	.desc_hs_epaddr = uvc_desc_ep_hs + 2,
};

static struct uvc_data uvc_data = {
	.err = 0,
};

BUILD_ASSERT(DT_INST_ON_BUS(0, usb), "Not assigned to a USB device controller");

DEVICE_DT_INST_DEFINE(0, uvc_preinit, NULL, &uvc_data, &uvc_conf, POST_KERNEL,
		      CONFIG_VIDEO_INIT_PRIORITY, &uvc_video_api);

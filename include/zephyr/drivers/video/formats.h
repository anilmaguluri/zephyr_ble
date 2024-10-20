/*
 * Copyright (c) 2019 Linaro Limited.
 * Copyright (c) 2024 tinyVision.ai Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public definitions of Video formats.
 */

#ifndef ZEPHYR_INCLUDE_VIDEO_FORMATS_H_
#define ZEPHYR_INCLUDE_VIDEO_FORMATS_H_

/**
 * @brief Video Formats
 * @defgroup video_formats Video Formats
 * @since 4.0
 * @version 1.0.0
 * @ingroup video_interface
 * @{
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Four-character-code uniquely identifying the pixel format
 */
#define VIDEO_FOURCC(a, b, c, d)                                                                   \
	((uint32_t)(a) | ((uint32_t)(b) << 8) | ((uint32_t)(c) << 16) | ((uint32_t)(d) << 24))

/**
 * @brief Convert a four-character string to a four-character-code
 *
 * This converts a string literal or variable into a four-character-code
 * as defined by @ref VIDEO_FOURCC.
 *
 * @param str String to be converted
 * @return Four-character-code.
 */
#define VIDEO_FOURCC_FROM_STR(str) VIDEO_FOURCC((str)[0], (str)[1], (str)[2], (str)[3])

/**
 * @defgroup video_pixel_formats Video pixel formats
 * The @c | character separate the pixel, and spaces separate the bytes.
 * The uppercase letter represents the most significant bit.
 * The lowercase letters represent the rest of the bits.
 * @{
 */

/**
 * @name Bayer formats (R, G, B channels).
 * @{
 */

/**
 * @verbatim
 * | Bbbbbbbb | Gggggggg | Bbbbbbbb | Gggggggg | Bbbbbbbb | Gggggggg | ...
 * | Gggggggg | Rrrrrrrr | Gggggggg | Rrrrrrrr | Gggggggg | Rrrrrrrr | ...
 * @endverbatim
 */
#define VIDEO_PIX_FMT_BGGR8      VIDEO_FOURCC('B', 'A', '8', '1')
#define VIDEO_PIX_FMT_BGGR8_BITS 8

/**
 * @verbatim
 * | Bbbbbbbb | Gggggggg | Bbbbbbbb | Gggggggg | Bbbbbbbb | Gggggggg | ...
 * | Rrrrrrrr | Gggggggg | Rrrrrrrr | Gggggggg | Rrrrrrrr | Gggggggg | ...
 * @endverbatim
 */
#define VIDEO_PIX_FMT_GBRG8      VIDEO_FOURCC('G', 'B', 'R', 'G')
#define VIDEO_PIX_FMT_GBRG8_BITS 8

/**
 * @verbatim
 * | Gggggggg | Rrrrrrrr | Gggggggg | Rrrrrrrr | Gggggggg | Rrrrrrrr | ...
 * | Bbbbbbbb | Gggggggg | Bbbbbbbb | Gggggggg | Bbbbbbbb | Gggggggg | ...
 * @endverbatim
 */
#define VIDEO_PIX_FMT_GRBG8      VIDEO_FOURCC('G', 'R', 'B', 'G')
#define VIDEO_PIX_FMT_GRBG8_BITS 8

/**
 * @verbatim
 * | Rrrrrrrr | Gggggggg | Rrrrrrrr | Gggggggg | Rrrrrrrr | Gggggggg | ...
 * | Gggggggg | Bbbbbbbb | Gggggggg | Bbbbbbbb | Gggggggg | Bbbbbbbb | ...
 * @endverbatim
 */
#define VIDEO_PIX_FMT_RGGB8      VIDEO_FOURCC('R', 'G', 'G', 'B')
#define VIDEO_PIX_FMT_RGGB8_BITS 8

/**
 * @}
 */

/**
 * @name RGB formats
 * Per-color (R, G, B) channels.
 * @{
 */

/**
 * 5-bit blue followed by 6-bit green followed by 5-bit red, in little-endian over two bytes.
 * @verbatim
 * | gggRrrrr | BbbbbGgg | ...
 * @endverbatim
 */
#define VIDEO_PIX_FMT_RGB565      VIDEO_FOURCC('R', 'G', 'B', 'P')
#define VIDEO_PIX_FMT_RGB565_BITS 16

/**
 * There is an empty (X) byte for each pixel.
 * @verbatim
 * | Xxxxxxxx Rrrrrrrr Gggggggg Bbbbbbbb | ...
 * @endverbatim
 */
#define VIDEO_PIX_FMT_XRGB32      VIDEO_FOURCC('B', 'X', '2', '4')
#define VIDEO_PIX_FMT_XRGB32_BITS 32

/**
 * @}
 */

/**
 * @name YUV formats
 * Luminance (Y) and chrominance (U, V) channels.
 * @{
 */

/**
 * There is either a missing channel per pixel, U or V.
 * The value is to be averaged over 2 pixels to get the value of individual pixel.
 * @verbatim
 * | Yyyyyyyy Uuuuuuuu | Yyyyyyyy Vvvvvvvv | ...
 * @endverbatim
 */
#define VIDEO_PIX_FMT_YUYV      VIDEO_FOURCC('Y', 'U', 'Y', 'V')
#define VIDEO_PIX_FMT_YUYV_BITS 16

/**
 * There is an empty (X) byte for each pixel.
 * @verbatim
 * | Xxxxxxxx Yyyyyyyy Uuuuuuuu Vvvvvvvv | ...
 * @endverbatim
 */
#define VIDEO_PIX_FMT_XYUV32      VIDEO_FOURCC('X', 'Y', 'U', 'V')
#define VIDEO_PIX_FMT_XYUV32_BITS 32

/**
 * @}
 */

/**
 * @name Compressed formats
 * @{
 */

/**
 * Both JPEG (single frame) and Motion-JPEG (MJPEG, multiple JPEG frames concatenated)
 */
#define VIDEO_PIX_FMT_JPEG VIDEO_FOURCC('J', 'P', 'E', 'G')

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_VIDEO_H_ */

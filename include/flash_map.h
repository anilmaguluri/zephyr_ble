/*
 * Copyright (c) 2017 Nordic Semiconductor ASA
 * Copyright (c) 2015 Runtime Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_FLASH_MAP_H_
#define ZEPHYR_INCLUDE_FLASH_MAP_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 *
 * Provides abstraction of flash regions for type of use.
 * I.e. dude where's my image?
 *
 * System will contain a map which contains flash areas. Every
 * region will contain flash identifier, offset within flash and length.
 *
 * 1. This system map could be in a file within filesystem (Initializer
 * must know/figure out where the filesystem is at).
 * 2. Map could be at fixed location for project (compiled to code)
 * 3. Map could be at specific place in flash (put in place at mfg time).
 *
 * Note that the map you use must be valid for BSP it's for,
 * match the linker scripts when platform executes from flash,
 * and match the target offset specified in download script.
 */
#include <zephyr/types.h>
#include <stddef.h>
#include <sys/types.h>

#define SOC_FLASH_0_ID 0	/* device_id for SoC flash memory driver */
#define SPI_FLASH_0_ID 1	/* device_id for external SPI flash driver */

struct flash_area {
	u8_t fa_id;
	u8_t fa_device_id;
	u16_t pad16;
	off_t fa_off;
	size_t fa_size;
	const char *fa_dev_name;
};

struct flash_sector {
	off_t fs_off;
	size_t fs_size;
};

/*
 * Start using flash area.
 */
int flash_area_open(u8_t id, const struct flash_area **fa);

void flash_area_close(const struct flash_area *fa);

/*
 * Read/write/erase. Offset is relative from beginning of flash area.
 */
int flash_area_read(const struct flash_area *fa, off_t off, void *dst,
		    size_t len);
int flash_area_write(const struct flash_area *fa, off_t off, const void *src,
		     size_t len);
int flash_area_erase(const struct flash_area *fa, off_t off, size_t len);
/* int flash_area_is_empty(const struct flash_area *, bool *); */

/*
 * Alignment restriction for flash writes.
 */
u8_t flash_area_align(const struct flash_area *fa);

/*
 * Given flash area ID, return info about sectors within the area.
 */
int flash_area_get_sectors(int fa_id, u32_t *count,
			   struct flash_sector *sectors);

/**
 * Check whether given flash area has supporting flash driver
 * in the system.
 *
 * @param fa Flash area.
 *
 * @return 1 On success. -ENODEV if no driver match.
 */
int flash_area_has_driver(const struct flash_area *fa);

/**
 * Get driver for given flash area.
 *
 * @param fa Flash area.
 *
 * @return device driver.
 */
struct device *flash_area_get_device(const struct flash_area *fa);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_FLASH_MAP_H_ */

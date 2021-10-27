/*
 * Copyright (c) 2020 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <kernel.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(gdbstub);

#include <sys/util.h>

#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <toolchain.h>
#include <sys/types.h>
#include <sys/util.h>

#include <debug/gdbstub.h>
#include "gdbstub_backend.h"

/* +1 is for the NULL character added during receive */
#define GDB_PACKET_SIZE     (CONFIG_GDBSTUB_BUF_SZ + 1)

/* GDB remote serial protocol does not define errors value properly
 * and handle all error packets as the same the code error is not
 * used. There are informal values used by others gdbstub
 * implementation, like qemu. Lets use the same here.
 */
#define GDB_ERROR_GENERAL   "E01"
#define GDB_ERROR_MEMORY    "E14"
#define GDB_ERROR_OVERFLOW  "E22"

static bool not_first_start;

/* Empty memory region array */
__weak const struct gdb_mem_region gdb_mem_region_array[0];

/* Number of memory regions, default to 0 */
__weak const size_t gdb_mem_num_regions;

/**
 * Given a starting address and length of a memory block, find a memory
 * region descriptor from the memory region array where the memory block
 * fits inside the memory region.
 *
 * @param addr Starting address of the memory block
 * @param len  Length of the memory block
 *
 * @return Pointer to the memory region description if found.
 *         NULL if not found.
 */
static inline const
struct gdb_mem_region *find_memory_region(const uintptr_t addr, const size_t len)
{
	const struct gdb_mem_region *r, *ret = NULL;
	unsigned int idx;

	for (idx = 0; idx < gdb_mem_num_regions; idx++) {
		r = &gdb_mem_region_array[idx];

		if ((addr >= r->start) &&
		    (addr < r->end) &&
		    ((addr + len) >= r->start) &&
		    ((addr + len) < r->end)) {
			ret = r;
			break;
		}
	}

	return ret;
}

bool gdb_mem_can_read(const uintptr_t addr, const size_t len, uint8_t *align)
{
	bool ret = false;
	const struct gdb_mem_region *r;

	if (gdb_mem_num_regions == 0) {
		/*
		 * No region is defined.
		 * Assume memory access is not restricted, and there is
		 * no alignment requirement.
		 */
		*align = 1;
		ret = true;
	} else {
		r = find_memory_region(addr, len);
		if (r != NULL) {
			if ((r->attributes & GDB_MEM_REGION_READ) ==
			    GDB_MEM_REGION_READ) {
				if (r->alignment > 0) {
					*align = r->alignment;
				} else {
					*align = 1;
				}
				ret = true;
			}
		}
	}

	return ret;
}

bool gdb_mem_can_write(const uintptr_t addr, const size_t len, uint8_t *align)
{
	bool ret = false;
	const struct gdb_mem_region *r;

	if (gdb_mem_num_regions == 0) {
		/*
		 * No region is defined.
		 * Assume memory access is not restricted, and there is
		 * no alignment requirement.
		 */
		*align = 1;
		ret = true;
	} else {
		r = find_memory_region(addr, len);
		if (r != NULL) {
			if ((r->attributes & GDB_MEM_REGION_WRITE) ==
			    GDB_MEM_REGION_WRITE) {
				if (r->alignment > 0) {
					*align = r->alignment;
				} else {
					*align = 1;
				}

				ret = true;
			}
		}
	}

	return ret;
}

size_t gdb_bin2hex(const uint8_t *buf, size_t buflen, char *hex, size_t hexlen)
{
	if ((hexlen + 1) < buflen * 2) {
		return 0;
	}

	for (size_t i = 0; i < buflen; i++) {
		if (hex2char(buf[i] >> 4, &hex[2 * i]) < 0) {
			return 0;
		}
		if (hex2char(buf[i] & 0xf, &hex[2 * i + 1]) < 0) {
			return 0;
		}
	}

	return 2 * buflen;
}

/**
 * Add preamble and termination to the given data.
 *
 * It returns 0 if the packet was acknowledge, -1 otherwise.
 */
static int gdb_send_packet(const uint8_t *data, size_t len)
{
	uint8_t buf[2];
	uint8_t checksum = 0;

	/* Send packet start */
	z_gdb_putchar('$');

	/* Send packet data and calculate checksum */
	while (len-- > 0) {
		checksum += *data;
		z_gdb_putchar(*data++);
	}

	/* Send the checksum */
	z_gdb_putchar('#');

	if (gdb_bin2hex(&checksum, 1, buf, sizeof(buf)) == 0) {
		return -1;
	}

	z_gdb_putchar(buf[0]);
	z_gdb_putchar(buf[1]);

	if (z_gdb_getchar() == '+') {
		return 0;
	}

	/* Just got an invalid response */
	return -1;
}

/**
 * Receives one whole GDB packet.
 *
 * @retval  0 Success
 * @retval -1 Checksum error
 * @retval -2 Incoming packet too large
 */
static int gdb_get_packet(uint8_t *buf, size_t buf_len, size_t *len)
{
	uint8_t ch = '0';
	uint8_t expected_checksum, checksum = 0;
	uint8_t checksum_buf[2];

	/* Wait for packet start */
	checksum = 0;

	/* wait for the start character, ignore the rest */
	while (ch != '$') {
		ch = z_gdb_getchar();
	}

	*len = 0;
	/* Read until receive '#' */
	while (true) {
		ch = z_gdb_getchar();

		if (ch == '#') {
			break;
		}

		/* Only put into buffer if not full */
		if (*len < (buf_len - 1)) {
			buf[*len] = ch;
		}

		checksum += ch;
		(*len)++;
	}

	buf[*len] = '\0';

	/* Get checksum now */
	checksum_buf[0] = z_gdb_getchar();
	checksum_buf[1] = z_gdb_getchar();

	if (hex2bin(checksum_buf, 2, &expected_checksum, 1) == 0) {
		return -1;
	}

	/* Verify checksum */
	if (checksum != expected_checksum) {
		LOG_DBG("Bad checksum. Got 0x%x but was expecting: 0x%x",
			checksum, expected_checksum);
		/* NACK packet */
		z_gdb_putchar('-');
		return -1;
	}

	/* ACK packet */
	z_gdb_putchar('+');

	if (*len >= (buf_len - 1)) {
		return -2;
	} else {
		return 0;
	}
}

/**
 * Read data from a given memory address and length.
 *
 * @return Number of bytes read from memory, otherwise -1
 */
static int gdb_mem_read(uint8_t *buf, size_t buf_len,
			uintptr_t addr, size_t len)
{
	uint8_t data, align;
	size_t pos, count = 0;

	if (len > buf_len) {
		count = -1;
		goto out;
	}

	if (!gdb_mem_can_read(addr, len, &align)) {
		count = -1;
		goto out;
	}

	/* Read from system memory */
	for (pos = 0; pos < len; pos++) {
		data = *(uint8_t *)(addr + pos);
		count += gdb_bin2hex(&data, 1, buf + count, buf_len - count);
	}

out:
	return count;
}

/**
 * Write data to a given memory address and length.
 *
 * @return 0 if successful, otherwise -1
 */
static int gdb_mem_write(const uint8_t *buf, uintptr_t addr,
			 size_t len)
{
	uint8_t data, align;
	int ret = 0;

	if (!gdb_mem_can_write(addr, len, &align)) {
		ret = -1;
		goto out;
	}

	while (len > 0) {
		size_t ret = hex2bin(buf, 2, &data, sizeof(data));

		if (ret == 0) {
			ret = -1;
			goto out;
		}

		*(uint8_t *)addr = data;

		addr++;
		buf += 2;
		len--;
	}

	ret = 0;

out:
	return ret;
}

/**
 * Send a exception packet "T <value>"
 */
static int gdb_send_exception(uint8_t *buf, size_t len, uint8_t exception)
{
	size_t size;

	*buf = 'T';
	size = gdb_bin2hex(&exception, 1, buf + 1, len - 1);
	if (size == 0) {
		return -1;
	}

	/* Related to 'T' */
	size++;

	return gdb_send_packet(buf, size);
}

/**
 * Synchronously communicate with gdb on the host
 */
int z_gdb_main_loop(struct gdb_ctx *ctx)
{
	/* 'static' modifier is intentional so the buffer
	 * is not declared inside running stack, which may
	 * not have enough space.
	 */
	static uint8_t buf[GDB_PACKET_SIZE];

	enum loop_state {
		RECEIVING,
		CONTINUE,
		ERROR,
	} state;

	state = RECEIVING;

	/* Only send exception if this is not the first
	 * GDB break.
	 */
	if (not_first_start) {
		gdb_send_exception(buf, sizeof(buf), ctx->exception);
	} else {
		not_first_start = true;
	}

#define CHECK_ERROR(condition)			\
	{					\
		if ((condition)) {		\
			state = ERROR;		\
			break;			\
		}				\
	}

#define CHECK_SYMBOL(c)					\
	{							\
		CHECK_ERROR(ptr == NULL || *ptr != (c));	\
		ptr++;						\
	}

#define CHECK_INT(arg)							\
	{								\
		arg = strtol((const char *)ptr, (char **)&ptr, 16);	\
		CHECK_ERROR(ptr == NULL);				\
	}

	while (state == RECEIVING) {
		uint8_t *ptr;
		size_t data_len, pkt_len;
		uintptr_t addr;
		int ret;

		ret = gdb_get_packet(buf, sizeof(buf), &pkt_len);
		if ((ret == -1) || (ret == -2)) {
			/*
			 * Send error and wait for next packet.
			 *
			 * -1: Checksum error.
			 * -2: Packet too big.
			 */
			gdb_send_packet(GDB_ERROR_GENERAL, 3);
			continue;
		}

		if (pkt_len == 0) {
			continue;
		}

		ptr = buf;

		switch (*ptr++) {

		/**
		 * Read from the memory
		 * Format: m addr,length
		 */
		case 'm':
			CHECK_INT(addr);
			CHECK_SYMBOL(',');
			CHECK_INT(data_len);

			/* Read Memory */

			/*
			 * GDB ask the guest to read parameters when
			 * the user request backtrace. If the
			 * parameter is a NULL pointer this will cause
			 * a fault. Just send a packet informing that
			 * this address is invalid
			 */
			if (addr == 0L) {
				gdb_send_packet(GDB_ERROR_MEMORY, 3);
				break;
			}
			ret = gdb_mem_read(buf, sizeof(buf), addr, data_len);
			CHECK_ERROR(ret == -1);
			gdb_send_packet(buf, ret);
			break;

		/**
		 * Write to memory
		 * Format: M addr,length:val
		 */
		case 'M':
			CHECK_INT(addr);
			CHECK_SYMBOL(',');
			CHECK_INT(data_len);
			CHECK_SYMBOL(':');

			if (addr == 0L) {
				gdb_send_packet(GDB_ERROR_MEMORY, 3);
				break;
			}

			/* Write Memory */
			pkt_len = gdb_mem_write(ptr, addr, data_len);
			CHECK_ERROR(pkt_len == -1);
			gdb_send_packet("OK", 2);
			break;

		/*
		 * Continue ignoring the optional address
		 * Format: c addr
		 */
		case 'c':
			arch_gdb_continue();
			state = CONTINUE;
			break;

		/*
		 * Step one instruction ignoring the optional address
		 * s addr..addr
		 */
		case 's':
			arch_gdb_step();
			state = CONTINUE;
			break;

		/*
		 * Read all registers
		 * Format: g
		 */
		case 'g':
			pkt_len = arch_gdb_reg_readall(ctx, buf, sizeof(buf));
			CHECK_ERROR(pkt_len == 0);
			gdb_send_packet(buf, pkt_len);
			break;

		/**
		 * Write the value of the CPU registers
		 * Fromat: G XX...
		 */
		case 'G':
			pkt_len = arch_gdb_reg_writeall(ctx, ptr, pkt_len - 1);
			CHECK_ERROR(pkt_len == 0);
			gdb_send_packet("OK", 2);
			break;

		/**
		 * Read the value of a register
		 * Format: p n
		 */
		case 'p':
			CHECK_INT(addr);

			/* Read Register */
			pkt_len = arch_gdb_reg_readone(ctx, buf, sizeof(buf), addr);
			CHECK_ERROR(pkt_len == 0);
			gdb_send_packet(buf, pkt_len);
			break;

		/**
		 * Write data into a specific register
		 * Format: P register=value
		 */
		case 'P':
			CHECK_INT(addr);
			CHECK_SYMBOL('=');

			pkt_len = arch_gdb_reg_writeone(ctx, ptr, strlen(ptr), addr);
			CHECK_ERROR(pkt_len == 0);
			gdb_send_packet("OK", 2);
			break;


		/* What cause the pause  */
		case '?':
			gdb_send_exception(buf, sizeof(buf),
					   ctx->exception);
			break;

		/*
		 * Not supported action
		 */
		default:
			gdb_send_packet(NULL, 0);
			break;
		}

		/*
		 * If this is an recoverable error, send an error message to
		 * GDB and continue the debugging session.
		 */
		if (state == ERROR) {
			gdb_send_packet(GDB_ERROR_GENERAL, 3);
			state = RECEIVING;
		}
	}

#undef CHECK_ERROR
#undef CHECK_INT
#undef CHECK_SYMBOL

	return 0;
}

int gdb_init(const struct device *arg)
{
	ARG_UNUSED(arg);

	if (z_gdb_backend_init() == -1) {
		LOG_ERR("Could not initialize gdbstub backend.");
		return -1;
	}

	arch_gdb_init();
	return 0;
}

SYS_INIT(gdb_init, PRE_KERNEL_2, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

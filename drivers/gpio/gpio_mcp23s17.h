/*
 * Copyright (c) 2019 Geanix ApS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Header file for the MCP23S17 driver.
 */

#ifndef ZEPHYR_DRIVERS_GPIO_GPIO_MCP23S17_H_
#define ZEPHYR_DRIVERS_GPIO_GPIO_MCP23S17_H_

#include <kernel.h>

#include <gpio.h>
#include <spi.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Register definitions */
#define REG_IODIR_PORTA			0x00
#define REG_IODIR_PORTB			0x01
#define REG_IPOL_PORTA			0x02
#define REG_IPOL_PORTB			0x03
#define REG_GPINTEN_PORTA		0x04
#define REG_GPINTEN_PORTB		0x05
#define REG_DEFVAL_PORTA		0x06
#define REG_DEFVAL_PORTB		0x07
#define REG_INTCON_PORTA		0x08
#define REG_INTCON_PORTB		0x09
#define REG_GPPU_PORTA			0x0A
#define REG_GPPU_PORTB			0x0B
#define REG_INTF_PORTA			0x0E
#define REG_INTF_PORTB			0x0F
#define REG_INTCAP_PORTA		0x10
#define REG_INTCAP_PORTB		0x11
#define REG_GPIO_PORTA			0x12
#define REG_GPIO_PORTB			0x13
#define REG_OLAT_PORTA			0x14
#define REG_OLAT_PORTB			0x15

#define MCP23S17_ADDR			0x40
#define MCP23S17_READBIT		0x01

/** Store the port 0/1 data for each register pair. */
union mcp23s17_port_data {
	u16_t all;
	u8_t port[2];
	u8_t byte[2];
};

/** Configuration data */
struct mcp23s17_config {
	const char * const spi_dev_name;
	const u16_t slave;
	const u32_t freq;
	const char * const cs_dev;
	const u32_t cs_pin;
};


/** Runtime driver data */
struct mcp23s17_drv_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;

	/** Master SPI device */
	struct device *spi;
	struct spi_config spi_cfg;
	struct spi_cs_control mcp23s17_cs_ctrl;

	struct k_sem lock;

	struct {
		union mcp23s17_port_data iodir;
		union mcp23s17_port_data ipol;
		union mcp23s17_port_data gpinten;
		union mcp23s17_port_data defval;
		union mcp23s17_port_data intcon;
		union mcp23s17_port_data iocon;
		union mcp23s17_port_data gppu;
		union mcp23s17_port_data intf;
		union mcp23s17_port_data intcap;
		union mcp23s17_port_data gpio;
		union mcp23s17_port_data olat;
	} reg_cache;
};

#ifdef __cplusplus
}
#endif

#endif	/* ZEPHYR_DRIVERS_GPIO_GPIO_MCP23S17_H_ */

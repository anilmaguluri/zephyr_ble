/*
 * Copyright (c) 2019 Piotr Mienkowski
 * Copyright (c) 2017 ARM Ltd
 * Copyright (c) 2015-2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public APIs for GPIO drivers
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_GPIO_H_
#define ZEPHYR_INCLUDE_DRIVERS_GPIO_H_

#include <sys/__assert.h>
#include <sys/slist.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <device.h>
#include <dt-bindings/gpio/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief GPIO Driver APIs
 * @defgroup gpio_interface GPIO Driver APIs
 * @ingroup io_interfaces
 * @{
 */

/**
 * @name GPIO input/output configuration flags
 * @{
 */

/** Enables pin as input. */
#define GPIO_INPUT              (1U << 8)

/** Enables pin as output, no change to the output state. */
#define GPIO_OUTPUT             (1U << 9)

/** Disables pin for both input and output. */
#define GPIO_DISCONNECTED	0

/** @cond INTERNAL_HIDDEN */

/* Initializes output to a low state. */
#define GPIO_OUTPUT_INIT_LOW    (1U << 10)

/* Initializes output to a high state. */
#define GPIO_OUTPUT_INIT_HIGH   (1U << 11)

/* Initializes output based on logic level */
#define GPIO_OUTPUT_INIT_LOGICAL (1U << 12)

/** @endcond */

/** Configures GPIO pin as output and initializes it to a low state. */
#define GPIO_OUTPUT_LOW         (GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW)
/** Configures GPIO pin as output and initializes it to a high state. */
#define GPIO_OUTPUT_HIGH        (GPIO_OUTPUT | GPIO_OUTPUT_INIT_HIGH)
/** Configures GPIO pin as output and initializes it to a logic 0. */
#define GPIO_OUTPUT_INACTIVE    (GPIO_OUTPUT |			\
				 GPIO_OUTPUT_INIT_LOW |		\
				 GPIO_OUTPUT_INIT_LOGICAL)
/** Configures GPIO pin as output and initializes it to a logic 1. */
#define GPIO_OUTPUT_ACTIVE      (GPIO_OUTPUT |			\
				 GPIO_OUTPUT_INIT_HIGH |	\
				 GPIO_OUTPUT_INIT_LOGICAL)

/** @} */

/**
 * @name GPIO interrupt configuration flags
 * The `GPIO_INT_*` flags are used to specify how input GPIO pins will trigger
 * interrupts. The interrupts can be sensitive to pin physical or logical level.
 * Interrupts sensitive to pin logical level take into account GPIO_ACTIVE_LOW
 * flag. If a pin was configured as Active Low, physical level low will be
 * considered as logical level 1 (an active state), physical level high will
 * be considered as logical level 0 (an inactive state).
 * @{
 */

/** Disables GPIO pin interrupt. */
#define GPIO_INT_DISABLE               (1U << 13)

/** @cond INTERNAL_HIDDEN */

/* Enables GPIO pin interrupt. */
#define GPIO_INT_ENABLE                (1U << 14)

/* GPIO interrupt is sensitive to logical levels.
 *
 * This is a component flag that should be combined with other
 * `GPIO_INT_*` flags to produce a meaningful configuration.
 */
#define GPIO_INT_LEVELS_LOGICAL        (1U << 15)

/* GPIO interrupt is edge sensitive.
 *
 * Note: by default interrupts are level sensitive.
 *
 * This is a component flag that should be combined with other
 * `GPIO_INT_*` flags to produce a meaningful configuration.
 */
#define GPIO_INT_EDGE                  (1U << 16)

/* Trigger detection when input state is (or transitions to) physical low or
 * logical 0 level.
 *
 * This is a component flag that should be combined with other
 * `GPIO_INT_*` flags to produce a meaningful configuration.
 */
#define GPIO_INT_LOW_0                 (1U << 17)

/* Trigger detection on input state is (or transitions to) physical high or
 * logical 1 level.
 *
 * This is a component flag that should be combined with other
 * `GPIO_INT_*` flags to produce a meaningful configuration.
 */
#define GPIO_INT_HIGH_1                (1U << 18)

/** @endcond */

/** Configures GPIO interrupt to be triggered on pin rising edge and enables it.
 */
#define GPIO_INT_EDGE_RISING           (GPIO_INT_ENABLE | \
					GPIO_INT_EDGE | \
					GPIO_INT_HIGH_1)

/** Configures GPIO interrupt to be triggered on pin falling edge and enables
 * it.
 */
#define GPIO_INT_EDGE_FALLING          (GPIO_INT_ENABLE | \
					GPIO_INT_EDGE | \
					GPIO_INT_LOW_0)

/** Configures GPIO interrupt to be triggered on pin rising or falling edge and
 * enables it.
 */
#define GPIO_INT_EDGE_BOTH             (GPIO_INT_ENABLE | \
					GPIO_INT_EDGE | \
					GPIO_INT_LOW_0 | \
					GPIO_INT_HIGH_1)

/** Configures GPIO interrupt to be triggered on pin physical level low and
 * enables it.
 */
#define GPIO_INT_LEVEL_LOW             (GPIO_INT_ENABLE | \
					GPIO_INT_LOW_0)

/** Configures GPIO interrupt to be triggered on pin physical level high and
 * enables it.
 */
#define GPIO_INT_LEVEL_HIGH            (GPIO_INT_ENABLE | \
					GPIO_INT_HIGH_1)

/** Configures GPIO interrupt to be triggered on pin state change to logical
 * level 0 and enables it.
 */
#define GPIO_INT_EDGE_TO_INACTIVE      (GPIO_INT_ENABLE | \
					GPIO_INT_LEVELS_LOGICAL | \
					GPIO_INT_EDGE | \
					GPIO_INT_LOW_0)

/** Configures GPIO interrupt to be triggered on pin state change to logical
 * level 1 and enables it.
 */
#define GPIO_INT_EDGE_TO_ACTIVE        (GPIO_INT_ENABLE | \
					GPIO_INT_LEVELS_LOGICAL | \
					GPIO_INT_EDGE | \
					GPIO_INT_HIGH_1)

/** Configures GPIO interrupt to be triggered on pin logical level 0 and enables
 * it.
 */
#define GPIO_INT_LEVEL_INACTIVE        (GPIO_INT_ENABLE | \
					GPIO_INT_LEVELS_LOGICAL | \
					GPIO_INT_LOW_0)

/** Configures GPIO interrupt to be triggered on pin logical level 1 and enables
 * it.
 */
#define GPIO_INT_LEVEL_ACTIVE          (GPIO_INT_ENABLE | \
					GPIO_INT_LEVELS_LOGICAL | \
					GPIO_INT_HIGH_1)

/** @} */

/** Enable GPIO pin debounce.
 *
 * @note Drivers that do not support a debounce feature should ignore
 * this flag rather than rejecting the configuration with -ENOTSUP.
 */
#define GPIO_INT_DEBOUNCE              (1U << 19)

/**
 * @name GPIO drive strength flags
 * The `GPIO_DS_*` flags are used with `gpio_pin_configure` to specify the drive
 * strength configuration of a GPIO pin.
 *
 * The drive strength of individual pins can be configured
 * independently for when the pin output is low and high.
 *
 * The `GPIO_DS_*_LOW` enumerations define the drive strength of a pin
 * when output is low.

 * The `GPIO_DS_*_HIGH` enumerations define the drive strength of a pin
 * when output is high.
 *
 * The interface supports two different drive strengths:
 * `DFLT` - The lowest drive strength supported by the HW
 * `ALT` - The highest drive strength supported by the HW
 *
 * On hardware that supports only one standard drive strength, both
 * `DFLT` and `ALT` have the same behavior.
 * @{
 */
/** @cond INTERNAL_HIDDEN */
#define GPIO_DS_LOW_POS 20
#define GPIO_DS_LOW_MASK (0x3U << GPIO_DS_LOW_POS)
/** @endcond */

/** Default drive strength standard when GPIO pin output is low.
 */
#define GPIO_DS_DFLT_LOW (0x0U << GPIO_DS_LOW_POS)

/** Alternative drive strength when GPIO pin output is low.
 * For hardware that does not support configurable drive strength
 * use the default drive strength.
 */
#define GPIO_DS_ALT_LOW (0x1U << GPIO_DS_LOW_POS)

/** @cond INTERNAL_HIDDEN */
#define GPIO_DS_HIGH_POS 22
#define GPIO_DS_HIGH_MASK (0x3U << GPIO_DS_HIGH_POS)
/** @endcond */

/** Default drive strength when GPIO pin output is high.
 */
#define GPIO_DS_DFLT_HIGH (0x0U << GPIO_DS_HIGH_POS)

/** Alternative drive strength when GPIO pin output is high.
 * For hardware that does not support configurable drive strengths
 * use the default drive strength.
 */
#define GPIO_DS_ALT_HIGH (0x1U << GPIO_DS_HIGH_POS)
/** @} */

/** @name Deprecated Flags
 * @{
 */

/** @cond INTERNAL_HIDDEN */
#define GPIO_DIR_MASK		(GPIO_INPUT | GPIO_OUTPUT)
/** @endcond */

/** Legacy flag indicating pin is configured as input only.
 *
 * @deprecated Replace with `GPIO_INPUT`.
 */
#define GPIO_DIR_IN             GPIO_INPUT

/** Legacy flag indicating pin is configured as output.
 *
 * @deprecated Replace with `GPIO_OUTPUT`.
 */
#undef GPIO_DIR_OUT
#define GPIO_DIR_OUT            GPIO_OUTPUT

/** Legacy flag indicating pin is disconnected when GPIO pin output is low.
 *
 * @deprecated Replace with `GPIO_OPEN_SOURCE`.
 */
#define GPIO_DS_DISCONNECT_LOW  GPIO_OPEN_SOURCE

/** Legacy flag indicating pin is disconnected when GPIO pin output is high.
 *
 * @deprecated Replace with `GPIO_OPEN_DRAIN`.
 */
#define GPIO_DS_DISCONNECT_HIGH GPIO_OPEN_DRAIN

/** @cond INTERNAL_HIDDEN */
#define GPIO_PUD_SHIFT		4
#define GPIO_PUD_MASK		(0x3U << GPIO_PUD_SHIFT)
/** @endcond */

/** Pin is neither pull-up nor pull-down.
 *
 * @deprecated Not used any more
 */
#define GPIO_PUD_NORMAL		0U

/** Enable GPIO pin pull-up.
 *
 * @deprecated Replace with `GPIO_PULL_UP`.
 */
#undef GPIO_PUD_PULL_UP
#define GPIO_PUD_PULL_UP	GPIO_PULL_UP

/** Enable GPIO pin pull-down.
 *
 * @deprecated Replace with `GPIO_PULL_DOWN`.
 */
#undef GPIO_PUD_PULL_DOWN
#define GPIO_PUD_PULL_DOWN	GPIO_PULL_DOWN

/** Legacy flag indicating that interrupt is enabled.
 *
 * @deprecated Replace with `GPIO_INT_ENABLE`.
 */
#define GPIO_INT                GPIO_INT_ENABLE

/** Legacy flag indicating that interrupt is level sensitive.
 *
 * @deprecated Replace with `GPIO_INT_LEVEL_LOW`, `GPIO_INT_LEVEL_HIGH`.
 */
#define GPIO_INT_LEVEL          (0U << 14)

/** Legacy flag setting indicating signal or interrupt active level.
 *
 * This flag was used both to indicate a signal's active level, and to
 * indicate the level associated with an interrupt on a signal.  As
 * active level is also relevant to output signals the two
 * interpretations have been separated.  The legacy value supports
 * testing for interrupt level as this is the most common use in
 * existing code.
 *
 * @deprecated Replace with `GPIO_ACTIVE_LOW` or `GPIO_INT_LOW_0`
 * depending on intent.
 */
#undef GPIO_INT_ACTIVE_LOW
#define GPIO_INT_ACTIVE_LOW     GPIO_INT_LOW_0

/** Legacy flag setting indicating signal or interrupt active level.
 *
 * This flag was used both to indicate a signal's active level, and to
 * indicate the level associated with an interrupt on a signal.  As
 * active level is also relevant to output signals the two
 * interpretations have been separated.  The legacy value supports
 * testing for interrupt level as this is the most common use in
 * existing code.
 *
 * @deprecated Replace with `GPIO_ACTIVE_HIGH` or `GPIO_INT_HIGH_1`
 * depending on intent.
 */
#undef GPIO_INT_ACTIVE_HIGH
#define GPIO_INT_ACTIVE_HIGH    GPIO_INT_HIGH_1

/** Legacy flag indicating interrupt triggers on both rising and falling edge.
 *
 * @deprecated Replace with `GPIO_INT_EDGE_BOTH`.
 */
#define GPIO_INT_DOUBLE_EDGE    GPIO_INT_EDGE_BOTH

/** @cond INTERNAL_HIDDEN */
#define GPIO_POL_SHIFT		0
#define GPIO_POL_MASK		(1U << GPIO_POL_SHIFT)
/** @endcond */

/** Legacy flag indicating that GPIO pin polarity is normal.
 *
 * @deprecated Replace with `GPIO_ACTIVE_HIGH`.
 */
#define GPIO_POL_NORMAL		GPIO_ACTIVE_HIGH

/** Legacy flag indicating that GPIO pin polarity is inverted.
 *
 * @deprecated Replace with `GPIO_ACTIVE_LOW`.
 */
#define GPIO_POL_INV		GPIO_ACTIVE_LOW

/** @} */

/** @cond INTERNAL_HIDDEN */
#define GPIO_ACCESS_BY_PIN 0
#define GPIO_ACCESS_BY_PORT 1
/**
 * @endcond
 */

/**
 * @brief Identifies a set of pins associated with a port.
 *
 * The pin with index n is present in the set if and only if the bit
 * identified by (1U << n) is set.
 */
typedef u32_t gpio_port_pins_t;

/**
 * @brief Provides values for a set of pins associated with a port.
 *
 * The value for a pin with index n is high (physical mode) or active
 * (logical mode) if and only if the bit identified by (1U << n) is set.
 * Otherwise the value for the pin is low (physical mode) or inactive
 * (logical mode).
 *
 * Values of this type are often paired with a `gpio_port_pins_t` value
 * that specifies which encoded pin values are valid for the operation.
 */
typedef u32_t gpio_port_value_t;

/**
 * @brief Maximum number of pins that are supported by `gpio_port_pins_t`.
 */
#define GPIO_MAX_PINS_PER_PORT (sizeof(gpio_port_pins_t) * __CHAR_BIT__)

/**
 * This structure is common to all GPIO drivers and is expected to be the first
 * element in the driver's struct driver_data decleration.
 */
struct gpio_driver_data {
	gpio_port_pins_t invert;
};

struct gpio_callback;

/**
 * @typedef gpio_callback_handler_t
 * @brief Define the application callback handler function signature
 *
 * @param "struct device *port" Device struct for the GPIO device.
 * @param "struct gpio_callback *cb" Original struct gpio_callback
 *        owning this handler
 * @param "u32_t pins" Mask of pins that triggers the callback handler
 *
 * Note: cb pointer can be used to retrieve private data through
 * CONTAINER_OF() if original struct gpio_callback is stored in
 * another private structure.
 */
typedef void (*gpio_callback_handler_t)(struct device *port,
					struct gpio_callback *cb,
					gpio_port_pins_t pins);

/**
 * @brief GPIO callback structure
 *
 * Used to register a callback in the driver instance callback list.
 * As many callbacks as needed can be added as long as each of them
 * are unique pointers of struct gpio_callback.
 * Beware such structure should not be allocated on stack.
 *
 * Note: To help setting it, see gpio_init_callback() below
 */
struct gpio_callback {
	/** This is meant to be used in the driver and the user should not
	 * mess with it (see drivers/gpio/gpio_utils.h)
	 */
	sys_snode_t node;

	/** Actual callback function being called when relevant. */
	gpio_callback_handler_t handler;

	/** A mask of pins the callback is interested in, if 0 the callback
	 * will never be called. Such pin_mask can be modified whenever
	 * necessary by the owner, and thus will affect the handler being
	 * called or not. The selected pins must be configured to trigger
	 * an interrupt.
	 */
	gpio_port_pins_t pin_mask;
};

/**
 * @cond INTERNAL_HIDDEN
 *
 * For internal use only, skip these in public documentation.
 */

/* Used by driver api function pin_interrupt_configure, these are defined
 * in terms of the public int flags so we can just mask and pass them
 * through to the driver api
 */
enum gpio_int_mode {
	GPIO_INT_MODE_DISABLED = GPIO_INT_DISABLE,
	GPIO_INT_MODE_LEVEL = GPIO_INT_ENABLE,
	GPIO_INT_MODE_EDGE = GPIO_INT_ENABLE | GPIO_INT_EDGE,
};

enum gpio_int_trig {
	/* Trigger detection when input state is (or transitions to)
	 * physical low. (Edge Failing or Active Low) */
	GPIO_INT_TRIG_LOW = GPIO_INT_LOW_0,
	/* Trigger detection when input state is (or transitions to)
	 * physical high. (Edge Rising or Active High) */
	GPIO_INT_TRIG_HIGH = GPIO_INT_HIGH_1,
	/* Trigger detection on pin rising or falling edge. */
	GPIO_INT_TRIG_BOTH = GPIO_INT_LOW_0 | GPIO_INT_HIGH_1,
};

struct gpio_driver_api {
	int (*config)(struct device *port, int access_op, u32_t pin, int flags);
	int (*write)(struct device *port, int access_op, u32_t pin,
		     u32_t value);
	int (*read)(struct device *port, int access_op, u32_t pin,
		    u32_t *value);
	int (*port_get_raw)(struct device *port, gpio_port_value_t *value);
	int (*port_set_masked_raw)(struct device *port, gpio_port_pins_t mask,
				   gpio_port_value_t value);
	int (*port_set_bits_raw)(struct device *port, gpio_port_pins_t pins);
	int (*port_clear_bits_raw)(struct device *port, gpio_port_pins_t pins);
	int (*port_toggle_bits)(struct device *port, gpio_port_pins_t pins);
	int (*pin_interrupt_configure)(struct device *port, unsigned int pin,
				       enum gpio_int_mode, enum gpio_int_trig);
	int (*manage_callback)(struct device *port, struct gpio_callback *cb,
			       bool set);
	int (*enable_callback)(struct device *port, int access_op, u32_t pin);
	int (*disable_callback)(struct device *port, int access_op, u32_t pin);
	u32_t (*get_pending_int)(struct device *dev);
};

__syscall int gpio_config(struct device *port, int access_op, u32_t pin,
			  int flags);

static inline int z_impl_gpio_config(struct device *port, int access_op,
				    u32_t pin, int flags)
{
	const struct gpio_driver_api *api =
		(const struct gpio_driver_api *)port->driver_api;

	return api->config(port, access_op, pin, flags);
}

__syscall int gpio_write(struct device *port, int access_op, u32_t pin,
			 u32_t value);

static inline int z_impl_gpio_write(struct device *port, int access_op,
				   u32_t pin, u32_t value)
{
	const struct gpio_driver_api *api =
		(const struct gpio_driver_api *)port->driver_api;

	return api->write(port, access_op, pin, value);
}

__syscall int gpio_read(struct device *port, int access_op, u32_t pin,
			u32_t *value);

static inline int z_impl_gpio_read(struct device *port, int access_op,
				  u32_t pin, u32_t *value)
{
	const struct gpio_driver_api *api =
		(const struct gpio_driver_api *)port->driver_api;

	return api->read(port, access_op, pin, value);
}

__syscall int gpio_enable_callback(struct device *port, int access_op,
				   u32_t pin);

static inline int z_impl_gpio_enable_callback(struct device *port,
					     int access_op, u32_t pin)
{
	const struct gpio_driver_api *api =
		(const struct gpio_driver_api *)port->driver_api;

	if (api->enable_callback == NULL) {
		return -ENOTSUP;
	}

	return api->enable_callback(port, access_op, pin);
}

__syscall int gpio_disable_callback(struct device *port, int access_op,
				    u32_t pin);

static inline int z_impl_gpio_disable_callback(struct device *port,
					      int access_op, u32_t pin)
{
	const struct gpio_driver_api *api =
		(const struct gpio_driver_api *)port->driver_api;

	if (api->disable_callback == NULL) {
		return -ENOTSUP;
	}

	return api->disable_callback(port, access_op, pin);
}
/**
 * @endcond
 */

/**
 * @brief Configure pin interrupt.
 *
 * @note This function can also be used to configure interrupts on pins
 *       not controlled directly by the GPIO module. That is, pins which are
 *       routed to other modules such as I2C, SPI, UART.
 *
 * @param port Pointer to device structure for the driver instance.
 * @param pin Pin number.
 * @param flags Interrupt configuration flags as defined by GPIO_INT_*.
 *
 * @retval 0 If successful.
 * @retval -ENOTSUP If any of the configuration options is not supported
 *                  (unless otherwise directed by flag documentation).
 * @retval -EINVAL  Invalid argument.
 * @retval -EBUSY   Interrupt line required to configure pin interrupt is
 *                  already in use.
 * @retval -EIO I/O error when accessing an external GPIO chip.
 * @retval -EWOULDBLOCK if operation would block.
 */
__syscall int gpio_pin_interrupt_configure(struct device *port,
		unsigned int pin, unsigned int flags);

static inline int z_impl_gpio_pin_interrupt_configure(struct device *port,
		unsigned int pin, unsigned int flags)
{
	const struct gpio_driver_api *api =
		(const struct gpio_driver_api *)port->driver_api;
	const struct gpio_driver_data *const data =
		(const struct gpio_driver_data *)port->driver_data;
	enum gpio_int_trig trig;
	enum gpio_int_mode mode;

	__ASSERT(pin < GPIO_MAX_PINS_PER_PORT, "Invalid pin number");

	__ASSERT_NO_MSG((flags & GPIO_INT_DEBOUNCE) == 0);

	__ASSERT((flags & (GPIO_INT_DISABLE | GPIO_INT_ENABLE))
		 != (GPIO_INT_DISABLE | GPIO_INT_ENABLE),
		 "Cannot both enable and disable interrupts");

	__ASSERT((flags & (GPIO_INT_DISABLE | GPIO_INT_ENABLE)) != 0U,
		 "Must either enable or disable interrupts");

	__ASSERT(((flags & GPIO_INT_ENABLE) == 0) ||
		 ((flags & GPIO_INT_EDGE) != 0) ||
		 ((flags & (GPIO_INT_LOW_0 | GPIO_INT_HIGH_1)) !=
		  (GPIO_INT_LOW_0 | GPIO_INT_HIGH_1)),
		 "Only one of GPIO_INT_LOW_0, GPIO_INT_HIGH_1 can be "
		 "enabled for a level interrupt.");

	__ASSERT(((flags & GPIO_INT_ENABLE) == 0) ||
		 ((flags & (GPIO_INT_LOW_0 | GPIO_INT_HIGH_1)) != 0),
		 "At least one of GPIO_INT_LOW_0, GPIO_INT_HIGH_1 has to be "
		 "enabled.");

	if (((flags & GPIO_INT_LEVELS_LOGICAL) != 0) &&
	    ((data->invert & BIT(pin)) != 0)) {
		/* Invert signal bits */
		flags ^= (GPIO_INT_LOW_0 | GPIO_INT_HIGH_1);
	}

	trig = (enum gpio_int_trig)(flags & (GPIO_INT_LOW_0 | GPIO_INT_HIGH_1));
	mode = (enum gpio_int_mode)(flags & (GPIO_INT_EDGE | GPIO_INT_DISABLE | GPIO_INT_ENABLE));

	return api->pin_interrupt_configure(port, pin, mode, trig);
}

/**
 * @brief Configure a single pin.
 *
 * @param port Pointer to device structure for the driver instance.
 * @param pin Pin number to configure.
 * @param flags Flags for pin configuration: 'GPIO input/output configuration
 *        flags', 'GPIO drive strength flags', 'GPIO pin drive flags', 'GPIO pin
 *        bias flags', GPIO_INT_DEBOUNCE.
 *
 * @retval 0 If successful.
 * @retval -ENOTSUP if any of the configuration options is not supported
 *                  (unless otherwise directed by flag documentation).
 * @retval -EINVAL Invalid argument.
 * @retval -EIO I/O error when accessing an external GPIO chip.
 * @retval -EWOULDBLOCK if operation would block.
 */
static inline int gpio_pin_configure(struct device *port, u32_t pin,
				     unsigned int flags)
{
	const struct gpio_driver_api *api =
		(const struct gpio_driver_api *)port->driver_api;
	struct gpio_driver_data *data =
		(struct gpio_driver_data *)port->driver_data;
	int ret;

	__ASSERT(pin < GPIO_MAX_PINS_PER_PORT, "Invalid pin number");

	__ASSERT((flags & (GPIO_PULL_UP | GPIO_PULL_DOWN)) !=
		 (GPIO_PULL_UP | GPIO_PULL_DOWN),
		 "Pull Up and Pull Down should not be enabled simultaneously");

	__ASSERT((flags & GPIO_OUTPUT) != 0 || (flags & GPIO_SINGLE_ENDED) == 0,
		 "Output needs to be enabled for 'Open Drain', 'Open Source' "
		 "mode to be supported");

	__ASSERT_NO_MSG((flags & GPIO_SINGLE_ENDED) != 0 ||
			(flags & GPIO_LINE_OPEN_DRAIN) == 0);

	__ASSERT((flags & (GPIO_OUTPUT_INIT_LOW | GPIO_OUTPUT_INIT_HIGH)) == 0
		 || (flags & GPIO_OUTPUT) != 0,
		 "Output needs to be enabled to be initialized low or high");

	__ASSERT((flags & (GPIO_OUTPUT_INIT_LOW | GPIO_OUTPUT_INIT_HIGH))
		 != (GPIO_OUTPUT_INIT_LOW | GPIO_OUTPUT_INIT_HIGH),
		 "Output cannot be initialized low and high");

	if (((flags & GPIO_OUTPUT_INIT_LOGICAL) != 0)
	    && ((flags & (GPIO_OUTPUT_INIT_LOW | GPIO_OUTPUT_INIT_HIGH)) != 0)
	    && ((flags & GPIO_ACTIVE_LOW) != 0)) {
		flags ^= GPIO_OUTPUT_INIT_LOW | GPIO_OUTPUT_INIT_HIGH
			| GPIO_OUTPUT_INIT_LOGICAL;
	}

	ret = gpio_config(port, GPIO_ACCESS_BY_PIN, pin, flags);
	if (ret != 0) {
		return ret;
	}

	if ((flags & GPIO_ACTIVE_LOW) != 0) {
		data->invert |= BIT(pin);
	} else {
		data->invert &= ~BIT(pin);
	}
	if (((flags & (GPIO_INT_DISABLE | GPIO_INT_ENABLE)) != 0U)
	    && (api->pin_interrupt_configure != NULL)) {
		flags &= ~GPIO_INT_DEBOUNCE;
		ret = z_impl_gpio_pin_interrupt_configure(port, pin, flags);
	}

	return ret;
}

/**
 * @brief Get physical level of all input pins in a port.
 *
 * A low physical level on the pin will be interpreted as value 0. A high
 * physical level will be interpreted as value 1. This function ignores
 * GPIO_ACTIVE_LOW flag.
 *
 * Value of a pin with index n will be represented by bit n in the returned
 * port value.
 *
 * @param port Pointer to the device structure for the driver instance.
 * @param value Pointer to a variable where pin values will be stored.
 *
 * @retval 0 If successful.
 * @retval -EIO I/O error when accessing an external GPIO chip.
 * @retval -EWOULDBLOCK if operation would block.
 */
__syscall int gpio_port_get_raw(struct device *port, gpio_port_value_t *value);

static inline int z_impl_gpio_port_get_raw(struct device *port,
					   gpio_port_value_t *value)
{
	const struct gpio_driver_api *api =
		(const struct gpio_driver_api *)port->driver_api;

	return api->port_get_raw(port, value);
}

/**
 * @brief Get logical level of all input pins in a port.
 *
 * Get logical level of an input pin taking into account GPIO_ACTIVE_LOW flag.
 * If pin is configured as Active High, a low physical level will be interpreted
 * as logical value 0. If pin is configured as Active Low, a low physical level
 * will be interpreted as logical value 1.
 *
 * Value of a pin with index n will be represented by bit n in the returned
 * port value.
 *
 * @param port Pointer to the device structure for the driver instance.
 * @param value Pointer to a variable where pin values will be stored.
 *
 * @retval 0 If successful.
 * @retval -EIO I/O error when accessing an external GPIO chip.
 * @retval -EWOULDBLOCK if operation would block.
 */
static inline int gpio_port_get(struct device *port, gpio_port_value_t *value)
{
	const struct gpio_driver_data *const data =
			(const struct gpio_driver_data *)port->driver_data;
	int ret;

	ret = gpio_port_get_raw(port, value);
	if (ret == 0) {
		*value ^= data->invert;
	}

	return ret;
}

/**
 * @brief Set physical level of output pins in a port.
 *
 * Writing value 0 to the pin will set it to a low physical level. Writing
 * value 1 will set it to a high physical level. This function ignores
 * GPIO_ACTIVE_LOW flag.
 *
 * Pin with index n is represented by bit n in mask and value parameter.
 *
 * @param port Pointer to the device structure for the driver instance.
 * @param mask Mask indicating which pins will be modified.
 * @param value Value assigned to the output pins.
 *
 * @retval 0 If successful.
 * @retval -EIO I/O error when accessing an external GPIO chip.
 * @retval -EWOULDBLOCK if operation would block.
 */
__syscall int gpio_port_set_masked_raw(struct device *port,
		gpio_port_pins_t mask, gpio_port_value_t value);

static inline int z_impl_gpio_port_set_masked_raw(struct device *port,
		gpio_port_pins_t mask, gpio_port_value_t value)
{
	const struct gpio_driver_api *api =
		(const struct gpio_driver_api *)port->driver_api;

	return api->port_set_masked_raw(port, mask, value);
}

/**
 * @brief Set logical level of output pins in a port.
 *
 * Set logical level of an output pin taking into account GPIO_ACTIVE_LOW flag.
 * Value 0 sets the pin in logical 0 / inactive state. Value 1 sets the pin in
 * logical 1 / active state. If pin is configured as Active High, the default,
 * setting it in inactive state will force the pin to a low physical level. If
 * pin is configured as Active Low, setting it in inactive state will force the
 * pin to a high physical level.
 *
 * Pin with index n is represented by bit n in mask and value parameter.
 *
 * @param port Pointer to the device structure for the driver instance.
 * @param mask Mask indicating which pins will be modified.
 * @param value Value assigned to the output pins.
 *
 * @retval 0 If successful.
 * @retval -EIO I/O error when accessing an external GPIO chip.
 * @retval -EWOULDBLOCK if operation would block.
 */
static inline int gpio_port_set_masked(struct device *port,
		gpio_port_pins_t mask, gpio_port_value_t value)
{
	const struct gpio_driver_data *const data =
			(const struct gpio_driver_data *)port->driver_data;

	value ^= data->invert;

	return gpio_port_set_masked_raw(port, mask, value);
}

/**
 * @brief Set physical level of selected output pins to high.
 *
 * @param port Pointer to the device structure for the driver instance.
 * @param pins Value indicating which pins will be modified.
 *
 * @retval 0 If successful.
 * @retval -EIO I/O error when accessing an external GPIO chip.
 * @retval -EWOULDBLOCK if operation would block.
 */
__syscall int gpio_port_set_bits_raw(struct device *port,
				     gpio_port_pins_t pins);

static inline int z_impl_gpio_port_set_bits_raw(struct device *port,
						gpio_port_pins_t pins)
{
	const struct gpio_driver_api *api =
		(const struct gpio_driver_api *)port->driver_api;

	return api->port_set_bits_raw(port, pins);
}

/**
 * @brief Set logical level of selected output pins to active.
 *
 * @param port Pointer to the device structure for the driver instance.
 * @param pins Value indicating which pins will be modified.
 *
 * @retval 0 If successful.
 * @retval -EIO I/O error when accessing an external GPIO chip.
 * @retval -EWOULDBLOCK if operation would block.
 */
static inline int gpio_port_set_bits(struct device *port, gpio_port_pins_t pins)
{
	return gpio_port_set_masked(port, pins, pins);
}

/**
 * @brief Set physical level of selected output pins to low.
 *
 * @param port Pointer to the device structure for the driver instance.
 * @param pins Value indicating which pins will be modified.
 *
 * @retval 0 If successful.
 * @retval -EIO I/O error when accessing an external GPIO chip.
 * @retval -EWOULDBLOCK if operation would block.
 */
__syscall int gpio_port_clear_bits_raw(struct device *port,
				       gpio_port_pins_t pins);

static inline int z_impl_gpio_port_clear_bits_raw(struct device *port,
						  gpio_port_pins_t pins)
{
	const struct gpio_driver_api *api =
		(const struct gpio_driver_api *)port->driver_api;

	return api->port_clear_bits_raw(port, pins);
}

/**
 * @brief Set logical level of selected output pins to inactive.
 *
 * @param port Pointer to the device structure for the driver instance.
 * @param pins Value indicating which pins will be modified.
 *
 * @retval 0 If successful.
 * @retval -EIO I/O error when accessing an external GPIO chip.
 * @retval -EWOULDBLOCK if operation would block.
 */
static inline int gpio_port_clear_bits(struct device *port,
				       gpio_port_pins_t pins)
{
	return gpio_port_set_masked(port, pins, 0);
}

/**
 * @brief Toggle level of selected output pins.
 *
 * @param port Pointer to the device structure for the driver instance.
 * @param pins Value indicating which pins will be modified.
 *
 * @retval 0 If successful.
 * @retval -EIO I/O error when accessing an external GPIO chip.
 * @retval -EWOULDBLOCK if operation would block.
 */
__syscall int gpio_port_toggle_bits(struct device *port, gpio_port_pins_t pins);

static inline int z_impl_gpio_port_toggle_bits(struct device *port,
					       gpio_port_pins_t pins)
{
	const struct gpio_driver_api *api =
		(const struct gpio_driver_api *)port->driver_api;

	return api->port_toggle_bits(port, pins);
}

/**
 * @brief Set physical level of selected output pins.
 *
 * @param port Pointer to the device structure for the driver instance.
 * @param set_pins Value indicating which pins will be set to high.
 * @param clear_pins Value indicating which pins will be set to low.
 *
 * @retval 0 If successful.
 * @retval -EIO I/O error when accessing an external GPIO chip.
 * @retval -EWOULDBLOCK if operation would block.
 */
static inline int gpio_port_set_clr_bits_raw(struct device *port,
		gpio_port_pins_t set_pins, gpio_port_pins_t clear_pins)
{
	__ASSERT((set_pins & clear_pins) == 0, "Set and Clear pins overlap");

	return gpio_port_set_masked_raw(port, set_pins | clear_pins, set_pins);
}

/**
 * @brief Set logical level of selected output pins.
 *
 * @param port Pointer to the device structure for the driver instance.
 * @param set_pins Value indicating which pins will be set to active.
 * @param clear_pins Value indicating which pins will be set to inactive.
 *
 * @retval 0 If successful.
 * @retval -EIO I/O error when accessing an external GPIO chip.
 * @retval -EWOULDBLOCK if operation would block.
 */
static inline int gpio_port_set_clr_bits(struct device *port,
		gpio_port_pins_t set_pins, gpio_port_pins_t clear_pins)
{
	__ASSERT((set_pins & clear_pins) == 0, "Set and Clear pins overlap");

	return gpio_port_set_masked(port, set_pins | clear_pins, set_pins);
}

/**
 * @brief Get physical level of an input pin.
 *
 * A low physical level on the pin will be interpreted as value 0. A high
 * physical level will be interpreted as value 1. This function ignores
 * GPIO_ACTIVE_LOW flag.
 *
 * @param port Pointer to the device structure for the driver instance.
 * @param pin Pin number.
 *
 * @retval 1 If pin physical level is high.
 * @retval 0 If pin physical level is low.
 * @retval -EIO I/O error when accessing an external GPIO chip.
 * @retval -EWOULDBLOCK if operation would block.
 */
static inline int gpio_pin_get_raw(struct device *port, unsigned int pin)
{
	gpio_port_value_t value;
	int ret;

	__ASSERT(pin < GPIO_MAX_PINS_PER_PORT, "Invalid pin number");

	ret = gpio_port_get_raw(port, &value);
	if (ret == 0) {
		ret = (value & BIT(pin)) != 0 ? 1 : 0;
	}

	return ret;
}

/**
 * @brief Get logical level of an input pin.
 *
 * Get logical level of an input pin taking into account GPIO_ACTIVE_LOW flag.
 * If pin is configured as Active High, a low physical level will be interpreted
 * as logical value 0. If pin is configured as Active Low, a low physical level
 * will be interpreted as logical value 1.
 *
 * Note: If pin is configured as Active High, the default, gpio_pin_get()
 *       function is equivalent to gpio_pin_get_raw().
 *
 * @param port Pointer to the device structure for the driver instance.
 * @param pin Pin number.
 *
 * @retval 1 If pin logical value is 1 / active.
 * @retval 0 If pin logical value is 0 / inactive.
 * @retval -EIO I/O error when accessing an external GPIO chip.
 * @retval -EWOULDBLOCK if operation would block.
 */
static inline int gpio_pin_get(struct device *port, unsigned int pin)
{
	gpio_port_value_t value;
	int ret;

	__ASSERT(pin < GPIO_MAX_PINS_PER_PORT, "Invalid pin number");

	ret = gpio_port_get(port, &value);
	if (ret == 0) {
		ret = (value & BIT(pin)) != 0 ? 1 : 0;
	}

	return ret;
}

/**
 * @brief Set physical level of an output pin.
 *
 * Writing value 0 to the pin will set it to a low physical level. Writing any
 * value other than 0 will set it to a high physical level. This function
 * ignores GPIO_ACTIVE_LOW flag.
 *
 * @param port Pointer to the device structure for the driver instance.
 * @param pin Pin number.
 * @param value Value assigned to the pin.
 *
 * @retval 0 If successful.
 * @retval -EIO I/O error when accessing an external GPIO chip.
 * @retval -EWOULDBLOCK if operation would block.
 */
static inline int gpio_pin_set_raw(struct device *port, unsigned int pin,
				   int value)
{
	int ret;

	__ASSERT(pin < GPIO_MAX_PINS_PER_PORT, "Invalid pin number");

	if (value != 0)	{
		ret = gpio_port_set_bits_raw(port, BIT(pin));
	} else {
		ret = gpio_port_clear_bits_raw(port, BIT(pin));
	}

	return ret;
}

/**
 * @brief Set logical level of an output pin.
 *
 * Set logical level of an output pin taking into account GPIO_ACTIVE_LOW flag.
 * Value 0 sets the pin in logical 0 / inactive state. Any value other than 0
 * sets the pin in logical 1 / active state. If pin is configured as Active
 * High, the default, setting it in inactive state will force the pin to a low
 * physical level. If pin is configured as Active Low, setting it in inactive
 * state will force the pin to a high physical level.
 *
 * Note: If pin is configured as Active High, gpio_pin_set() function is
 *       equivalent to gpio_pin_set_raw().
 *
 * @param port Pointer to the device structure for the driver instance.
 * @param pin Pin number.
 * @param value Value assigned to the pin.
 *
 * @retval 0 If successful.
 * @retval -EIO I/O error when accessing an external GPIO chip.
 * @retval -EWOULDBLOCK if operation would block.
 */
static inline int gpio_pin_set(struct device *port, unsigned int pin, int value)
{
	const struct gpio_driver_data *const data =
			(const struct gpio_driver_data *)port->driver_data;

	__ASSERT(pin < GPIO_MAX_PINS_PER_PORT, "Invalid pin number");

	if (data->invert & BIT(pin)) {
		value = (value != 0) ? 0 : 1;
	}

	return gpio_pin_set_raw(port, pin, value);
}

/**
 * @brief Toggle pin level.
 *
 * @param port Pointer to the device structure for the driver instance.
 * @param pin Pin number.
 *
 * @retval 0 If successful.
 * @retval -EIO I/O error when accessing an external GPIO chip.
 * @retval -EWOULDBLOCK if operation would block.
 */
static inline int gpio_pin_toggle(struct device *port, unsigned int pin)
{
	__ASSERT(pin < GPIO_MAX_PINS_PER_PORT, "Invalid pin number");

	return gpio_port_toggle_bits(port, BIT(pin));
}

/**
 * @brief Write the data value to a single pin.
 * @param port Pointer to the device structure for the driver instance.
 * @param pin Pin number where the data is written.
 * @param value Value set on the pin.
 * @return 0 if successful, negative errno code on failure.
 *
 * @deprecated Replace with gpio_pin_set_raw() or gpio_pin_set().
 */
static inline int gpio_pin_write(struct device *port, u32_t pin,
				 u32_t value)
{
	return gpio_write(port, GPIO_ACCESS_BY_PIN, pin, value);
}

/**
 * @brief Read the data value of a single pin.
 *
 * Read the input state of a pin, returning the value 0 or 1.
 *
 * @param port Pointer to the device structure for the driver instance.
 * @param pin Pin number where data is read.
 * @param value Integer pointer to receive the data values from the pin.
 * @return 0 if successful, negative errno code on failure.
 *
 * @deprecated Replace with gpio_pin_get_raw() or gpio_pin_get().
 */
static inline int gpio_pin_read(struct device *port, u32_t pin,
				u32_t *value)
{
	return gpio_read(port, GPIO_ACCESS_BY_PIN, pin, value);
}

/**
 * @brief Helper to initialize a struct gpio_callback properly
 * @param callback A valid Application's callback structure pointer.
 * @param handler A valid handler function pointer.
 * @param pin_mask A bit mask of relevant pins for the handler
 */
static inline void gpio_init_callback(struct gpio_callback *callback,
				      gpio_callback_handler_t handler,
				      gpio_port_pins_t pin_mask)
{
	__ASSERT(callback, "Callback pointer should not be NULL");
	__ASSERT(handler, "Callback handler pointer should not be NULL");

	callback->handler = handler;
	callback->pin_mask = pin_mask;
}

/**
 * @brief Add an application callback.
 * @param port Pointer to the device structure for the driver instance.
 * @param callback A valid Application's callback structure pointer.
 * @return 0 if successful, negative errno code on failure.
 *
 * @note Callbacks may be added to the device from within a callback
 * handler invocation, but whether they are invoked for the current
 * GPIO event is not specified.
 *
 * Note: enables to add as many callback as needed on the same port.
 */
static inline int gpio_add_callback(struct device *port,
				    struct gpio_callback *callback)
{
	const struct gpio_driver_api *api =
		(const struct gpio_driver_api *)port->driver_api;

	if (api->manage_callback == NULL) {
		return -ENOTSUP;
	}

	return api->manage_callback(port, callback, true);
}

/**
 * @brief Remove an application callback.
 * @param port Pointer to the device structure for the driver instance.
 * @param callback A valid application's callback structure pointer.
 * @return 0 if successful, negative errno code on failure.
 *
 * @warning It is explicitly permitted, within a callback handler, to
 * remove the registration for the callback that is running, i.e. @p
 * callback.  Attempts to remove other registrations on the same
 * device may result in undefined behavior, including failure to
 * invoke callbacks that remain registered and unintended invocation
 * of removed callbacks.
 *
 * Note: enables to remove as many callbacks as added through
 *       gpio_add_callback().
 */
static inline int gpio_remove_callback(struct device *port,
				       struct gpio_callback *callback)
{
	const struct gpio_driver_api *api =
		(const struct gpio_driver_api *)port->driver_api;

	if (api->manage_callback == NULL) {
		return -ENOTSUP;
	}

	return api->manage_callback(port, callback, false);
}

/**
 * @brief Enable callback(s) for a single pin.
 * @param port Pointer to the device structure for the driver instance.
 * @param pin Pin number where the callback function is enabled.
 * @return 0 if successful, negative errno code on failure.
 *
 * Note: Depending on the driver implementation, this function will enable
 *       the pin to trigger an interruption. So as a semantic detail, if no
 *       callback is registered, of course none will be called.
 *
 * @deprecated Replace with ``gpio_pin_interrupt_configure()`` with
 * ``GPIO_INT_ENABLE`` along with other interrupt configuration flags.
 */
static inline int gpio_pin_enable_callback(struct device *port, u32_t pin)
{
	return gpio_enable_callback(port, GPIO_ACCESS_BY_PIN, pin);
}

/**
 * @brief Disable callback(s) for a single pin.
 * @param port Pointer to the device structure for the driver instance.
 * @param pin Pin number where the callback function is disabled.
 * @return 0 if successful, negative errno code on failure.
 *
 * @deprecated Replace with ``gpio_pin_interrupt_configure()`` with
 * ``GPIO_INT_DISABLE``.
 */
static inline int gpio_pin_disable_callback(struct device *port, u32_t pin)
{
	return gpio_disable_callback(port, GPIO_ACCESS_BY_PIN, pin);
}

/**
 * @brief Function to get pending interrupts
 *
 * The purpose of this function is to return the interrupt
 * status register for the device.
 * This is especially useful when waking up from
 * low power states to check the wake up source.
 *
 * @param dev Pointer to the device structure for the driver instance.
 *
 * @retval status != 0 if at least one gpio interrupt is pending.
 * @retval 0 if no gpio interrupt is pending.
 */
__syscall int gpio_get_pending_int(struct device *dev);

static inline int z_impl_gpio_get_pending_int(struct device *dev)
{
	const struct gpio_driver_api *api =
		(const struct gpio_driver_api *)dev->driver_api;

	if (api->get_pending_int == NULL) {
		return -ENOTSUP;
	}

	return api->get_pending_int(dev);
}

/** @cond INTERNAL_HIDDEN */
struct gpio_pin_config {
	char *gpio_controller;
	u32_t gpio_pin;
};

#define GPIO_DECLARE_PIN_CONFIG_IDX(_idx)	\
	struct gpio_pin_config gpio_pin_ ##_idx
#define GPIO_DECLARE_PIN_CONFIG		\
	GPIO_DECLARE_PIN_CONFIG_IDX()

#define GPIO_PIN_IDX(_idx, _controller, _pin)	\
	.gpio_pin_ ##_idx = {			\
		.gpio_controller = (_controller),\
		.gpio_pin = (_pin),		\
	}
#define GPIO_PIN(_controller, _pin)		\
	GPIO_PIN_IDX(, _controller, _pin)

#define GPIO_GET_CONTROLLER_IDX(_idx, _conf)	\
	((_conf)->gpio_pin_ ##_idx.gpio_controller)
#define GPIO_GET_PIN_IDX(_idx, _conf)	\
	((_conf)->gpio_pin_ ##_idx.gpio_pin)

#define GPIO_GET_CONTROLLER(_conf)	GPIO_GET_CONTROLLER_IDX(, _conf)
#define GPIO_GET_PIN(_conf)		GPIO_GET_PIN_IDX(, _conf)
/** @endcond */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#include <syscalls/gpio.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_GPIO_H_ */

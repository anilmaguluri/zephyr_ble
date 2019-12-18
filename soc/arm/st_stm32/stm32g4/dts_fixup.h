/*
 * Copyright (c) 2019 Richard Osterloh <richard.osterloh@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* SoC level DTS fixup file */

#define DT_NUM_IRQ_PRIO_BITS			DT_ARM_V7M_NVIC_E000E100_ARM_NUM_IRQ_PRIORITY_BITS

#define DT_GPIO_STM32_GPIOA_BASE_ADDRESS	DT_ST_STM32_GPIO_48000000_BASE_ADDRESS
#define DT_GPIO_STM32_GPIOA_CLOCK_CONTROLLER	DT_ST_STM32_GPIO_48000000_CLOCKS_CONTROLLER
#define DT_GPIO_STM32_GPIOA_LABEL		DT_ST_STM32_GPIO_48000000_LABEL
#define DT_GPIO_STM32_GPIOA_SIZE		DT_ST_STM32_GPIO_48000000_SIZE
#define DT_GPIO_STM32_GPIOA_CLOCK_BITS		DT_ST_STM32_GPIO_48000000_CLOCKS_BITS
#define DT_GPIO_STM32_GPIOA_CLOCK_BUS		DT_ST_STM32_GPIO_48000000_CLOCKS_BUS

#define DT_GPIO_STM32_GPIOB_BASE_ADDRESS	DT_ST_STM32_GPIO_48000400_BASE_ADDRESS
#define DT_GPIO_STM32_GPIOB_CLOCK_CONTROLLER	DT_ST_STM32_GPIO_48000400_CLOCKS_CONTROLLER
#define DT_GPIO_STM32_GPIOB_LABEL		DT_ST_STM32_GPIO_48000400_LABEL
#define DT_GPIO_STM32_GPIOB_SIZE		DT_ST_STM32_GPIO_48000400_SIZE
#define DT_GPIO_STM32_GPIOB_CLOCK_BITS		DT_ST_STM32_GPIO_48000400_CLOCKS_BITS
#define DT_GPIO_STM32_GPIOB_CLOCK_BUS		DT_ST_STM32_GPIO_48000400_CLOCKS_BUS

#define DT_GPIO_STM32_GPIOC_BASE_ADDRESS	DT_ST_STM32_GPIO_48000800_BASE_ADDRESS
#define DT_GPIO_STM32_GPIOC_CLOCK_CONTROLLER	DT_ST_STM32_GPIO_48000800_CLOCKS_CONTROLLER
#define DT_GPIO_STM32_GPIOC_LABEL		DT_ST_STM32_GPIO_48000800_LABEL
#define DT_GPIO_STM32_GPIOC_SIZE		DT_ST_STM32_GPIO_48000800_SIZE
#define DT_GPIO_STM32_GPIOC_CLOCK_BITS		DT_ST_STM32_GPIO_48000800_CLOCKS_BITS
#define DT_GPIO_STM32_GPIOC_CLOCK_BUS		DT_ST_STM32_GPIO_48000800_CLOCKS_BUS

#define DT_GPIO_STM32_GPIOD_BASE_ADDRESS	DT_ST_STM32_GPIO_48000C00_BASE_ADDRESS
#define DT_GPIO_STM32_GPIOD_CLOCK_CONTROLLER	DT_ST_STM32_GPIO_48000C00_CLOCKS_CONTROLLER
#define DT_GPIO_STM32_GPIOD_LABEL		DT_ST_STM32_GPIO_48000C00_LABEL
#define DT_GPIO_STM32_GPIOD_SIZE		DT_ST_STM32_GPIO_48000C00_SIZE
#define DT_GPIO_STM32_GPIOD_CLOCK_BITS		DT_ST_STM32_GPIO_48000C00_CLOCKS_BITS
#define DT_GPIO_STM32_GPIOD_CLOCK_BUS		DT_ST_STM32_GPIO_48000C00_CLOCKS_BUS

#define DT_GPIO_STM32_GPIOE_BASE_ADDRESS	DT_ST_STM32_GPIO_48001000_BASE_ADDRESS
#define DT_GPIO_STM32_GPIOE_CLOCK_CONTROLLER	DT_ST_STM32_GPIO_48001000_CLOCKS_CONTROLLER
#define DT_GPIO_STM32_GPIOE_LABEL		DT_ST_STM32_GPIO_48001000_LABEL
#define DT_GPIO_STM32_GPIOE_SIZE		DT_ST_STM32_GPIO_48001000_SIZE
#define DT_GPIO_STM32_GPIOE_CLOCK_BITS		DT_ST_STM32_GPIO_48001000_CLOCKS_BITS
#define DT_GPIO_STM32_GPIOE_CLOCK_BUS		DT_ST_STM32_GPIO_48001000_CLOCKS_BUS

#define DT_GPIO_STM32_GPIOF_BASE_ADDRESS	DT_ST_STM32_GPIO_48001400_BASE_ADDRESS
#define DT_GPIO_STM32_GPIOF_CLOCK_CONTROLLER	DT_ST_STM32_GPIO_48001400_CLOCKS_CONTROLLER
#define DT_GPIO_STM32_GPIOF_LABEL		DT_ST_STM32_GPIO_48001400_LABEL
#define DT_GPIO_STM32_GPIOF_SIZE		DT_ST_STM32_GPIO_48001400_SIZE
#define DT_GPIO_STM32_GPIOF_CLOCK_BITS		DT_ST_STM32_GPIO_48001400_CLOCKS_BITS
#define DT_GPIO_STM32_GPIOF_CLOCK_BUS		DT_ST_STM32_GPIO_48001400_CLOCKS_BUS

#define DT_GPIO_STM32_GPIOG_BASE_ADDRESS	DT_ST_STM32_GPIO_48001800_BASE_ADDRESS
#define DT_GPIO_STM32_GPIOG_CLOCK_CONTROLLER	DT_ST_STM32_GPIO_48001800_CLOCKS_CONTROLLER
#define DT_GPIO_STM32_GPIOG_LABEL		DT_ST_STM32_GPIO_48001800_LABEL
#define DT_GPIO_STM32_GPIOG_SIZE		DT_ST_STM32_GPIO_48001800_SIZE
#define DT_GPIO_STM32_GPIOG_CLOCK_BITS		DT_ST_STM32_GPIO_48001800_CLOCKS_BITS
#define DT_GPIO_STM32_GPIOG_CLOCK_BUS		DT_ST_STM32_GPIO_48001800_CLOCKS_BUS

#define DT_UART_STM32_USART_1_BASE_ADDRESS	DT_ST_STM32_USART_40013800_BASE_ADDRESS
#define DT_UART_STM32_USART_1_BAUD_RATE		DT_ST_STM32_USART_40013800_CURRENT_SPEED
#define DT_UART_STM32_USART_1_IRQ_PRI		DT_ST_STM32_USART_40013800_IRQ_0_PRIORITY
#define DT_UART_STM32_USART_1_NAME		DT_ST_STM32_USART_40013800_LABEL
#define DT_USART_1_IRQ				DT_ST_STM32_USART_40013800_IRQ_0
#define DT_UART_STM32_USART_1_CLOCK_BITS	DT_ST_STM32_USART_40013800_CLOCKS_BITS
#define DT_UART_STM32_USART_1_CLOCK_BUS		DT_ST_STM32_USART_40013800_CLOCKS_BUS
#define DT_UART_STM32_USART_1_HW_FLOW_CONTROL	DT_ST_STM32_USART_40013800_HW_FLOW_CONTROL

#define DT_UART_STM32_USART_2_BASE_ADDRESS	DT_ST_STM32_USART_40004400_BASE_ADDRESS
#define DT_UART_STM32_USART_2_BAUD_RATE		DT_ST_STM32_USART_40004400_CURRENT_SPEED
#define DT_UART_STM32_USART_2_IRQ_PRI		DT_ST_STM32_USART_40004400_IRQ_0_PRIORITY
#define DT_UART_STM32_USART_2_NAME		DT_ST_STM32_USART_40004400_LABEL
#define DT_USART_2_IRQ				DT_ST_STM32_USART_40004400_IRQ_0
#define DT_UART_STM32_USART_2_CLOCK_BITS	DT_ST_STM32_USART_40004400_CLOCKS_BITS
#define DT_UART_STM32_USART_2_CLOCK_BUS		DT_ST_STM32_USART_40004400_CLOCKS_BUS
#define DT_UART_STM32_USART_2_HW_FLOW_CONTROL	DT_ST_STM32_USART_40004400_HW_FLOW_CONTROL

#define DT_UART_STM32_USART_3_BASE_ADDRESS	DT_ST_STM32_USART_40004800_BASE_ADDRESS
#define DT_UART_STM32_USART_3_BAUD_RATE		DT_ST_STM32_USART_40004800_CURRENT_SPEED
#define DT_UART_STM32_USART_3_IRQ_PRI		DT_ST_STM32_USART_40004800_IRQ_0_PRIORITY
#define DT_UART_STM32_USART_3_NAME		DT_ST_STM32_USART_40004800_LABEL
#define DT_USART_3_IRQ				DT_ST_STM32_USART_40004800_IRQ_0
#define DT_UART_STM32_USART_3_CLOCK_BITS	DT_ST_STM32_USART_40004800_CLOCKS_BITS
#define DT_UART_STM32_USART_3_CLOCK_BUS		DT_ST_STM32_USART_40004800_CLOCKS_BUS
#define DT_UART_STM32_USART_3_HW_FLOW_CONTROL	DT_ST_STM32_USART_40004800_HW_FLOW_CONTROL

#define DT_UART_STM32_UART_4_BASE_ADDRESS	DT_ST_STM32_UART_40004C00_BASE_ADDRESS
#define DT_UART_STM32_UART_4_BAUD_RATE		DT_ST_STM32_UART_40004C00_CURRENT_SPEED
#define DT_UART_STM32_UART_4_IRQ_PRI		DT_ST_STM32_UART_40004C00_IRQ_0_PRIORITY
#define DT_UART_STM32_UART_4_NAME		DT_ST_STM32_UART_40004C00_LABEL
#define DT_UART_4_IRQ				DT_ST_STM32_UART_40004C00_IRQ_0
#define DT_UART_STM32_UART_4_CLOCK_BITS		DT_ST_STM32_UART_40004C00_CLOCKS_BITS
#define DT_UART_STM32_UART_4_CLOCK_BUS		DT_ST_STM32_UART_40004C00_CLOCKS_BUS
#define DT_UART_STM32_UART_4_HW_FLOW_CONTROL	DT_ST_STM32_UART_40004C00_HW_FLOW_CONTROL

#define DT_UART_STM32_LPUART_1_BASE_ADDRESS	DT_ST_STM32_LPUART_40008000_BASE_ADDRESS
#define DT_UART_STM32_LPUART_1_BAUD_RATE	DT_ST_STM32_LPUART_40008000_CURRENT_SPEED
#define DT_UART_STM32_LPUART_1_IRQ_PRI		DT_ST_STM32_LPUART_40008000_IRQ_0_PRIORITY
#define DT_UART_STM32_LPUART_1_NAME		DT_ST_STM32_LPUART_40008000_LABEL
#define DT_LPUART_1_IRQ				DT_ST_STM32_LPUART_40008000_IRQ_0
#define DT_UART_STM32_LPUART_1_CLOCK_BITS	DT_ST_STM32_LPUART_40008000_CLOCKS_BITS
#define DT_UART_STM32_LPUART_1_CLOCK_BUS	DT_ST_STM32_LPUART_40008000_CLOCKS_BUS
#define DT_UART_STM32_LPUART_1_HW_FLOW_CONTROL	DT_ST_STM32_LPUART_40008000_HW_FLOW_CONTROL

#define DT_FLASH_DEV_BASE_ADDRESS		DT_ST_STM32G4_FLASH_CONTROLLER_40022000_BASE_ADDRESS
#define DT_FLASH_DEV_NAME			DT_ST_STM32G4_FLASH_CONTROLLER_40022000_LABEL

#define DT_I2C_1_BASE_ADDRESS			DT_ST_STM32_I2C_V2_40005400_BASE_ADDRESS
#define DT_I2C_1_EVENT_IRQ_PRI			DT_ST_STM32_I2C_V2_40005400_IRQ_EVENT_PRIORITY
#define DT_I2C_1_ERROR_IRQ_PRI			DT_ST_STM32_I2C_V2_40005400_IRQ_ERROR_PRIORITY
#define DT_I2C_1_NAME			DT_ST_STM32_I2C_V2_40005400_LABEL
#define DT_I2C_1_EVENT_IRQ			DT_ST_STM32_I2C_V2_40005400_IRQ_EVENT
#define DT_I2C_1_ERROR_IRQ			DT_ST_STM32_I2C_V2_40005400_IRQ_ERROR
#define DT_I2C_1_BITRATE			DT_ST_STM32_I2C_V2_40005400_CLOCK_FREQUENCY
#define DT_I2C_1_CLOCK_BITS			DT_ST_STM32_I2C_V2_40005400_CLOCKS_BITS
#define DT_I2C_1_CLOCK_BUS			DT_ST_STM32_I2C_V2_40005400_CLOCKS_BUS

#define DT_I2C_2_BASE_ADDRESS			DT_ST_STM32_I2C_V2_40005800_BASE_ADDRESS
#define DT_I2C_2_EVENT_IRQ_PRI			DT_ST_STM32_I2C_V2_40005800_IRQ_EVENT_PRIORITY
#define DT_I2C_2_ERROR_IRQ_PRI			DT_ST_STM32_I2C_V2_40005800_IRQ_ERROR_PRIORITY
#define DT_I2C_2_NAME			DT_ST_STM32_I2C_V2_40005800_LABEL
#define DT_I2C_2_EVENT_IRQ			DT_ST_STM32_I2C_V2_40005800_IRQ_EVENT
#define DT_I2C_2_ERROR_IRQ			DT_ST_STM32_I2C_V2_40005800_IRQ_ERROR
#define DT_I2C_2_BITRATE			DT_ST_STM32_I2C_V2_40005800_CLOCK_FREQUENCY
#define DT_I2C_2_CLOCK_BITS			DT_ST_STM32_I2C_V2_40005800_CLOCKS_BITS
#define DT_I2C_2_CLOCK_BUS			DT_ST_STM32_I2C_V2_40005800_CLOCKS_BUS

#define DT_I2C_3_BASE_ADDRESS			DT_ST_STM32_I2C_V2_40007800_BASE_ADDRESS
#define DT_I2C_3_EVENT_IRQ_PRI			DT_ST_STM32_I2C_V2_40007800_IRQ_EVENT_PRIORITY
#define DT_I2C_3_ERROR_IRQ_PRI			DT_ST_STM32_I2C_V2_40007800_IRQ_ERROR_PRIORITY
#define DT_I2C_3_NAME			DT_ST_STM32_I2C_V2_40007800_LABEL
#define DT_I2C_3_EVENT_IRQ			DT_ST_STM32_I2C_V2_40007800_IRQ_EVENT
#define DT_I2C_3_ERROR_IRQ			DT_ST_STM32_I2C_V2_40007800_IRQ_ERROR
#define DT_I2C_3_BITRATE			DT_ST_STM32_I2C_V2_40007800_CLOCK_FREQUENCY
#define DT_I2C_3_CLOCK_BITS			DT_ST_STM32_I2C_V2_40007800_CLOCKS_BITS
#define DT_I2C_3_CLOCK_BUS			DT_ST_STM32_I2C_V2_40007800_CLOCKS_BUS

#define DT_RTC_0_BASE_ADDRESS			DT_ST_STM32_RTC_40002800_BASE_ADDRESS
#define DT_RTC_0_IRQ_PRI			DT_ST_STM32_RTC_40002800_IRQ_0_PRIORITY
#define DT_RTC_0_IRQ				DT_ST_STM32_RTC_40002800_IRQ_0
#define DT_RTC_0_NAME				DT_ST_STM32_RTC_40002800_LABEL
#define DT_RTC_0_CLOCK_BITS			DT_ST_STM32_RTC_40002800_CLOCKS_BITS
#define DT_RTC_0_CLOCK_BUS			DT_ST_STM32_RTC_40002800_CLOCKS_BUS

#define DT_SPI_1_BASE_ADDRESS			DT_ST_STM32_SPI_FIFO_40013000_BASE_ADDRESS
#define DT_SPI_1_IRQ_PRI			DT_ST_STM32_SPI_FIFO_40013000_IRQ_0_PRIORITY
#define DT_SPI_1_NAME				DT_ST_STM32_SPI_FIFO_40013000_LABEL
#define DT_SPI_1_IRQ				DT_ST_STM32_SPI_FIFO_40013000_IRQ_0
#define DT_SPI_1_CLOCK_BITS			DT_ST_STM32_SPI_FIFO_40013000_CLOCKS_BITS
#define DT_SPI_1_CLOCK_BUS			DT_ST_STM32_SPI_FIFO_40013000_CLOCKS_BUS

#define DT_SPI_2_BASE_ADDRESS			DT_ST_STM32_SPI_FIFO_40003800_BASE_ADDRESS
#define DT_SPI_2_IRQ_PRI			DT_ST_STM32_SPI_FIFO_40003800_IRQ_0_PRIORITY
#define DT_SPI_2_NAME				DT_ST_STM32_SPI_FIFO_40003800_LABEL
#define DT_SPI_2_IRQ				DT_ST_STM32_SPI_FIFO_40003800_IRQ_0
#define DT_SPI_2_CLOCK_BITS			DT_ST_STM32_SPI_FIFO_40003800_CLOCKS_BITS
#define DT_SPI_2_CLOCK_BUS			DT_ST_STM32_SPI_FIFO_40003800_CLOCKS_BUS

#define DT_SPI_3_BASE_ADDRESS			DT_ST_STM32_SPI_FIFO_40003C00_BASE_ADDRESS
#define DT_SPI_3_IRQ_PRI			DT_ST_STM32_SPI_FIFO_40003C00_IRQ_0_PRIORITY
#define DT_SPI_3_NAME				DT_ST_STM32_SPI_FIFO_40003C00_LABEL
#define DT_SPI_3_IRQ				DT_ST_STM32_SPI_FIFO_40003C00_IRQ_0
#define DT_SPI_3_CLOCK_BITS			DT_ST_STM32_SPI_FIFO_40003C00_CLOCKS_BITS
#define DT_SPI_3_CLOCK_BUS			DT_ST_STM32_SPI_FIFO_40003C00_CLOCKS_BUS

#define DT_USB_BASE_ADDRESS			DT_ST_STM32_USB_40005C00_BASE_ADDRESS
#define DT_USB_IRQ				DT_ST_STM32_USB_40005C00_IRQ_USB
#define DT_USB_IRQ_PRI				DT_ST_STM32_USB_40005C00_IRQ_USB_PRIORITY
#define DT_USB_NUM_BIDIR_ENDPOINTS		DT_ST_STM32_USB_40005C00_NUM_BIDIR_ENDPOINTS
#define DT_USB_RAM_SIZE				DT_ST_STM32_USB_40005C00_RAM_SIZE
#define DT_USB_CLOCK_BITS			DT_ST_STM32_USB_40005C00_CLOCKS_BITS
#define DT_USB_CLOCK_BUS			DT_ST_STM32_USB_40005C00_CLOCKS_BUS

#define DT_PWM_STM32_1_DEV_NAME             	DT_ST_STM32_PWM_40012C00_PWM_LABEL
#define DT_PWM_STM32_1_PRESCALER		DT_ST_STM32_PWM_40012C00_PWM_ST_PRESCALER
#define DT_TIM_STM32_1_BASE_ADDRESS		DT_ST_STM32_TIMERS_40012C00_BASE_ADDRESS
#define DT_TIM_STM32_1_CLOCK_BITS		DT_ST_STM32_TIMERS_40012C00_CLOCKS_BITS
#define DT_TIM_STM32_1_CLOCK_BUS		DT_ST_STM32_TIMERS_40012C00_CLOCKS_BUS

#define DT_PWM_STM32_2_DEV_NAME			DT_ST_STM32_PWM_40000000_PWM_LABEL
#define DT_PWM_STM32_2_PRESCALER		DT_ST_STM32_PWM_40000000_PWM_ST_PRESCALER
#define DT_TIM_STM32_2_BASE_ADDRESS		DT_ST_STM32_TIMERS_40000000_BASE_ADDRESS
#define DT_TIM_STM32_2_CLOCK_BITS		DT_ST_STM32_TIMERS_40000000_CLOCKS_BITS
#define DT_TIM_STM32_2_CLOCK_BUS		DT_ST_STM32_TIMERS_40000000_CLOCKS_BUS

#define DT_PWM_STM32_3_DEV_NAME			DT_ST_STM32_PWM_40000400_PWM_LABEL
#define DT_PWM_STM32_3_PRESCALER		DT_ST_STM32_PWM_40000400_PWM_ST_PRESCALER
#define DT_TIM_STM32_3_BASE_ADDRESS		DT_ST_STM32_TIMERS_40000400_BASE_ADDRESS
#define DT_TIM_STM32_3_CLOCK_BITS		DT_ST_STM32_TIMERS_40000400_CLOCKS_BITS
#define DT_TIM_STM32_3_CLOCK_BUS		DT_ST_STM32_TIMERS_40000400_CLOCKS_BUS

#define DT_PWM_STM32_4_DEV_NAME			DT_ST_STM32_PWM_40000800_PWM_LABEL
#define DT_PWM_STM32_4_PRESCALER		DT_ST_STM32_PWM_40000800_PWM_ST_PRESCALER
#define DT_TIM_STM32_4_BASE_ADDRESS		DT_ST_STM32_TIMERS_40000800_BASE_ADDRESS
#define DT_TIM_STM32_4_CLOCK_BITS		DT_ST_STM32_TIMERS_40000800_CLOCKS_BITS
#define DT_TIM_STM32_4_CLOCK_BUS		DT_ST_STM32_TIMERS_40000800_CLOCKS_BUS

#define DT_PWM_STM32_5_DEV_NAME			DT_ST_STM32_PWM_40000C00_PWM_LABEL
#define DT_PWM_STM32_5_PRESCALER		DT_ST_STM32_PWM_40000C00_PWM_ST_PRESCALER
#define DT_TIM_STM32_5_BASE_ADDRESS		DT_ST_STM32_TIMERS_40000C00_BASE_ADDRESS
#define DT_TIM_STM32_5_CLOCK_BITS		DT_ST_STM32_TIMERS_40000C00_CLOCKS_BITS
#define DT_TIM_STM32_5_CLOCK_BUS		DT_ST_STM32_TIMERS_40000C00_CLOCKS_BUS

#define DT_PWM_STM32_6_DEV_NAME			DT_ST_STM32_PWM_40001000_PWM_LABEL
#define DT_PWM_STM32_6_PRESCALER		DT_ST_STM32_PWM_40001000_PWM_ST_PRESCALER
#define DT_TIM_STM32_6_BASE_ADDRESS		DT_ST_STM32_TIMERS_40001000_BASE_ADDRESS
#define DT_TIM_STM32_6_CLOCK_BITS		DT_ST_STM32_TIMERS_40001000_CLOCKS_BITS
#define DT_TIM_STM32_6_CLOCK_BUS		DT_ST_STM32_TIMERS_40001000_CLOCKS_BUS

#define DT_PWM_STM32_7_DEV_NAME			DT_ST_STM32_PWM_40001400_PWM_LABEL
#define DT_PWM_STM32_7_PRESCALER		DT_ST_STM32_PWM_40001400_PWM_ST_PRESCALER
#define DT_TIM_STM32_7_BASE_ADDRESS		DT_ST_STM32_TIMERS_40001400_BASE_ADDRESS
#define DT_TIM_STM32_7_CLOCK_BITS		DT_ST_STM32_TIMERS_40001400_CLOCKS_BITS
#define DT_TIM_STM32_7_CLOCK_BUS		DT_ST_STM32_TIMERS_40001400_CLOCKS_BUS

#define DT_PWM_STM32_8_DEV_NAME			DT_ST_STM32_PWM_40013400_PWM_LABEL
#define DT_PWM_STM32_8_PRESCALER		DT_ST_STM32_PWM_40013400_PWM_ST_PRESCALER
#define DT_TIM_STM32_8_BASE_ADDRESS		DT_ST_STM32_TIMERS_40013400_BASE_ADDRESS
#define DT_TIM_STM32_8_CLOCK_BITS		DT_ST_STM32_TIMERS_40013400_CLOCKS_BITS
#define DT_TIM_STM32_8_CLOCK_BUS		DT_ST_STM32_TIMERS_40013400_CLOCKS_BUS

#define DT_PWM_STM32_15_DEV_NAME		DT_ST_STM32_PWM_40014000_PWM_LABEL
#define DT_PWM_STM32_15_PRESCALER		DT_ST_STM32_PWM_40014000_PWM_ST_PRESCALER
#define DT_TIM_STM32_15_BASE_ADDRESS		DT_ST_STM32_TIMERS_40014000_BASE_ADDRESS
#define DT_TIM_STM32_15_CLOCK_BITS		DT_ST_STM32_TIMERS_40014000_CLOCKS_BITS
#define DT_TIM_STM32_15_CLOCK_BUS		DT_ST_STM32_TIMERS_40014000_CLOCKS_BUS

#define DT_PWM_STM32_16_DEV_NAME		DT_ST_STM32_PWM_40014400_PWM_LABEL
#define DT_PWM_STM32_16_PRESCALER		DT_ST_STM32_PWM_40014400_PWM_ST_PRESCALER
#define DT_TIM_STM32_16_BASE_ADDRESS		DT_ST_STM32_TIMERS_40014400_BASE_ADDRESS
#define DT_TIM_STM32_16_CLOCK_BITS		DT_ST_STM32_TIMERS_40014400_CLOCKS_BITS
#define DT_TIM_STM32_16_CLOCK_BUS		DT_ST_STM32_TIMERS_40014400_CLOCKS_BUS

#define DT_PWM_STM32_17_DEV_NAME		DT_ST_STM32_PWM_40014800_PWM_LABEL
#define DT_PWM_STM32_17_PRESCALER		DT_ST_STM32_PWM_40014800_PWM_ST_PRESCALER
#define DT_TIM_STM32_17_BASE_ADDRESS		DT_ST_STM32_TIMERS_40014800_BASE_ADDRESS
#define DT_TIM_STM32_17_CLOCK_BITS		DT_ST_STM32_TIMERS_40014800_CLOCKS_BITS
#define DT_TIM_STM32_17_CLOCK_BUS		DT_ST_STM32_TIMERS_40014800_CLOCKS_BUS

#define DT_WDT_0_NAME				DT_INST_0_ST_STM32_WATCHDOG_LABEL

#define DT_WWDT_0_BASE_ADDRESS			DT_INST_0_ST_STM32_WINDOW_WATCHDOG_BASE_ADDRESS
#define DT_WWDT_0_NAME				DT_INST_0_ST_STM32_WINDOW_WATCHDOG_LABEL
#define DT_WWDT_0_IRQ				DT_INST_0_ST_STM32_WINDOW_WATCHDOG_IRQ_0
#define DT_WWDT_0_IRQ_PRI			DT_INST_0_ST_STM32_WINDOW_WATCHDOG_IRQ_0_PRIORITY
#define DT_WWDT_0_CLOCK_BITS			DT_INST_0_ST_STM32_WINDOW_WATCHDOG_CLOCKS_BITS
#define DT_WWDT_0_CLOCK_BUS			DT_INST_0_ST_STM32_WINDOW_WATCHDOG_CLOCKS_BUS

/* End of SoC Level DTS fixup file */

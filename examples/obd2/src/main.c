/**
 * Copyright (C) 2019, Sergey Shcherbakov
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * Example description:
 *
 * Basic CAN OBD-II example. It reads Speed and RPM using CAN OBD-II
 * protocol @500kbps and display result on a LCD.
 *
 * A sa reference for services and PIDs used Wiki:
 * https://en.wikipedia.org/wiki/OBD-II_PIDs
 */

// Std headers
#include <stddef.h>

// libopencm3 headers
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/cm3/nvic.h>

// Drivers
#include "../../libopencm3-drivers/include/hd44780.h"

// Default OBD-II ID
#define OBD2_BROADCAT_ID	(0x7df)

// ECU ID
#define ECU_ID				(0x7E8)

// OBD-II service mask
#define SERVICE_MASK		(~0x40)


// Very simple array size calculation function
#define ARRAY_SIZE(array) \
	(sizeof(array) / sizeof(array[0]))

// Configuration for system clock
const struct rcc_clock_scale sysclock_conf = {
	.pllm = 8,
	.plln = 288,
	.pllp = 2,
	.pllq = 6,
	.pllr = 0,
	.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,

	.hpre = RCC_CFGR_HPRE_DIV_NONE,	/* AHB = SYSCLK = 144 MHz */
	.ppre1 = RCC_CFGR_PPRE_DIV_4,	/* APB1 = SYSCLK/4 = 36MHz */
	.ppre2 = RCC_CFGR_PPRE_DIV_2,	/* APB2 = SYSCLK/2 = 72MHz */
	.voltage_scale = PWR_SCALE1,

	.flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_LATENCY_4WS,

	.ahb_frequency  = 144000000,
	.apb1_frequency = 36000000,
	.apb2_frequency = 72000000,
};


// Description of PINS that used as a bus for HD44780 display
static struct hd44780_bus lcd_bus = {
	.rs = {GPIOE, GPIO7},
	.e = {GPIOE, GPIO11},
	.rnw = {GPIOE, GPIO10},
	.db7 = {GPIOE, GPIO15},
	.db6 = {GPIOE, GPIO14},
	.db5 = {GPIOE, GPIO13},
	.db4 = {GPIOE, GPIO12},
};

static volatile uint8_t obd2Speed;
static volatile float obd2Rpm;

// Variable that used for counting ms
static volatile uint32_t tick_ms_count;

// Basic sleep function that uses TICK and busy loop
void sleep_ms(uint32_t ms)
{
	uint32_t current_ms = tick_ms_count;
	while ((tick_ms_count - current_ms) < ms);
}

void gpio_setup()
{
	// Enable GPIOE port clock
	rcc_periph_clock_enable(RCC_GPIOE);

	// Preconfigure backlight to 'on' state
	gpio_set(GPIOE, GPIO9);

	// Configure backlight GPIO
	gpio_mode_setup(GPIOE,
		GPIO_MODE_OUTPUT,
		GPIO_PUPD_NONE,
		GPIO9);
}

static void systick_setup(void)
{
	// 144MHz / 8 == 18MHz counts per second
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);

	// 18000000/1000 = 18000 overflows per second - every 1ms one interrupt
	// SysTick interrupt every N clock pulses: set reload to N-1
	systick_set_reload(sysclock_conf.ahb_frequency / 8 / 1000 - 1);

	// Enable tick interrupts
	systick_interrupt_enable();

	// Start tick counter
	systick_counter_enable();
}

void sys_tick_handler(void)
{
	tick_ms_count++;
}

static void can_setup(void)
{
	// Enable peripheral clocks
	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_CAN1);

	// Configure CAN RX and TX pins
	gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO0 | GPIO1);
	gpio_set_af(GPIOD, GPIO_AF9, GPIO0 | GPIO1);

	// Reset CAN
	can_reset(CAN1);

	// CAN1 init configuration - 500kbps
	int err = can_init(CAN1,
			false,				/* TTCM: Time triggered comm mode? */
			true,				/* ABOM: Automatic bus-off management? */
			false,				/* AWUM: Automatic wakeup mode? */
			false,				/* NART: No automatic retransmission? */
			false,				/* RFLM: Receive FIFO locked mode? */
			false,				/* TXFP: Transmit FIFO priority? */
			CAN_BTR_SJW_1TQ,		/* SJW value */
			CAN_BTR_TS1_6TQ,		/* TS1 value */
			CAN_BTR_TS2_1TQ,		/* TS2 value */
			9,				/* BRP+1: Baud rate prescaler */
			false,				/* Loopback mode? */
			false);				/* Silent mode? */

	if (err != 0) {
		// Halt - stop on break point and wait for debugger
		__asm__("BKPT");
	}

	// Init CAN1 filter #0 init
	// XXX: it's actually not configured at all, but it should be set for CAN functionality
	can_filter_id_mask_32bit_init(
			0,     /* Filter ID */
			0,     /* CAN ID */
			0,     /* CAN ID mask */
			0,     /* FIFO assignment (here: FIFO0) */
			true); /* Enable the filter. */

	// NVIC setup
	nvic_enable_irq(NVIC_CAN1_RX0_IRQ);
	nvic_set_priority(NVIC_CAN1_RX0_IRQ, 1);

	// Enable CAN1 RX interrupt
	can_enable_irq(CAN1, CAN_IER_FMPIE0);
}

void parseService01(uint8_t pid, uint8_t len, uint8_t *data)
{
	switch (pid) {
		// RPM
		case 0x0c:
			if (len != 2) {
				// Skip unknown data size
				return;
			}

			obd2Rpm = (256.0 * data[0] + data[1]) / 4;
			return;

		// Speed
		case 0x0d:
			if (len != 1) {
				// Skip unknown data size
				return;
			}

			obd2Speed = data[0];
			return;

		default:
			return;
	}
}

void parseResp(uint32_t id, uint8_t dlc, uint8_t *data)
{
	if (ECU_ID != id) {
		// Skip non-ECU ID's
		return;
	}

	if (dlc == 0) {
		// Skip all empty frames, most probably it's RTR requests
		return;
	}

	if (data[0] < 1) {
		// Skip empty OBD-II frames
		return;
	}

	switch (data[1] & SERVICE_MASK) {
		// Service $01
		case 0x01:
			parseService01(data[2], data[0] - 2, data + 3);
			return;

		default:
			return;
	}
}

// CAN1 RX0 interrupt handler
void can1_rx0_isr(void)
{
	uint32_t id;
	bool ext;
	bool rtr;
	uint8_t fmi;
	uint8_t dlc;
	uint8_t data[8];

	can_receive(CAN1,
		0,			/* FIFO #0 */
		true,		/* Release buffer after getting data */
		&id,
		&ext,
		&rtr,
		&fmi,
		&dlc,
		data,
		NULL);		/* Timestamp is not used now */

	parseResp(id, dlc, data);
}

int queryRpm(void)
{
	uint8_t query[3] = {0x02, 0x01, 0x0c};

	return can_transmit(CAN1,
			OBD2_BROADCAT_ID,
			false,
			false,
			sizeof(query),
			query);
}

int querySpeed(void)
{
	uint8_t query[3] = {0x02, 0x01, 0x0d};

	return can_transmit(CAN1,
			OBD2_BROADCAT_ID,
			false,
			false,
			sizeof(query),
			query);
}

int main(void)
{
	uint8_t speed;
	float rpm = 0;
	float rpm_prev = -1;

	rcc_clock_setup_pll(&sysclock_conf);

	gpio_setup();
	systick_setup();
	hd44780_init(&lcd_bus, 16, false, 2, false);
	can_setup();

	hd44780_printf("Globallogic\nEducation");
	sleep_ms(3000);

	hd44780_clear();

	while (1) {
		CM_ATOMIC_BLOCK() {
			speed = obd2Speed;
			rpm = obd2Rpm;
		}

		if (rpm != rpm_prev) {
			hd44780_clear();

			if (rpm > 0) {
				hd44780_printf_xy(0, 0, "Speed = %3d km/h", speed);
				hd44780_printf_xy(0, 1, "RPM = %8.02f", rpm);
			} else {
				hd44780_printf("<<Start engine>>");
			}

			rpm_prev = rpm;
		}

		queryRpm();
		querySpeed();

		sleep_ms(500);
	}

	return 0;
}

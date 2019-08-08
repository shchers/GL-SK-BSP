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
 * CAN will be configured on 500kbps. All incoming messages
 *  will be displayed on screen. Pressing buttons SW1-SW5
 *  will generate messages with DLC=1 to id 0x5A5.
 */

// Std headers
#include <stddef.h>
#include <string.h> // <- only for memcpy

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
#include "../../libopencm3-drivers/include/keys.h"

// Default CAN address
#define DFLT_CAN_ID	(0x5A5)

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

// Descriptor of PINS that used as a keys
static struct keys_s keys[] = {
	// SW1
	{
		.port = GPIOC,
		.gpio = GPIO11,
		.pup = false,
		.nc = false,
	},
	// SW2
	{
		.port = GPIOA,
		.gpio = GPIO15,
		.pup = false,
		.nc = false,
	},
	// SW3
	{
		.port = GPIOC,
		.gpio = GPIO9,
		.pup = false,
		.nc = false,
	},
	// SW4
	{
		.port = GPIOC,
		.gpio = GPIO6,
		.pup = false,
		.nc = false,
	},
	// SW5
	{
		.port = GPIOC,
		.gpio = GPIO8,
		.pup = false,
		.nc = false,
	},
};

// Basic storage for CAN data frame
struct can_frame_s {
	uint32_t id;
	bool ext;
	bool rtr;
	uint8_t fmi;
	uint8_t dlc;
	uint8_t data[8];
};

static volatile struct can_frame_s frame;

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
		0,		/* FIFO #0 */
		true,		/* Release buffer after getting data */
		&id,
		&ext,
		&rtr,
		&fmi,
		&dlc,
		data,
		NULL);		/* Timestamp is not used now */

	/* XXX: We need to use intermediate variables to avoid "... discards
	 *      'volatile' qualifier ..." warning */
	frame.id = id;
	frame.ext = ext;
	frame.rtr = rtr;
	frame.fmi = fmi;
	frame.dlc = dlc;

	/* XXX: We can just copy data bytes, but not used cells will have
	 *      garbage, so let's clean it before printing to LCD */
	memset((uint8_t*)frame.data, 0, 8);
	if (dlc) {
		memcpy((uint8_t*)frame.data, data, dlc);
	}
}

int main(void)
{
	struct can_frame_s f;
	int id;
	uint8_t d;

	rcc_clock_setup_pll(&sysclock_conf);
	gpio_setup();
	systick_setup();
	hd44780_init(&lcd_bus, 16, false, 2, false);
	keys_setup(keys, sizeof(keys)/sizeof(keys[0]));
	can_setup();

	hd44780_printf("Globallogic\nEducation");
	sleep_ms(3000);

	hd44780_clear();

	while (1) {
		// XXX: It makes code more safe, but generally it need to be replaced with queue
		CM_ATOMIC_BLOCK() {
			f = frame;
		}

		// XXX: 29-bit CAN ID is not supported
		hd44780_printf_xy(0, 0, "ID = %03X DLC = %d", f.id, f.dlc);
		hd44780_printf_xy(0, 1, "%02X%02X%02X%02X%02X%02X%02X%02X",
				f.data[0],
				f.data[1],
				f.data[2],
				f.data[3],
				f.data[4],
				f.data[5],
				f.data[6],
				f.data[7]);

		// Scan all keys and send corresponding key code if pressed
		for (id = 0; id < ARRAY_SIZE(keys); id++) {
			if (key_pressed(keys, id)) {
				d = id + 0xa0;

				can_transmit(CAN1,
					DFLT_CAN_ID,
					false,
					false,
					1,
					&d);
			}
		}

		sleep_ms(10);
	}

	return 0;
}

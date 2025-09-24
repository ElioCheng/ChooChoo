#include "arch/rpi.h"
#include "klog.h"
#include "printf.h"
#include "timer/timer.h"
#include "types.h"
#include <stdarg.h>

#ifndef MMIO_BASE
#define MMIO_BASE (char *)0xFE000000
#endif

/*********** GPIO CONFIGURATION ********************************/

static char *const GPIO_BASE = (char *)(MMIO_BASE + 0x200000);
static const u32 GPFSEL_OFFSETS[6] = { 0x00, 0x04, 0x08, 0x0c, 0x10, 0x14 };
static const u32 GPIO_PUP_PDN_CNTRL_OFFSETS[4] = { 0xe4, 0xe8, 0xec, 0xf0 };

#define GPFSEL_REG(reg) (*(u32 *)(GPIO_BASE + GPFSEL_OFFSETS[reg]))
#define GPIO_PUP_PDN_CNTRL_REG(reg) (*(u32 *)(GPIO_BASE + GPIO_PUP_PDN_CNTRL_OFFSETS[reg]))

// function control settings for GPIO pins
static const u32 GPIO_INPUT = 0x00;
static const u32 GPIO_OUTPUT = 0x01;
static const u32 GPIO_ALTFN0 = 0x04;
static const u32 GPIO_ALTFN1 = 0x05;
static const u32 GPIO_ALTFN2 = 0x06;
static const u32 GPIO_ALTFN3 = 0x07;
static const u32 GPIO_ALTFN4 = 0x03;
static const u32 GPIO_ALTFN5 = 0x02;

// pup/pdn resistor settings for GPIO pins
static const u32 GPIO_NONE = 0x00;
static const u32 GPIO_PUP = 0x01;
static const u32 GPIO_PDP = 0x02;

static const u32 GPIO_GPSET0 = 0x1c;
static const u32 GPIO_GPCLR0 = 0x28;

static void setup_gpio(u32 pin, u32 setting, u32 resistor)
{
	u32 reg = pin / 10;
	u32 shift = (pin % 10) * 3;
	u32 status = GPFSEL_REG(reg); // read status
	status &= ~(7u << shift); // clear bits
	status |= (setting << shift); // set bits
	GPFSEL_REG(reg) = status;

	reg = pin / 16;
	shift = (pin % 16) * 2;
	status = GPIO_PUP_PDN_CNTRL_REG(reg); // read status
	status &= ~(3u << shift); // clear bits
	status |= (resistor << shift); // set bits
	GPIO_PUP_PDN_CNTRL_REG(reg) = status; // write back
}

// GPIO initialization, to be called before UART functions.
// For UART3 (line 2 on the RPi hat), we need to configure the GPIO to route
// the uart control and data signals to the GPIO pins 4-7 expected by the hat.
// GPIO pins 14 & 15 already configured by boot loader, but redo for clarity.
void gpio_init()
{
	setup_gpio(4, GPIO_ALTFN4, GPIO_NONE);
	setup_gpio(5, GPIO_ALTFN4, GPIO_NONE);
	setup_gpio(6, GPIO_ALTFN4, GPIO_NONE);
	setup_gpio(7, GPIO_ALTFN4, GPIO_NONE);
	setup_gpio(14, GPIO_ALTFN0, GPIO_NONE);
	setup_gpio(15, GPIO_ALTFN0, GPIO_NONE);

	// Setup debug pins
	for (int pin = 16; pin <= 23; pin++) {
		setup_gpio(pin, GPIO_OUTPUT, GPIO_NONE);
	}
}

void set_gpio_pin(u32 pin, u32 value)
{
	u32 reg = pin / 32;
	u32 shift = pin % 32;
	u32 *gpio_set = (u32 *)(GPIO_BASE + GPIO_GPSET0); // GPSET0/1
	u32 *gpio_clr = (u32 *)(GPIO_BASE + GPIO_GPCLR0); // GPCLR0/1

	if (value) {
		gpio_set[reg] = (1 << shift);
	} else {
		gpio_clr[reg] = (1 << shift);
	}
}

#define GPIO_INDICATOR_START_PIN 16
#define GPIO_INDICATOR_END_PIN 23

// Update the GPIO indicator pins to show the value (lowest 8 bits)
void update_gpio_indicator(u32 value)
{
	for (u32 pin = GPIO_INDICATOR_START_PIN; pin <= GPIO_INDICATOR_END_PIN; pin++) {
		set_gpio_pin(pin, (value & (1 << (GPIO_INDICATOR_END_PIN - pin))) ? 1 : 0);
	}
}

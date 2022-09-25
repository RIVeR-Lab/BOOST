#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>
#include "uartBase.h"
#include "gpsManager.h"

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN	DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0	""
#define PIN	0
#define FLAGS	0
#endif

// USART2
#define USART2_ID DT_NODELABEL(usart2)

#if DT_NODE_HAS_STATUS(USART2_ID, okay)
#define USART2_LABEL DT_LABEL(USART2_ID)
#else
#error "Unsupported board"
#endif

void main(void)
{
	printk("Main Function\n");

	const struct device *led0_dev;
	bool led_is_on = true;
	int ret;

	led0_dev = device_get_binding(LED0);
	if (led0_dev == NULL) {
		return;
	}

	ret = gpio_pin_configure(led0_dev, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return;
	}

	const struct device *uart2Dev;
	uart2Dev = device_get_binding(USART2_LABEL);
	uartBase uart2(*uart2Dev);
	gpsManager gpsManager(uart2);

	gpsManager.create();
	gpsManager.start();


	while (1) {
		gpio_pin_set(led0_dev, PIN, (int)led_is_on);
		led_is_on = !led_is_on;

		k_msleep(SLEEP_TIME_MS);
	}
}
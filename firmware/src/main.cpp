#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>
#include "uartBase.h"
#include "gpsManager.h"
#include "logging/log.h"
#include <debug/thread_analyzer.h>
#include <stdio.h>

LOG_MODULE_REGISTER(mainThread, LOG_LEVEL_DBG);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

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
	gpsManager.initialize();
	gpsManager.start();

	thread_analyzer_print();

	while (1) {
		gpio_pin_set(led0_dev, PIN, (int)led_is_on);
		led_is_on = !led_is_on;

		// char buffer [100];
  		// snprintf(buffer, 100, "Lat: %f\n", gpsManager.getLat());
		// printk("%s", buffer);

		

		// printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
		// printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
		// printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
		// printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
		// printInt(gps.location.age(), gps.location.isValid(), 5);
		// printDateTime(gps.date, gps.time);
		// printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
		// printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
		// printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
		// printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);

		k_msleep(SLEEP_TIME_MS);
	}
}
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
#include "testGpsConsumerManager.h"
#include "mainThread.h"

LOG_MODULE_REGISTER(main_cpp, LOG_LEVEL_DBG);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000





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

	theMainThread.create();
	theMainThread.initialize();
	theMainThread.start();

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
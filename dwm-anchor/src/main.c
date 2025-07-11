#include <zephyr.h>
#include <sys/printk.h>

#include "ble_device.h"

#define STACKSIZE 1024
#define PRIORITY_BLE 99
#define PRIORITY_MAIN 90
#define DELAY_TIME   K_MSEC(1000)

extern int dw_main(void);

#include "ble_device.h"

void bluetooth_thread(void * id, void * unused1, void * unused2)
{
    printk("START: %s\n", __func__);

    // k_sleep(K_MSEC(500));

	ble_device_init();

	while(1) {/* spin */}
}

K_THREAD_DEFINE(bluetooth_id, STACKSIZE, bluetooth_thread,
                NULL, NULL, NULL, PRIORITY_BLE, 0, 0);

void main_thread(void * id, void * unused1, void * unused2)
{
    printk("START: %s\n", __func__);
	dw_main();

	while(1) {/* spin */}
}

K_THREAD_DEFINE(main_id, STACKSIZE, main_thread,
                NULL, NULL, NULL, PRIORITY_MAIN, 0, 0);
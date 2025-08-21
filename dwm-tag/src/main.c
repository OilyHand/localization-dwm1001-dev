#include <zephyr.h>
#include <sys/printk.h>

#define STACKSIZE 1024
#define PRIORITY_SPI  1
#define PRIORITY_BLE  3
#define PRIORITY_MAIN 5
#define DELAY_TIME K_MSEC(1000)

extern int dw_main(void);


/*---------------------------------------------------------------------------*/
/*  Bluetooth Thread                                                         */
/*---------------------------------------------------------------------------*/
// #include "ble_device.h"

// void bluetooth_thread(void * id, void * unused1, void * unused2)
// {
// 	ble_device_init();
// 	while(1){/* spin */}
// }

// K_THREAD_DEFINE(bluetooth_id, STACKSIZE, bluetooth_thread,
//                 NULL, NULL, NULL, PRIORITY_BLE, 0, 0);

/*---------------------------------------------------------------------------*/
/*  SPI Master Thread                                                        */
/*---------------------------------------------------------------------------*/
#include "spi_master.h"

void spi_master_thread(void * id, void * unused1, void * unused2)
{
    spi_master_init();
	while(1){/* spin */}
}

K_THREAD_DEFINE(spi_master_id, STACKSIZE, spi_master_thread,
                NULL, NULL, NULL, PRIORITY_SPI, 0, 0);


/*---------------------------------------------------------------------------*/
/*  Main Thread                                                              */
/*---------------------------------------------------------------------------*/
// void main_thread(void * id, void * unused1, void * unused2)
// {
//     k_sleep( K_MSEC(1000));
// 	dw_main();
// 	while(1){/* spin */}
// }

// K_THREAD_DEFINE(main_id, STACKSIZE, main_thread,
//                 NULL, NULL, NULL, PRIORITY_MAIN, 0, 0);x``
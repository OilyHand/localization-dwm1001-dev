#include <zephyr.h>
#include <sys/printk.h>

#define STACKSIZE 1024
#define PRIORITY 99
#define DELAY_TIME   K_MSEC(1000)

extern int dw_main(void);

void main_thread(void * id, void * unused1, void * unused2)
{
    printk("%s\n", __func__);

    k_sleep( K_MSEC(1000));

	dw_main();

	while(1) { /* spin */}
}

K_THREAD_DEFINE(main_id, STACKSIZE, main_thread,
                NULL, NULL, NULL, PRIORITY, 0, 0);
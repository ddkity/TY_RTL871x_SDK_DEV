#include "FreeRTOS.h"
#include "task.h"
#include "diag.h"
#include "main.h"

#define TUYA_CONSOLE 0
extern void console_init(void);

static void app_init_thread(void *param)
{
    #include "semphr.h" 
    extern xSemaphoreHandle sync_sem; // define in wlan_network.c by nzy 
    
    // waiting to intial sync
    if(sync_sem) {
        xSemaphoreTake(sync_sem, portMAX_DELAY);
        vSemaphoreDelete(sync_sem);
        sync_sem = NULL;
    }

    extern void user_main(void); // user entry
    user_main();

    /* Kill init thread after all init tasks done */
    vTaskDelete(NULL);
}

static void app_init_entry(void)
{
    if(xTaskCreate(app_init_thread, ((const char*)"app_init"), 2048, NULL, tskIDLE_PRIORITY + 3 + PRIORITIE_OFFSET, NULL) != pdPASS)
        printf("\n\r%s xTaskCreate(init_thread) failed", __FUNCTION__);
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void main(void)
{
	if ( rtl_cryptoEngine_init() != 0 ) {
		DiagPrintf("crypto engine init failed\r\n");
	}

	/* Initialize log uart and at command service */
    #if (TUYA_CONSOLE) || (MP)
	//console_init();	
	ReRegisterPlatformLogUart();
    #endif

	/* pre-processor of application example */
	//pre_example_entry();

	/* wlan intialization */
#if defined(CONFIG_WIFI_NORMAL) && defined(CONFIG_NETWORK)
	wlan_network();
#endif

    #if !MP
	/* application init */
	app_init_entry();
    #endif

	/*Enable Schedule, Start Kernel*/
#if defined(CONFIG_KERNEL) && !TASK_SCHEDULER_DISABLED
	#ifdef PLATFORM_FREERTOS
	vTaskStartScheduler();
	#endif
#else
    #if (TUYA_CONSOLE) || (MP)
	RtlConsolTaskRom(NULL);
	#endif
#endif
}

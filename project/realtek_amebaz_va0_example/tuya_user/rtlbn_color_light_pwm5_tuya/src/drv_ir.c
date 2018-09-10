#include "tuya_device.h"
#include "adapter_platform.h"
#include "tuya_cloud_error_code.h"
#include "tuya_iot_wifi_api.h"
#include "tuya_cloud_types.h"
#include "tuya_led.h"
#include "tuya_uart.h"
#include "tuya_gpio.h"
#include "tuya_key.h"
#include "uni_time_queue.h"
#include "gw_intf.h"
#include "uni_log.h"
#include "uni_thread.h"
#include "sys_api.h"
#include "gpio_test.h"
#include "pwmout_api.h"


#include "timer_api.h"
#include "gpio_irq_api.h"

#define         TIMER_IRQ_PERIOD                    10           //US
#define         GPIO_IRQ_PIN                        PA_22


void ir_timer_irq_handler(uint32_t id)
{

}


VOID ir_gpio_irq_handler (uint32_t id, gpio_irq_event event)
{

    
}


STATIC gtimer_t timer_ir;
STATIC gpio_irq_t gpio_ir;

VOID ir_remote_hard_init(VOID)
{
    gpio_irq_init(&gpio_ir, GPIO_IRQ_PIN, ir_gpio_irq_handler, NULL);
    gpio_irq_set(&gpio_ir, IRQ_FALL, 1);   // Falling Edge Trigger
    gpio_irq_enable(&gpio_ir);

    //
    gtimer_init(&timer_ir, TIMER3);
    gtimer_start_periodical(&timer_ir, 10, (void*)ir_timer_irq_handler, NULL);
}

OPERATE_RET ir_remote_init(VOID)
{
    ir_remote_hard_init();
    
}

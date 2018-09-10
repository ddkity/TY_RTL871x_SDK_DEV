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

#include "light_driver.h"
#include "light_handler.h"
#include "light_config.h"
#include "light_prod_test.h"

VOID send_light_data(u8 R_value, u8 G_value, u8 B_value, u8 CW_value, u8 WW_value)
{
    //PR_INFO("R:%d G:%d B:%d c:%d w:%d", R_value, G_value, B_value, CW_value, WW_value);
    BYTE_T value[1] = {0x00};

    value[0] = CW_value;

    light_pwm_send_data(value, 1);
}

VOID light_hardware_init(VOID)
{
    light_drv_pwm_init();
}



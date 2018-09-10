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


#define PWM_PERIOD  1000
#define PWM_MIN     0
#define PWM_MAX     PWM_PERIOD
                    
//u32 duty[CHAN_NUM] = {100};
u32 pwm_val = 0;

u32 io_info[CHAN_NUM] ={
    PWM_0_OUT_IO_NUM,
    PWM_1_OUT_IO_NUM,
//    PWM_2_OUT_IO_NUM,
//    PWM_3_OUT_IO_NUM,
//    PWM_4_OUT_IO_NUM,
};

STATIC void pwm_set_duty(u32 duty, u8 channel)
{
    STATIC pwmout_t pwm_info;

    pwm_info.pwm_idx = pwmout_pin2chan(io_info[channel]);
    pwmout_pulsewidth_us(&pwm_info, duty);
}

STATIC VOID pwm_init(u32 period, u32 *duty, u32 pwm_channel_num, u32*pin_info_list)
{
    u8 i;
    STATIC pwmout_t pwm_info;

    for (i=0;i<pwm_channel_num;i++){
        pwmout_init(&pwm_info, pin_info_list[i]);
        pwmout_period_us(&pwm_info, period);
        pwmout_pulsewidth_us(&pwm_info, *duty);
    }
}

OPERATE_RET light_drv_pwm_init(VOID)
{
    //pwm init
    pwm_init(PWM_PERIOD, &pwm_val, CHAN_NUM, io_info);

    return OPRT_OK;
}

//VOID light_pwm_send_data(u8 R_value, u8 G_value, u8 B_value, u8 CW_value, u8 WW_value)
VOID light_pwm_send_data(unsigned char *ptr,uint16_t number)
{
    BYTE_T i;

    for(i = 0; i < number; i ++) {
    	pwm_val =  *ptr * PWM_MAX  / 255;
    	pwm_set_duty(pwm_val, i);

        ptr ++;
    }

}


#ifndef __IRDECODE_H__
#define __IRDECODE_H__

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
#include "timer_api.h"
#include "gpio_irq_api.h"


//hw_timer_define
STATIC gtimer_t ir_timer;
STATIC SEM_HANDLE get_switch_sem;
STATIC gpio_irq_t ir_irq;
STATIC gpio_t test_gpio_x;

STATIC gtimer_t test_timer;


#define irCntAllowance   10
#define PRINTIRCNT      0
#define IR_GPIO_NUM     PA_19
//#define TEST_GPIO_NUM   PA_19
#define TIMER_CNT_MAX   ((145 + irCntAllowance))


typedef enum {
	KEY_1 = 0x83,//开关
	KEY_2 = 0x82,//冷暖
	KEY_3 = 0X81,//
	KEY_4 = 0x80,//
	
	KEY_5 = 0x87,//变暖
	KEY_6 = 0x86,//
	KEY_7 = 0x85,//
	KEY_8 = 0x84,//
	
	KEY_9  = 0x8b,//
	KEY_10 = 0x8a,//
	KEY_11 = 0x89,//
	KEY_12 = 0x88,//
	
	KEY_13 = 0X8f,//
	KEY_14 = 0x8e,//
	KEY_15 = 0x8d,//
	KEY_16 = 0x8c,//
	
	KEY_17 = 0x93,//
	KEY_18 = 0x92,//
	KEY_19 = 0x91,//
	KEY_20 = 0x90,//

	KEY_21 = 0x97,//跳变
	KEY_22 = 0x96,//渐变
	KEY_23 = 0x95,//上层
	KEY_24 = 0x94//下层
}IRCMD;

typedef enum {
	KEY_a = 0xe0,//设定
	KEY_b = 0x20,//夜灯
	KEY_c = 0X60,//30
	KEY_d = 0x10,//60	
	KEY_e = 0x80,//开关
	KEY_f = 0x40,//明
	KEY_g = 0xc0,//暗
}IRCMD_new;


typedef enum {
	IRCODEERROR = -1,
	IRCODE0,
	IRCODE1,
	IRCODESTART,
	IRCODEREPEAT	
}IRCODE;

typedef struct{
	volatile BOOL timer_switch;
	volatile UINT timer_val;
	volatile UINT cur_timer_val;
}SFT_TIMER;

typedef struct{
	SFT_TIMER IrDecodeTimer;
	SEM_HANDLE ir_cmddeal_sem;
	THRD_HANDLE ir_thread;
}IRDEAL;

//VOID gra_change_timer_cb(void);
VOID UserInit(void);
VOID UserIrCmdDeal(IRCMD cmd,IRCODE irType);
//VOID UserIrCmdDeal_test(IRCMD cmd,IRCODE irType);


#endif

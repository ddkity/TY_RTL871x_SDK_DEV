/***********************************************************
*  File: tuya_device.h
*  Author: nzy
*  Date: 20180526
***********************************************************/
#ifndef _TUYA_DEVICE_H
    #define _TUYA_DEVICE_H

    //#include "sys_adapter.h"
    //#include "error_code.h"
    #include "tuya_cloud_types.h"
    
#ifdef __cplusplus
	extern "C" {
#endif

#ifdef  __DEVICE_GLOBALS
    #define __DEVICE_EXT
#else
    #define __DEVICE_EXT extern
#endif

/***********************************************************
*************************micro define***********************
***********************************************************/
// device information define
#define DEV_SW_VERSION USER_SW_VER
#define PRODUCT_KEY "wKIbgibLBL7vdEA3"

#define DEF_DEV_ABI DEV_SINGLE
/***********************************************************
*************************variable define********************
***********************************************************/
#define USE_NEW_PRODTEST

#define         WF_RESET_KEY            TY_GPIOA_15
#define         WF_DIR_LED              TY_GPIOA_5          //BLUE

#define         IR_GPIO_NUM             PA_19

#define         ir_send_led_stop()     tuya_set_led_light_type(wf_light,OL_LOW, 0, 0)
#define         ir_send_led_start()      tuya_set_led_light_type(wf_light,OL_HIGH, 0, 0)

/***********************************************************
*  Function: pre_device_init
*  Input: none
*  Output: none
*  Return: none
*  Note: to initialize device before device_init
***********************************************************/
__DEVICE_EXT \
VOID pre_device_init(VOID);


/***********************************************************
*************************function define********************
***********************************************************/
/***********************************************************
*  Function: device_init
*  Input: 
*  Output: 
*  Return: 
***********************************************************/
__DEVICE_EXT \
OPERATE_RET device_init(VOID);


#ifdef __cplusplus
}
#endif
#endif


/***********************************************************
*  File: tuya_device.c
*  Author: hjh
*  Date: 20180526
***********************************************************/
#define __DEVICE_GLOBALS
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

// 
#include "light_driver.h"
#include "light_handler.h"
#include "light_config.h"
#include "light_prod_test.h"

/***********************************************************
*************************micro define***********************
***********************************************************/
#define APP_DEBUG   0


/*********************trans function************************/



/***********************************************************
*  Function: gpio_test
*  Input: none
*  Output: none
*  Return: none
*  Note: For production testing
***********************************************************/
BOOL_T gpio_test(VOID)
{
    return gpio_test_cb(RTL_BOARD_WR3);
}

/***********************************************************
*  Function: pre_device_init
*  Input: none
*  Output: none
*  Return: none
*  Note: to initialize device before device_init
***********************************************************/
VOID pre_device_init(VOID)
{
    PR_DEBUG("%s",tuya_iot_get_sdk_info());
    PR_DEBUG("%s:%s",APP_BIN_NAME,DEV_SW_VERSION);
#if APP_DEBUG
    PR_DEBUG("DEBUG MODE");
#else
    SetLogManageAttr(LOG_LEVEL_INFO);
#endif
}
/***********************************************************
*  Function: get_wf_gw_status
*  Input: 
*  Output: 
*  Return: 
***********************************************************/
STATIC VOID get_wf_gw_status(IN CONST GW_WIFI_NW_STAT_E stat)
{
    light_wf_gw_status(stat);
    
    return;
}
/***********************************************************
*  Function: app_init
*  Input: 
*  Output: 
*  Return: 
***********************************************************/
VOID app_init(VOID)
{
    //prod init
    app_cfg_set(GWCM_SPCL_MODE, prod_test);
    
    app_light_init();
}
/***********************************************************
*  Function: device_differ_init
*  Input: 
*  Output: 
*  Return: 
***********************************************************/
STATIC OPERATE_RET device_differ_init(VOID)
{
	
    return OPRT_OK;
}
/***********************************************************
*  Function: device_cb
*  Input: 
*  Output: 
*  Return: 
***********************************************************/
VOID device_cb(IN CONST TY_RECV_OBJ_DP_S *dp)
{
    BOOL op_led = FALSE;
    
    if(NULL == dp) {
        PR_ERR("dp error");
        return;
    }

    UCHAR_T nxt = dp->dps_cnt;
    PR_DEBUG("dp_cnt:%d", nxt);

#if 0
    while (nxt){
        nxt--;
        sl_datapoint_proc(dp->dps[nxt]);
        op_led = TRUE;
    };
#else
    for (UCHAR_T i=0;i<nxt;i++){
        light_data_handler(dp->dps[i]);
        op_led = TRUE;
    };
#endif


    if(TRUE == op_led ) {
        light_data_save();
        op_led = FALSE;
    }

#if 0
    init_upload_proc();//all dp data send
#else
    dp_upload_proc(dp);
#endif
}

/***********************************************************
*  Function: device_init
*  Input: 
*  Output: 
*  Return: 
***********************************************************/
OPERATE_RET device_init(VOID)
{
    OPERATE_RET op_ret = OPRT_OK;
    // tuya_iot_wf_nw_cfg_ap_pri_set(TRUE);//change to ap mode

    TY_IOT_CBS_S wf_cbs = {
        NULL,\
        NULL,\
        NULL,\
        device_cb,\
        NULL,\
        NULL,\
        NULL,
    };

    op_ret = tuya_iot_wf_soc_dev_init(0,&wf_cbs,PRODUCT_KEY,DEV_SW_VERSION);
    if(OPRT_OK != op_ret) {
        PR_ERR("tuya_iot_wf_soc_dev_init err:%02x",op_ret);
        return OPRT_COM_ERROR;
    }

    op_ret = tuya_iot_reg_get_wf_nw_stat_cb(get_wf_gw_status);
    if(OPRT_OK != op_ret) {
        PR_ERR("tuya_iot_reg_get_wf_nw_stat_cb err:%02x",op_ret);
        return op_ret;
    }

    op_ret = device_differ_init();
    if(op_ret != OPRT_OK) {
        return op_ret;
    }

    return op_ret;
}


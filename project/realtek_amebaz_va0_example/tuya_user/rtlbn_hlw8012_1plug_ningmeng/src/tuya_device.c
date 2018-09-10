/***********************************************************
*  File: tuya_device.c
*  Author: wym
*  Date: 20180412
***********************************************************/
#define _TUYA_DEVICE_GLOBAL
#include "tuya_device.h"
#include "adapter_platform.h"
#include "tuya_iot_wifi_api.h"
#include "tuya_led.h"
#include "tuya_uart.h"
#include "tuya_gpio.h"
#include "tuya_key.h"
#include "hw_table.h"
#include "uni_time_queue.h"
#include "gw_intf.h"
#include "uni_log.h"
#include "uni_thread.h"
#include "app_dltj.h"

/***********************************************************
*************************micro define***********************
***********************************************************/

#define GWCM_MOD GWCM_LOW_POWER

typedef enum
{
	DP_SWITCH = 1,//开关
	DP_COUNT_DOWN = 9,//倒计时
}DP_ID;

/***********************************************************
*************************variable define********************
***********************************************************/
BOOL_T is_count_down = TRUE; // 倒计时开关
BOOL_T count_downing = FALSE; // 倒计时开关

TIMER_ID cd_timer;// 倒计时定时器

OPERATE_RET dp_upload_proc(CHAR_T *jstr);
VOID Start_boot_up(VOID);

// 倒计时回调
STATIC VOID cd_timer_cb(UINT_T timerID,PVOID pTimerArg);
STATIC VOID prod_test(BOOL_T flag, CHAR_T rssi);

/***********************************************************
*************************function define********************
***********************************************************/

/***********************************************************
*  Function: app_init
*  Input: none
*  Output: none
*  Return: none
*  Note: called by user_main
***********************************************************/
VOID app_init(VOID) {
    app_cfg_set(GWCM_MOD,prod_test);

    return ;
}

VOID status_changed_cb(IN CONST GW_STATUS_E status)
{
    PR_DEBUG("1111111111111111111111111gw status changed to %d", status);
    
}

OPERATE_RET get_file_data_cb(IN CONST FW_UG_S *fw, IN CONST UINT_T total_len, IN CONST UINT_T offset,
                                     IN CONST BYTE_T *data, IN CONST UINT_T len, OUT UINT_T *remain_len, IN PVOID pri_data)
{
    PR_DEBUG("2222222222222222222222222Rev File Data");
    PR_DEBUG("Total_len:%d ", total_len);
    PR_DEBUG("Offset:%d Len:%d", offset, len);

    return OPRT_OK;
}

VOID upgrade_notify_cb(IN CONST FW_UG_S *fw, IN CONST INT_T download_result, IN PVOID pri_data)
{
    PR_DEBUG("33333333333333333333333333333333333333333download  Finish");
    PR_DEBUG("download_result:%d", download_result);
    set_ele_thread_state(ELE_NORMAL);
}

VOID gw_ug_inform_cb(IN CONST FW_UG_S *fw)
{
    PR_DEBUG("44444444444444444444444444444444444444Rev GW Upgrade Info");
    PR_DEBUG("fw->fw_url:%s", fw->fw_url);
    PR_DEBUG("fw->fw_md5:%s", fw->fw_md5);
    PR_DEBUG("fw->sw_ver:%s", fw->sw_ver);
    PR_DEBUG("fw->file_size:%d", fw->file_size);
    set_ele_thread_state(ELE_SYS_OTA);
    tuya_iot_upgrade_gw(fw, get_file_data_cb, upgrade_notify_cb, NULL);
}

VOID dev_dp_query_cb(IN CONST TY_DP_QUERY_S *dp_qry)
{
    PR_DEBUG("555555555555555555555Recv DP Query Cmd");
    PR_DEBUG("cid :%s,cnt:%d,dpid:%s", dp_qry->cid, dp_qry->cnt, dp_qry->dpid);
}

VOID dev_obj_dp_cb(IN CONST TY_RECV_OBJ_DP_S *dp)
{
#if 1
    INT_T i = 0,ch_index;
    char buff[30];
    char time[32];
    TIME_T curtime;
    for(i = 0;i < dp->dps_cnt;i++) {
        PR_DEBUG("dpid:%d",dp->dps[i].dpid);
        PR_DEBUG("type:%d",dp->dps[i].type);
        PR_DEBUG("time_stamp:%d",dp->dps[i].time_stamp);
        //PR_NOTICE("********************");
        switch(dp->dps[i].type) {
            case PROP_BOOL: {
                PR_DEBUG("dps bool value is %d",dp->dps[i].value.dp_bool);
                ch_index = hw_set_channel_by_dpid(&g_hw_table, dp->dps[i].dpid, dp->dps[i].value.dp_bool);
                 PR_DEBUG("%d,%d",ch_index,g_hw_table.channel_num);
                if( ch_index >= 0)
                {
                    // 找到则上报现通道状态
                    if(g_hw_table.channels[ch_index].stat)
                    {
                        sprintf(buff, "{\"%d\":true}", g_hw_table.channels[ch_index].dpid);
                    }
                    else
                    {
                        sprintf(buff, "{\"%d\":false}", g_hw_table.channels[ch_index].dpid);
                    }                
                    // 推送通道数据
                    dp_upload_proc(buff);

                    // 倒计时
                    if((is_count_down)&&(g_hw_table.channels[ch_index].cd_sec >= 0 ))
                    //if(FALSE)
                    {
                        // 关闭倒计时
                        g_hw_table.channels[ch_index].cd_sec = -1;
                        sprintf(buff, "{\"%d\":0}", g_hw_table.channels[ch_index].cd_dpid);// 回复倒计时0
                        // 推送倒计时数据
                        dp_upload_proc(buff);
                    }
                }
                
            }
            break;
            
            case PROP_VALUE: { 
                PR_NOTICE("dps value is %d",dp->dps[i].value.dp_value);
                ch_index = hw_find_channel_by_cd_dpid(&g_hw_table, dp->dps[i].dpid);
                if(ch_index >= 0)
                {
                    // 找到 设定倒计时状态
                    if(dp->dps[i].value.dp_value == 0)
                    {
                        // 关闭倒计时
                        g_hw_table.channels[ch_index].cd_sec = -1;
                        sprintf(buff, "{\"%d\":0}", g_hw_table.channels[ch_index].cd_dpid);// 回复倒计时0
                        dp_upload_proc(buff);
                    }
                    else
                    {
                        g_hw_table.channels[ch_index].cd_sec = dp->dps[i].value.dp_value;// 倒计时开始
                        sprintf(buff, "{\"%d\":%d}",g_hw_table.channels[ch_index].cd_dpid,g_hw_table.channels[ch_index].cd_sec);// 回复倒计时
                        dp_upload_proc(buff);
                    }

                }
            }
            break;
            
            case PROP_ENUM: {
                PR_DEBUG("dps value is %d",dp->dps[i].value.dp_enum);
                
            }
            break;
            
            case PROP_BITMAP: {
                PR_DEBUG("dps value is %d",dp->dps[i].value.dp_bitmap);
            }
            break;

            case PROP_STR: {
                PR_DEBUG("dps value is %s",dp->dps[i].value.dp_str);
            }
            break;
        }
    }

    PR_DEBUG("dp->cid:%s dp->dps_cnt:%d",dp->cid,dp->dps_cnt);
#endif

}

VOID dev_raw_dp_cb(IN CONST TY_RECV_RAW_DP_S *dp)
{
    PR_DEBUG("raw data dpid:%d",dp->dpid);

    PR_DEBUG("recv len:%d",dp->len);
    #if 0
    INT_T i = 0;
    
    for(i = 0;i < dp->len;i++) {
        PR_DEBUG_RAW("%02X ",dp->data[i]);
    }
    #endif
    PR_DEBUG_RAW("\n");
    PR_DEBUG("end");
}

STATIC VOID __get_wf_status(IN CONST GW_WIFI_NW_STAT_E stat)
{
    hw_set_wifi_led_stat(&g_hw_table, stat);
    wf_nw_stat_inform(stat);
    return;
}

/***********************************************************
*  Function: gpio_test
*  Input: none
*  Output: none
*  Return: none
*  Note: For production testing
***********************************************************/
BOOL_T gpio_test(VOID)
{
    return TRUE;
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
    PR_DEBUG("%s:%s",APP_BIN_NAME,DEV_SW_VERSION);
//    SetLogManageAttr(LOG_LEVEL_INFO);
}

/***********************************************************
*  Function: device_init 
*  Input: none
*  Output: none
*  Return: none
***********************************************************/
OPERATE_RET device_init(VOID)
{
    OPERATE_RET op_ret = OPRT_OK;

    // tuya_iot_wf_nw_cfg_ap_pri_set(TRUE);
    
    TY_IOT_CBS_S wf_cbs = {
        status_changed_cb,\
        gw_ug_inform_cb,\
        NULL,
        dev_obj_dp_cb,\
        dev_raw_dp_cb,\
        dev_dp_query_cb,\
        NULL,
    };
    op_ret = tuya_iot_wf_soc_dev_init(GWCM_MOD,&wf_cbs,PRODUCT_KEY,DEV_SW_VERSION);
    if(OPRT_OK != op_ret) {
        PR_ERR("tuya_iot_wf_soc_dev_init err:%d",op_ret);
        return -1;
    }

    op_ret = tuya_iot_reg_get_wf_nw_stat_cb(__get_wf_status);
    if(OPRT_OK != op_ret) {
        PR_ERR("tuya_iot_reg_get_wf_nw_stat_cb err:%d",op_ret);
        return op_ret;
    }
    
   // PR_NOTICE("**************************************************************************************");

    init_hw(&g_hw_table);

    if(is_count_down){
        op_ret = sys_add_timer(cd_timer_cb, NULL, &cd_timer);      // 倒计时定时器
        if(OPRT_OK != op_ret) {
            return op_ret;
        }
        else{
            PR_DEBUG("cd_timer ID:%d",cd_timer);
            sys_start_timer(cd_timer, 1000, TIMER_CYCLE);
        }
    }
    op_ret= dltj_config_init(D_NORMAL_MODE);
    if(OPRT_OK != op_ret) {
        PR_ERR("dltj init err!");
        return op_ret;
    }
    INT_T size = SysGetHeapSize();//lql
	PR_NOTICE("device_init ok  free_mem_size:%d",size);
    return OPRT_OK;
}


STATIC VOID cd_timer_cb(UINT_T timerID,PVOID pTimerArg)
{
    int i;// 通道号
    char buff[20];
    // 遍历通道
    for(i=0; i<g_hw_table.channel_num; ++i)
    {
        if(g_hw_table.channels[i].cd_sec < 0)
        {
            continue;// 通道计时关闭
        }
        else
        {
            // 通道计时中
            --g_hw_table.channels[i].cd_sec;

            if(g_hw_table.channels[i].cd_sec <= 0)// 计时到达
            {
                // 置反通道状态
                hw_trig_channel(&g_hw_table, i);
                // 上报通道状态
                if(g_hw_table.channels[i].stat)
                {
                    sprintf(buff, "{\"%d\":true}", g_hw_table.channels[i].dpid);
                }
                else
                {
                    sprintf(buff, "{\"%d\":false}", g_hw_table.channels[i].dpid);
                }
                dp_upload_proc(buff);
                
                // 上报通道倒计时状态
                sprintf(buff, "{\"%d\":0}",  g_hw_table.channels[i].cd_dpid);
                dp_upload_proc(buff);
                g_hw_table.channels[i].cd_sec = -1; // 关闭通道定时
            }
            else
            {
                // 计时未到达
                // 每30s的整数倍上报一次
                if(g_hw_table.channels[i].cd_sec % 30 == 0)
                {
                    sprintf(buff, "{\"%d\":%d}", g_hw_table.channels[i].cd_dpid, g_hw_table.channels[i].cd_sec);
                    dp_upload_proc(buff);
                }
            }
        }

    }
}




OPERATE_RET dp_upload_proc(CHAR_T *jstr)
{
    OPERATE_RET op_ret;
    op_ret = sf_obj_dp_report_async(get_gw_cntl()->gw_if.id,jstr,false);
    if(OPRT_OK != op_ret) {
        PR_ERR("sf_obj_dp_report op_ret:%d",op_ret);
        return op_ret;
    }else {
        return OPRT_OK;
    }
}

VOID Start_boot_up(VOID)
{
#if 1
    UCHAR i=0;
    char buff[30];

    for(i = 0; i < g_hw_table.channel_num; i++)
    {
        if(g_hw_table.channels[i].stat){
            sprintf(buff, "{\"%d\":true}", g_hw_table.channels[i].dpid);
        }
        else{
            sprintf(buff, "{\"%d\":false}", g_hw_table.channels[i].dpid);
        }
        dp_upload_proc(buff);

        if(is_count_down){
            g_hw_table.channels[i].cd_sec = -1;
            sprintf(buff, "{\"%d\":0}", g_hw_table.channels[i].cd_dpid);// 回复倒计时0
            
            // 推送倒计时数据
            dp_upload_proc(buff);
        }
   }
#endif

}





STATIC VOID prod_test(BOOL_T flag, CHAR_T rssi)
{
    OPERATE_RET op_ret =OPRT_OK;
    PR_DEBUG("prod rssi:%d",rssi);
    op_ret = init_hw(&g_hw_table);
    if(OPRT_OK != op_ret) {
        PR_ERR("hw init err!");
        return;
    }
    tuya_set_led_light_type(g_hw_table.wifi_stat_led_handle,OL_FLASH_HIGH,1500,LED_TIMER_UNINIT);//1500ms flash
    op_ret= dltj_config_init(D_CAL_START_MODE);
    if(OPRT_OK != op_ret) {
        PR_ERR("dltj init err!");
        return;
    }
    hw_set_channel(&g_hw_table, 0, TRUE);
    if(OPRT_OK != ele_cnt_init(D_CAL_START_MODE)){
        PR_ERR("ele cnt test fail");
        hw_set_channel(&g_hw_table, 0, FALSE);
        tuya_set_led_light_type(g_hw_table.wifi_stat_led_handle,OL_HIGH,0,0);
        PR_DEBUG("FAIL");
    }else{
        hw_set_channel(&g_hw_table, 0, FALSE);
        tuya_set_led_light_type(g_hw_table.wifi_stat_led_handle,OL_LOW,0,0);
        PR_DEBUG("OK");
    }

}


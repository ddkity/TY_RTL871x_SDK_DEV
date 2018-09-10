/***********************************************************
*  File: tuya_device.c
*  Author: lql
*  Date: 20171128
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
#include "sys_api.h"
#include "gpio_test.h"
/***********************************************************
*************************micro define***********************
***********************************************************/

#define LED_CHANNEL_NUM      2     //LED通道个数
#define KEY_CHANNEL_NUM      2     //KEY通道个数
#define KEY_TIMER_MS         20    //key timer inteval

#define WF_RST_KEY   TY_GPIOA_0      //重置建
#define KEY_RST_TIME       3000      //按键重置时间:ms
#define BOOT_KEY_OK  1
#define WF_KEY_TEST1  TY_GPIOA_5     //检测开机按键

/***********************************************************
*************************variable define********************
***********************************************************/
STATIC GW_WF_CFG_MTHD_SEL gwcm_mode_user = GWCM_OLD;
STATIC BOOL_T is_count_down = TRUE; // 倒计时开关
STATIC TIMER_ID cd_timer;// 倒计时定时器
STATIC TIMER_ID rst_timer;//按键重置复位4s不操作
STATIC INT_T cd_upload_period = 30;// 倒计时状态上报周期 单位:秒

/***********************************************************
*************************function declaration***************
***********************************************************/
VOID Start_boot_up(VOID);
STATIC VOID cd_timer_cb(UINT_T timerID,PVOID pTimerArg);

/***********************************************************
*************************function define********************
***********************************************************/
STATIC VOID protest_switch_timer_cb(UINT timerID,PVOID pTimerArg)
{
    INT_T i;// 通道号
    for(i=0; i<g_hw_table.channel_num; ++i) {
        hw_trig_channel(&g_hw_table, i);
        g_hw_table.channels[i].prtest_swti1_count++;
        if(g_hw_table.channels[i].prtest_swti1_count<3){ 
         sys_start_timer(g_hw_table.switch_timer,500,TIMER_ONCE);
        }else{
            g_hw_table.channels[i].prtest_swti1_count=0;
        }
    }
}

VOID prod_test(BOOL_T flag, CHAR_T rssi)
{
    if(FALSE == flag) {
        PR_ERR("No Auth");
        return;
    }

    OPERATE_RET op_ret;
    PR_DEBUG("dev_test_start_cb");
    PR_DEBUG("rssi:%d", rssi);

    op_ret = sys_add_timer(protest_switch_timer_cb,NULL,&g_hw_table.switch_timer);
    if(OPRT_OK != op_ret) {
        PR_ERR("sys_add_timer switch_timer err");
        return ;
    }

    prod_test_init_hw(&g_hw_table);
    return;
}

/***********************************************************
*  Function: app_init
*  Input: none
*  Output: none
*  Return: none
*  Note: called by user_main
***********************************************************/
VOID app_init(VOID) 
{
    app_cfg_set(GWCM_LOW_POWER,prod_test);//GWCM_LOW_POWER
    gwcm_mode_user = GWCM_SPCL_MODE;
}

VOID status_changed_cb(IN CONST GW_STATUS_E status)
{
    PR_DEBUG("gw status changed to %d", status);
}

OPERATE_RET get_file_data_cb(IN CONST FW_UG_S *fw, IN CONST UINT_T total_len, IN CONST UINT_T offset,
                                     IN CONST BYTE_T *data, IN CONST UINT_T len, OUT UINT_T *remain_len, IN PVOID pri_data)
{
    PR_DEBUG("Rev File Data");
    PR_DEBUG("Total_len:%d ", total_len);
    PR_DEBUG("Offset:%d Len:%d", offset, len);

    return OPRT_OK;
}

VOID upgrade_notify_cb(IN CONST FW_UG_S *fw, IN CONST INT_T download_result, IN PVOID pri_data)
{
    PR_DEBUG("download  Finish");
    PR_DEBUG("download_result:%d", download_result);
}

VOID gw_ug_inform_cb(IN CONST FW_UG_S *fw)
{
    PR_DEBUG("Rev GW Upgrade Info");
    PR_DEBUG("fw->fw_url:%s", fw->fw_url);
    PR_DEBUG("fw->fw_md5:%s", fw->fw_md5);
    PR_DEBUG("fw->sw_ver:%s", fw->sw_ver);
    PR_DEBUG("fw->file_size:%d", fw->file_size);

    tuya_iot_upgrade_gw(fw, get_file_data_cb, upgrade_notify_cb, NULL);
}

VOID dev_dp_query_cb(IN CONST TY_DP_QUERY_S *dp_qry)
{
    PR_DEBUG("Recv DP Query Cmd");
}

OPERATE_RET upload_channel_stat(IN UINT_T ch_idx)
{
    OPERATE_RET op_ret;
    INT_T count_sec = 0;
    INT_T dp_idx = 0;
    INT_T dp_cnt = 0;

    dp_cnt = (is_count_down)?2:1;
    TY_OBJ_DP_S *dp_arr = (TY_OBJ_DP_S *)Malloc(dp_cnt*SIZEOF(TY_OBJ_DP_S));
    if(NULL == dp_arr) {
        PR_ERR("malloc failed");
        return OPRT_MALLOC_FAILED;
    }

    memset(dp_arr, 0, dp_cnt*SIZEOF(TY_OBJ_DP_S));

    dp_arr[dp_idx].dpid = g_hw_table.channels[ch_idx].dpid;
    dp_arr[dp_idx].type = PROP_BOOL;
    dp_arr[dp_idx].time_stamp = 0;
    dp_arr[dp_idx].value.dp_bool = g_hw_table.channels[ch_idx].stat;
    dp_idx ++;

    if(is_count_down) {
        dp_arr[dp_idx].dpid = g_hw_table.channels[ch_idx].cd_dpid;
        dp_arr[dp_idx].type = PROP_VALUE;
        dp_arr[dp_idx].time_stamp = 0;
        if(g_hw_table.channels[ch_idx].cd_sec >= 0) {
            dp_arr[dp_idx].value.dp_value = g_hw_table.channels[ch_idx].cd_sec;
        }else {
            dp_arr[dp_idx].value.dp_value = 0;
        }
        dp_idx ++;
    }

    if(dp_idx != dp_cnt) {
        PR_ERR("dp_idx:%d,dp_cnt:%d",dp_idx,dp_cnt);
        Free(dp_arr);
        dp_arr = NULL;
        return OPRT_COM_ERROR;
    }

    op_ret = dev_report_dp_json_async(get_gw_cntl()->gw_if.id,dp_arr,dp_cnt);
    Free(dp_arr);
    dp_arr = NULL;
    if(OPRT_OK != op_ret) {
        PR_ERR("upload_all_dp_stat op_ret:%d",op_ret);
    }

    return op_ret;
}

OPERATE_RET upload_all_dp_stat(VOID)
{
    OPERATE_RET op_ret;
    INT_T count_sec = 0;
    INT_T ch_idx = 0, dp_idx = 0;
    INT_T dp_cnt = 0;

    if(is_count_down) {
        dp_cnt = g_hw_table.channel_num*2;
    }else {
        dp_cnt = g_hw_table.channel_num;
    }
    
    TY_OBJ_DP_S *dp_arr = (TY_OBJ_DP_S *)Malloc(dp_cnt*SIZEOF(TY_OBJ_DP_S));
    if(NULL == dp_arr) {
        PR_ERR("malloc failed");
        return OPRT_MALLOC_FAILED;
    }

    memset(dp_arr, 0, dp_cnt*SIZEOF(TY_OBJ_DP_S));
    for(ch_idx = 0,dp_idx = 0; (ch_idx < g_hw_table.channel_num)&&(dp_idx < dp_cnt); ch_idx++,dp_idx++) {
        dp_arr[dp_idx].dpid = g_hw_table.channels[ch_idx].dpid;
        dp_arr[dp_idx].type = PROP_BOOL;
        dp_arr[dp_idx].time_stamp = 0;
        dp_arr[dp_idx].value.dp_bool = g_hw_table.channels[ch_idx].stat;

        if(is_count_down) {
            dp_idx++;
            dp_arr[dp_idx].dpid = g_hw_table.channels[ch_idx].cd_dpid;
            dp_arr[dp_idx].type = PROP_VALUE;
            dp_arr[dp_idx].time_stamp = 0;
            if(g_hw_table.channels[ch_idx].cd_sec >= 0) {
                dp_arr[dp_idx].value.dp_value = g_hw_table.channels[ch_idx].cd_sec;
            }else {
                dp_arr[dp_idx].value.dp_value = 0;
            }
        }
    }

    op_ret = dev_report_dp_json_async(get_gw_cntl()->gw_if.id,dp_arr,dp_cnt);
    Free(dp_arr);
    dp_arr = NULL;
    if(OPRT_OK != op_ret) {
        PR_ERR("upload_all_dp_stat op_ret:%d",op_ret);
    }

    return op_ret;
}

VOID dev_obj_dp_cb(IN CONST TY_RECV_OBJ_DP_S *dp)
{
    OPERATE_RET op_ret;
    UCHAR_T i = 0;
    UCHAR_T dp_cnt = 0;
    UINT_T ch_index = 0;
    
    for(i = 0;i < dp->dps_cnt;i++) {
        switch(dp->dps[i].dpid) {
        case DP_SWITCH: {
            ch_index = hw_set_channel_by_dpid(&g_hw_table, dp->dps[i].dpid, dp->dps[i].value.dp_bool);
            if( ch_index >= 0)
            {
                if((is_count_down)&&(g_hw_table.channels[ch_index].cd_sec >= 0)) {
                    g_hw_table.channels[ch_index].cd_sec = -1;
                }
                upload_channel_stat(ch_index);
            }
        }
        break;

        case DP_COUNT_DOWN: {
            ch_index = hw_find_channel_by_cd_dpid(&g_hw_table, dp->dps[i].dpid);
            if(ch_index >= 0) {
                // 找到 设定倒计时状态
                TY_OBJ_DP_S count_dp;
                count_dp.dpid = DP_COUNT_DOWN;
                count_dp.type = PROP_VALUE;
                count_dp.time_stamp = 0;

                if(dp->dps[i].value.dp_value == 0) {
                    // 关闭倒计时
                    g_hw_table.channels[ch_index].cd_sec = -1;
                    count_dp.value.dp_value = 0; 
                    PR_DEBUG("close count down");
                }else {
                    g_hw_table.channels[ch_index].cd_sec = dp->dps[i].value.dp_value;// 倒计时开始
                    count_dp.value.dp_value = dp->dps[i].value.dp_value;
                    PR_DEBUG("start count down %d sec",dp->dps[i].value.dp_value);
                }
                op_ret = dev_report_dp_json_async(dp->cid,&count_dp,1);
                if(OPRT_OK != op_ret) {
                    PR_ERR("dev_report_dp_json_async op_ret:%d",op_ret);
                    break;
                }
            }
        }
        break;

        default:
        break;
        }
    }
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
    return (gpio_test_cb(RTL_BOARD_WR3)||gpio_test_cb(RTL_BOARD_WR2)||gpio_test_cb(RTL_BOARD_WR1));
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
    SetLogManageAttr(LOG_LEVEL_INFO);
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
    op_ret = tuya_iot_wf_soc_dev_init(GWCM_LOW_POWER,&wf_cbs,PRODUCT_KEY,DEV_SW_VERSION);
    if(OPRT_OK != op_ret) {
        PR_ERR("tuya_iot_wf_soc_dev_init err:%d",op_ret);
        return -1;
    }

    op_ret = tuya_iot_reg_get_wf_nw_stat_cb(__get_wf_status);
    if(OPRT_OK != op_ret) {
        PR_ERR("tuya_iot_reg_get_wf_nw_stat_cb err:%d",op_ret);
        return op_ret;
    }
 
    init_hw(&g_hw_table);
	
    if(is_count_down) {   
        op_ret = sys_add_timer(cd_timer_cb, NULL, &cd_timer);      // 倒计时定时器
        if(OPRT_OK != op_ret) {
            return op_ret;
        }else {
            PR_NOTICE("cd_timer ID:%d",cd_timer);
            sys_start_timer(cd_timer, 1000, TIMER_CYCLE);
        }
    }

    INT_T size = SysGetHeapSize();//lql
    PR_NOTICE("device_init ok  free_mem_size:%d",size);
    return OPRT_OK;
}


STATIC VOID cd_timer_cb(UINT_T timerID,PVOID pTimerArg)
{
    UCHAR_T i = 0;// 通道号
    OPERATE_RET op_ret;

    // 遍历通道
    for(i = 0; i<g_hw_table.channel_num; i++) {
        if(g_hw_table.channels[i].cd_sec < 0) {
            continue;// 通道计时关闭
        }else {
            // 通道计时中
            --g_hw_table.channels[i].cd_sec;
            if(g_hw_table.channels[i].cd_sec <= 0) {// 计时到达
                g_hw_table.channels[i].cd_sec = -1; // 关闭通道定时
                // 置反通道状态
                hw_trig_channel(&g_hw_table, i);
                upload_channel_stat(i);
            }else {
                // 计时未到达
                // 每30s的整数倍上报一次
                if(g_hw_table.channels[i].cd_sec % 30 == 0) {
                    upload_channel_stat(i);
                }
            }
        }
    }
}

VOID Start_boot_up(VOID) {
    upload_all_dp_stat();
}

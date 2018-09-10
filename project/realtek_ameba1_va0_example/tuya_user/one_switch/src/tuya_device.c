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

/***********************************************************
*************************micro define***********************
***********************************************************/

#define LED_CHANNEL_NUM      2     //LED通道个数
#define KEY_CHANNEL_NUM      2     //KEY通道个数
#define KEY_TIMER_MS      	 20    //key timer inteval

#define WF_RST_KEY   TY_GPIOA_0      //重置建
#define KEY_RST_TIME       3000            //按键重置时间 ms

/***********************************************************
*************************variable define********************
***********************************************************/
BOOL is_count_down = TRUE; // 倒计时开关
BOOL count_downing = FALSE; // 倒计时开关

TIMER_ID cd_timer;// 倒计时定时器
int cd_upload_period = 30;// 倒计时状态上报周期 单位:秒

OPERATE_RET dp_upload_proc(CHAR *jstr);
OPERATE_RET dp_upload_proc_sync(CHAR *jstr,CHAR *time);
STATIC  VOID cdtimeExcute(PVOID pArg);
VOID Start_boot_up(VOID);

// 倒计时回调
STATIC VOID cd_timer_cb(UINT timerID,PVOID pTimerArg);
/***********************************************************
*************************function define********************
***********************************************************/
VOID status_changed_cb(IN CONST GW_STATUS_E status)
{
    PR_DEBUG("gw status changed to %d", status);
}
VOID app_init(VOID) {
    return ;
}

OPERATE_RET get_file_data_cb(IN CONST FW_UG_S *fw, IN CONST UINT total_len, IN CONST UINT offset,
                                     IN CONST BYTE *data, IN CONST UINT len, OUT UINT *remain_len, IN PVOID pri_data)
{
    PR_DEBUG("Rev File Data");
    PR_DEBUG("Total_len:%d ", total_len);
    PR_DEBUG("Offset:%d Len:%d", offset, len);

    return OPRT_OK;
}

VOID upgrade_notify_cb(IN CONST FW_UG_S *fw, IN CONST INT download_result, IN PVOID pri_data)
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

VOID dev_obj_dp_cb(IN CONST TY_RECV_OBJ_DP_S *dp)
{
#if 1
    INT i = 0,ch_index;
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
                        //dp_upload_proc(buff);
                        curtime = uni_time_get_posix();
                        sprintf(time,"{\"%d\":%u}",g_hw_table.channels[i].cd_dpid,curtime);
                        dp_upload_proc_sync(buff,time);
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
                        //dp_upload_proc(buff);
                        curtime = uni_time_get_posix();
                        sprintf(time,"{\"%d\":%u}",g_hw_table.channels[i].cd_dpid,curtime);
                        dp_upload_proc_sync(buff,time);
                    }
                    else
                    {
                        g_hw_table.channels[ch_index].cd_sec = dp->dps[i].value.dp_value;// 倒计时开始
                        sprintf(buff, "{\"%d\":%d}",g_hw_table.channels[ch_index].cd_dpid,g_hw_table.channels[ch_index].cd_sec);// 回复倒计时
                        //dp_upload_proc(buff);
                        curtime = uni_time_get_posix();
                        sprintf(time,"{\"%d\":%u}",g_hw_table.channels[i].cd_dpid,curtime);
                        //PR_NOTICE("$%$%%%%%%%%%%%%%%%%%%%%");
                        dp_upload_proc_sync(buff,time);
                        
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
#if 0
    OPERATE_RET op_ret = OPRT_OK;
    op_ret = dev_report_dp_json_async(dp->cid,dp->dps,dp->dps_cnt);
    if(OPRT_OK != op_ret) {
        PR_ERR("dev_report_dp_json_async error:%d",op_ret);
    }
    //dev_report_dp_json_async(IN CONST CHAR *dev_id,IN CONST TY_OBJ_DP_S *dp_data,IN CONST UINT cnt)
    //dev_report_dp_stat_sync(IN CONST CHAR *dev_id,IN CONST TY_OBJ_DP_S *dp_data,IN CONST UINT cnt,IN CONST UINT timeout)
#endif
}

VOID dev_raw_dp_cb(IN CONST TY_RECV_RAW_DP_S *dp)
{
    PR_DEBUG("dpid:%d",dp->dpid);

    PR_DEBUG("recv len:%d",dp->len);
    INT i = 0;
    for(i = 0;i < dp->len;i++) {
        PR_DEBUG_RAW("%02X ",dp->data[i]);
    }
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
BOOL gpio_test(VOID)
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
        dev_obj_dp_cb,\
        dev_raw_dp_cb,\
        dev_dp_query_cb,\
        NULL,
    }; 
    op_ret = tuya_iot_wf_soc_dev_init(TY_APP_CFG_WF,GWCM_OLD,&wf_cbs,PRODUCT_KEY,DEV_SW_VERSION);
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

   //用于倒计时功能的线程
    #if 1
    THRD_HANDLE cd_time_handle = NULL;
    THRD_PARAM_S thrd_param;
    thrd_param.priority = TRD_PRIO_4;
    thrd_param.stackDepth = 1024*2;
    thrd_param.thrdname = "cd_timer";
    op_ret = CreateAndStart(&cd_time_handle,NULL,NULL,\
                           cdtimeExcute,NULL,&thrd_param);
    if(OPRT_OK != op_ret) {
        return op_ret;
    }
    #endif   
    
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

    INT size = SysGetHeapSize();//lql
	PR_NOTICE("device_init ok  free_mem_size:%d",size);
    return OPRT_OK;
}


STATIC VOID cd_timer_cb(UINT timerID,PVOID pTimerArg)
{
    int i;// 通道号
    char buff[20];
    char time[32];
    TIME_T curtime;
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
                g_hw_table.channels[i].respond_cd_count = 0;
                g_hw_table.channels[i].cd_sec = -1;
                #if 0
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
                //dp_upload_proc(buff);
                curtime = uni_time_get_posix();
                sprintf(time,"{\"%d\":%u}",g_hw_table.channels[i].cd_dpid,curtime);
                dp_upload_proc_sync(buff,time);
                g_hw_table.channels[i].cd_sec = -1; // 关闭通道定时
                #endif
            }
            else
            {
                // 计时未到达
                // 每30s的整数倍上报一次
                if(g_hw_table.channels[i].cd_sec % 30 == 0)
                {
                    g_hw_table.channels[i].respond_cd_count = g_hw_table.channels[i].cd_sec;
                    #if 0
                    sprintf(buff, "{\"%d\":%d}", g_hw_table.channels[i].cd_dpid, g_hw_table.channels[i].cd_sec);
                    //dp_upload_proc(buff);
                    curtime = uni_time_get_posix();
                    sprintf(time,"{\"%d\":%u}",g_hw_table.channels[i].cd_dpid,curtime);
                    dp_upload_proc_sync(buff,time);
                    #endif
                }
            }
        }

    }
}




OPERATE_RET dp_upload_proc(CHAR *jstr)
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

OPERATE_RET dp_upload_proc_sync(CHAR *jstr,CHAR *time)
{
#if 0
    OPERATE_RET op_ret;
    //PR_NOTICE("dp_upload_proc_sync");
    op_ret = sf_stat_dp_report_sync(get_gw_cntl()->gw_if.id, jstr, time, 20000);
    PR_NOTICE("********************************************************************");
    if(OPRT_OK != op_ret) {
        PR_ERR("sf_obj_dp_report op_ret:%d",op_ret);
        return op_ret;
    }else {
        return OPRT_OK;
    }
#endif
#if 1
    OPERATE_RET op_ret;
    op_ret = sf_obj_dp_report_async(get_gw_cntl()->gw_if.id,jstr,false);
    if(OPRT_OK != op_ret) {
        PR_ERR("sf_obj_dp_report op_ret:%d",op_ret);
        return op_ret;
    }else {
        return OPRT_OK;
    }
#endif
}


STATIC VOID cdtimeExcute(PVOID pArg){
    int i;// 通道号
    char buff[20];
    char time[32];
    TIME_T curtime;
    while(1){
        // 遍历通道
        SystemSleep(1000);
        for(i=0; i<g_hw_table.channel_num; ++i)
        {
            if(g_hw_table.channels[i].respond_cd_count < 0)
            {
                continue;// 通道计时关闭
            }
            else
            {
                // 通道计时中
                if(g_hw_table.channels[i].respond_cd_count == 0)// 计时到达
                {
                    #if 1
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
                    //dp_upload_proc(buff);
                    curtime = uni_time_get_posix();
                    sprintf(time,"{\"%d\":%u}",g_hw_table.channels[i].cd_dpid,curtime);
                    dp_upload_proc_sync(buff,time);
                    //g_hw_table.channels[i].cd_sec = -1; // 关闭通道定时
                    #endif
                    g_hw_table.channels[i].respond_cd_count = -1;
                }
                else
                {
                    // 计时未到达
                    // 每30s的整数倍上报一次
                    #if 1
                    sprintf(buff, "{\"%d\":%d}", g_hw_table.channels[i].cd_dpid, g_hw_table.channels[i].respond_cd_count);
                    //dp_upload_proc(buff);
                    curtime = uni_time_get_posix();
                    sprintf(time,"{\"%d\":%u}",g_hw_table.channels[i].cd_dpid,curtime);
                    dp_upload_proc_sync(buff,time);
                    g_hw_table.channels[i].respond_cd_count = -1;
                    #endif
                }
            }

        }
    }
}


VOID Start_boot_up(VOID)
{

    UCHAR i=0;
    
    char buff[30];
    char time[32];
    TIME_T curtime;
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
            //dp_upload_proc(buff);
            curtime = uni_time_get_posix();
            sprintf(time,"{\"%d\":%u}",g_hw_table.channels[i].cd_dpid,curtime);
            dp_upload_proc_sync(buff,time);
        
        }
   }

}

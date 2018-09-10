/***********************************************************
*  File: tuya_device.c
*  Author: qiuping.wang
*  Date: 20180531
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
#include "kv_storge.h"//flash读写头文件

/***********************************************************
*************************micro define***********************
***********************************************************/
#define LED_CHANNEL_NUM      2     //LED通道个数
#define KEY_CHANNEL_NUM      2     //KEY通道个数
#define KEY_TIMER_MS      	 20    //key timer inteval

//存储状态
//断电记忆
#define DEF_MODE_VAR "def_mode"
#define  STORE_OFF_MOD "save_off_stat"
//上电状态存储
#define  STORE_CHANGE "init_stat_save"
#define FASE_SW_CNT_KEY "fsw_cnt_key"


#define NUM 1  //通道数
//app控制上电状态，默认下次重启生效
#define INIT_OFF  0  //上电关
#define INIT_ON   1  //上电开
#define INIT_MEM  2  //断电记忆
UCHAR_T init_stat = INIT_OFF;


enum _HAL_RESET_REASON{
	REASON_DEFAULT_RST = 0,         /**< normal startup by power on */
//	REASON_WDT_RST,             /**< hardware watch dog reset */
//	REASON_EXCEPTION_RST,       /**< exception reset, GPIO status won't change */
//	REASON_SOFT_WDT_RST,        /**< software watch dog reset, GPIO status won't change */
	REASON_SOFT_RESTART = 9,        /**< software restart ,system_restart , GPIO status won't //change */
//	REASON_DEEP_SLEEP_AWAKE,    /**< wake up from deep-sleep */
//	REASON_EXT_SYS_RST          /**< external system reset */
};

/***********************************************************
*************************variable define********************
***********************************************************/
BOOL_T is_count_down = TRUE; // 倒计时开关
BOOL_T count_downing = FALSE; // 倒计时开关
volatile BOOL is_save_stat =  true;     // 断电记忆开关
TIMER_ID save_stat_timer;  //断电记忆
TIMER_ID save_init_timer;  //上电状态
GW_WF_CFG_MTHD_SEL gwcm_mode_user = GWCM_OLD;

TIMER_ID cd_timer;// 倒计时定时器
int cd_upload_period = 30;// 倒计时状态上报周期 单位:秒
//wifi状态更新
GW_WIFI_NW_STAT_E wifi_stat ;
extern void cJSON_AddItemToObject(cJSON *object,const char *string,cJSON *item);

VOID Start_boot_up(VOID);

// 倒计时回调
STATIC VOID cd_timer_cb(UINT_T timerID,PVOID pTimerArg);
//断电记忆
void read_saved_stat(void);  // 读取存储的通道状态
STATIC VOID save_stat_timer_cb(UINT timerID,PVOID pTimerArg); // 状态储存回调 用于断电记忆
//上电状态记忆
STATIC OPERATE_RET read_def_mode(IN OUT UCHAR_T *mode);
OPERATE_RET write_def_mode(UCHAR_T def_stat);

/***********************************************************
*************************function define********************
***********************************************************/
STATIC VOID protest_switch_timer_cb(UINT timerID,PVOID pTimerArg)
{
    INT_T i;// 通道号
    for(i=0; i<g_hw_table.channel_num; ++i) {
        hw_trig_channel(&g_hw_table, i);
        g_hw_table.channels[i].prtest_swti1_count++;
        if(g_hw_table.channels[i].prtest_swti1_count<3) { 
         sys_start_timer(g_hw_table.switch_timer,500,TIMER_ONCE);
        }else {
            g_hw_table.channels[i].prtest_swti1_count=0;
        }
    }
}

VOID prod_test(BOOL_T flag, CHAR_T rssi)
{
    OPERATE_RET op_ret;
	PR_DEBUG("dev_test_start_cb");
	PR_DEBUG("rssi:%d", rssi);
	
    op_ret = sys_add_timer(protest_switch_timer_cb,NULL,&g_hw_table.switch_timer);
    if(OPRT_OK != op_ret) {
        PR_ERR("sys_add_timer switch_timer err");
        return ;
    }

    prod_test_init_hw(&g_hw_table);
    
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
    dev_reset_judge();
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
#if 1
    INT_T i = 0,ch_index;
    char buff[30];
	UINT_T buff_len;
    char time[32];
    TIME_T curtime;
    for(i = 0;i < dp->dps_cnt;i++) {
        PR_DEBUG("dpid:%d type:%d time_stamp:%d",dp->dps[i].dpid,dp->dps[i].type,dp->dps[i].time_stamp);
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
				if(is_save_stat) {
		        	sys_start_timer(save_stat_timer, 5000, TIMER_ONCE);    // 启动断电记忆存储定时器
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
	            
	        case DP_POWER_MODE: {
	            PR_DEBUG("dps value is %d",dp->dps[i].value.dp_enum);
				TY_OBJ_DP_S mode_dp;
                mode_dp.dpid = DP_POWER_MODE;
                mode_dp.type = PROP_ENUM;
                mode_dp.time_stamp = 0;
	            //找到该通道设备上电状态dpid：通电/断电/断电记忆
	            CONST UCHAR_T mode[] = {INIT_OFF,INIT_ON,INIT_MEM};
				UCHAR_T idx = dp->dps[i].value.dp_enum;
				
				if((idx >= 0)&&(idx <= 2)) {
					PR_DEBUG("idx = %d",idx);
					//app设定上电默认断电或者app未做设置默认上电断电状态
	                mode_dp.value.dp_enum = mode[idx];
	                init_stat = mode[idx];
	                PR_DEBUG("mode_dp.value.dp_value = %d",mode_dp.value.dp_value);
	                op_ret = dev_report_dp_json_async(get_gw_cntl()->gw_if.id,&mode_dp,1);//超时时间单位秒
				    if(OPRT_OK != op_ret) {
				        PR_ERR("dev_report_dp_json_async op_ret:%d",op_ret);
				    }
					write_def_mode(init_stat);
				}
	        }
	        break;
	            
	        default:
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
BOOL gpio_test(VOID)
{
	return gpio_test_cb(RTL_BOARD_WR3) || gpio_test_cb(RTL_BOARD_WR1) || gpio_test_cb(RTL_BOARD_WR2);
}


/***********************************************************
*  Function: get_reset_cnt
*  Input: none
*  Output: none
*  Return: none
*  Note: to get the cnt on_off for device
************************************************************/
STATIC INT_T get_reset_cnt(VOID)
{
	OPERATE_RET op_ret;
	INT_T cnt = 0;
	UCHAR *buf;
    UINT_T buf_len;
    
	buf = Malloc(32);
	if(NULL == buf) {
		PR_ERR("Malloc failed");
		goto ERR_EXT;
	}

	op_ret = kvs_read(FASE_SW_CNT_KEY, &buf, &buf_len);
	if(OPRT_OK != op_ret){
		PR_ERR("get_reset_cnt failed");
		Free(buf);
		goto ERR_EXT;
	}
	PR_DEBUG("buf:%s",buf);
	
	cJSON *root = NULL;
	root = cJSON_Parse(buf);
	if(NULL == root) {
		Free(buf);
		goto ERR_EXT;
	}
	Free(buf);

	cJSON *json;
	json = cJSON_GetObjectItem(root,"fsw_cnt_key");
	if(NULL == json) {
		cnt = 0;
	}else{
		cnt = json->valueint;
	}
	
	cJSON_Delete(root);
	return cnt;
		
ERR_EXT:	
	return 0;
}

/***********************************************************
*  Function: set_reset_cnt
*  Input: none
*  Output: none
*  Return: none
*  Note: to set the cnt on_off for device
************************************************************/
STATIC OPERATE_RET set_reset_cnt(INT_T val)
{
	OPERATE_RET op_ret;
	CHAR_T *out = NULL;

	cJSON *root_test = NULL;
	root_test = cJSON_CreateObject();
	if(NULL == root_test) {
		PR_ERR("json creat failed");
		goto ERR_EXT;
	}
	
	cJSON_AddNumberToObject(root_test, "fsw_cnt_key", val);
	
	out=cJSON_PrintUnformatted(root_test);
	cJSON_Delete(root_test);
	if(NULL == out) {
		PR_ERR("cJSON_PrintUnformatted err:");
		Free(out);
		goto ERR_EXT;
	}
	PR_DEBUG("out[%s]", out);
	op_ret = kvs_write(FASE_SW_CNT_KEY, out, strlen(out)); 
	if(OPRT_OK != op_ret) {
		PR_ERR("data write psm err: %02x",op_ret);
		Free(out);
		goto ERR_EXT;
	}
	Free(out);
	return OPRT_OK;
ERR_EXT:	
	return OPRT_COM_ERROR;

}

/***********************************************************
*  Function: reset_fsw_cnt_cb
*  Input: none
*  Output: none
*  Return: none
*  Note: to clear the cnt on_off for device
************************************************************/
STATIC VOID reset_fsw_cnt_cb(UINT timerID,PVOID pTimerArg)
{
    PR_DEBUG("%s",__FUNCTION__);

    set_reset_cnt(0);
}


/***********************************************************
*  Function: dev_reset_judge
*  Input: none
*  Output: none
*  Return: none
*  Note: to init the reset reason judge for device
************************************************************/
VOID dev_reset_judge(VOID)
{
	OPERATE_RET op_ret;
	UCHAR_T rst_cnt;
	UCHAR_T rst_num;

	//CHAR_T *rst_inf = system_get_rst_info();
	//PR_NOTICE("%s", rst_inf);
	//rst_num = atoi(rst_inf+23);
	//PR_DEBUG("rst_inf:%d", rst_num);

	//if(rst_num == REASON_DEFAULT_RST){
		rst_cnt = get_reset_cnt();
		PR_NOTICE("rst_cnt:%d", rst_cnt+1);
		set_reset_cnt(rst_cnt+1);
		TIMER_ID reset_timer;
		op_ret = sys_add_timer(reset_fsw_cnt_cb,NULL,&reset_timer);
		if(OPRT_OK != op_ret) {
			PR_ERR("reset_fsw_cnt timer add err:%02x",op_ret);
			return;
		}else {
			sys_start_timer(reset_timer,5000,TIMER_ONCE);
		}
	//}
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
   	//tuya_iot_wf_nw_cfg_ap_pri_set(TRUE);


    TY_IOT_CBS_S wf_cbs = {
        status_changed_cb,\
        gw_ug_inform_cb,\
        NULL,
        dev_obj_dp_cb,\
        dev_raw_dp_cb,\
        dev_dp_query_cb,\
        NULL,
    };
    op_ret = tuya_iot_wf_soc_dev_init(gwcm_mode_user,&wf_cbs,PRODUCT_KEY,DEV_SW_VERSION);//GWCM_LOW_POWER初始化
    if(OPRT_OK != op_ret) {
        PR_ERR("tuya_iot_wf_soc_dev_init err:%d",op_ret);
        return -1;
    }

	
	if(get_reset_cnt() >= 3){
		set_reset_cnt(0);
		gw_wifi_reset(WRT_AUTO);
	}
	
	op_ret = read_def_mode(&init_stat);
	PR_DEBUG("init_stat = %d",init_stat);
	if(OPRT_OK != op_ret) {
		init_stat = INIT_OFF;
	}
	
	init_hw(&g_hw_table);
    op_ret = tuya_iot_reg_get_wf_nw_stat_cb(__get_wf_status);//实时获取wifi状态
    if(OPRT_OK != op_ret) {
        PR_ERR("tuya_iot_reg_get_wf_nw_stat_cb err:%d",op_ret);
        return op_ret;
    }

	
	//上电读断电记忆
	if(is_save_stat) {  //is_save_stat
		if(init_stat == INIT_MEM) {
			PR_DEBUG("is_save_stat = %d",is_save_stat);
     		read_saved_stat();                 //读取存储的状态 
		}	
        op_ret = sys_add_timer(save_stat_timer_cb, NULL, &save_stat_timer);// 状态储存定时器 用于断电记忆
        if(OPRT_OK != op_ret) {
            return op_ret;
        }
    }

    if(is_count_down) { //is_count_down 
        op_ret = sys_add_timer(cd_timer_cb, NULL, &cd_timer);      // 倒计时定时器
        if(OPRT_OK != op_ret) {
            return op_ret;
        }
        else{
            PR_NOTICE("cd_timer ID:%d",cd_timer);
            sys_start_timer(cd_timer, 1000, TIMER_CYCLE);
        }
    }

	INT_T size = SysGetHeapSize();//lql
	PR_DEBUG("device_init ok  free_mem_size:%d",size);
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

//断电记忆
STATIC VOID save_stat_timer_cb(UINT timerID,PVOID pTimerArg)
{
    OPERATE_RET op_ret;
    int i;

    // 存储当前状态
    PR_DEBUG("save stat");
    IN CONST BYTE_T buff[48] = "{\"power\":[";
    for(i=0; i<g_hw_table.channel_num; ++i) {
        if((g_hw_table.channels[i].stat)&&(init_stat == INIT_MEM)) {
            strcat(buff, "true");
        }
        else if((!g_hw_table.channels[i].stat)||(init_stat == INIT_MEM)) {
            strcat(buff, "false");
        }
		
        if(i < g_hw_table.channel_num -1) {
            strcat(buff, ",");
        }else {
            strcat(buff, "]}");
        }
    }
    PR_DEBUG("%s", buff);

     op_ret = wd_common_write(STORE_OFF_MOD,buff,strlen(buff));
    if(OPRT_OK != op_ret) {
        PR_DEBUG("kvs_write err:%02x",op_ret);
    }
}

void read_saved_stat(void)
{

	PR_DEBUG("_______________SAVE________________");
    OPERATE_RET op_ret;
    cJSON *root = NULL, *js_power = NULL, *js_ch_stat = NULL;
    UINT_T buff_len = 0;
    UCHAR_T *buff = NULL;

    int i;

    // 读取存储的状态json {"power":[false,true...]}
    op_ret = wd_common_read(STORE_OFF_MOD,&buff,&buff_len);
    if(OPRT_OK != op_ret) {
        PR_DEBUG("msf_get_single err:%02x",op_ret);
        return ;
    }
    // 分析json
    PR_DEBUG("read stat: %s", buff);
    root = cJSON_Parse(buff);
	Free(buff);
    // 如果json分析出错 退出函数
    if(NULL == root) {
        PR_ERR("cjson parse err");
        return;
    }

    js_power = cJSON_GetObjectItem(root, "power");
    // 如果找不到power对象 退出函数
    if(NULL == js_power) {
        PR_ERR("cjson get power error");
        goto JSON_PARSE_ERR;
    }

	UCHAR_T count = cJSON_GetArraySize(js_power);
	if(count != g_hw_table.channel_num) {
    	for(i = 0;i< g_hw_table.channel_num; ++i) {
			hw_set_channel(&g_hw_table, i, FALSE);
    	}
		return;
    }
	
	for(i=0; i< g_hw_table.channel_num; ++i) {
        js_ch_stat = cJSON_GetArrayItem(js_power, i);
        if(js_ch_stat == NULL) {
            PR_ERR("cjson %d ch stat not found", i);
            goto JSON_PARSE_ERR;
        }else {
            if(js_ch_stat->type == cJSON_True) {
                hw_set_channel(&g_hw_table, i, TRUE);
            }else {
                hw_set_channel(&g_hw_table, i, FALSE);
            }  
        }
    }

JSON_PARSE_ERR:
    cJSON_Delete(root);

}
UCHAR_T get_def_mode()
{
	return init_stat;
}

OPERATE_RET write_def_mode(UCHAR_T def_stat)
{
	OPERATE_RET op_ret;
	cJSON *root = NULL;
	UCHAR_T  *out = NULL;

	root = cJSON_CreateObject();
	if(NULL == root) {
		PR_ERR("cJSON ERROR");
		return OPRT_CR_CJSON_ERR;
	}
	
	cJSON_AddNumberToObject(root,DEF_MODE_VAR,def_stat);
	out = cJSON_PrintUnformatted(root);
	cJSON_Delete(root);
	if(NULL == out) {
		PR_ERR("out = NULL");			
		return OPRT_MALLOC_FAILED;
	}
	
	PR_DEBUG("OUT = %s ",out);
	op_ret = wd_common_write(STORE_CHANGE,out,strlen(out));
    Free(out);
	if(OPRT_OK != op_ret) {			
		PR_ERR("wd_common_write err:%d",op_ret);
		return op_ret;
	}
	return OPRT_OK;
}

STATIC OPERATE_RET read_def_mode(IN OUT UCHAR_T *mode)
{
	OPERATE_RET op_ret;
	cJSON *root = NULL;
    UCHAR_T *buff = NULL;
	UINT_T buff_len = 0;
	
	// 读取存储的状态json {"def_mode":{0}}
	op_ret = wd_common_read(STORE_CHANGE,&buff,&buff_len);	
	if(OPRT_OK != op_ret) {
		PR_ERR("wd_common_read err:%d",op_ret);
		return op_ret;
	}
	// 分析json
	PR_DEBUG("wd_common_read: %s", buff);
	root = cJSON_Parse(buff);//将json数据解析成json结构体
	Free(buff);
	// 如果json分析出错 退出函数
	if(NULL == root) {
		PR_ERR("cjson parse err");
		return OPRT_CJSON_PARSE_ERR;
	}

	cJSON *js_json = cJSON_GetObjectItem(root,DEF_MODE_VAR);
	if(NULL == js_json) {
		cJSON_Delete(root);
		return OPRT_COM_ERROR;
	}
	cJSON_Delete(root);
	*mode = js_json->valueint;
	return OPRT_OK;
}

VOID Start_boot_up(VOID)
{
    upload_all_dp_stat();
}

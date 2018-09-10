/***********************************************************
File: hw_table.h 
Author: 徐靖航 JingHang Xu
Date: 2017-06-01
Description:
    开关插座等控制继电器开关为主的设备模板
    无倒计时 版本            
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
//存储状态
//断电记忆
#define POWER_MEMORY    "power"

//上电状态存储
#define  STORE_CHANGE "init_stat_save"

#define WM_SUCCESS 0
#define WM_FAIL 1

#define CHANNEL_NUM  (g_hw_table.channel_num)

#define GWCM_MODE(wf_mode)  (((wf_mode) == LOW_POWER) ? (GWCM_LOW_POWER) : (GWCM_OLD_PROD) )
/***********************************************************
*************************typedef define***********************
***********************************************************/
//产测
typedef struct{
UCHAR_T      num;
UCHAR_T      relay_act_cnt;
TIMER_ID     relay_tm;
}PT_RELAY_ACT_INFOR;

/***********************************************************
*************************variable define********************
***********************************************************/
TIMER_ID wfl_timer;

BOOL is_save_stat = TRUE; // 断电记忆开
TIMER_ID save_stat_timer;
TIMER_ID cd_timer;// 倒计时定时器
int cd_upload_period = 30;// 倒计时状态上报周期 单位:秒

// 继电器动作定时器回调
TIMER_ID all_relay_tm;         // 所有继电器动作定时器句柄
PT_RELAY_ACT_INFOR  *pt_channel_tbl = NULL;

/***********************************************************
*************************function define********************
***********************************************************/
STATIC VOID key_process(TY_GPIO_PORT_E port,PUSH_KEY_TYPE_E type,INT_T cnt);
STATIC VOID wfl_timer_cb(UINT timerID,PVOID pTimerArg);
STATIC OPERATE_RET dp_upload_proc(CHAR_T *jstr);
// 状态同步回调   
void update_all_stat(void);

// 状态储存回调 用于断电记忆
STATIC VOID save_stat_timer_cb(UINT timerID,PVOID pTimerArg);
// 倒计时回调
STATIC VOID cd_timer_cb(UINT timerID,PVOID pTimerArg);
// 读取存储的通道状态
OPERATE_RET read_saved_channel_stat(OUT CHAR_T *st_arr, IN INT_T ch_num);
STATIC VOID prod_test(BOOL flag, CHAR_T rssi);

#define HW_JSON "{\"sw\":{\"ch\":[{\"bt\":{\"pin\":{\"value\":0},\"lv\":{\"value\":false}},\"df\":{\"value\":false},\"dpid\":{\"value\":1},\"cddpid\":{\"value\":9},\"rl\":{\"pin\":{\"value\":5},\"lv\":{\"value\":true}},\"led\":{\"pin\":{\"value\":12},\"lv\":{\"value\":false}}},{\"bt\":{\"pin\":{\"value\":14},\"lv\":{\"value\":false}},\"df\":{\"value\":false},\"dpid\":{\"value\":2},\"cddpid\":{\"value\":10},\"rl\":{\"pin\":{\"value\":15},\"lv\":{\"value\":true}},\"led\":{\"pin\":{\"value\":18},\"lv\":{\"value\":false}}}],\"tch\":{\"tbt\":{\"pin\":{\"value\":22},\"lv\":{\"value\":false}}},\"net\":{\"netn\":{\"value\":null},\"owm\":{\"value\":true},\"rsthold\":{\"value\":3},\"wfst\":{\"pin\":{\"value\":23},\"lv\":{\"value\":false}},\"nety\":{\"value\":null}}}}"
// 产测相关代码 ======================================================================
// 继电器动作定时器回调


/***********************************************************
*  Function: app_init
*  Input: none
*  Output: none
*  Return: none
*  Note: called by user_main
***********************************************************/
VOID app_init(VOID) 
{
    CHAR_T *pConfig = NULL;   
	UINT len = 0;
	cJSON *js_hw_table = NULL;
	OPERATE_RET op_ret;// 注册操作的返回值

//    tuya_iot_oem_set(TRUE);
    
    js_hw_table = cJSON_Parse(HW_JSON);

	if(NULL == js_hw_table)
    {
    	PR_ERR("ERROR!");
		return ;
    }
    
    // 用json初始化硬件表
    int ret = hw_set_by_json(&g_hw_table, js_hw_table);
    cJSON_Delete(js_hw_table);
    if(ret != 0) {
        PR_ERR("ret:%d",ret);
        return;
    }

    app_cfg_set(GWCM_MODE(g_hw_table.wf_mode), prod_test);

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
    //SetLogManageAttr(LOG_LEVEL_INFO);
}


// 产测相关代码 ======================================================================
/***********************************************************
*  Function: 单个继电器动作定时器回调
*  Input: 
*  Output: 
*  Return: 
***********************************************************/
STATIC VOID single_relay_tm_cb(UINT timerID,PVOID pTimerArg)
{
    UCHAR_T num;

    PR_DEBUG("relay timer callback");
    // 置反目标通道

    num = *((UCHAR_T*)pTimerArg);
    
    PR_DEBUG("% ch action", num);
    if(num >= CHANNEL_NUM)
    {
        PR_ERR("TimerArg Err !");
        return;
    }
    if(NULL == pt_channel_tbl)
    {
        PR_ERR("pt_channel_tbl pointer null!");
        return;
    }

    hw_trig_channel(&g_hw_table, num);
    pt_channel_tbl[num].relay_act_cnt++;

    // 如果执行少于三次则再次启动定时器
    if(pt_channel_tbl[num].relay_act_cnt < 3){
        sys_start_timer(pt_channel_tbl[num].relay_tm, 500, TIMER_ONCE);
    }else{
        pt_channel_tbl[num].relay_act_cnt = 0;
    }
}

/***********************************************************
*  Function: 所有电器依次动作三次
*  Input: 
*  Output: 
*  Return: 
***********************************************************/
STATIC VOID all_relay_tm_cb(UINT timerID,PVOID pTimerArg)
{
    static UCHAR_T act_cnt = 0;
    static UCHAR_T target_ch = 0;

    PR_DEBUG("all relay timer callback");
    
	if( target_ch < CHANNEL_NUM)
	{
		hw_trig_channel(&g_hw_table, target_ch);
	    act_cnt++;
  	}
    // 如果执行少于三次则再次启动定时器
    //所有通道都执行完毕
    if( target_ch >= CHANNEL_NUM ){
        act_cnt = 0;
        target_ch = 0;
    }
    //单通道三次未执行完毕
    else if(act_cnt < 3){
        sys_start_timer(all_relay_tm, 500, TIMER_ONCE);
    }
    //单通道三次执行完毕，切换通道
    else{
        act_cnt = 0;
        target_ch++;
        sys_start_timer(all_relay_tm, 500, TIMER_ONCE);	
    }
}
/***********************************************************
*  Function: 产测时的按键回调
*  Input: 
*  Output: 
*  Return: 
***********************************************************/
STATIC VOID pt_key_process(TY_GPIO_PORT_E port,PUSH_KEY_TYPE_E type,INT_T cnt)
{
    PR_DEBUG("gpio_no: %d",port);
    PR_DEBUG("type: %d",type);
    PR_DEBUG("cnt: %d",cnt);

    INT_T i;//通道下标 通道号为通道下标

    /*******总控******/
     if((g_hw_table.power_button.type !=  IO_DRIVE_LEVEL_NOT_EXIST)&&\
        (g_hw_table.power_button.key_cfg.port == port))
     {
        if(NORMAL_KEY == type){
			if(IsThisSysTimerRun(all_relay_tm)==false){
				sys_start_timer(all_relay_tm, 500, TIMER_ONCE);
		    }
        }
     }

    /*******分控*******/
    for(i=0; i<CHANNEL_NUM; i++)
    {
        if((g_hw_table.channels[i].button.type  !=  IO_DRIVE_LEVEL_NOT_EXIST)&&\
           (g_hw_table.channels[i].button.key_cfg.port == port))
        {
            if(NORMAL_KEY == type){
                if(IsThisSysTimerRun(pt_channel_tbl[i].relay_tm)==false){
                    sys_start_timer(pt_channel_tbl[i].relay_tm, 500, TIMER_ONCE);
                }
            }
        }
    }
     
}

/***********************************************************
*  Function: 产测回调
*  Input: 
*  Output: 
*  Return: 
***********************************************************/
STATIC VOID prod_test(BOOL flag, CHAR_T rssi)
{
    INT_T i;
    BOOL is_single_ctrl = FALSE;

    // TODO: 信号值测试
    if(flag == FALSE)
    {
        PR_ERR("no auth");
        return;
    }
    
    PR_NOTICE("product test mode");
    OPERATE_RET op_ret;// 注册操作的返回值
    // 注册产测需要使用的硬件 ----------------------------

    // 硬件配置初始化 配置完后所有通道默认无效
    op_ret = hw_init(&g_hw_table, pt_key_process);
    if(OPRT_OK != op_ret) {
        PR_ERR("op_ret:%d",op_ret);
        return;
    }
    //设置通道初始状态
    for(i=0; i<CHANNEL_NUM; i++){
        hw_set_channel(&g_hw_table, i, FALSE);
    }
    
    // 主控继电器动作定时器
    if(g_hw_table.power_button.type !=  IO_DRIVE_LEVEL_NOT_EXIST)
    {
        op_ret = sys_add_timer(all_relay_tm_cb, NULL, &all_relay_tm);
        if(OPRT_OK != op_ret) {
            PR_ERR("add relay_tm err");
            return ;
        }
    }
    //判断是否存在分控
    for(i=0; i<CHANNEL_NUM; i++){
        if(g_hw_table.channels[i].button.type !=  IO_DRIVE_LEVEL_NOT_EXIST){
            is_single_ctrl = TRUE;
        }
    }
    //分控继电器动作定时器
    if(TRUE == is_single_ctrl)
    {
        pt_channel_tbl = (PT_RELAY_ACT_INFOR*)Malloc(CHANNEL_NUM * sizeof(PT_RELAY_ACT_INFOR));
        if(NULL == pt_channel_tbl){
            PR_ERR("Malloc Failed!");
            return;
        }
        for(i=0; i<CHANNEL_NUM; i++)
        {
            pt_channel_tbl[i].num = i; 
            if(g_hw_table.channels[i].button.type !=  IO_DRIVE_LEVEL_NOT_EXIST){
                op_ret = sys_add_timer(single_relay_tm_cb, &pt_channel_tbl[i].num, &pt_channel_tbl[i].relay_tm);
                if(OPRT_OK != op_ret) {
                    PR_ERR("add relay_tm err");
                    return ;
                }
            }
        }
    }
    
    // 注册结束 -----------------------------------------

    // 产测流程 - 步骤1 wifi指示灯控制为配网状态 快闪wifi状态指示灯
    hw_set_wifi_led_stat(&g_hw_table, STAT_UNPROVISION);
}

/***********************************************************
*  Function: wifi状态改变回调
*  Input: 
*  Output: 
*  Return: 
***********************************************************/
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

/***********************************************************
*  Function: object类型dp数据解析，发送回复帧
*  Input:   dp 待解析的数据指针
*  Output: 
*  Return: 
***********************************************************/
VOID dev_obj_dp_cb(IN CONST TY_RECV_OBJ_DP_S *dp)
{
    INT_T i = 0,ch_index;
    cJSON *js_back = NULL;
    char name[10] = {0};

    js_back = cJSON_CreateObject();
    if(NULL == js_back)
    {
        PR_DEBUG("Json CreateObject Fail");
        return;
    }
    
    for(i = 0;i < dp->dps_cnt;i++) {
        PR_DEBUG("dpid:%d",dp->dps[i].dpid);
        PR_DEBUG("type:%d",dp->dps[i].type);
        PR_DEBUG("time_stamp:%d",dp->dps[i].time_stamp);
        switch(dp->dps[i].type) {
            case PROP_BOOL: 
                PR_DEBUG("dps bool value is %d",dp->dps[i].value.dp_bool);
                ch_index = hw_set_channel_by_dpid(&g_hw_table, dp->dps[i].dpid, dp->dps[i].value.dp_bool);
                PR_DEBUG("%d,%d",ch_index,CHANNEL_NUM);
                if( ch_index >= 0)
                {
                    //组回复帧
                    sprintf(name, "%d", g_hw_table.channels[ch_index].dpid);
                    cJSON_AddBoolToObject(js_back, name,g_hw_table.channels[ch_index].stat);
                    PR_DEBUG("g_hw_table.channels[%d].cd_dpid: %d", ch_index, g_hw_table.channels[ch_index].cd_dpid);
                    if(g_hw_table.channels[ch_index].cd_dpid != DPID_NOT_EXIST){
                        sprintf(name, "%d", g_hw_table.channels[ch_index].cd_dpid);
                        cJSON_AddNumberToObject(js_back, name, REV_COUNT_TIME(g_hw_table.channels[ch_index].cd_sec));
                    }
                    
                    if(is_save_stat)
                        sys_start_timer(save_stat_timer, 5000, TIMER_ONCE);
                }
            break;
            case PROP_VALUE: 
                PR_NOTICE("dps value is %d",dp->dps[i].value.dp_value);
                ch_index = hw_find_channel_by_cd_dpid(&g_hw_table, dp->dps[i].dpid);
                if(ch_index >= 0)
                {
                    // 找到 设定倒计时状态
                    g_hw_table.channels[ch_index].cd_sec = GET_COUNT_TIME(dp->dps[i].value.dp_value);
                    //组回复帧
                    sprintf(name, "%d", g_hw_table.channels[ch_index].cd_dpid);
                    cJSON_AddNumberToObject(js_back, name, dp->dps[i].value.dp_value);
                }
            break;
            
            case PROP_ENUM: 
                PR_DEBUG("dps value is %d",dp->dps[i].value.dp_enum);
            break;
            
            case PROP_BITMAP:
                PR_DEBUG("dps value is %d",dp->dps[i].value.dp_bitmap);
            break;

            case PROP_STR: 
                PR_DEBUG("dps value is %s",dp->dps[i].value.dp_str);
            break;
        }
    }
    //发送回复帧
    char *pbuff = cJSON_PrintUnformatted(js_back);
    cJSON_Delete(js_back);
    dp_upload_proc(pbuff);
    Free(pbuff);
    
    PR_DEBUG("dp->cid:%s dp->dps_cnt:%d",dp->cid,dp->dps_cnt);
}
/***********************************************************
*  Function: raw类型dp数据解析
*  Input:   dp 待解析的数据指针
*  Output: 
*  Return: 
***********************************************************/
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

/***********************************************************
*  Function: 设备查询命令回调
*  Input:   dp_qry 接收到的查询命令
*  Output: 
*  Return: 
***********************************************************/
VOID dev_dp_query_cb(IN CONST TY_DP_QUERY_S *dp_qry)
{
    PR_DEBUG("Recv DP Query Cmd");
}
/***********************************************************
*  Function: 获取wifi状态
*  Input: 
*  Output: 
*  Return: 
***********************************************************/
STATIC VOID __get_wf_status(IN CONST GW_WIFI_NW_STAT_E stat)
{
    hw_set_wifi_led_stat(&g_hw_table, stat);
    return;
}

/***********************************************************
*  Function: 设备状态初始化
*  Input: 
*  Output:
*  Return:
***********************************************************/
OPERATE_RET dev_default_stat_init(VOID)
{
    INT_T i;
    OPERATE_RET ret;
    CHAR_T *pch_state;

    pch_state = Malloc(CHANNEL_NUM);
    if(NULL == pch_state){
        PR_ERR("malloc fail");
        return OPRT_MALLOC_FAILED;
    }
    
    memset(pch_state, 0x00, CHANNEL_NUM);
    
    //查询是否要开启断电记忆
    is_save_stat = FALSE;
    for(i=0; i<CHANNEL_NUM; i++)
    {
        if(TRUE == g_hw_table.channels[i].default_stat){
            is_save_stat = TRUE;
            break;
        }
    }

    if( is_save_stat )
    {   // 状态储存定时器 用于断电记忆
        ret = sys_add_timer(save_stat_timer_cb, NULL, &save_stat_timer);
        if(OPRT_OK != ret) {
            return ret;
        }
    
       //读出所有通道状态
        ret = read_saved_channel_stat(pch_state, CHANNEL_NUM);
        if(ret != OPRT_OK){
            PR_ERR("read stat failed!");
        }
        
        //将不需要断电记忆的通道置无效
        for(i=0; i<CHANNEL_NUM; i++)
        {
            if(FALSE == g_hw_table.channels[i].default_stat)
                pch_state[i] = FALSE;
        }
    }

    //设置通道状态
    for(i=0; i<CHANNEL_NUM; i++){
        hw_set_channel(&g_hw_table, i, pch_state[i]);
    }
    
    //释放内存
    Free(pch_state);

    return OPRT_OK;
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

    TY_IOT_CBS_S wf_cbs = {
        status_changed_cb,\
        gw_ug_inform_cb,\
        NULL,
        dev_obj_dp_cb,\
        dev_raw_dp_cb,\
        dev_dp_query_cb,\
        NULL,
    };

	PR_DEBUG("g_hw_table.wf_mode:%d",g_hw_table.wf_mode);
    op_ret = tuya_iot_wf_soc_dev_init(GWCM_MODE(g_hw_table.wf_mode),&wf_cbs,PRODECT_KEY,DEV_SW_VERSION);

    if(OPRT_OK != op_ret) {
        PR_ERR("tuya_iot_wf_soc_dev_init err:%d",op_ret);
        return op_ret;
    }

    op_ret = tuya_iot_reg_get_wf_nw_stat_cb(__get_wf_status);
    if(OPRT_OK != op_ret) {
        PR_ERR("tuya_iot_reg_get_wf_nw_stat_cb err:%d",op_ret);
        return op_ret;
    }

    // 硬件配置初始化 配置完后所有通道默认无效
    op_ret = hw_init(&g_hw_table, key_process);
    if(OPRT_OK != op_ret) {
        PR_ERR("hw_init:%d",op_ret);
        return op_ret;
    }
    
    //设备状态初始化
    //判断是否需要开启断电记忆功能
    dev_default_stat_init();

    op_ret = sys_add_timer(cd_timer_cb, NULL, &cd_timer);      // 倒计时定时器
    if(OPRT_OK != op_ret) {
        return op_ret;
    }
    else{
        PR_DEBUG("cd_timer ID:%d",cd_timer);
        sys_start_timer(cd_timer, 1000, TIMER_CYCLE);
    }

    INT_T size = SysGetHeapSize();//lql
	PR_NOTICE("device_init ok  free_mem_size:%d",size);
	
    return op_ret;
}


/***********************************************************
*  Function: 按键处理
*  Input: 
*  Output: 
*  Return: 
***********************************************************/
STATIC VOID key_process(TY_GPIO_PORT_E port,PUSH_KEY_TYPE_E type,INT_T cnt)
{
    INT_T i;//通道下标 
    char  *pbuff = NULL;
    cJSON *root = NULL;
    char name[10] = {0};


    PR_DEBUG("port: %d",port);
    PR_DEBUG("type: %d",type);
    PR_DEBUG("cnt: %d",cnt);

    /*******总控******/
     if((g_hw_table.power_button.type !=  IO_DRIVE_LEVEL_NOT_EXIST)&&\
        (g_hw_table.power_button.key_cfg.port == port))
     {
         if(LONG_KEY == type) {
            tuya_iot_wf_gw_unactive(WRT_AUTO);//AP  smart cfg   切换
         }else if(NORMAL_KEY == type){
    		BOOL is_every_active = TRUE;
            // 遍历通道计算是否全为有效
            for(i=0; i<CHANNEL_NUM; ++i) {
                is_every_active = is_every_active && g_hw_table.channels[i].stat;
            }
            // 如果全为有效 则全部关闭,否则全打开
            for(i=0; i<CHANNEL_NUM; ++i){
                hw_set_channel(&g_hw_table, i, !is_every_active);
            }
            // 启动断电记忆存储定时器
            if(is_save_stat)
            {
                sys_start_timer(save_stat_timer, 5000, TIMER_ONCE);
            }
            update_all_stat();
         }
         
         return;
     }

    
    /*******分控*****/
    for(i = 0; i < CHANNEL_NUM; i++)
    {
        // 如果通道io号和目前触发的一致 则认为该通道按钮事件到达
        if((g_hw_table.channels[i].button.type  !=  IO_DRIVE_LEVEL_NOT_EXIST)&&\
           (g_hw_table.channels[i].button.key_cfg.port == port))
        {   
            if(LONG_KEY == type) {
                tuya_iot_wf_gw_unactive(WRT_AUTO);//AP  smart cfg   切换
            }else if(NORMAL_KEY == type){
                root = cJSON_CreateObject();
                if(NULL == root)
                {
                    PR_DEBUG("Json CreateObject Fail");
                    return;
                }

                hw_trig_channel(&g_hw_table, i);
                
                //组回复帧
                sprintf(name, "%d", g_hw_table.channels[i].dpid);
                cJSON_AddBoolToObject(root, name, g_hw_table.channels[i].stat);
                
                if(g_hw_table.channels[i].cd_dpid != DPID_NOT_EXIST ){
                    sprintf(name, "%d", g_hw_table.channels[i].cd_dpid);
                    cJSON_AddNumberToObject(root, name, REV_COUNT_TIME(g_hw_table.channels[i].cd_sec));
                }
                
                // 启动断电记忆存储定时器
                if(is_save_stat)
                {
                    sys_start_timer(save_stat_timer, 5000, TIMER_ONCE);
                }

                // 发送数据
                pbuff = cJSON_PrintUnformatted(root);
                cJSON_Delete(root);
                dp_upload_proc(pbuff);
                Free(pbuff);  
                
                // 启动断电记忆存储定时器
                if(is_save_stat)
                {
                    sys_start_timer(save_stat_timer, 5000, TIMER_ONCE);
                }
            }
            
            return;
        }
    }
}

/***********************************************************
*  Function: 数据上报
*  Input: 
*  Output: 
*  Return: 
***********************************************************/
STATIC OPERATE_RET dp_upload_proc(CHAR_T *jstr)
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

/***********************************************************
*  Function: 存储的通道状态 定时回调函数
*  Input:
*  Output:
*  Return:   读取结果
***********************************************************/
STATIC VOID save_stat_timer_cb(UINT timerID,PVOID pTimerArg)
{
	OPERATE_RET op_ret;
	INT_T i;
	cJSON *root = NULL, *js_power = NULL;
	UCHAR_T  *out = NULL;

    root = cJSON_CreateObject();
    if(NULL == root)
    {
        PR_DEBUG("cJSON CreateObject Fail!");
        return;
    }
    js_power = cJSON_CreateArray();
    if(NULL == js_power)
    {
        PR_DEBUG("cJSON CreateArry Fail!");
        cJSON_Delete(root);
        return;
    }

    cJSON_AddItemToObject(root, POWER_MEMORY, js_power);
    
    PR_DEBUG("save stat");
    for(i=0; i<CHANNEL_NUM; ++i)
    {
        cJSON_AddItemToArray(js_power,cJSON_CreateBool(g_hw_table.channels[i].stat));
    }

    out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if( NULL == out)
    {
        PR_ERR("cJSON_PrintUnformatted failed!");
        return ;
    }
	PR_DEBUG("%s", out);
	
	op_ret = kvs_write(STORE_CHANGE,out,strlen(out));
    Free(out);
    
	if(OPRT_OK != op_ret) {			
		PR_ERR("kvs_read err:%02x",op_ret);
	}
}

/***********************************************************
*  Function: 读取存储的通道状态
*  Input:    ch_num   需要读取的通道数 
*  Output:   st_arr   读取通道状态
*  Return:   读取结果
***********************************************************/
OPERATE_RET read_saved_channel_stat(OUT CHAR_T *st_arr, IN INT_T ch_num)
{
    OPERATE_RET op_ret;
    cJSON *root = NULL, *js_power = NULL, *js_ch_stat = NULL;
    UCHAR_T *pbuff = Malloc(128);
    int i;

     memset(pbuff, 0x00, 128);
    // 读取存储的状态json {"power":[false,true...]}
    op_ret = kvs_read(STORE_CHANGE, &pbuff, 128);
    if(OPRT_OK != op_ret) {
        PR_NOTICE("Power stat not exist: %02x, this may be a new product.",op_ret);
        return op_ret;
    }
    
    // 分析json
    PR_DEBUG("read stat: %s", pbuff);
    root = cJSON_Parse(pbuff);
    Free(pbuff);
    // 如果json分析出错 退出函数
    if(root == NULL){
        PR_ERR("cjson parse err");
        return OPRT_COM_ERROR;
    }

    js_power = cJSON_GetObjectItem(root, POWER_MEMORY);
    // 如果找不到power对象 退出函数
    if(NULL == js_power) {
        PR_ERR("cjson get power error");
        cJSON_Delete(root);
        return OPRT_COM_ERROR;
    }

    for(i=0; i< ch_num; ++i)
    {
        js_ch_stat = cJSON_GetArrayItem(js_power, i);
        if(js_ch_stat == NULL)
        {
            PR_ERR("cjson %d ch stat not found", i);
            cJSON_Delete(root);
            return OPRT_COM_ERROR;
        }
        else
        {
            if(js_ch_stat->type == cJSON_True){
                *st_arr++ = (CHAR_T)TRUE;
            }
            else{
                *st_arr++ = (CHAR_T)FALSE;
            }
        }
    }
    
    cJSON_Delete(root);
    return OPRT_OK;
}
/***********************************************************
*  Function: 倒计时处理
*  Input:   
*  Output:
*  Return: 
***********************************************************/
STATIC VOID cd_timer_cb(UINT timerID,PVOID pTimerArg)
{
    int i;// 通道号
    cJSON *root = NULL;
    CHAR_T *pbuff = NULL;
    CHAR_T name[10] = {0};
    
    // 遍历通道
    for(i=0; i<CHANNEL_NUM; ++i)
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

                if( NULL == root)
                    root = cJSON_CreateObject();
                if(NULL == root){
                    PR_DEBUG("Json CreateObject Fail");
                    return;
                }

                // 上报通道状态
                sprintf(name, "%d", g_hw_table.channels[i].dpid);
                cJSON_AddBoolToObject(root, name, g_hw_table.channels[i].stat);
                if(g_hw_table.channels[i].cd_dpid != DPID_NOT_EXIST)
                {
                    sprintf(name, "%d", g_hw_table.channels[i].cd_dpid);
                    cJSON_AddNumberToObject(root, name, REV_COUNT_TIME(g_hw_table.channels[i].cd_sec));
                }
            }
            else
            {
                // 计时未到达
                // 每30s的整数倍上报一次
                if((g_hw_table.channels[i].cd_dpid != DPID_NOT_EXIST)&&\
                   (g_hw_table.channels[i].cd_sec % 30 == 0))
                {
                    if( NULL == root)
                        root = cJSON_CreateObject();
                    if(NULL == root){
                        PR_DEBUG("Json CreateObject Fail");
                        continue;
                    }
                    // 上报通道状态
                    sprintf(name, "%d", g_hw_table.channels[i].cd_dpid);
                    cJSON_AddNumberToObject(root, name, REV_COUNT_TIME(g_hw_table.channels[i].cd_sec));
                }
            }
        }
    }
    //发送数据
    if(NULL != root)
    {
        pbuff = cJSON_PrintUnformatted(root);
        cJSON_Delete(root);
        dp_upload_proc(pbuff);
        Free(pbuff);
    }
}
/***********************************************************
*  Function: 发送所有通道信息
*  Input:   
*  Output:
*  Return: 
***********************************************************/
VOID update_all_stat(VOID)
{
    int i, val = 0;
    char *pbuff = NULL;
    cJSON* root = NULL;
    char name[10] = {0};

    root = cJSON_CreateObject();
    if(NULL == root)
    {
        PR_DEBUG("Json CreateObject Fail");
        return;
    }
    
    for(i=0; i<CHANNEL_NUM; ++i) 
    {
        sprintf(name, "%d", g_hw_table.channels[i].dpid);
        cJSON_AddBoolToObject(root, name, g_hw_table.channels[i].stat);
        if(g_hw_table.channels[i].cd_dpid != DPID_NOT_EXIST)
        {
            sprintf(name, "%d", g_hw_table.channels[i].cd_dpid);
            cJSON_AddNumberToObject(root, name, REV_COUNT_TIME(g_hw_table.channels[i].cd_sec));
        }
    }
    
    // 发送数据
    PR_DEBUG("update all channel stat");
    pbuff = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    dp_upload_proc(pbuff);
    Free(pbuff);

}
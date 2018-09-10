/***********************************************************
*  File: app_dltj.c
*  Author: litao
*  Date: 170704
***********************************************************
*  Author: wym
*  Date: 180411
*  Description:重构，移植到realtek平台中
***********************************************************/

#define _APP_DLTJ_GLOBAL
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
#include "uni_time.h"





/***********************************************************
*************************micro define***********************
***********************************************************/
#if _APP_DLTJ_DEBUG
#define APP_DLTJ_DEBUG \
        PR_DEBUG
#else
#define APP_DLTJ_DEBUG(...)
#endif


#define WM_SUCCESS 0
#define WM_FAIL 1


#define DEVICE_MOD "device_mod"
#define DEVICE_PART "device_part"
#define ELE_SAVE_KEY "ele"
#define TEM_ELE_SAVE_KEY "tem_ele"

#define TIME_POSIX_2016 1451577600

#define ADD_ELE_THRE 100//临时电量累加处理阈值
#define ADD_TIME_MIN_THRE 60//电量处理最小时间间隔
#define ADD_TIME_MAX_THRE 1800//电量处理最大时间间隔，单位S
#define REPT_THRE_CURR 5//电流上报变化阈值，单位%
#define REPT_THRE_VOL 2//电压上报变化阈值，单位%
#define REPT_THRE_PWR 5//功率上报变化阈值，单位%
#define ELE_SLEEP_TIME 5//电量统计上层采样间隔
#define _IS_OVER_PROTECT 0//是否存在过流保护功能
#define LIMIT_CURRENT 10000//过流保护限值，单位1mA
#define _IF_REPT_COE   TRUE  //是否上报电量校准系数


/***********************************************************
*************************variable define********************
***********************************************************/
typedef struct{
    u8 edpid;
    u8 idpid;
    u8 pdpid;
    u8 vdpid;
	u8 prdpid;
	u8 vrefdpid;
	u8 irefdpid;
	u8 prefdpid;
    u8 erefdpid;
}AppDltjInfo_S,*pAppDltjInfo_S;

typedef struct 
{
    UINT_T unix_time;
	UINT_T ele_energy;
}LOC_DATA;

typedef struct 
{
	UINT_T add_ele;//下层电量统计驱动中累加的电量
	UINT_T cur_posix;//当前时间戳
	UINT_T loc_data_cnt;//本地电量数据组的元素个数
	UINT_T cur_current;
	UINT_T cur_vol;
	UINT_T cur_power;
	UINT_T last_current;
	UINT_T last_vol;
	UINT_T last_power;
	UINT_T tem_ele_val;//存放已经累加的但还未上报或存储到local_data中的电量
	LOC_DATA ele_sav_data[10];//本地电量数据组
}POWER_CNT;
STATIC POWER_CNT power_cnt;





STATIC ELE_THREAD_STATE ele_state;
STATIC INT_T cur_time;//距离上一次上报之后的时间，单位比1S略大一些
STATIC UINT_T last_tem_ele;//上一次电量处理时的临时电量值


typedef struct{
    TIMER_ID                    app_dltj_timer_id;
    THRD_HANDLE                 app_dltj_thread;
    SEM_HANDLE                  app_dltj_sem;
    AppDltjInfo_S	            app_dltj_info;
}AppDltj_S,*pAppDltj_S;
AppDltj_S mAppDltj;


/***********************************************************
*************************function define********************
***********************************************************/
STATIC P_THRD_FUNC app_dltj_proc(PVOID pArg);
STATIC BOOL same_day_judge(UINT_T u_time1, UINT_T u_time2);
STATIC OPERATE_RET get_time_posix(UINT_T *curPosix);
STATIC OPERATE_RET set_tem_ele_val(INT_T val);
STATIC OPERATE_RET get_tem_ele_val(INT_T *val);
STATIC OPERATE_RET set_ele_data(VOID);
STATIC OPERATE_RET get_ele_data(VOID);
STATIC VOID addto_local_data(UINT_T time, UINT_T ele);
STATIC INT_T report_cur_data(VOID);
STATIC OPERATE_RET update_ele_data(UINT_T time, UINT_T ele_value);
STATIC VOID report_local_data(VOID);
STATIC VOID save_reported_data();
STATIC BOOL value_range_judge(UINT_T JudgedValue, UINT_T TargetValue, UINT_T range);
STATIC INT_T report_coe_data(VOID);





OPERATE_RET dltj_config_init(int mode)
{
    DLTJ_CONFIG dltj_config_data;
    dltj_config_data.epin = TY_GPIOA_18;
    dltj_config_data.ivpin = TY_GPIOA_23;
    dltj_config_data.ivcpin.pin = TY_GPIOA_15;
    dltj_config_data.ivcpin.type = IO_DRIVE_LEVEL_HIGH;
    dltj_config_data.v_ref = 1680;
    dltj_config_data.i_ref = 34586;
    dltj_config_data.p_ref = 98363;
    dltj_config_data.e_ref = 212;
    dltj_config_data.v_def = 2200;
    dltj_config_data.i_def = 392;
    dltj_config_data.p_def = 864;
    dltj_config_data.if_have = TRUE;
    dltj_config_data.edpid = DP_ELE;
    dltj_config_data.idpid = DP_CURRENT;
    dltj_config_data.pdpid = DP_POWER;
    dltj_config_data.vdpid = DP_VOLTAGE;
    dltj_config_data.prdpid = DP_PTRSLT;
    dltj_config_data.vrefdpid = DP_VREF;
    dltj_config_data.irefdpid = DP_IREF;
    dltj_config_data.prefdpid = DP_PREF;
    dltj_config_data.erefdpid = DP_EREF;
    OPERATE_RET op_ret;
    if(mode == D_NORMAL_MODE){
        op_ret = app_dltj_init(&(dltj_config_data));
        if(OPRT_OK != op_ret){
            return op_ret;
        }
    }else if(mode == D_CAL_START_MODE){
        hlw8012_init(&(dltj_config_data));
    }
    return OPRT_OK;
}


OPERATE_RET app_dltj_init(DLTJ_CONFIG *dltj)
{
    OPERATE_RET op_ret;

    hlw8012_init(dltj);
    
    mAppDltj.app_dltj_info.edpid = dltj->edpid;
    mAppDltj.app_dltj_info.idpid = dltj->idpid;
    mAppDltj.app_dltj_info.vdpid = dltj->vdpid;
    mAppDltj.app_dltj_info.pdpid = dltj->pdpid;

    mAppDltj.app_dltj_info.prdpid = dltj->prdpid;
    mAppDltj.app_dltj_info.vrefdpid = dltj->vrefdpid;
    mAppDltj.app_dltj_info.irefdpid = dltj->irefdpid;
    mAppDltj.app_dltj_info.prefdpid = dltj->prefdpid;
    mAppDltj.app_dltj_info.erefdpid = dltj->erefdpid;
    if(OPRT_OK == ele_cnt_init(D_NORMAL_MODE)){
        if(OPRT_OK != get_ele_data()){
            PR_ERR("get ele data err...");
        }
        if(OPRT_OK != get_tem_ele_val(&power_cnt.tem_ele_val)){
            PR_ERR("get tem ele data err...");
        }


        //app thread
        THRD_PARAM_S thrd_param;
        thrd_param.priority = TRD_PRIO_4;
        thrd_param.stackDepth = 1024*2;
        thrd_param.thrdname = "app_dltj_task";
        op_ret = CreateAndStart(&mAppDltj.app_dltj_thread,NULL,NULL,app_dltj_proc,NULL,&thrd_param);
        if(op_ret != OPRT_OK) {
            PR_ERR("creat ele thread err...");
            return op_ret;
        }
        APP_DLTJ_DEBUG("app_dltj_init success............................................");
        //some timer
        return OPRT_OK;
   }
}

STATIC VOID ele_par_handle()
{
    get_ele_par(&power_cnt.cur_power,&power_cnt.cur_vol,&power_cnt.cur_current);
    APP_DLTJ_DEBUG("cur:%d power:%d vol:%d",power_cnt.cur_current,power_cnt.cur_power,power_cnt.cur_vol);
    if(ele_state == ELE_NORMAL){
        if(WM_SUCCESS != report_cur_data()){
            APP_DLTJ_DEBUG("report cur data err !!!");
        }else{
            save_reported_data();
        }
    }
    if(_IS_OVER_PROTECT){
        if(power_cnt.cur_current >= LIMIT_CURRENT){
            //over_protect();
        }
    }
}
/*联网激活前，所有新增加的电量存储在power_cnt.tem_ele_val中，并存储在TEM_ELE_SAVE_KEY区，
 激活之后，将power_cnt.tem_ele_val上报并清空TEM_ELE_SAVE_KEY，此后对于新增加的电量，先尝试
 上报，上报失败则存在ele_sav_data数组中，联网状态下电量线程每次循环会尝试上报ele_sav_data数组*/
STATIC VOID add_ele_handle()
{
    get_ele(&power_cnt.add_ele);
    power_cnt.tem_ele_val += power_cnt.add_ele;
    APP_DLTJ_DEBUG("add_ele = %d,tem_ele_val = %d",power_cnt.add_ele,power_cnt.tem_ele_val);
    power_cnt.add_ele = 0;
    //电量处理事件触发
    if((power_cnt.tem_ele_val - last_tem_ele>= ADD_ELE_THRE && cur_time >= ADD_TIME_MIN_THRE )\
        || cur_time >= ADD_TIME_MAX_THRE){
        if(power_cnt.tem_ele_val - last_tem_ele > 0){
            if(ELE_NOT_ACTIVE == ele_state){
                set_tem_ele_val(power_cnt.tem_ele_val);
                APP_DLTJ_DEBUG("set tem ele val :%d success...",power_cnt.tem_ele_val);
            }else if(ELE_NORMAL == ele_state){
                if(OPRT_OK != update_ele_data(power_cnt.cur_posix, power_cnt.tem_ele_val)){
                    addto_local_data(power_cnt.cur_posix, power_cnt.tem_ele_val);
                }
                power_cnt.tem_ele_val = 0;
            }else if(ELE_UNCONNECT == ele_state || ELE_SYS_OTA == ele_state){
                addto_local_data(power_cnt.cur_posix, power_cnt.tem_ele_val);
                power_cnt.tem_ele_val = 0;
            }
            last_tem_ele = power_cnt.tem_ele_val;
        }
        cur_time = 0;
    }
    if(ele_state == ELE_NORMAL){
        if(power_cnt.loc_data_cnt != 0){
            report_local_data();
        }
    }
}

STATIC P_THRD_FUNC app_dltj_proc(PVOID pArg)
{ 
    cur_time = 0;
    last_tem_ele = power_cnt.tem_ele_val;
    ele_state = ELE_NOT_ACTIVE;

    GW_WIFI_NW_STAT_E wf_nw_stat;
    get_wf_gw_nw_status(&wf_nw_stat);
    
    while(1){
    #if 1
        switch(ele_state){
        case ELE_NOT_ACTIVE:
            if(GNS_WAN_VALID == get_gw_nw_status()&& OPRT_OK == get_time_posix(&power_cnt.cur_posix)){
                ele_state = ELE_NORMAL;//首次联网激活，将临时电量上报并清零临时电量内存
                if(power_cnt.tem_ele_val){
                    if(OPRT_OK != update_ele_data(power_cnt.cur_posix, power_cnt.tem_ele_val)){
                        addto_local_data(power_cnt.cur_posix, power_cnt.tem_ele_val);
                    }
                    power_cnt.tem_ele_val = 0;
                    last_tem_ele = 0;
                    cur_time = 0;
                    set_tem_ele_val(power_cnt.tem_ele_val);
                }
                if(_IF_REPT_COE){
                    if(WM_SUCCESS != report_coe_data()){
                        PR_DEBUG("report coe data fail!!!");
                    }
                }
            }
            break;
        case ELE_NORMAL:
        case ELE_UNCONNECT:
            break;
        case ELE_SYS_OTA://OTA结束成功将重启,失败将继续工作
            break;
        }
        get_time_posix(&power_cnt.cur_posix);
        PR_DEBUG("time posix:%d", power_cnt.cur_posix);
        APP_DLTJ_DEBUG("Curr ele state :%d",ele_state);
        update_ele_data(power_cnt.cur_posix,100);
        ele_par_handle();
        add_ele_handle();
        //PR_NOTICE("remain size:%d",system_get_free_heap_size());
   #else
           power_cnt.cur_power +=10;
           power_cnt.cur_current +=10;
           power_cnt.cur_vol +=10;
           report_cur_data();
           get_time_posix(&power_cnt.cur_posix);
           APP_DLTJ_DEBUG("power_cnt.cur_posix :%d",power_cnt.cur_posix);
           last_tem_ele += 100;
           update_ele_data(power_cnt.cur_posix,last_tem_ele);
   #endif

        SystemSleep(ELE_SLEEP_TIME*1000);
        cur_time += ELE_SLEEP_TIME;
        APP_DLTJ_DEBUG("Curr time:%d",cur_time);
    }

}

STATIC BOOL same_day_judge(UINT_T u_time1, UINT_T u_time2)
{
	return (u_time1+28800)/86400 == (u_time2+28800)/86400 ? TRUE: FALSE;
}

STATIC OPERATE_RET get_time_posix(UINT_T *curPosix)
{
    UINT tmp_posix;
    tmp_posix = uni_time_get_posix();
    if(tmp_posix < TIME_POSIX_2016) {
        APP_DLTJ_DEBUG("get curPosix err!!!");
        return OPRT_INVALID_PARM;
    }
    *curPosix = tmp_posix;
    return OPRT_OK;
}



STATIC OPERATE_RET set_tem_ele_val(INT_T val)
{
	OPERATE_RET op_ret;
    cJSON *root = NULL;
	UCHAR *buf = NULL;

    root = cJSON_CreateObject();
    if(NULL == root) {
		PR_ERR("cJSON_CreateObject error");
		return OPRT_CJSON_GET_ERR;
	}
    
    cJSON_AddNumberToObject(root, "tem_ele", val);
    buf = cJSON_PrintUnformatted(root);
    if(NULL == buf) {
        PR_ERR("buf is NULL");
        cJSON_Delete(root);
        return OPRT_COM_ERROR;
    }    
    cJSON_Delete(root);
    op_ret = flash_self_if_write(TEM_ELE_SAVE_KEY, buf);
	if(OPRT_OK != op_ret) {
		APP_DLTJ_DEBUG("set tem ele err...........................:%02x",op_ret);
        Free(buf);
		return op_ret;
	}
    
    Free(buf);
    return OPRT_OK;    
}

STATIC OPERATE_RET get_tem_ele_val(INT_T *val)
{
	OPERATE_RET op_ret;
    cJSON *root = NULL, *json = NULL;
	UCHAR *buf;

    buf = (UCHAR *)Malloc(256);
	if(NULL == buf) {
		PR_ERR("malloc error");
		*val = 0;
		return OPRT_MALLOC_FAILED;
	}
    UINT_T buf_len;
    op_ret = kvs_read(TEM_ELE_SAVE_KEY, &buf,&buf_len);
	if(OPRT_OK != op_ret) {
		APP_DLTJ_DEBUG("msf_get_single err:%02x",op_ret);
        Free(buf);
		*val = 0;
		return op_ret;
	}
    PR_DEBUG("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!tem ele buf:%s",buf);
	root = cJSON_Parse(buf);
	if(NULL == root) {
		PR_ERR("cjson parse");
        goto JSON_PARSE_ERR;
	}

    json = cJSON_GetObjectItem(root,"tem_ele");
    if(NULL == json) {
        PR_ERR("cjson get ");
        goto JSON_PARSE_ERR;
	}

    *val = json->valueint;
    PR_DEBUG("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!get tem ele data:%d",*val);
    cJSON_Delete(root);
    Free(buf);
    return OPRT_OK;

JSON_PARSE_ERR:
    cJSON_Delete(root);
    Free(buf);
	*val = 0;
    return OPRT_COM_ERROR;
}

STATIC OPERATE_RET set_ele_data(VOID)
{
	OPERATE_RET op_ret;
    cJSON *arrayItem = NULL, *array = NULL;
	UCHAR *buf = NULL;
	INT_T i;

	array = cJSON_CreateArray();    
	if(NULL == array) {        
		PR_ERR("cJSON_CreateArray err");        
		return OPRT_CJSON_GET_ERR;    
	}

	for(i=0; i<power_cnt.loc_data_cnt; i++){
		arrayItem = cJSON_CreateObject();
	    if(NULL == arrayItem) {
			cJSON_Delete(array);
			PR_ERR("cJSON_CreateObject error");
			return OPRT_CJSON_GET_ERR;
		}
		cJSON_AddNumberToObject(arrayItem, "time", power_cnt.ele_sav_data[i].unix_time);
		cJSON_AddNumberToObject(arrayItem, "ele", power_cnt.ele_sav_data[i].ele_energy);
		cJSON_AddItemToArray(array,arrayItem);
	}
    buf = cJSON_PrintUnformatted(array);
    if(NULL == buf) {
        PR_ERR("buf is NULL");
        cJSON_Delete(array);
        return OPRT_COM_ERROR;
    }    
	APP_DLTJ_DEBUG("set data buf:%s",buf);
	
	op_ret = flash_self_if_write(ELE_SAVE_KEY,buf);
	if(OPRT_OK != op_ret) {
		APP_DLTJ_DEBUG("msf_get_single err:%02x",op_ret);
        cJSON_Delete(array);
        Free(buf);
		return op_ret;
	}
	
    cJSON_Delete(array);
    Free(buf);
    return OPRT_OK;    
}

STATIC OPERATE_RET get_ele_data(VOID)
{
	OPERATE_RET op_ret;
    cJSON *arrayItem = NULL, *array = NULL, *json = NULL;
	UCHAR *buf=NULL;
	INT_T i;

    buf = (UCHAR *)Malloc(1024);
	if(NULL == buf) {
		PR_ERR("malloc error");
		power_cnt.loc_data_cnt = 0;
		return OPRT_MALLOC_FAILED;
	}
    UINT_T buf_len;
	op_ret = kvs_read(ELE_SAVE_KEY,&buf,&buf_len);
	if(OPRT_OK != op_ret) {
		APP_DLTJ_DEBUG("msf_get_single err:%02x",op_ret);
		power_cnt.loc_data_cnt = 0;
        Free(buf);
		return op_ret;
	}
	
	APP_DLTJ_DEBUG("get ele data buf:%s",buf);
	array = cJSON_Parse(buf);
	if(NULL == array) {
		PR_ERR("cjson parse err");
		power_cnt.loc_data_cnt = 0;
	    Free(buf);
	    return OPRT_COM_ERROR;
	}

	Free(buf);
	power_cnt.loc_data_cnt = cJSON_GetArraySize(array);

	for(i=0; i<power_cnt.loc_data_cnt; i++){
		arrayItem = cJSON_GetArrayItem(array, i);
		if(arrayItem){
			json = cJSON_GetObjectItem(arrayItem,"time");
			if(json == NULL){
				cJSON_Delete(array);
				return OPRT_COM_ERROR;
			}
			power_cnt.ele_sav_data[i].unix_time = json->valueint;
			json = cJSON_GetObjectItem(arrayItem,"ele");
			if(json == NULL){
				cJSON_Delete(array);
				return OPRT_COM_ERROR;
			}
				
			power_cnt.ele_sav_data[i].ele_energy = json->valueint;
			APP_DLTJ_DEBUG("i:%d,time:%d,ele:%d",i,power_cnt.ele_sav_data[i].unix_time,\
			power_cnt.ele_sav_data[i].ele_energy);
		}
	}
	cJSON_Delete(array);
    return OPRT_OK;
}






//最多存10组电量，按天分组
STATIC VOID addto_local_data(UINT_T time, UINT_T ele)
{
	INT_T i;
	APP_DLTJ_DEBUG("time:%d,ele:%d",time,ele);
	APP_DLTJ_DEBUG("try to add to local data...");

	if(power_cnt.loc_data_cnt != 0){
		if(time == 0){
			power_cnt.ele_sav_data[power_cnt.loc_data_cnt-1].ele_energy += ele;
			return;
		}
		//电量按天存在flash中
		if(same_day_judge(power_cnt.ele_sav_data[power_cnt.loc_data_cnt-1].unix_time,time)){
			power_cnt.ele_sav_data[power_cnt.loc_data_cnt-1].unix_time = time;
			power_cnt.ele_sav_data[power_cnt.loc_data_cnt-1].ele_energy += ele;
			APP_DLTJ_DEBUG("Same day ...%d %d",power_cnt.ele_sav_data[power_cnt.loc_data_cnt-1].ele_energy,ele);
		}else{
			if(power_cnt.loc_data_cnt == 10){
				//for(i=0; i<9; i++){
				//	power_cnt.ele_sav_data[i].unix_time = power_cnt.ele_sav_data[i+1].unix_time;
				//	power_cnt.ele_sav_data[i].ele_energy = power_cnt.ele_sav_data[i+1].ele_energy;
				//}
				//power_cnt.ele_sav_data[power_cnt.loc_data_cnt-1].unix_time = time;
				power_cnt.ele_sav_data[power_cnt.loc_data_cnt-1].ele_energy += ele;//达到最大存储数目之后，将后续电量存在最后一天
			}else{
				power_cnt.ele_sav_data[power_cnt.loc_data_cnt].unix_time = time;
				power_cnt.ele_sav_data[power_cnt.loc_data_cnt].ele_energy = ele;
				power_cnt.loc_data_cnt++;
			}

		}
	}else{
		power_cnt.ele_sav_data[0].unix_time = time;
		power_cnt.ele_sav_data[0].ele_energy = ele;
		power_cnt.loc_data_cnt++;
	}
	APP_DLTJ_DEBUG("set loc_data_cnt : %d",power_cnt.loc_data_cnt);
	if(OPRT_OK != set_ele_data()){
		APP_DLTJ_DEBUG("set ele to flash err ...");
	}
}


STATIC INT_T report_cur_data(VOID)
{
    char dpid_str[8];
/*
    GW_WIFI_STAT_E wf_stat = get_wf_gw_status();
    if(STAT_UNPROVISION == wf_stat || \
       STAT_STA_UNCONN == wf_stat || \
       (get_gw_status() != STAT_WORK)) {
        return WM_FAIL;
    }
*/
    cJSON *root = NULL;
    root = cJSON_CreateObject();
    if(NULL == root) {
        return WM_FAIL;
    }

    if(!value_range_judge(power_cnt.cur_current,power_cnt.last_current,REPT_THRE_CURR) ||\
       !value_range_judge(power_cnt.cur_power,power_cnt.last_power,REPT_THRE_PWR) ||\
       !value_range_judge(power_cnt.cur_vol,power_cnt.last_vol,REPT_THRE_VOL)){
        sprintf(dpid_str, "%d", mAppDltj.app_dltj_info.idpid);
        cJSON_AddNumberToObject(root,dpid_str,power_cnt.cur_current);
        sprintf(dpid_str, "%d", mAppDltj.app_dltj_info.pdpid);
    	cJSON_AddNumberToObject(root,dpid_str,power_cnt.cur_power);
    	sprintf(dpid_str, "%d", mAppDltj.app_dltj_info.vdpid);
    	cJSON_AddNumberToObject(root,dpid_str,power_cnt.cur_vol); 
    }else{
        APP_DLTJ_DEBUG("No ele data need to report");
        cJSON_Delete(root);
        return WM_FAIL;
    }

    char *out;
    out=cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if(NULL == out) {
        PR_ERR("cJSON_PrintUnformatted err:");
        return WM_FAIL;
    }

    //APP_DLTJ_DEBUG("out len:%d",strlen(out));
    APP_DLTJ_DEBUG("out[%s]", out);
    OPERATE_RET op_ret = sf_obj_dp_report_async(get_gw_cntl()->gw_if.id,out,false);
    if(OPRT_OK != op_ret) {
		Free(out);
        PR_ERR("sf_obj_dp_report op_ret:%d",op_ret);
        return WM_FAIL;
    }
	Free(out);
    return WM_SUCCESS;
   
}

STATIC OPERATE_RET update_ele_data(UINT_T time, UINT_T ele_value)
{
#if 1
	OPERATE_RET op_ret;
	TY_OBJ_DP_S *stat_data;
	INT_T size = 0;
	size = sizeof(TY_OBJ_DP_S);//????????????????如何分配
	APP_DLTJ_DEBUG("size:%d",size);
	stat_data = (TY_OBJ_DP_S *)Malloc(size);
	memset(stat_data, 0, size);
	stat_data->dpid = mAppDltj.app_dltj_info.edpid;
	stat_data->time_stamp = time;
	stat_data->type = PROP_VALUE;
	stat_data->value.dp_enum = ele_value;
    APP_DLTJ_DEBUG("Upload tem ele time:%d, value:%d!!!!!",time,ele_value);
    op_ret = dev_report_dp_stat_sync(get_gw_cntl()->gw_if.id,stat_data,\
                                                   1,5);//??????????

	if(OPRT_OK != op_ret){
		PR_ERR("Ele report fail %d!!!", op_ret);
		Free(stat_data);
		return OPRT_COM_ERROR;
	}
	Free(stat_data);
	APP_DLTJ_DEBUG("Ele report success...");
	return OPRT_OK;
#endif
}

STATIC VOID report_local_data(VOID)
{
	INT_T i,j;
	UINT_T val_size = power_cnt.loc_data_cnt;
	INT_T cha_flag = 0;
	if(val_size == 0)
		return;
/*
    if((wifi_stat != STAT_STA_CONN && wifi_stat != STAT_AP_STA_CONN) || \
       (get_gw_status() < STAT_WORK) ) {
        APP_DLTJ_DEBUG("unconnect...");
        return;
    }
*/
	for(i=0; i<val_size; i++){
		if(update_ele_data(power_cnt.ele_sav_data[0].unix_time,power_cnt.ele_sav_data[0].ele_energy) != OPRT_OK){
			break;
		}
		APP_DLTJ_DEBUG("Upload cnt:%d,curr_i:%d,unix_time:%d,ele_energy:%d success...",power_cnt.loc_data_cnt,i,\
		        power_cnt.ele_sav_data[0].unix_time,power_cnt.ele_sav_data[0].ele_energy);
		cha_flag = 1;
		for(j=0; j<val_size-1; j++){
			power_cnt.ele_sav_data[j].unix_time = power_cnt.ele_sav_data[j+1].unix_time;
			power_cnt.ele_sav_data[j].ele_energy = power_cnt.ele_sav_data[j+1].ele_energy;
		}
		power_cnt.loc_data_cnt -= 1;
	}
	
	if(cha_flag == 1){
		if(OPRT_OK != set_ele_data()){
			PR_ERR("set ele to flash err ...");
		}
	}
}


STATIC VOID save_reported_data()
{
    power_cnt.last_current = power_cnt.cur_current;
    power_cnt.last_vol = power_cnt.cur_vol;
    power_cnt.last_power = power_cnt.cur_power;
}




STATIC INT_T report_coe_data(VOID)
{
    char dpid_str[8];
    cJSON *root = NULL;
    root = cJSON_CreateObject();
    if(NULL == root) {
        return WM_FAIL;
    }
    
    RPT_REF dltj_ref_data;
    get_dltj_ref_data(&dltj_ref_data.prod_rslt,&dltj_ref_data.v_ref,&dltj_ref_data.i_ref,\
                      &dltj_ref_data.p_ref,&dltj_ref_data.e_ref);
                      
    APP_DLTJ_DEBUG("!!!product result: %d, v_ref:%d, i_ref:%d, p_ref:%d, e_ref:%d",dltj_ref_data.prod_rslt,\
            dltj_ref_data.v_ref,dltj_ref_data.i_ref,dltj_ref_data.p_ref,dltj_ref_data.e_ref);
    sprintf(dpid_str, "%d", mAppDltj.app_dltj_info.prdpid);
    cJSON_AddNumberToObject(root,dpid_str,dltj_ref_data.prod_rslt);
    sprintf(dpid_str, "%d", mAppDltj.app_dltj_info.vrefdpid);
    cJSON_AddNumberToObject(root,dpid_str,dltj_ref_data.v_ref);
    sprintf(dpid_str, "%d", mAppDltj.app_dltj_info.irefdpid);
	cJSON_AddNumberToObject(root,dpid_str,dltj_ref_data.i_ref);
	sprintf(dpid_str, "%d", mAppDltj.app_dltj_info.prefdpid);
	cJSON_AddNumberToObject(root,dpid_str,dltj_ref_data.p_ref);
	sprintf(dpid_str, "%d", mAppDltj.app_dltj_info.erefdpid);
	cJSON_AddNumberToObject(root,dpid_str,dltj_ref_data.e_ref);

    char *out;
    out=cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if(NULL == out) {
        PR_ERR("cJSON_PrintUnformatted err:");
        return WM_FAIL;
    }
    
    APP_DLTJ_DEBUG("out[%s]", out);
    OPERATE_RET op_ret = sf_obj_dp_report_async(get_gw_cntl()->gw_if.id,out,false);
    if(OPRT_OK != op_ret) {
		Free(out);
        PR_ERR("sf_obj_dp_report op_ret:%d",op_ret);
        return WM_FAIL;
    }
	Free(out);
	APP_DLTJ_DEBUG("report ref coe success!!! product result:%d, v_ref:%d, i_ref:%d, p_ref:%d, e_ref:%d",\
	    dltj_ref_data.v_ref,dltj_ref_data.i_ref,dltj_ref_data.p_ref,dltj_ref_data.e_ref,dltj_ref_data.prod_rslt);
    return WM_SUCCESS;
   
}

//判断JudgedValue是否在TargetValue的正负range%范围之内，如果是返回1，如果否返回0
//JudgedValue为被判定的值，TargetValue为被判定值的目标值，range为浮动范围0-100(%).
STATIC BOOL value_range_judge(UINT_T JudgedValue, UINT_T TargetValue, UINT_T range)
{
    if((JudgedValue * 100 >= TargetValue * (100 - range)) && \
        (JudgedValue * 100 <= TargetValue * (100 + range))){
        return TRUE;
    }else{
        return FALSE;
    }
}

VOID wf_nw_stat_inform(GW_WIFI_NW_STAT_E stat)
{
    if(ele_state == ELE_NORMAL || ele_state == ELE_UNCONNECT){
        if(stat == STAT_AP_CLOUD_CONN || stat == STAT_CLOUD_CONN){
            ele_state = ELE_NORMAL;
        }else{
            ele_state = ELE_UNCONNECT;
        }
    }
}

VOID set_ele_thread_state(ELE_THREAD_STATE instate)
{
    ele_state = instate;
}





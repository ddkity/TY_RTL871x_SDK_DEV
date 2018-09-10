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

extern BOOL flash_scene_flag;

STATIC VOID init_upload_proc(VOID);
STATIC VOID start_gra_change(TIME_MS delay_time);
STATIC VOID hw_test_timer_cb(uint32_t id);
//STATIC INT_T get_reset_cnt(VOID);
STATIC OPERATE_RET dev_inf_set(VOID);

/*******************light pwm io define********************/

GW_WIFI_NW_STAT_E wifi_stat;    /* wifi 当前状态标识 */

/* 云端对应的数据点 */
#define DPID_1               1  //开关
#define DPID_3               3  //亮度


/* 灯光数据结构体，值由dp_data转换而来 */
typedef struct {
	UCHAR WHITE_VAL;
	UCHAR LAST_WHITE_VAL;
	UCHAR FIN_WHITE_VAL;
}LIGHT_DATA_DEF;

STATIC LIGHT_DATA_DEF light_data;

typedef struct
{
	BOOL scale_flag;
    THRD_HANDLE gra_thread;
    SEM_HANDLE gra_key_sem;
}L_GRA_CHANGE_DEF;
STATIC L_GRA_CHANGE_DEF gra_change;

typedef struct
{
	MUTEX_HANDLE  mutex;
    THRD_HANDLE flash_scene_thread;
    xSemaphoreHandle flash_scene_sem;
}FLASH_SCENE_HANDLE_DEF;
STATIC FLASH_SCENE_HANDLE_DEF flash_scene_handle;

/* tuya云端下发的数据点对应的值 */
typedef struct
{
	BOOL SWITCH;
    UCHAR BRIGHT;
}DP_DEF;
STATIC DP_DEF dp_data;

/* 定义系统重启的原因 */
typedef enum {
    REASON_DEFAULT_RST = 0,         /**< normal startup by power on */
//    REASON_WDT_RST,             /**< hardware watch dog reset */
//    REASON_EXCEPTION_RST,       /**< exception reset, GPIO status won't change */
//    REASON_SOFT_WDT_RST,        /**< software watch dog reset, GPIO status won't change */
    REASON_SOFT_RESTART = 9,        /**< software restart ,system_restart , GPIO status won't change */
//    REASON_DEEP_SLEEP_AWAKE,    /**< wake up from deep-sleep */
//    REASON_EXT_SYS_RST          /**< external system reset */
} rst_reason;

//定时器
TIMER_ID wf_stat_dir;
TIMER_ID timer_init_dpdata;
TIMER_ID gradua_timer;
TIMER_ID timer;
TIMER_ID data_save_timer;

/* 这三个变量是干什么用的 */
STATIC UINT_T irq_cnt = 0;
STATIC UINT_T num_cnt = 0;  /* 云端下发场景数据的时候，有会分为几组颜色的情况，然后各组颜色轮流切换，这个变量就是指当前处于哪个组 */
STATIC INT_T flash_dir = 0;

BOOL sta_cha_flag = FALSE;

//hw_timer_define
STATIC gtimer_t hw_timer;

/****************************************************************
*   公共函数
****************************************************************/
STATIC USHORT byte_combine(UCHAR hight, UCHAR low)
{
    USHORT temp;
    temp = (hight << 8) | low;
    return temp;
}
STATIC VOID char_change(UINT temp, UCHAR *hight, UCHAR *low)
{
    *hight = (temp & 0xff00) >> 8;
    *low = temp & 0x00ff;
}

STATIC INT_T string_combine_byte(u32 a,u32 b)
{
    INT_T combine_data = (a<<4)|(b&0xf);
    return combine_data;
}
STATIC INT_T string_combine_short(u32 a,u32 b, u32 c,u32 d)
{
    INT_T combine_data = (a<<12)|(b<<8)|(c<<4)|(d&0xf);
    return combine_data;
}

STATIC INT_T ABS(INT_T value)
{
	if(value < 0){
		return 0-value;
	}else{
		return value;
	}
}

STATIC UCHAR get_max_value(UCHAR a, UCHAR b, UCHAR c, UCHAR d, UCHAR e)
{
	int x = a > b ? a : b; //1次比较，1次赋值
	int y = c > d ? c : d; //1次比较，1次赋值
	int z = x > y ? x : y;
	return z > e ? z : e;  //1次比较
}

STATIC CHAR_T* my_itoa(int num,char*str,int radix)
{
    /*索引表*/
    char index[]="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    unsigned unum;/*中间变量*/
    int i=0,j,k;
    char temp;
    /*确定unum的值*/
    if(radix==10&&num<0)/*十进制负数*/
    {
        unum=(unsigned)-num;
        str[i++]='-';
    }
    else unum=(unsigned)num;/*其他情况*/
    /*转换*/
    do{
        str[i++]=index[unum%(unsigned)radix];
        unum/=radix;
    }while(unum);
    str[i]='\0';
    /*逆序*/
    if(str[0]=='-')k=1;/*十进制负数*/
    else k=0;
    for(j=k;j<=(i-1)/2;j++)
    {
        temp=str[j];
        str[j]=str[i-1+k-j];
        str[i-1+k-j]=temp;
    }
    return str;
}

static unsigned char abcd_to_asc(unsigned char ucBcd)
{
	unsigned char ucAsc = 0;

	ucBcd &= 0x0f;
	if (ucBcd <= 9)
		ucAsc = ucBcd + '0';
	else
		ucAsc = ucBcd + 'A' - 10;
	return (ucAsc);
}

void BcdToAsc_Api(char * sAscBuf, unsigned char * sBcdBuf, int iAscLen)
{
	int i, j;j = 0;

	if((sBcdBuf == NULL) || (sAscBuf == NULL) || (iAscLen < 0))
		return;

	for (i = 0; i < iAscLen / 2; i++)
	{
		sAscBuf[j] = (sBcdBuf[i] & 0xf0) >> 4;
		sAscBuf[j] = abcd_to_asc(sAscBuf[j]);
		j++;
		sAscBuf[j] = sBcdBuf[i] & 0x0f;
		sAscBuf[j] = abcd_to_asc(sAscBuf[j]);
		j++;
	}
	if (iAscLen % 2)
	{
		sAscBuf[j] = (sBcdBuf[i] & 0xf0) >> 4;
		sAscBuf[j] = abcd_to_asc(sAscBuf[j]);
	}
}

/******************************************************************************
*   分割线
*******************************************************************************/
STATIC VOID get_light_data(VOID)
{
    light_data.FIN_WHITE_VAL = dp_data.BRIGHT;
}

STATIC VOID start_gra_change(TIME_MS delay_time)
{
	gra_change.scale_flag = FALSE;
    gtimer_reload(&hw_timer, 1000*delay_time);
    gtimer_start(&hw_timer);
}

VOID light_data_handler(TY_OBJ_DP_S root)
{
    UCHAR dpid, type;
	u16_t len, rawlen;

    dpid = root.dpid;

    switch(dpid) {
        case DPID_1:
            PR_DEBUG("{\"%d\":\"%d\"}", dpid, root.value.dp_bool);
			switch(root.value.dp_bool) {
			    case cJSON_False:
			        //关灯
			        dp_data.SWITCH = FALSE;
					MutexLock(flash_scene_handle.mutex);
					light_data.WHITE_VAL = 0;
					light_data.FIN_WHITE_VAL = 0;
                    gtimer_stop(&hw_timer);
					send_light_data(0x00, 0x00, 0x00, light_data.WHITE_VAL, 0x00);  /* huangjituan 马上关灯 */
					MutexUnLock(flash_scene_handle.mutex);
			        break;

			    case cJSON_True:
					//开灯
			        MutexLock(flash_scene_handle.mutex);
					dp_data.SWITCH = TRUE;
                    get_light_data();
                    MutexUnLock(flash_scene_handle.mutex);
					start_gra_change(NORMAL_DELAY_TIME);
			        break;

			    default:
			        break;
			}
			break;

        case DPID_3:
            PR_DEBUG("{\"%d\":\"%d\"}", dpid, root.value.dp_value);
			if(root.value.dp_value < 11 || root.value.dp_value >255){
				PR_ERR("the data length is %d",len);
			}else {
				dp_data.BRIGHT = root.value.dp_value;
				get_light_data();
				start_gra_change(NORMAL_DELAY_TIME);
			}
			break;

        default:
            break;
    }
}

VOID light_data_save(VOID)
{
    if(!IsThisSysTimerRun(data_save_timer)){
        sys_start_timer(data_save_timer,5000,TIMER_CYCLE);
    }
}

/* 根据wifi状态设置指示灯 */
STATIC VOID wfl_timer_cb(UINT timerID,PVOID pTimerArg)
{
    OPERATE_RET op_ret;
    STATIC UINT last_wf_stat = 0xffffffff;
	STATIC BOOL config_flag = FALSE;

    if(last_wf_stat != wifi_stat) {
        PR_DEBUG("wifi_stat:%d",wifi_stat);
		PR_DEBUG("size:%d",SysGetHeapSize());
        switch(wifi_stat) {
            case STAT_UNPROVISION: {
				config_flag = TRUE;
                flash_scene_flag = FALSE;
                sys_start_timer(wf_stat_dir, 250, TIMER_CYCLE); /* 快闪 */
            }
            break;

            case STAT_AP_STA_UNCFG: {
				 config_flag = TRUE;
                 flash_scene_flag = FALSE;
                 sys_start_timer(wf_stat_dir, 1500, TIMER_CYCLE);   /* 慢闪 */
            }
            break;

            case STAT_STA_DISC:
			case STAT_LOW_POWER:
				 if(IsThisSysTimerRun(wf_stat_dir)){
				 	sys_stop_timer(wf_stat_dir);            /* 灭灯 */
				 }
				 if(wifi_stat == STAT_STA_DISC) {
				 	PR_DEBUG("config_flag:%d",config_flag);     /* 有配置的时候，默认配置,在哪里?  */
				 	if(config_flag == TRUE){
						config_flag = FALSE;
						reset_light_sta();
					}
				    PR_DEBUG("STAT_STA_UNCONN");
                 }else {
                	send_light_data(0, 0, 0, BRIGHT_INIT_VALUE, 0);     /* 低功耗状态下白灯亮 */
                    PR_DEBUG("LOW POWER");
                 }
                 break;
            case STAT_STA_CONN:
            case STAT_AP_STA_CONN:
            case STAT_CLOUD_CONN:
            case STAT_AP_CLOUD_CONN:
                flash_scene_flag = TRUE;        /* 其他联网状态下正常是否亮灯 */
                break;
            default:
                break;
        }

        last_wf_stat = wifi_stat;
    }
}

/*  上传数据点到云端 */
STATIC VOID idu_timer_cb(UINT timerID,PVOID pTimerArg)
{
    if((wifi_stat == STAT_CLOUD_CONN) || (wifi_stat == STAT_AP_CLOUD_CONN)){
        init_upload_proc();
        sys_stop_timer(timer_init_dpdata);
    }
}

/* wifi配网指示灯显示 */
STATIC VOID wf_direct_timer_cb(UINT timerID,PVOID pTimerArg)
{
    STATIC INT_T flag = 0;
    if(flag == 0) {
        flag = 1;
		send_light_data(0, 0, 0, 0, 0);
    }else {
        flag = 0;
		send_light_data(0, 0, 0, BRIGHT_INIT_VALUE, 0);
    }

}

STATIC VOID data_save_timer_cb(UINT timerID,PVOID pTimerArg)
{
     dev_inf_set();
	 sys_stop_timer(data_save_timer);
}

/* 配网之后需要重新初始化状态 */
STATIC VOID set_default_dp_data(VOID)
{
	dp_data.SWITCH = TRUE;
	dp_data.BRIGHT= BRIGHT_INIT_VALUE;

	dev_inf_set();
}

STATIC OPERATE_RET dev_inf_set(VOID)
{
    OPERATE_RET op_ret;
    INT_T i = 0;
	CHAR_T *out = NULL;

	cJSON *root_test = NULL;
	root_test = cJSON_CreateObject();
	if(NULL == root_test) {
		PR_ERR("json creat failed");
		goto ERR_EXT;
	}

	cJSON_AddBoolToObject(root_test, "switch", dp_data.SWITCH);
	cJSON_AddNumberToObject(root_test, "bright", dp_data.BRIGHT);

	out=cJSON_PrintUnformatted(root_test);
	cJSON_Delete(root_test);
	if(NULL == out) {
		PR_ERR("cJSON_PrintUnformatted err:");
        Free(out);
		goto ERR_EXT;
	}
	PR_DEBUG("write flash[%s]", out);
	op_ret = kvs_write(DP_DATA_KEY, out, strlen(out));
	if(OPRT_OK != op_ret) {
		PR_ERR("data write flash err: %02x",op_ret);
		Free(out);
		goto ERR_EXT;
	}
	Free(out);
    return OPRT_OK;
ERR_EXT:
	return OPRT_COM_ERROR;
}

STATIC OPERATE_RET dev_inf_get(VOID)
{
    OPERATE_RET op_ret;

    UCHAR *buf;
    UINT_T buf_len;

    buf = Malloc(378);

    if(NULL == buf) {
		PR_ERR("Malloc failed");
		goto ERR_EXT;
	}

    op_ret = kvs_read(DP_DATA_KEY, &buf, &buf_len);
	if(OPRT_OK != op_ret){
		PR_ERR("msf_get_single failed,err:%02x", op_ret);
		Free(buf);
		goto ERR_EXT;
	}
	PR_DEBUG("buf:%s",buf);

    cJSON *root = NULL;
	root = cJSON_Parse(buf);
	if(NULL == root) {
		return OPRT_CJSON_PARSE_ERR;
	}
	Free(buf);

    cJSON *json;

    json = cJSON_GetObjectItem(root,"switch");
	if(NULL == json) {
		dp_data.SWITCH = TRUE;
	}else{
		dp_data.SWITCH = json->valueint;
	}

	if(FALSE == dp_data.SWITCH) {
        CHAR_T *rst_inf = system_get_rst_info();
        CHAR_T rst_num = atoi(rst_inf+23);
        PR_DEBUG("rst_inf->reaso is %d", rst_num);
        if(rst_num == REASON_DEFAULT_RST) {
            dp_data.SWITCH = TRUE;
        }
	}

    json = cJSON_GetObjectItem(root,"bright");
	if(NULL == json) {
		dp_data.BRIGHT = BRIGHT_INIT_VALUE;
	}else{
		dp_data.BRIGHT = json->valueint;
	}

	cJSON_Delete(root);
	return OPRT_OK;

ERR_EXT:
    set_default_dp_data();
    return OPRT_COM_ERROR;
}

/* 上传某个数据点 */
VOID dp_upload_proc(IN CONST TY_RECV_OBJ_DP_S *dp)
{
    cJSON *root_test = NULL;
    OPERATE_RET op_ret;
    CHAR_T *out = NULL;
    DEV_CNTL_N_S *dev_cntl;
    DP_CNTL_S *dp_cntl =  NULL;

    if(NULL == dp) {
        PR_ERR("dp error");
        return;
    }

    root_test = cJSON_CreateObject();
	if(NULL == root_test) {
		return;
	}

    UCHAR_T nxt = dp->dps_cnt;

    for (UCHAR_T i=0;i<nxt;i++){
        switch(dp->dps[i].dpid){
            case DPID_1:
                cJSON_AddBoolToObject(root_test, "1", dp_data.SWITCH);
                break;

            case DPID_3:
                cJSON_AddNumberToObject(root_test, "3", dp_data.BRIGHT);
                break;

            default:
                break;
        };
    };

    out=cJSON_PrintUnformatted(root_test);
	cJSON_Delete(root_test);
	if(NULL == out) {
		PR_ERR("cJSON_PrintUnformatted err:");
        Free(out);
		return;
	}

	PR_DEBUG("dp_out[%s]", out);
    op_ret = sf_obj_dp_report_async(get_gw_cntl()->gw_if.id, out, FALSE);
    if(OPRT_OK != op_ret) {
        PR_ERR("sf_obj_dp_report err:%02x",op_ret);
        Free(out);
        return;
    }
	Free(out);
	return;

}

/* 上传所有数据点 */
STATIC VOID init_upload_proc(VOID)
{
    cJSON *root_test = NULL;
    OPERATE_RET op_ret;
    CHAR_T *out = NULL;

    DEV_CNTL_N_S *dev_cntl = get_gw_cntl()->dev;
	if( dev_cntl == NULL )
		return;
	DP_CNTL_S *dp_cntl =  NULL;
	dp_cntl = &dev_cntl->dp[1];

	root_test = cJSON_CreateObject();
	if(NULL == root_test) {
		return;
	}

	cJSON_AddBoolToObject(root_test, "1", dp_data.SWITCH);
    cJSON_AddNumberToObject(root_test, "3", dp_data.BRIGHT);

    out=cJSON_PrintUnformatted(root_test);
	cJSON_Delete(root_test);
	if(NULL == out) {
		PR_ERR("cJSON_PrintUnformatted err:");
        Free(out);
		return;
	}
	PR_DEBUG("out[%s]", out);
    op_ret = sf_obj_dp_report_async(get_gw_cntl()->gw_if.id, out, FALSE);
    if(OPRT_OK != op_ret) {
        PR_ERR("sf_obj_dp_report err:%02x",op_ret);
        Free(out);
        return;
    }
	Free(out);
	return;

}

STATIC VOID light_gra_change(PVOID pArg)
{
    signed int delata_white = 0;
	signed int delata_warm = 0;
    UCHAR MAX_VALUE;
	STATIC FLOAT_T w_scale;
	STATIC FLOAT_T ww_scale;
	UINT_T WHITE_GRA_STEP = 1;
	UINT_T WARM_GRA_STEP = 1;
    PR_DEBUG("%s",__FUNCTION__);

	while(1)
	{
	    WaitSemaphore(gra_change.gra_key_sem);
		send_light_data(0x00, 0x00, 0x00, light_data.FIN_WHITE_VAL, 0x00);
		gtimer_stop(&hw_timer);
	}
}

STATIC VOID flash_scene_change(PVOID pArg)
{
	UCHAR_T require_time;
    PR_DEBUG("%s",__FUNCTION__);
	while(1)
	{
		MutexLock(flash_scene_handle.mutex);
		if(dp_data.SWITCH == TRUE && flash_scene_flag == TRUE ){
            ;
		}
		MutexUnLock(flash_scene_handle.mutex);
		SystemSleep(20);
	}
}

STATIC OPERATE_RET light_handler_init(VOID)
{
    OPERATE_RET op_ret;

    gtimer_init(&hw_timer, TIMER2);
    gtimer_start_periodical(&hw_timer, 1000, (void*)hw_test_timer_cb, (uint32_t)&hw_timer);
    gtimer_stop(&hw_timer);

    //register device
    op_ret = kvs_write(DEVICE_MOD, DEVICE_PART, strlen(DEVICE_PART));
    if(op_ret != OPRT_OK) {
		PR_ERR("msf_register_module err:%02x",op_ret);
		return op_ret;
    }

    op_ret = CreateMutexAndInit(&flash_scene_handle.mutex);
    if(op_ret != OPRT_OK) {
        return op_ret;
    }

    op_ret = CreateAndInitSemaphore(&gra_change.gra_key_sem, 0, 1);
    if(OPRT_OK != op_ret) {
        return op_ret;
    }

    THRD_PARAM_S thrd_param;
    thrd_param.stackDepth = 1024+512;
    thrd_param.priority = TRD_PRIO_2;
    thrd_param.thrdname = "gra_task";
    op_ret = CreateAndStart(&gra_change.gra_thread, NULL, NULL, light_gra_change, NULL, &thrd_param);
    if(op_ret != OPRT_OK) {
        return op_ret;
    }

    thrd_param.stackDepth = 1024+512;
    thrd_param.priority = TRD_PRIO_2;
    thrd_param.thrdname = "flash_scene_task";
    op_ret = CreateAndStart(&flash_scene_handle.flash_scene_thread, NULL, NULL, flash_scene_change, NULL, &thrd_param);
	if(op_ret != OPRT_OK) {
        return op_ret;
    }

    //dev_inf_get();

    op_ret = sys_add_timer(wfl_timer_cb,NULL,&timer);
    if(OPRT_OK != op_ret) {
        return op_ret;
    }else {
        //sys_start_timer(timer,300,TIMER_CYCLE);//turn to get_wf_gw_status
    }

    op_ret = sys_add_timer(idu_timer_cb,NULL,&timer_init_dpdata);
    if(OPRT_OK != op_ret) {
        return op_ret;
    }else {
        sys_start_timer(timer_init_dpdata,300,TIMER_CYCLE);
    }

    #if 0
    op_ret = sys_add_timer(wf_direct_timer_cb,NULL,&wf_stat_dir);
    if(OPRT_OK != op_ret) {
        return op_ret;
    }
    #endif

	op_ret = sys_add_timer(data_save_timer_cb,NULL,&data_save_timer);
    if(OPRT_OK != op_ret) {
        return op_ret;
    }

    return OPRT_OK;
}

VOID reset_light_sta(VOID)
{
    //read flash datas
    dev_inf_get();

    if(dp_data.SWITCH == TRUE){

		get_light_data();

        light_data.LAST_WHITE_VAL = light_data.FIN_WHITE_VAL/RESO_VAL;
		//light_data.LAST_WARM_VAL = light_data.FIN_WARM_VAL/RESO_VAL;
		send_light_data(0x00, 0x00, 0x00, light_data.FIN_WHITE_VAL, 0x00);
	}
}

STATIC VOID reset_fsw_cnt_cb(UINT timerID,PVOID pTimerArg)
{
    PR_DEBUG("%s",__FUNCTION__);

    set_reset_cnt(0);
}

STATIC VOID hw_test_timer_cb(uint32_t id)
{
	PostSemaphore(gra_change.gra_key_sem);
}

OPERATE_RET set_reset_cnt(INT_T val)
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

INT_T get_reset_cnt(VOID)
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

VOID light_wf_gw_status(IN CONST GW_WIFI_NW_STAT_E stat)
{
    wifi_stat = stat;
    PR_NOTICE("wifi_stat:%d",wifi_stat);

    if((STAT_UNPROVISION == wifi_stat)||(STAT_AP_STA_UNCFG == wifi_stat)){
		set_default_dp_data();
	}

    sys_start_timer(timer,300,TIMER_ONCE);
}

UCHAR_T g_rst_num = 0xff;
VOID dev_reset_judge(VOID)
{
    OPERATE_RET op_ret;
    UCHAR_T rst_cnt;
    UCHAR_T rst_num;

    CHAR_T *rst_inf = system_get_rst_info();
    PR_NOTICE("%s", rst_inf);
    rst_num = atoi(rst_inf+23);
    PR_DEBUG("rst_inf:%d", rst_num);
    g_rst_num = rst_num;

    if(rst_num == REASON_DEFAULT_RST){
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
	}
}


OPERATE_RET app_light_init(VOID)
{
    OPERATE_RET op_ret;

    if(get_reset_cnt() >= 5)
	{
		return;
	}

    //light_hardware_init();

    op_ret = light_handler_init();
    if(OPRT_OK != op_ret)
        return op_ret;

    //reset_light_sta();
    //dev_reset_judge();
}

void power_up_count_judge_smcfg (void)
{
	if(get_reset_cnt() >= 5){
		set_reset_cnt(0);
		set_default_dp_data();
		PR_INFO("**********************gw_wifi_reset(WRT_AUTO)******************");
		//gw_wifi_reset(WRT_AUTO);
		 tuya_iot_wf_gw_unactive(WRT_AUTO);
		PR_INFO("**********************gw_wifi_reset(WRT_AUTO)*****************");
	}
}

void pre_light_init (void)
{
	OPERATE_RET op_ret = OPRT_OK;
	op_ret = sys_add_timer(wf_direct_timer_cb,NULL,&wf_stat_dir);
	if(OPRT_OK != op_ret) {
		return op_ret;
	}

    light_hardware_init();
	if(get_reset_cnt() >= 5)
	{
        send_light_data(0, 0, 0, BRIGHT_INIT_VALUE/8, 0);
	    sys_start_timer(wf_stat_dir, 250, TIMER_CYCLE);
	}
	else
	{
		op_ret = wd_gw_wsm_read(&(get_gw_cntl()->gw_wsm));

		if (get_gw_cntl()->gw_wsm.nc_tp == GWNS_UNCFG_SMC)
		{
			send_light_data(0, 0, 0, BRIGHT_INIT_VALUE/8, 0);
			sys_start_timer(wf_stat_dir, 250, TIMER_CYCLE);
		}
		else if (get_gw_cntl()->gw_wsm.nc_tp == GWNS_UNCFG_AP)
		{
			send_light_data(0, 0, 0, BRIGHT_INIT_VALUE/8, 0);
			sys_start_timer(wf_stat_dir, 1500, TIMER_CYCLE);
		}
		else
		{
			reset_light_sta();

            #if 0
            switch(dp_data.WORK_MODE)
    		{
    			case WHITE_MODE:
    			case COLOUR_MODE:
    			case SCENE_MODE:
                    break;

                default:
                    //xTaskCreate(pre_app_init_theard, ((const char*)"pre_app_init_theard"), 512, NULL, tskIDLE_PRIORITY + 1 + PRIORITIE_OFFSET, NULL);
                    send_light_data(flash_light_data.data_group[0].RED_VAL, flash_light_data.data_group[0].GREEN_VAL, flash_light_data.data_group[0].BLUE_VAL,0x00,0x00);
                    break;
            }
            #endif
		}
	}
}



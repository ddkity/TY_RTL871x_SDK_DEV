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

#include "ir_remote.h"

//#include "ir_proc.h"
//#include "ir_proc_uart.h"

/***********************************************************
*************************micro define***********************
***********************************************************/
#define APP_DEBUG   0

/*******************light pwm io define********************/
LED_HANDLE wf_light = NULL;
BYTE prod_test_state = 0;

GW_WIFI_NW_STAT_E wifi_stat;


#define PIN_DI              13
#define PIN_DCKI            15

#define DPID_1               1
#define DPID_2               2
#define DPID_3               3
#define DPID_4               4
#define DPID_5               5
#define DPID_6               6
#define DPID_7               7
#define DPID_8               8
#define DPID_9               9
#define DPID_10              10


typedef enum {
    REASON_DEFAULT_RST = 0,         /**< normal startup by power on */
//    REASON_WDT_RST,             /**< hardware watch dog reset */
//    REASON_EXCEPTION_RST,       /**< exception reset, GPIO status won't change */
//    REASON_SOFT_WDT_RST,        /**< software watch dog reset, GPIO status won't change */
    REASON_SOFT_RESTART = 9,        /**< software restart ,system_restart , GPIO status won't change */
//    REASON_DEEP_SLEEP_AWAKE,    /**< wake up from deep-sleep */
//    REASON_EXT_SYS_RST          /**< external system reset */
} rst_reason;

TIMER_ID wf_stat_dir;
TIMER_ID timer_init_dpdata;
TIMER_ID gradua_timer;
TIMER_ID timer;
TIMER_ID data_save_timer;

STATIC UINT_T irq_cnt = 0;
STATIC UINT_T num_cnt = 0;
STATIC INT_T flash_dir = 0;

BOOL sta_cha_flag = FALSE;

//hw_timer_define
STATIC gtimer_t hw_timer;

VOID prod_test(BOOL_T flag, CHAR_T rssi);
VOID light_init(VOID);
VOID reset_light_sta(VOID);
VOID dev_reset_judge(VOID);
VOID pwm_init(u32 period, u32 *duty, u32 pwm_channel_num, u32*pin_info_list);
void pwm_set_duty(u32 duty, u8 channel);
VOID device_cb(IN CONST TY_RECV_OBJ_DP_S *dp);
VOID device_raw_cb(IN CONST TY_RECV_RAW_DP_S *dp);
STATIC VOID send_light_data(u8 R_value, u8 G_value, u8 B_value, u8 CW_value, u8 WW_value);
STATIC VOID get_light_data(VOID);
STATIC OPERATE_RET dev_inf_set(VOID);
STATIC OPERATE_RET dev_inf_get(VOID);
STATIC OPERATE_RET device_differ_init(VOID);
STATIC VOID get_wf_gw_status(IN CONST GW_WIFI_NW_STAT_E stat);
STATIC VOID init_upload_proc(VOID);
STATIC VOID set_default_dp_data(VOID);
STATIC INT_T get_reset_cnt(VOID);
STATIC OPERATE_RET set_reset_cnt(INT_T val);
STATIC VOID sl_datapoint_proc(TY_OBJ_DP_S root);
STATIC VOID start_gra_change(TIME_MS delay_time);
STATIC VOID hw_test_timer_cb(uint32_t id);
STATIC dp_upload_proc(IN CONST TY_RECV_OBJ_DP_S *dp);
STATIC VOID key_process(TY_GPIO_PORT_E gpio_no, PUSH_KEY_TYPE_E type, INT_T cnt);
STATIC OPERATE_RET sl_datapoint_trans(cJSON *root, TY_OBJ_DP_S dps);



/*********************trans function************************/
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

STATIC INT_T ty_get_enum_id(UCHAR dpid, UCHAR *enum_str)
{
	UCHAR i = 0;
	UCHAR enum_id = 0;
	DP_CNTL_S *dp_cntl =  NULL;	
	DEV_CNTL_N_S *dev_cntl = get_gw_cntl()->dev;

	for(i = 0; i < dev_cntl->dp_num; i++) {
		if(dev_cntl->dp[i].dp_desc.dp_id == dpid) {
			dp_cntl = &dev_cntl->dp[i];
			break;
		}
	}

	if(i >= dev_cntl->dp_num) {
		PR_ERR("not find enum_str");
		return -1;
	}

	if( dp_cntl == NULL ) {
		PR_ERR("dp_cntl is NULL");
		return -1;
	}

	for( i = 0; i < dp_cntl->prop.prop_enum.cnt; i++ )
	{
		if( strcmp(enum_str, dp_cntl->prop.prop_enum.pp_enum[i]) == 0 )
			break;
	}

	return i;	
}

STATIC UCHAR *ty_get_enum_str(DP_CNTL_S *dp_cntl, UCHAR enum_id)
{
	if( dp_cntl == NULL ) {
		return NULL;
	}

	if( enum_id >= dp_cntl->prop.prop_enum.cnt ) {
		return NULL;
	}
	
	return dp_cntl->prop.prop_enum.pp_enum[enum_id];	
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

VOID app_init(VOID)
{
    OPERATE_RET op_ret;

    // create led handle
    op_ret = tuya_create_led_handle(WF_DIR_LED, TRUE, &wf_light);
    if(OPRT_OK  != op_ret) {
        return;
    }

    app_cfg_set(GWCM_LOW_POWER, prod_test);

    //op_ret = device_differ_init();
    if(op_ret != OPRT_OK) {
        return;
    }
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
        device_raw_cb,\
        NULL,\
        NULL,
    };

    op_ret = tuya_iot_wf_soc_dev_init(0,&wf_cbs,PRODUCT_KEY,DEV_SW_VERSION);
    if(OPRT_OK != op_ret) {
        PR_ERR("tuya_iot_wf_soc_dev_init err:%02x",op_ret);
        return OPRT_COM_ERROR;
    }

    op_ret = device_differ_init();
    if(op_ret != OPRT_OK) {
        return;
    }

    /* 初始化IR */
    op_ret = ir_remote_init();

    return op_ret;
}

VOID device_cb(IN CONST TY_RECV_OBJ_DP_S *dp)
{
    cJSON *root = NULL;
    OPERATE_RET op_ret;
    
    if(NULL == dp) {
        PR_ERR("dp error");
        return;
    }

    root = cJSON_CreateObject();
	if(NULL == root) {
		PR_ERR("json creat failed");
		return;
	}
    
    UCHAR_T nxt = dp->dps_cnt;
    PR_DEBUG("dp_cnt:%d", nxt);

    for (UCHAR_T i=0;i<nxt;i++){
        op_ret = sl_datapoint_trans(root, dp->dps[i]);
        if (op_ret != OPRT_OK){
            cJSON_Delete(root);
            return;
        }
    };

    CHAR_T *buf = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if(NULL == buf) {
        PR_ERR("cJSON_PrintUnformatted err:");
        Free(buf);
        return;
    }

    PR_DEBUG("the receive buf is %s",buf);
    ir_dp_handle(buf);

    Free(buf);
}

VOID device_raw_cb(IN CONST TY_RECV_RAW_DP_S *dp)
{
    cJSON *root = NULL;
    CHAR_T dps_id[5];

    if(NULL == dp) {
        PR_ERR("dp error");
        return;
    }

    UINT_T base64_len = 2*dp->len +1;
    CHAR_T *base64_buf = Malloc(base64_len);
    if(NULL == base64_buf) {
        return OPRT_MALLOC_FAILED;
    }

    root = cJSON_CreateObject();
	if(NULL == root) {
        Free(base64_buf);
		PR_ERR("json creat failed");
		return;
	}

    INT_T status = base64_encode(base64_buf, &base64_len, dp->data, dp->len);
    PR_DEBUG("base64_encode:%d", status);
    
    sprintf(dps_id, "%d", dp->dpid);
    
    cJSON_AddNumberToObject(root, "1", 3);
    cJSON_AddStringToObject(root, dps_id, base64_buf);

    CHAR_T *buf = cJSON_PrintUnformatted(root);
    Free(base64_buf);
    cJSON_Delete(root);
    if(NULL == buf) {
        PR_ERR("cJSON_PrintUnformatted err:");
        Free(buf);
        return;
    }

    PR_DEBUG("the receive buf is %s",buf);
    ir_dp_handle(buf);

    Free(buf);
}

STATIC VOID get_wf_gw_status(IN CONST GW_WIFI_NW_STAT_E stat)
{
    wifi_stat = stat;
    PR_NOTICE("wifi_stat:%d",wifi_stat);

    return;
}

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
				PR_DEBUG("STAT_UNPROVISION");
                tuya_set_led_light_type(wf_light,OL_FLASH_HIGH,250,0xffff);
                }
                break;  
            
            case STAT_AP_STA_UNCFG:
            case STAT_AP_STA_DISC:
            case STAT_AP_STA_CONN: {
				 PR_DEBUG("STAT_AP_STA_UNCONN");
                 tuya_set_led_light_type(wf_light,OL_FLASH_HIGH,1500,0xffff);
                 }
                 break;
            
            case STAT_STA_DISC:
			case STAT_LOW_POWER:
				 if(wifi_stat == STAT_STA_DISC) {
				 	PR_DEBUG("STAT_STA_UNCONN");
                    tuya_set_led_light_type(wf_light,OL_HIGH,0,0);
                 }else {
                	PR_DEBUG("LOW POWER");
                    tuya_set_led_light_type(wf_light,OL_HIGH,0,0);
                 }
                 break;
            case STAT_STA_CONN: 
            case STAT_CLOUD_CONN:
            case STAT_AP_CLOUD_CONN: {
                PR_DEBUG("STAT_STA_CONN");
                tuya_set_led_light_type(wf_light,OL_LOW,0,0);
                }
                break;
            default:
                break;
        }
        last_wf_stat = wifi_stat;
    }
}

STATIC OPERATE_RET device_differ_init(VOID)
{
    OPERATE_RET op_ret;    
    
    op_ret = tuya_iot_reg_get_wf_nw_stat_cb(get_wf_gw_status);
    if(OPRT_OK != op_ret) {
        PR_ERR("tuya_iot_reg_get_wf_nw_stat_cb err:%02x",op_ret);
        return op_ret;
    }

    // key process init
    op_ret = key_init(NULL,0,25);
    if(OPRT_OK  != op_ret) {
        return op_ret;
    }

    // register key to process
    KEY_USER_DEF_S key_cfg;
    key_cfg.port = WF_RESET_KEY;
    key_cfg.long_key_time = 3000;
    key_cfg.call_back = key_process;
    key_cfg.low_level_detect = TRUE;
    key_cfg.lp_tp  = LP_ONCE_TRIG;
    key_cfg.seq_key_detect_time = 400;
    op_ret = reg_proc_key(&key_cfg);
    if(OPRT_OK  != op_ret) {
        return op_ret;
    }
    
    op_ret = sys_add_timer(wfl_timer_cb,NULL,&timer);
    if(OPRT_OK != op_ret) {
        return op_ret;
    }else {
        sys_start_timer(timer,300,TIMER_CYCLE);
    }
    
    return OPRT_OK;
}


VOID prod_test(BOOL_T flag, CHAR_T rssi)
{
    
}


STATIC OPERATE_RET sl_datapoint_trans(cJSON *root, TY_OBJ_DP_S dps)
{
    CHAR_T dps_id[5];

    if ((dps.dpid == 1) && (dps.value.dp_enum == 3)){
        return OPRT_COM_ERROR;
    }
    
    snprintf(dps_id, 5, "%d", dps.dpid);
    PR_DEBUG("dp%d_type:%d", dps.dpid, dps.type);
    
    if (dps.type == PROP_BOOL){
        cJSON_AddBoolToObject(root, dps_id, dps.value.dp_bool);
    }else if (dps.type == PROP_VALUE){
        cJSON_AddNumberToObject(root, dps_id, dps.value.dp_value);
    }else if (dps.type == PROP_STR){
        cJSON_AddStringToObject(root, dps_id, dps.value.dp_str);
    }else if (dps.type == PROP_ENUM){
        cJSON_AddNumberToObject(root, dps_id, dps.value.dp_enum);
    }else if (dps.type == PROP_BITMAP){
        cJSON_AddNumberToObject(root, dps_id, dps.value.dp_bitmap);
    }

    return OPRT_OK;
}

STATIC VOID key_process(TY_GPIO_PORT_E gpio_no, PUSH_KEY_TYPE_E type, INT_T cnt)
{
    PR_DEBUG("gpio_no: %d",gpio_no);
    PR_DEBUG("type: %d",type);
    PR_DEBUG("cnt: %d",cnt);


    if(WF_RESET_KEY == gpio_no) {
        if(LONG_KEY == type) {
            gw_wifi_reset(WRT_AUTO);
        }else if(SEQ_KEY == type && cnt >= 5) { // data restore factory            
            //auto_select_wf_cfg();
        }else if(SEQ_KEY == type && cnt == 2) {
            ShowSysMemPoolInfo();
        }else if(SEQ_KEY == type && cnt == 3) {
            //system_print_meminfo();
            PR_NOTICE("remain size:%d",SysGetHeapSize());
        }
        else if(NORMAL_KEY == type) {
            PR_NOTICE("remain size:%d",SysGetHeapSize());

            ir_study_start();
            /*if(prod_test_state)
            {
                if (ir_busy_flag == 0){
                    if (!IsThisSysTimerRun(prod_test_timer)){
                        sys_start_timer(prod_test_timer, 300, TIMER_CYCLE);
                    }else{
                        PR_DEBUG("timer is alreadly run");
                    }
                }else{
                    PR_DEBUG("ir_status is busy");
                }
                button_cb();
            } else {
                
                GW_WIFI_STAT_E wf_stat = get_wf_gw_status();

                if(wf_stat == STAT_STA_CONN) {
                    ir_send_led_start();
                    sys_start_timer(power_stat, 3000, TIMER_ONCE);
                }
                #if 0
                //测试代码,用于按键进入学习功能.
                GW_WIFI_STAT_E wf_stat = get_wf_gw_status();
                if(wf_stat == STAT_STA_CONN)
                    ir_study_start();
                #endif
            }*/
         }
    }
}




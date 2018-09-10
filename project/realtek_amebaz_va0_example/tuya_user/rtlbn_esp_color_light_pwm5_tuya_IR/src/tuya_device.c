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

#include "irDecode.h"
 
/***********************************************************
*************************micro define***********************
***********************************************************/
#define APP_DEBUG   0

/*******************light pwm io define********************/
#define PWM_0_OUT_IO_NUM PA_14  //0channel--R

#define PWM_1_OUT_IO_NUM PA_15  //1channel--G

#define PWM_2_OUT_IO_NUM PA_22  //5channel--B

#define PWM_3_OUT_IO_NUM PA_5   //4channel--WC

#define PWM_4_OUT_IO_NUM PA_12  //3channel--WW

                    
#define CHAN_NUM    5
#define PWM_PERIOD  1000
#define PWM_MIN     0
#define PWM_MAX     PWM_PERIOD
                    
u32 duty[CHAN_NUM] = {100};
u32 pwm_val = 0;

u32 io_info[CHAN_NUM] ={
    PWM_0_OUT_IO_NUM,
    PWM_1_OUT_IO_NUM,
    PWM_2_OUT_IO_NUM,
    PWM_3_OUT_IO_NUM,
    PWM_4_OUT_IO_NUM,
};

typedef enum {
    FUC_TEST1 = 0,
	AGING_TEST,
	FUC_TEST2,
}TEST_MODE_DEF;

typedef enum {
	FUN_SUC = 0,
    NO_KEY,
	WIFI_TEST_ERR,
}FUN_TEST_RET;

typedef struct 
{
	UINT pmd_times;
	UINT aging_times;
	UINT aging_tested_time;
//	UINT aging_left_time;
	BOOL wf_test_ret;
	FUN_TEST_RET fun_test_ret;
	TEST_MODE_DEF test_mode;
	TIMER_ID fuc_test_timer;
	TIMER_ID aging_test_timer;
}TEST_DEF;
STATIC TEST_DEF test_handle;
#define AGING_TEST_TIME 30
#define AGING_TEST_C_TIME 10
#define AGING_TEST_W_TIME 10
#define AGING_TEST_RGB_TIME		(AGING_TEST_TIME - AGING_TEST_W_TIME)
#define TIME_SAVE_INTERVAL 1

#define     COLOUR_MODE_DEFAULT         "ff00000000ffff"

#define DEVICE_MOD "device_mod"
#define DEVICE_PART "device_part"
#define DP_DATA_KEY   "dp_data_key"
#define FASE_SW_CNT_KEY "fsw_cnt_key"
#define LIGHT_TEST_KEY   "light_test_key"
#define AGING_TESTED_TIME  "aging_tested_time"

#define BRIGHT_INIT_VALUE 255
#define COL_TEMP_INIT_VALUE 255
#define TEMP_FULL_VALUE 255
#define NORMAL_DELAY_TIME 3
#define RESO_VAL 4

STATIC UCHAR TEST_R_BRGHT = 10;
STATIC UCHAR TEST_G_BRGHT = 10;
STATIC UCHAR TEST_B_BRGHT = 10;
STATIC UCHAR TEST_W_BRGHT = 1;

volatile STATIC BOOL flash_scene_flag = TRUE;

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


typedef struct {
    UCHAR RED_VAL;
    UCHAR GREEN_VAL;
	UCHAR BLUE_VAL;
	UCHAR WHITE_VAL;
	UCHAR WARM_VAL;
    UCHAR LAST_RED_VAL;
    UCHAR LAST_GREEN_VAL;
	UCHAR LAST_BLUE_VAL;
	UCHAR LAST_WHITE_VAL;
	UCHAR LAST_WARM_VAL;
	UCHAR FIN_RED_VAL;
    UCHAR FIN_GREEN_VAL;
	UCHAR FIN_BLUE_VAL;
	UCHAR FIN_WHITE_VAL;
	UCHAR FIN_WARM_VAL;
	USHORT HUE;
	UCHAR SATURATION;
	UCHAR VALUE;
}LIGHT_DATA_DEF;
STATIC LIGHT_DATA_DEF light_data;

typedef enum {
    WHITE_MODE = 0,
    COLOUR_MODE,
    SCENE_MODE,
    ROUGUANG_SCENE,
    BINFENG_SCENE,
    XUANCAI_SCENE,
    BANLAN_SCENE,
}SCENE_MODE_E;

typedef struct 
{
	BOOL scale_flag;
    THRD_HANDLE gra_thread;
    SEM_HANDLE gra_key_sem;
}L_GRA_CHANGE_DEF;
STATIC L_GRA_CHANGE_DEF gra_change;

typedef struct 
{
    INT_T r_delata;
	INT_T g_delata;
	INT_T b_delata;
	MUTEX_HANDLE  mutex;
    THRD_HANDLE flash_scene_thread;
    xSemaphoreHandle flash_scene_sem;
}FLASH_SCENE_HANDLE_DEF;
STATIC FLASH_SCENE_HANDLE_DEF flash_scene_handle;

typedef struct 
{
	BOOL SWITCH;
	SCENE_MODE_E WORK_MODE;
    UCHAR BRIGHT;
	UCHAR COL_TEMPERATURE;
	UCHAR COLOUR_DATA[15];
	UCHAR SCENE_DATA[15];
	UCHAR ROUGUANG_SCENE_DATA[15];
	UCHAR BINFENG_SCENE_DATA[45];
	UCHAR XUANCAI_SCENE_DATA[15];
	UCHAR BANLAN_SCENE_DATA[45];
}DP_DEF;
STATIC DP_DEF dp_data;

typedef struct {
    UCHAR RED_VAL;
	UCHAR GREEN_VAL;
	UCHAR BLUE_VAL;
}DATA_GROUP_DEF;

typedef struct {
    UCHAR BRIGHT;
    UCHAR COL_TEMP;
	UCHAR SPEED;
	UCHAR NUM;
	DATA_GROUP_DEF data_group[6];
}FLASH_LIGHT_DATA_DEF;
STATIC FLASH_LIGHT_DATA_DEF flash_light_data;


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
    light_init();
    reset_light_sta();
    dev_reset_judge();
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
        sl_datapoint_proc(dp->dps[i]);
        op_led = TRUE;
    };
#endif


    if(TRUE == op_led ) {
        if(!IsThisSysTimerRun(data_save_timer)){
        	sys_start_timer(data_save_timer,5000,TIMER_CYCLE);
        }
        op_led = FALSE;
    }

#if 0
    init_upload_proc();//all dp data send
#else
    dp_upload_proc(dp);
#endif
}

STATIC VOID work_mode_change(SCENE_MODE_E mode)
{
	if(dp_data.WORK_MODE == mode){
		;
    }else {
    	dp_data.WORK_MODE = mode;
    	MutexLock(flash_scene_handle.mutex);
		get_light_data();
		switch(mode)
		{
			case WHITE_MODE:
			case COLOUR_MODE:
			case SCENE_MODE:
				start_gra_change(NORMAL_DELAY_TIME);
				break;
			case ROUGUANG_SCENE:
			case BINFENG_SCENE:
			case XUANCAI_SCENE:
			case BANLAN_SCENE:
				if(dp_data.WORK_MODE == BINFENG_SCENE || dp_data.WORK_MODE == XUANCAI_SCENE){
					gtimer_stop(&hw_timer);
				}
				sta_cha_flag = TRUE;
				num_cnt = 0;
				flash_dir = 0;
				light_data.LAST_RED_VAL = 0;
				light_data.LAST_GREEN_VAL = 0;
				light_data.LAST_BLUE_VAL = 0;
				light_data.LAST_WHITE_VAL = 0;
				light_data.LAST_WARM_VAL = 0;
				//send_light_data(0, 0, 0, 0, 0);//quxiao jinhao 0601
				break;
			default:
				break;	
		}
		MutexUnLock(flash_scene_handle.mutex);
    }
}

STATIC VOID start_gra_change(TIME_MS delay_time)
{
	gra_change.scale_flag = FALSE;
    gtimer_reload(&hw_timer, 1000*delay_time);
    gtimer_start(&hw_timer);
}

STATIC VOID sl_datapoint_proc(TY_OBJ_DP_S root)
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
					light_data.RED_VAL = 0;
					light_data.GREEN_VAL = 0;
					light_data.BLUE_VAL = 0;
					light_data.WHITE_VAL = 0;
					light_data.WARM_VAL = 0;
					light_data.FIN_RED_VAL = 0;
					light_data.FIN_GREEN_VAL = 0;
					light_data.FIN_BLUE_VAL = 0;
					light_data.FIN_WHITE_VAL = 0;
					light_data.FIN_WARM_VAL = 0;
					switch(dp_data.WORK_MODE)
					{
						case WHITE_MODE:
						case COLOUR_MODE:
						case SCENE_MODE:
							MutexUnLock(flash_scene_handle.mutex);
							start_gra_change(NORMAL_DELAY_TIME);
							break;
						case ROUGUANG_SCENE:
				        case BANLAN_SCENE:
							gtimer_stop(&hw_timer);
							send_light_data(light_data.RED_VAL, light_data.GREEN_VAL, light_data.BLUE_VAL, light_data.WHITE_VAL, light_data.WARM_VAL);
							MutexUnLock(flash_scene_handle.mutex);
							break;
						default:
							send_light_data(light_data.RED_VAL, light_data.GREEN_VAL, light_data.BLUE_VAL, light_data.WHITE_VAL, light_data.WARM_VAL);
							MutexUnLock(flash_scene_handle.mutex);
							break;	
					}
					
			        break;
			    case cJSON_True:
					//开灯
			        MutexLock(flash_scene_handle.mutex);
					dp_data.SWITCH = TRUE;
					get_light_data();
					switch(dp_data.WORK_MODE)
					{
						case WHITE_MODE:
						case COLOUR_MODE:
						case SCENE_MODE:
							MutexUnLock(flash_scene_handle.mutex);
							start_gra_change(NORMAL_DELAY_TIME);
							break;
						default:
							sta_cha_flag = TRUE;
							num_cnt = 0;
							flash_dir = 0;
							MutexUnLock(flash_scene_handle.mutex);
							break;	
					}
			        break;
			    default:
			        break;
			}
			break;
            
        case DPID_2:
            PR_DEBUG("{\"%d\":\"%d\"}", dpid, root.value.dp_enum);
			if(dp_data.SWITCH== FALSE){
			   break;
			}
            
			work_mode_change(root.value.dp_enum);
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
            
		case DPID_4:
            PR_DEBUG("{\"%d\":\"%d\"}", dpid, root.value.dp_value);
			if(root.value.dp_value < 0 || root.value.dp_value >255){
				PR_ERR("the data length is %d",len);	  
		    }else {
				dp_data.COL_TEMPERATURE = root.value.dp_value;
				get_light_data();
				start_gra_change(NORMAL_DELAY_TIME);
			} 
			break;

        case DPID_5:
            PR_DEBUG("{\"%d\":\"%s\"}", dpid, root.value.dp_str);
			len = strlen(root.value.dp_str);
			if(len != 14){
				PR_ERR("the data length is wrong: %d", len);   
		    }else {
		    	memcpy(dp_data.COLOUR_DATA, root.value.dp_str, len);
				get_light_data();
				start_gra_change(NORMAL_DELAY_TIME);

			} 
		    break;

        case DPID_6:
            PR_DEBUG("{\"%d\":\"%s\"}", dpid, root.value.dp_str);
			len = strlen(root.value.dp_str);
			if(len != 14){
				PR_ERR("the data length is wrong: %d", len);   
		    }else {
		    	memcpy(dp_data.SCENE_DATA, root.value.dp_str, len);
				get_light_data();
				start_gra_change(NORMAL_DELAY_TIME);
			} 
		    break;

        case DPID_7:
            PR_DEBUG("{\"%d\":\"%s\"}", dpid, root.value.dp_str);
			len = strlen(root.value.dp_str);
			if(len != 14){
				PR_ERR("the data length is wrong: %d", len);   
		    }else {
		    	memcpy(dp_data.ROUGUANG_SCENE_DATA, root.value.dp_str, len);
				MutexLock(flash_scene_handle.mutex);
				sta_cha_flag = TRUE;
				num_cnt = 0;
				flash_dir = 0;
				get_light_data();
				MutexUnLock(flash_scene_handle.mutex);
			} 
		break;
            
		case DPID_8:
            PR_DEBUG("{\"%d\":\"%s\"}", dpid, root.value.dp_str);
			len = strlen(root.value.dp_str);
			if(len > 44){
				PR_ERR("the data length is wrong: %d", len);   
		    }else {
		    	memcpy(dp_data.BINFENG_SCENE_DATA, root.value.dp_str, len);
				MutexLock(flash_scene_handle.mutex);
				sta_cha_flag = TRUE;
				num_cnt = 0;
				flash_dir = 0;
				get_light_data();
				MutexUnLock(flash_scene_handle.mutex);
			} 
		break;
            
		case DPID_9:
            PR_DEBUG("{\"%d\":\"%s\"}", dpid, root.value.dp_str);
			len = strlen(root.value.dp_str);
			if(len != 14){
				PR_ERR("the data length is wrong: %d", len);   
		    }else {
		    	memcpy(dp_data.XUANCAI_SCENE_DATA, root.value.dp_str, len);
				MutexLock(flash_scene_handle.mutex);
				sta_cha_flag = TRUE;
				num_cnt = 0;
				flash_dir = 0;
				get_light_data();
				MutexUnLock(flash_scene_handle.mutex);
			} 
		break;
            
		case DPID_10:
            PR_DEBUG("{\"%d\":\"%s\"}", dpid, root.value.dp_str);
			len = strlen(root.value.dp_str);
			if(len > 44){
				PR_ERR("the data length is wrong: %d", len);   
		    }else {
		    	memcpy(dp_data.BANLAN_SCENE_DATA, root.value.dp_str, len);
				MutexLock(flash_scene_handle.mutex);
				sta_cha_flag = TRUE;
				num_cnt = 0;
				flash_dir = 0;
				get_light_data();
				MutexUnLock(flash_scene_handle.mutex);
			} 
		break;
            
        default:
            break;
    }
}

STATIC VOID get_wf_gw_status(IN CONST GW_WIFI_NW_STAT_E stat)
{
    wifi_stat = stat;
    PR_NOTICE("wifi_stat:%d",wifi_stat);

    if((STAT_UNPROVISION == wifi_stat)||(STAT_AP_STA_UNCFG == wifi_stat)){
		set_default_dp_data();
	}
    
    sys_start_timer(timer,300,TIMER_ONCE);

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
				config_flag = TRUE;
                flash_scene_flag = FALSE;
                sys_start_timer(wf_stat_dir, 250, TIMER_CYCLE);
            }
            break;  
            
            case STAT_AP_STA_UNCFG: {
				 config_flag = TRUE;
                 flash_scene_flag = FALSE;
                 sys_start_timer(wf_stat_dir, 1500, TIMER_CYCLE);
            }
            break;
            
            case STAT_STA_DISC:
			case STAT_LOW_POWER:
				 if(IsThisSysTimerRun(wf_stat_dir)){
				 	sys_stop_timer(wf_stat_dir);
				 }
				 if(wifi_stat == STAT_STA_DISC) {
				 	PR_DEBUG("config_flag:%d",config_flag);
				 	if(config_flag == TRUE){
						config_flag = FALSE;
						reset_light_sta();
					}
				    PR_DEBUG("STAT_STA_UNCONN");
                 }else {
                	send_light_data(0, 0, 0, BRIGHT_INIT_VALUE, 0);
                    PR_DEBUG("LOW POWER");
                 }
                 break;
            case STAT_STA_CONN: 
            case STAT_AP_STA_CONN:
            case STAT_CLOUD_CONN:
            case STAT_AP_CLOUD_CONN:
                flash_scene_flag = TRUE;
                break;
            default:
                break;
        }
        last_wf_stat = wifi_stat;
    }
}


STATIC VOID idu_timer_cb(UINT timerID,PVOID pTimerArg)
{
    if((wifi_stat == STAT_CLOUD_CONN) || (wifi_stat == STAT_AP_CLOUD_CONN)){
        init_upload_proc();
        sys_stop_timer(timer_init_dpdata);
    }
}

STATIC VOID wf_direct_timer_cb(UINT timerID,PVOID pTimerArg)
{
    STATIC INT_T flag = 0; 
    if(flag == 0) {
        flag = 1;
		send_light_data(0, 0, 0, 0, 0);
//		gpio_write(&test_gpio_x,0);
    }else {
        flag = 0;
		send_light_data(0, 0, 0, BRIGHT_INIT_VALUE, 0);
//		gpio_write(&test_gpio_x,1);
    }
	
}

STATIC VOID data_save_timer_cb(UINT timerID,PVOID pTimerArg)
{
     dev_inf_set();
	 sys_stop_timer(data_save_timer);
}

STATIC OPERATE_RET device_differ_init(VOID)
{
    OPERATE_RET op_ret;    

	if(get_reset_cnt() >= 3){
		set_reset_cnt(0);
		set_default_dp_data();
		gw_wifi_reset(WRT_AUTO);
	}

    dev_inf_get();
	Infrared_Init();//IR

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
    
    op_ret = sys_add_timer(wf_direct_timer_cb,NULL,&wf_stat_dir);
    if(OPRT_OK != op_ret) {
        return op_ret;
    }

	op_ret = sys_add_timer(data_save_timer_cb,NULL,&data_save_timer);
    if(OPRT_OK != op_ret) {
        return op_ret;
    }
    return OPRT_OK;
}

STATIC dp_upload_proc(IN CONST TY_RECV_OBJ_DP_S *dp)
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
            case DPID_2:
                dev_cntl = get_gw_cntl()->dev;
            	if( dev_cntl == NULL )
            		return;
            	dp_cntl = &dev_cntl->dp[1];
                cJSON_AddStringToObject(root_test,"2",ty_get_enum_str(dp_cntl,(UCHAR)dp_data.WORK_MODE));
                break;
            case DPID_3:
                cJSON_AddNumberToObject(root_test, "3", dp_data.BRIGHT);
                break;
            case DPID_4:
                cJSON_AddNumberToObject(root_test, "4", dp_data.COL_TEMPERATURE);
                break;
            case DPID_5:
                cJSON_AddStringToObject(root_test, "5", dp_data.COLOUR_DATA);
                break;
            case DPID_6:
                cJSON_AddStringToObject(root_test, "6", dp_data.SCENE_DATA);
                break;
            case DPID_7:
                cJSON_AddStringToObject(root_test, "7", dp_data.ROUGUANG_SCENE_DATA);
                break;
            case DPID_8:
                cJSON_AddStringToObject(root_test, "8", dp_data.BINFENG_SCENE_DATA);
                break;
            case DPID_9:
                cJSON_AddStringToObject(root_test, "9", dp_data.XUANCAI_SCENE_DATA);
                break;
            case DPID_10:
                cJSON_AddStringToObject(root_test, "10", dp_data.BANLAN_SCENE_DATA);
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
	cJSON_AddStringToObject(root_test,"2",ty_get_enum_str(dp_cntl,(UCHAR)dp_data.WORK_MODE));
    cJSON_AddNumberToObject(root_test, "3", dp_data.BRIGHT);
	cJSON_AddNumberToObject(root_test, "4", dp_data.COL_TEMPERATURE);
	cJSON_AddStringToObject(root_test, "5", dp_data.COLOUR_DATA);
	cJSON_AddStringToObject(root_test, "6", dp_data.SCENE_DATA);
	cJSON_AddStringToObject(root_test, "7", dp_data.ROUGUANG_SCENE_DATA);
	cJSON_AddStringToObject(root_test, "8", dp_data.BINFENG_SCENE_DATA);
	cJSON_AddStringToObject(root_test, "9", dp_data.XUANCAI_SCENE_DATA);
	cJSON_AddStringToObject(root_test, "10", dp_data.BANLAN_SCENE_DATA);
    
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

STATIC OPERATE_RET get_light_test_flag(VOID)
{
	OPERATE_RET op_ret;
    UINT_T buf_len;

	UCHAR *buf;
	buf = Malloc(32);
	if(NULL == buf) {
		PR_ERR("Malloc failed");
		goto ERR_EXT;
	}

	op_ret = kvs_read(LIGHT_TEST_KEY, &buf, &buf_len);
	if(OPRT_OK != op_ret){
		PR_ERR("msf_get_single failed");
		Free(buf);
		goto ERR_EXT;
	}
	PR_DEBUG("buf:%s",buf);
	
	cJSON *root = NULL;
	root = cJSON_Parse(buf);
	if(NULL == root) {
		Free(buf);
		test_handle.test_mode = FUC_TEST1;
		return OPRT_CJSON_PARSE_ERR;
	}
	Free(buf);

	cJSON *json;
	json = cJSON_GetObjectItem(root,"test_mode");
	if(NULL == json) {
		test_handle.test_mode = FUC_TEST1;
	}else{
		test_handle.test_mode = json->valueint;
	}
	
	cJSON_Delete(root);
	return OPRT_OK;
		
ERR_EXT:	
	test_handle.test_mode = FUC_TEST1;
	return OPRT_COM_ERROR;

}

STATIC OPERATE_RET set_light_test_flag(VOID)
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
	
	cJSON_AddNumberToObject(root_test, "test_mode", test_handle.test_mode);
	
	out=cJSON_PrintUnformatted(root_test);
	cJSON_Delete(root_test);
	if(NULL == out) {
		PR_ERR("cJSON_PrintUnformatted err:");
		Free(out);
		goto ERR_EXT;
	}
	//PR_DEBUG("out[%s]", out);
	op_ret = kvs_write(LIGHT_TEST_KEY, out, strlen(out));
	if(OPRT_OK != op_ret) {
		PR_ERR("data write psm err: %d",op_ret);
		Free(out);
		goto ERR_EXT;
	}
	Free(out);
	return OPRT_OK;
ERR_EXT:	
	return OPRT_COM_ERROR;

}


STATIC OPERATE_RET get_aging_tested_time(VOID)
{
	OPERATE_RET op_ret;
    UINT_T buf_len;

	UCHAR *buf;
	buf = Malloc(64);
	if(NULL == buf) {
		PR_ERR("Malloc failed");
		goto ERR_EXT;
	}

	op_ret = kvs_read(AGING_TESTED_TIME, &buf, &buf_len);
	if(OPRT_OK != op_ret){
		PR_ERR("msf_get_single failed");
		Free(buf);
		goto ERR_EXT;
	}
	PR_DEBUG("buf:%s",buf);
	
	cJSON *root = NULL;
	root = cJSON_Parse(buf);
	if(NULL == root) {
		Free(buf);
		test_handle.aging_tested_time = 0;
		return OPRT_CJSON_PARSE_ERR;
	}
	Free(buf);

	cJSON *json;
	json = cJSON_GetObjectItem(root,"aging_tested_time");
	if(NULL == json) {
		test_handle.aging_tested_time = 0;
	}else{
		test_handle.aging_tested_time = json->valueint;
	}

	cJSON_Delete(root);
	return OPRT_OK;
		
ERR_EXT:
	test_handle.aging_tested_time = 0;
	return OPRT_COM_ERROR;

}

STATIC OPERATE_RET set_aging_tested_time(VOID)
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
	
	cJSON_AddNumberToObject(root_test, "aging_tested_time", test_handle.aging_tested_time);
	
	out=cJSON_PrintUnformatted(root_test);
	cJSON_Delete(root_test);
	if(NULL == out) {
		PR_ERR("cJSON_PrintUnformatted err:");
		Free(out);
		goto ERR_EXT;
	}
	PR_DEBUG("out[%s]", out);
	op_ret = kvs_write(AGING_TESTED_TIME, out, strlen(out));
	if(OPRT_OK != op_ret) {
		PR_ERR("data write psm err: %d",op_ret);
		Free(out);
		goto ERR_EXT;
	}
	Free(out);
	return OPRT_OK;
ERR_EXT:	
	return OPRT_COM_ERROR;

}

STATIC VOID aging_test_timer_cb(UINT timerID,PVOID pTimerArg)
{
//	test_handle.aging_times ++;
//	if(test_handle.aging_times % TIME_SAVE_INTERVAL == 0){
		test_handle.aging_tested_time ++;
		if(OPRT_OK != set_aging_tested_time()){
			send_light_data(0x00, 0x00, 0xff, 0x00, 0x00);
			sys_stop_timer(test_handle.aging_test_timer);
		}
//	}
	
	if(test_handle.aging_tested_time == AGING_TEST_TIME){
		sys_stop_timer(test_handle.aging_test_timer);
		send_light_data(0, 0xff, 0, 0, 0);
		test_handle.test_mode = FUC_TEST2;
		test_handle.aging_tested_time = 0;
		if(OPRT_OK != set_light_test_flag()){
			send_light_data(0x00, 0x00, 0xff, 0x00, 0x00);
		}
		
		if(OPRT_OK != set_aging_tested_time()){
			send_light_data(0x00, 0x00, 0xff, 0x00, 0x00);
		}
	}else{
//		if(AGING_TEST_RGB_TIME >= (test_handle.aging_left_time-test_handle.aging_times)){
//			send_light_data(0xff, 0xff, 0xff, 0x00, 0x00);
//		}
        if(test_handle.aging_tested_time >= (AGING_TEST_C_TIME + AGING_TEST_W_TIME)) {
            send_light_data(0xff, 0xff, 0xff, 0x00, 0x00);
        } else if(test_handle.aging_tested_time < AGING_TEST_C_TIME){
            send_light_data(0x00, 0x00, 0x00, 0xff, 0);
        } else if((test_handle.aging_tested_time >= AGING_TEST_C_TIME) && (test_handle.aging_tested_time < (AGING_TEST_C_TIME + AGING_TEST_W_TIME))) {
            send_light_data(0x00, 0x00, 0x00, 0, 0xff);
        }


	}
}

STATIC VOID fuc_test_timer_cb(UINT timerID,PVOID pTimerArg)
{
	test_handle.pmd_times ++;


	if(test_handle.test_mode == FUC_TEST1){
		 switch(test_handle.pmd_times % 6){
			case 1: send_light_data(TEST_R_BRGHT, 0, 0, 0, 0); break;
			case 2: send_light_data(0, TEST_G_BRGHT, 0, 0, 0); break;
			case 3: send_light_data(0, 0, TEST_B_BRGHT, 0, 0); break;
			case 4:	send_light_data(0, 0, 0, 0xff, 0); break;
            case 5: 
			case 0:	send_light_data(0, 0, 0, 0, 0XFF); break;
			default:break;
		}
	 }
	 else{
		switch(test_handle.pmd_times % 11){
		 	case 1: send_light_data(TEST_R_BRGHT, 0, 0, 0, 0); break;
			case 2: send_light_data(0, 0, 0, 0, 0); break;
			case 3: send_light_data(0, TEST_G_BRGHT, 0, 0, 0); break;
			case 4: send_light_data(0, 0, 0, 0, 0); break;
			case 5: send_light_data(0, 0, TEST_B_BRGHT, 0, 0); break;
			case 6: send_light_data(0, 0, 0, 0, 0); break;
			case 7: send_light_data(0, 0, 0, 0xff, 0); break;	
			case 8: send_light_data(0, 0, 0, 0, 0); break;
			case 9: 
			case 10:send_light_data(0, 0, 0, 0, 0xff); break;
			case 0: send_light_data(0, 0, 0, 0, 0); break;
			default:break;
		 }
	 }
	 
	if(test_handle.pmd_times == 120){
		if(test_handle.test_mode == FUC_TEST1){
			test_handle.test_mode = AGING_TEST;
			if(OPRT_OK != set_light_test_flag()) { 
				sys_stop_timer(test_handle.fuc_test_timer);
				send_light_data(0x00, 0x00, 0x00, 0x00, 0x00);
			}else{
				SystemReset();
			}
		}
	}
}


VOID prod_test(BOOL_T flag, CHAR_T rssi)
{
    OPERATE_RET op_ret;
	flash_scene_flag = FALSE;
	PR_DEBUG("rssi:%d", rssi);
	set_reset_cnt(0);
	//prod thread create and start

	if(OPRT_OK != get_light_test_flag()){
		PR_ERR("get_light_test_flag err.......");
	}
	
	if(test_handle.test_mode == AGING_TEST){
		get_aging_tested_time();
		//test_handle.aging_left_time = AGING_TEST_TIME - test_handle.aging_tested_time*TIME_SAVE_INTERVAL;
		op_ret = sys_add_timer(aging_test_timer_cb,NULL,&test_handle.aging_test_timer);
	    if(OPRT_OK != op_ret) {
			send_light_data(0x00, 0x00, 0x00, 0x00, 0x00);
	        return;
	    }
	    
		if(test_handle.aging_tested_time >= (AGING_TEST_C_TIME + AGING_TEST_W_TIME)) {
			send_light_data(0xff, 0xff, 0xff, 0x00, 0x00);
		} else if(test_handle.aging_tested_time < AGING_TEST_C_TIME){
			send_light_data(0x00, 0x00, 0x00, 0xff, 0);
		} else if((test_handle.aging_tested_time >= AGING_TEST_C_TIME) && ((test_handle.aging_tested_time < (AGING_TEST_C_TIME + AGING_TEST_W_TIME)))) {
			send_light_data(0x00, 0x00, 0x00, 0, 0xff);
		}
    	
		test_handle.aging_times = 0;
        sys_start_timer(test_handle.aging_test_timer, 60000, TIMER_CYCLE);
	} else {
		if(rssi < -60 || flag == FALSE) {
			send_light_data(TEST_R_BRGHT, 0, 0, 0, 0);
			return;
	    }
		
		op_ret = sys_add_timer(fuc_test_timer_cb,NULL,&test_handle.fuc_test_timer);
	    if(OPRT_OK != op_ret) {  
			send_light_data(0x00, 0x00, 0x00, 0x00, 0x00);
	        return;
	    }
		test_handle.pmd_times = 0;
		
		/*if(test_handle.test_mode == FUC_TEST1){
			sys_start_timer(test_handle.fuc_test_timer, 1000, TIMER_CYCLE);
		}else{
			sys_start_timer(test_handle.fuc_test_timer, 500, TIMER_CYCLE);
		}*/
		sys_start_timer(test_handle.fuc_test_timer, 1000, TIMER_CYCLE);
	}
	return;	
}

STATIC VOID light_gra_change(PVOID pArg)
{
	signed int delata_red = 0;
	signed int delata_green = 0;
    signed int delata_blue = 0;
    signed int delata_white = 0;
	signed int delata_warm = 0;
    UCHAR MAX_VALUE;
	STATIC FLOAT_T r_scale;
	STATIC FLOAT_T g_scale;
	STATIC FLOAT_T b_scale;
	STATIC FLOAT_T w_scale;
	STATIC FLOAT_T ww_scale;
	UINT_T RED_GRA_STEP = 1;
	UINT_T GREEN_GRA_STEP = 1;
	UINT_T BLUE_GRA_STEP = 1;
	UINT_T WHITE_GRA_STEP = 1;
	UINT_T WARM_GRA_STEP = 1;

    PR_DEBUG("%s",__FUNCTION__);

	while(1)
	{
	    WaitSemaphore(gra_change.gra_key_sem);
		if(light_data.WHITE_VAL != light_data.LAST_WHITE_VAL || light_data.WARM_VAL != light_data.LAST_WARM_VAL || light_data.RED_VAL != light_data.LAST_RED_VAL ||\
			light_data.GREEN_VAL != light_data.LAST_GREEN_VAL || light_data.BLUE_VAL != light_data.LAST_BLUE_VAL)
	    {
			delata_red = light_data.RED_VAL - light_data.LAST_RED_VAL;
			delata_green = light_data.GREEN_VAL - light_data.LAST_GREEN_VAL;
			delata_blue = light_data.BLUE_VAL - light_data.LAST_BLUE_VAL;
			delata_white = light_data.WHITE_VAL - light_data.LAST_WHITE_VAL;
			delata_warm = light_data.WARM_VAL - light_data.LAST_WARM_VAL;
			MAX_VALUE = get_max_value(ABS(delata_red), ABS(delata_green), ABS(delata_blue), ABS(delata_white), ABS(delata_warm));
			//PR_DEBUG("MAX: %d", MAX_VALUE);
			if(gra_change.scale_flag == FALSE){	
				r_scale = ABS(delata_red)/1.0/MAX_VALUE;
				g_scale = ABS(delata_green)/1.0/MAX_VALUE;
				b_scale = ABS(delata_blue)/1.0/MAX_VALUE;
				w_scale = ABS(delata_white)/1.0/MAX_VALUE;
				ww_scale = ABS(delata_warm)/1.0/MAX_VALUE;
				gra_change.scale_flag = TRUE;
			}
			if(MAX_VALUE == ABS(delata_red)){
				RED_GRA_STEP = 1;
			}else{
				RED_GRA_STEP =  ABS(delata_red) - MAX_VALUE*r_scale;
			}
			if(MAX_VALUE == ABS(delata_green)){
				GREEN_GRA_STEP = 1;
			}else{
				GREEN_GRA_STEP =  ABS(delata_green) - MAX_VALUE*g_scale;
			}
			if(MAX_VALUE == ABS(delata_blue)){
				BLUE_GRA_STEP = 1;
			}else{
				BLUE_GRA_STEP =  ABS(delata_blue) - MAX_VALUE*b_scale;
			}
			if(MAX_VALUE == ABS(delata_white)){
				WHITE_GRA_STEP = 1;
			}else{
				WHITE_GRA_STEP =  ABS(delata_white) - MAX_VALUE*w_scale;
			}
			if(MAX_VALUE == ABS(delata_warm)){
				WARM_GRA_STEP = 1;
			}else{
				WARM_GRA_STEP =  ABS(delata_warm) - MAX_VALUE*ww_scale;
			}

			if(delata_red != 0){
			    if(ABS(delata_red) < RED_GRA_STEP)
			    { 
					 light_data.LAST_RED_VAL += delata_red;
				}else{
					if(delata_red < 0)
						light_data.LAST_RED_VAL -= RED_GRA_STEP;
					else
						light_data.LAST_RED_VAL += RED_GRA_STEP;
				}
			}
			if(delata_green != 0){
			    if(ABS(delata_green) < GREEN_GRA_STEP)
			    { 
					 light_data.LAST_GREEN_VAL += delata_green;
				}else{
					if(delata_green < 0)
						light_data.LAST_GREEN_VAL -= GREEN_GRA_STEP;
					else
						light_data.LAST_GREEN_VAL += GREEN_GRA_STEP;
				}
			}
			if(delata_blue != 0){
			    if(ABS(delata_blue) < BLUE_GRA_STEP)
			    { 
					 light_data.LAST_BLUE_VAL += delata_blue;
				}else{
					if(delata_blue < 0)
						light_data.LAST_BLUE_VAL -= BLUE_GRA_STEP;
					else
						light_data.LAST_BLUE_VAL += BLUE_GRA_STEP;
				}
			}
			if(delata_white != 0){
			    if(ABS(delata_white) < WHITE_GRA_STEP)
			    { 
					 light_data.LAST_WHITE_VAL += delata_white;
				}else{
					if(delata_white < 0)
						light_data.LAST_WHITE_VAL -= WHITE_GRA_STEP;
					else
						light_data.LAST_WHITE_VAL += WHITE_GRA_STEP;
				}
			}
			if(delata_warm != 0){
			    if(ABS(delata_warm) < WARM_GRA_STEP)
			    { 
					 light_data.LAST_WARM_VAL += delata_warm;
				}else{
					if(delata_warm < 0)
						light_data.LAST_WARM_VAL -= WARM_GRA_STEP;
					else
						light_data.LAST_WARM_VAL += WARM_GRA_STEP;
				}
			}
			//PR_DEBUG("r:%d g:%d b:%d w:%d ",light_data.LAST_RED_VAL,light_data.LAST_GREEN_VAL,light_data.LAST_BLUE_VAL,light_data.LAST_WHITE_VAL);
			if(dp_data.SWITCH == FALSE && (dp_data.WORK_MODE == ROUGUANG_SCENE || dp_data.WORK_MODE == BANLAN_SCENE)){
				;
			}else{
				MutexLock(flash_scene_handle.mutex);
				if(dp_data.WORK_MODE != BINFENG_SCENE && dp_data.WORK_MODE != XUANCAI_SCENE)
					send_light_data(light_data.LAST_RED_VAL*RESO_VAL, light_data.LAST_GREEN_VAL*RESO_VAL, light_data.LAST_BLUE_VAL*RESO_VAL, light_data.LAST_WHITE_VAL*RESO_VAL, light_data.LAST_WARM_VAL*RESO_VAL);
				MutexUnLock(flash_scene_handle.mutex);
			}
		}else{
			if(dp_data.WORK_MODE != ROUGUANG_SCENE && dp_data.WORK_MODE != BANLAN_SCENE){		
				send_light_data(light_data.FIN_RED_VAL, light_data.FIN_GREEN_VAL, light_data.FIN_BLUE_VAL, light_data.FIN_WHITE_VAL, light_data.FIN_WARM_VAL);
				//PR_DEBUG("R:%d G:%d B:%d W:%d WW:%d",light_data.FIN_RED_VAL, light_data.FIN_GREEN_VAL, light_data.FIN_BLUE_VAL, light_data.FIN_WHITE_VAL, light_data.FIN_WARM_VAL);
				gtimer_stop(&hw_timer);
			}
		}
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
			switch(dp_data.WORK_MODE){
				case ROUGUANG_SCENE:
				case BANLAN_SCENE:
					if(dp_data.WORK_MODE == ROUGUANG_SCENE){
						require_time = flash_light_data.SPEED*2+15;
					}else{
						require_time = flash_light_data.SPEED+10;
					}
					if(sta_cha_flag == TRUE){
						irq_cnt = require_time;
						sta_cha_flag = FALSE;
					}else{
						irq_cnt ++;
					}
					if(irq_cnt >= require_time){
						if(flash_light_data.NUM == 1){
							if(flash_dir == 0){
								flash_dir = 1;
								light_data.FIN_RED_VAL = flash_light_data.data_group[0].RED_VAL;
								light_data.FIN_GREEN_VAL = flash_light_data.data_group[0].GREEN_VAL;
								light_data.FIN_BLUE_VAL = flash_light_data.data_group[0].BLUE_VAL;
								light_data.RED_VAL = light_data.FIN_RED_VAL/RESO_VAL;
								light_data.GREEN_VAL = light_data.FIN_GREEN_VAL/RESO_VAL;
								light_data.BLUE_VAL = light_data.FIN_BLUE_VAL/RESO_VAL;
							}else{
								flash_dir = 0;
								light_data.FIN_RED_VAL = 0;
								light_data.FIN_GREEN_VAL = 0;
								light_data.FIN_BLUE_VAL = 0;
								light_data.RED_VAL = 0;
								light_data.GREEN_VAL = 0;
								light_data.BLUE_VAL = 0;
							}
						}else{
							light_data.FIN_RED_VAL = flash_light_data.data_group[num_cnt].RED_VAL;
							light_data.FIN_GREEN_VAL = flash_light_data.data_group[num_cnt].GREEN_VAL;
							light_data.FIN_BLUE_VAL = flash_light_data.data_group[num_cnt].BLUE_VAL;
							light_data.RED_VAL = light_data.FIN_RED_VAL/RESO_VAL;
							light_data.GREEN_VAL = light_data.FIN_GREEN_VAL/RESO_VAL;
							light_data.BLUE_VAL = light_data.FIN_BLUE_VAL/RESO_VAL;
						}
						num_cnt ++;
						if(num_cnt == flash_light_data.NUM)
							 num_cnt = 0;
						irq_cnt = 0;

						if(dp_data.WORK_MODE == ROUGUANG_SCENE){
							start_gra_change(flash_light_data.SPEED*2/5+2+flash_light_data.SPEED*(255-flash_light_data.BRIGHT)/100/4);
						}else{
							start_gra_change(flash_light_data.SPEED/7+3);
						}
					}
					break;
				case BINFENG_SCENE:
				case XUANCAI_SCENE:
					if(dp_data.WORK_MODE == BINFENG_SCENE){
						require_time = flash_light_data.SPEED+7;
					}else{
						require_time = flash_light_data.SPEED+2;
					}
					if(sta_cha_flag == TRUE){
						irq_cnt = require_time;
						sta_cha_flag = FALSE;
					}else{
						irq_cnt ++;
					}
					if(irq_cnt >= require_time){
						if(flash_light_data.NUM == 1){
							if(flash_dir == 0){
								if(dp_data.SWITCH == TRUE)
									send_light_data(flash_light_data.data_group[0].RED_VAL, flash_light_data.data_group[0].GREEN_VAL, \
												flash_light_data.data_group[0].BLUE_VAL, 0, 0);
								flash_dir = 1;
							}else{
								flash_dir = 0;
								if(dp_data.SWITCH == TRUE)
									send_light_data(0, 0, 0, 0, 0);
							}
						}else{
							if(dp_data.SWITCH == TRUE)
								send_light_data(flash_light_data.data_group[num_cnt].RED_VAL, flash_light_data.data_group[num_cnt].GREEN_VAL, \
												flash_light_data.data_group[num_cnt].BLUE_VAL, 0, 0);
						}
						num_cnt ++;
						if(num_cnt == flash_light_data.NUM)
							 num_cnt = 0;
						irq_cnt = 0;
					}
					break;
				default:
					break;
			}
		}
		MutexUnLock(flash_scene_handle.mutex);
		SystemSleep(20);
	}
}

STATIC VOID hw_test_timer_cb(uint32_t id)
{
	PostSemaphore(gra_change.gra_key_sem);
}

VOID light_init(VOID)
{
    OPERATE_RET op_ret;

    //prod init
    app_cfg_set(GWCM_SPCL_MODE, prod_test);
//	set_proc_ssid();

    //pwm init
    pwm_init(PWM_PERIOD, &pwm_val, CHAN_NUM, io_info);

    gtimer_init(&hw_timer, TIMER2);
    gtimer_start_periodical(&hw_timer, 1000, (void*)hw_test_timer_cb, (uint32_t)&hw_timer);
    gtimer_stop(&hw_timer);

    //register device
    op_ret = kvs_write(DEVICE_MOD, DEVICE_PART, strlen(DEVICE_PART));
    if(op_ret != OPRT_OK) {
		PR_ERR("msf_register_module err:%02x",op_ret);
		return;
    }
    
    op_ret = CreateMutexAndInit(&flash_scene_handle.mutex);
    if(op_ret != OPRT_OK) {
        return ;
    }

    op_ret = CreateAndInitSemaphore(&gra_change.gra_key_sem, 0, 1);
    if(OPRT_OK != op_ret) {
        return ;
    }

    THRD_PARAM_S thrd_param;
    thrd_param.stackDepth = 1024+512;
    thrd_param.priority = TRD_PRIO_2;
    thrd_param.thrdname = "gra_task";
    op_ret = CreateAndStart(&gra_change.gra_thread, NULL, NULL, light_gra_change, NULL, &thrd_param);
    if(op_ret != OPRT_OK) {
        return ;
    }

    thrd_param.stackDepth = 1024+512;
    thrd_param.priority = TRD_PRIO_2;
    thrd_param.thrdname = "flash_scene_task";
    op_ret = CreateAndStart(&flash_scene_handle.flash_scene_thread, NULL, NULL, flash_scene_change, NULL, &thrd_param);
	if(op_ret != OPRT_OK) {
        return ;
    }
}

VOID pwm_init(u32 period, u32 *duty, u32 pwm_channel_num, u32*pin_info_list)
{
    u8 i;
    STATIC pwmout_t pwm_info;

    for (i=0;i<pwm_channel_num;i++){
        pwmout_init(&pwm_info, pin_info_list[i]);
        pwmout_period_us(&pwm_info, period);
        pwmout_pulsewidth_us(&pwm_info, *duty);
    }
}

STATIC VOID set_default_dp_data(VOID)
{
	dp_data.SWITCH = TRUE;
    dp_data.WORK_MODE = WHITE_MODE;
	dp_data.BRIGHT= BRIGHT_INIT_VALUE; 
	dp_data.COL_TEMPERATURE = COL_TEMP_INIT_VALUE;

    memcpy(dp_data.COLOUR_DATA, COLOUR_MODE_DEFAULT, 14);
	memcpy(dp_data.SCENE_DATA, "00ff0000000000", 14);
	memcpy(dp_data.ROUGUANG_SCENE_DATA, "ffff500100ff00", 14);
	memcpy(dp_data.BINFENG_SCENE_DATA, "ffff8003ff000000ff000000ff000000000000000000", 44);
	memcpy(dp_data.XUANCAI_SCENE_DATA, "ffff5001ff0000", 14);
	memcpy(dp_data.BANLAN_SCENE_DATA, "ffff0505ff000000ff00ffff00ff00ff0000ff000000", 44);
    
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
	cJSON_AddNumberToObject(root_test, "work_mode", dp_data.WORK_MODE);
	cJSON_AddNumberToObject(root_test, "bright", dp_data.BRIGHT);
	cJSON_AddNumberToObject(root_test, "temper", dp_data.COL_TEMPERATURE);
	cJSON_AddStringToObject(root_test, "colour_data", dp_data.COLOUR_DATA);
	cJSON_AddStringToObject(root_test, "scene_data", dp_data.SCENE_DATA);
	cJSON_AddStringToObject(root_test, "rouguang_scene_data", dp_data.ROUGUANG_SCENE_DATA);
	cJSON_AddStringToObject(root_test, "binfeng_scene_data", dp_data.BINFENG_SCENE_DATA);
	cJSON_AddStringToObject(root_test, "xuancai_scene_data", dp_data.XUANCAI_SCENE_DATA);
	cJSON_AddStringToObject(root_test, "banlan_scene_data", dp_data.BANLAN_SCENE_DATA);

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

    json = cJSON_GetObjectItem(root,"work_mode");
	if(NULL == json) {
		dp_data.WORK_MODE = WHITE_MODE;
	}else{
		dp_data.WORK_MODE = json->valueint;
	}
    
    json = cJSON_GetObjectItem(root,"bright");
	if(NULL == json) {
		dp_data.BRIGHT = BRIGHT_INIT_VALUE;
	}else{
		dp_data.BRIGHT = json->valueint;
	}
    
	json = cJSON_GetObjectItem(root,"temper");
	if(NULL == json) {
		dp_data.COL_TEMPERATURE= COL_TEMP_INIT_VALUE;
	}else{
		dp_data.COL_TEMPERATURE = json->valueint;
	}

    json = cJSON_GetObjectItem(root,"colour_data");
    if(NULL == json) {
		memcpy(dp_data.COLOUR_DATA, COLOUR_MODE_DEFAULT, 14);
	}else{
		memcpy(dp_data.COLOUR_DATA, json->valuestring, 14);
	}
    
	json = cJSON_GetObjectItem(root,"scene_data");
	if(NULL == json) {
		memcpy(dp_data.SCENE_DATA, "00ff0000000000", 14);
	}else{
		memcpy(dp_data.SCENE_DATA, json->valuestring, 14);
	}
    
	json = cJSON_GetObjectItem(root,"rouguang_scene_data");
	if(NULL == json) {
		memcpy(dp_data.ROUGUANG_SCENE_DATA, "ffff500100ff00", 14);
	}else{
		memcpy(dp_data.ROUGUANG_SCENE_DATA, json->valuestring, 14);
	}
    
	json = cJSON_GetObjectItem(root,"binfeng_scene_data");
	if(NULL == json) {
		memcpy(dp_data.BINFENG_SCENE_DATA, "ffff8003ff000000ff000000ff000000000000000000", 44);
	}else{
		memcpy(dp_data.BINFENG_SCENE_DATA, json->valuestring, 44);
	}
    
	json = cJSON_GetObjectItem(root,"xuancai_scene_data");
	if(NULL == json) {
		memcpy(dp_data.XUANCAI_SCENE_DATA, "ffff5001ff0000", 14);
	}else{
		memcpy(dp_data.XUANCAI_SCENE_DATA, json->valuestring, 14);
	}
    
	json = cJSON_GetObjectItem(root,"banlan_scene_data");
	if(NULL == json) {
		memcpy(dp_data.BANLAN_SCENE_DATA, "ffff0505ff000000ff00ffff00ff00ff0000ff000000", 44);
	}else{
		memcpy(dp_data.BANLAN_SCENE_DATA, json->valuestring, 44);
	}
    
	cJSON_Delete(root);
	return OPRT_OK;

ERR_EXT:    
    set_default_dp_data();
    return OPRT_COM_ERROR;
}

STATIC VOID get_light_data(VOID)
{
	INT_T i;
	switch(dp_data.WORK_MODE){
		case WHITE_MODE:
			light_data.FIN_WHITE_VAL = dp_data.BRIGHT*dp_data.COL_TEMPERATURE/255;
			light_data.FIN_WARM_VAL = dp_data.BRIGHT - light_data.FIN_WHITE_VAL;
			light_data.FIN_RED_VAL = 0;
			light_data.FIN_GREEN_VAL = 0;
			light_data.FIN_BLUE_VAL = 0;
			light_data.RED_VAL = light_data.FIN_RED_VAL/RESO_VAL;
			light_data.GREEN_VAL = light_data.FIN_GREEN_VAL/RESO_VAL;
			light_data.BLUE_VAL = light_data.FIN_BLUE_VAL/RESO_VAL;
			light_data.WHITE_VAL = light_data.FIN_WHITE_VAL/RESO_VAL;
			light_data.WARM_VAL = light_data.FIN_WARM_VAL/RESO_VAL;
			break;
		case COLOUR_MODE:
			light_data.FIN_RED_VAL = string_combine_byte(asc2hex(dp_data.COLOUR_DATA[0]), asc2hex(dp_data.COLOUR_DATA[1]));//gamma_9_correction
			light_data.FIN_GREEN_VAL = string_combine_byte(asc2hex(dp_data.COLOUR_DATA[2]), asc2hex(dp_data.COLOUR_DATA[3]));//gamma_5_correction
			light_data.FIN_BLUE_VAL = string_combine_byte(asc2hex(dp_data.COLOUR_DATA[4]), asc2hex(dp_data.COLOUR_DATA[5]));//gamma_6_70_correction
			light_data.FIN_WHITE_VAL = 0;
			light_data.FIN_WARM_VAL = 0;
			light_data.RED_VAL = light_data.FIN_RED_VAL/RESO_VAL;
			light_data.GREEN_VAL = light_data.FIN_GREEN_VAL/RESO_VAL;
			light_data.BLUE_VAL = light_data.FIN_BLUE_VAL/RESO_VAL;
			light_data.WHITE_VAL = light_data.FIN_WHITE_VAL/RESO_VAL;
			light_data.WARM_VAL = light_data.FIN_WARM_VAL/RESO_VAL;
			light_data.HUE = string_combine_short(asc2hex(dp_data.COLOUR_DATA[6]), asc2hex(dp_data.COLOUR_DATA[7]), asc2hex(dp_data.COLOUR_DATA[8]), asc2hex(dp_data.COLOUR_DATA[9]));
			light_data.SATURATION = string_combine_byte(asc2hex(dp_data.COLOUR_DATA[10]), asc2hex(dp_data.COLOUR_DATA[11]));
			light_data.VALUE= string_combine_byte(asc2hex(dp_data.COLOUR_DATA[12]), asc2hex(dp_data.COLOUR_DATA[13]));
			break;
		case SCENE_MODE:
			light_data.FIN_RED_VAL = string_combine_byte(asc2hex(dp_data.SCENE_DATA[0]), asc2hex(dp_data.SCENE_DATA[1]));
			light_data.FIN_GREEN_VAL = string_combine_byte(asc2hex(dp_data.SCENE_DATA[2]), asc2hex(dp_data.SCENE_DATA[3]));
			light_data.FIN_BLUE_VAL = string_combine_byte(asc2hex(dp_data.SCENE_DATA[4]), asc2hex(dp_data.SCENE_DATA[5]));
			light_data.FIN_WHITE_VAL = 0;
			light_data.FIN_WARM_VAL = 0;
			light_data.RED_VAL = light_data.FIN_RED_VAL/RESO_VAL;
			light_data.GREEN_VAL = light_data.FIN_GREEN_VAL/RESO_VAL;
			light_data.BLUE_VAL = light_data.FIN_BLUE_VAL/RESO_VAL;
			light_data.WHITE_VAL = light_data.FIN_WHITE_VAL/RESO_VAL;
			light_data.WARM_VAL = light_data.FIN_WARM_VAL/RESO_VAL;
			light_data.HUE = string_combine_short(asc2hex(dp_data.SCENE_DATA[6]), asc2hex(dp_data.SCENE_DATA[7]), asc2hex(dp_data.SCENE_DATA[8]), asc2hex(dp_data.SCENE_DATA[9]));
			light_data.SATURATION = string_combine_byte(asc2hex(dp_data.SCENE_DATA[10]), asc2hex(dp_data.SCENE_DATA[11]));
			light_data.VALUE= string_combine_byte(asc2hex(dp_data.SCENE_DATA[12]), asc2hex(dp_data.SCENE_DATA[13]));
			break;
		case ROUGUANG_SCENE:
			flash_light_data.BRIGHT = string_combine_byte(asc2hex(dp_data.ROUGUANG_SCENE_DATA[0]), asc2hex(dp_data.ROUGUANG_SCENE_DATA[1]));
			flash_light_data.COL_TEMP = string_combine_byte(asc2hex(dp_data.ROUGUANG_SCENE_DATA[2]), asc2hex(dp_data.ROUGUANG_SCENE_DATA[3]));
			flash_light_data.SPEED = string_combine_byte(asc2hex(dp_data.ROUGUANG_SCENE_DATA[4]), asc2hex(dp_data.ROUGUANG_SCENE_DATA[5]));
			flash_light_data.NUM = string_combine_byte(asc2hex(dp_data.ROUGUANG_SCENE_DATA[6]), asc2hex(dp_data.ROUGUANG_SCENE_DATA[7]));
			for(i=0; i<flash_light_data.NUM; i++){
				flash_light_data.data_group[i].RED_VAL = string_combine_byte(asc2hex(dp_data.ROUGUANG_SCENE_DATA[i*6+8]), asc2hex(dp_data.ROUGUANG_SCENE_DATA[i*6+9]));
				flash_light_data.data_group[i].GREEN_VAL = string_combine_byte(asc2hex(dp_data.ROUGUANG_SCENE_DATA[i*6+10]), asc2hex(dp_data.ROUGUANG_SCENE_DATA[i*6+11]));
				flash_light_data.data_group[i].BLUE_VAL = string_combine_byte(asc2hex(dp_data.ROUGUANG_SCENE_DATA[i*6+12]), asc2hex(dp_data.ROUGUANG_SCENE_DATA[i*6+13]));
			}
			light_data.FIN_WHITE_VAL = 0;
			light_data.WHITE_VAL = 0;
			light_data.FIN_WARM_VAL = 0;
			light_data.WARM_VAL = 0;
			break;
		case BINFENG_SCENE:
			flash_light_data.BRIGHT = string_combine_byte(asc2hex(dp_data.BINFENG_SCENE_DATA[0]), asc2hex(dp_data.BINFENG_SCENE_DATA[1]));
			flash_light_data.COL_TEMP = string_combine_byte(asc2hex(dp_data.BINFENG_SCENE_DATA[2]), asc2hex(dp_data.BINFENG_SCENE_DATA[3]));
			flash_light_data.SPEED = string_combine_byte(asc2hex(dp_data.BINFENG_SCENE_DATA[4]), asc2hex(dp_data.BINFENG_SCENE_DATA[5]));
			flash_light_data.NUM = string_combine_byte(asc2hex(dp_data.BINFENG_SCENE_DATA[6]), asc2hex(dp_data.BINFENG_SCENE_DATA[7]));
			for(i=0; i<flash_light_data.NUM; i++){
				flash_light_data.data_group[i].RED_VAL = string_combine_byte(asc2hex(dp_data.BINFENG_SCENE_DATA[i*6+8]), asc2hex(dp_data.BINFENG_SCENE_DATA[i*6+9]));
				flash_light_data.data_group[i].GREEN_VAL = string_combine_byte(asc2hex(dp_data.BINFENG_SCENE_DATA[i*6+10]), asc2hex(dp_data.BINFENG_SCENE_DATA[i*6+11]));
				flash_light_data.data_group[i].BLUE_VAL = string_combine_byte(asc2hex(dp_data.BINFENG_SCENE_DATA[i*6+12]), asc2hex(dp_data.BINFENG_SCENE_DATA[i*6+13]));
			}
			light_data.FIN_WHITE_VAL = 0;
			light_data.WHITE_VAL = 0;
			light_data.FIN_WARM_VAL = 0;
			light_data.WARM_VAL = 0;
			break;
		case XUANCAI_SCENE:
			flash_light_data.BRIGHT = string_combine_byte(asc2hex(dp_data.XUANCAI_SCENE_DATA[0]), asc2hex(dp_data.XUANCAI_SCENE_DATA[1]));
			flash_light_data.COL_TEMP = string_combine_byte(asc2hex(dp_data.XUANCAI_SCENE_DATA[2]), asc2hex(dp_data.XUANCAI_SCENE_DATA[3]));
			flash_light_data.SPEED = string_combine_byte(asc2hex(dp_data.XUANCAI_SCENE_DATA[4]), asc2hex(dp_data.XUANCAI_SCENE_DATA[5]));
			flash_light_data.NUM = string_combine_byte(asc2hex(dp_data.XUANCAI_SCENE_DATA[6]), asc2hex(dp_data.XUANCAI_SCENE_DATA[7]));
			for(i=0; i<flash_light_data.NUM; i++){
				flash_light_data.data_group[i].RED_VAL = string_combine_byte(asc2hex(dp_data.XUANCAI_SCENE_DATA[i*6+8]), asc2hex(dp_data.XUANCAI_SCENE_DATA[i*6+9]));
				flash_light_data.data_group[i].GREEN_VAL = string_combine_byte(asc2hex(dp_data.XUANCAI_SCENE_DATA[i*6+10]), asc2hex(dp_data.XUANCAI_SCENE_DATA[i*6+11]));
				flash_light_data.data_group[i].BLUE_VAL = string_combine_byte(asc2hex(dp_data.XUANCAI_SCENE_DATA[i*6+12]), asc2hex(dp_data.XUANCAI_SCENE_DATA[i*6+13]));
			}
			light_data.FIN_WHITE_VAL = 0;
			light_data.WHITE_VAL = 0;
			light_data.FIN_WARM_VAL = 0;
			light_data.WARM_VAL = 0;
			break;
		case BANLAN_SCENE:
			flash_light_data.BRIGHT = string_combine_byte(asc2hex(dp_data.BANLAN_SCENE_DATA[0]), asc2hex(dp_data.BANLAN_SCENE_DATA[1]));
			flash_light_data.COL_TEMP = string_combine_byte(asc2hex(dp_data.BANLAN_SCENE_DATA[2]), asc2hex(dp_data.BANLAN_SCENE_DATA[3]));
			flash_light_data.SPEED = string_combine_byte(asc2hex(dp_data.BANLAN_SCENE_DATA[4]), asc2hex(dp_data.BANLAN_SCENE_DATA[5]));
			flash_light_data.NUM = string_combine_byte(asc2hex(dp_data.BANLAN_SCENE_DATA[6]), asc2hex(dp_data.BANLAN_SCENE_DATA[7]));
			for(i=0; i<flash_light_data.NUM; i++){
				flash_light_data.data_group[i].RED_VAL = string_combine_byte(asc2hex(dp_data.BANLAN_SCENE_DATA[i*6+8]), asc2hex(dp_data.BANLAN_SCENE_DATA[i*6+9]));
				flash_light_data.data_group[i].GREEN_VAL = string_combine_byte(asc2hex(dp_data.BANLAN_SCENE_DATA[i*6+10]), asc2hex(dp_data.BANLAN_SCENE_DATA[i*6+11]));
				flash_light_data.data_group[i].BLUE_VAL = string_combine_byte(asc2hex(dp_data.BANLAN_SCENE_DATA[i*6+12]), asc2hex(dp_data.BANLAN_SCENE_DATA[i*6+13]));
			}
			light_data.FIN_WHITE_VAL = 0;
			light_data.WHITE_VAL = 0;
			light_data.FIN_WARM_VAL = 0;
			light_data.WARM_VAL = 0;
			break;
		default:
			break;
	}
}

void pwm_set_duty(u32 duty, u8 channel)
{
    STATIC pwmout_t pwm_info;

    pwm_info.pwm_idx = pwmout_pin2chan(io_info[channel]);
    pwmout_pulsewidth_us(&pwm_info, duty);
}


STATIC VOID send_light_data(u8 R_value, u8 G_value, u8 B_value, u8 CW_value, u8 WW_value)
{
	pwm_val = PWM_MAX*R_value/255;
	pwm_set_duty(pwm_val, 0);
	pwm_val = PWM_MAX*G_value/255;
	pwm_set_duty(pwm_val, 1);
	pwm_val = PWM_MAX*B_value/255;
	pwm_set_duty(pwm_val, 2);
	pwm_val = PWM_MAX*CW_value/255;
	pwm_set_duty(pwm_val, 3);
	pwm_val = PWM_MAX*WW_value/255;
	pwm_set_duty(pwm_val, 4);
}

VOID reset_light_sta(VOID)
{
    //read flash datas
    dev_inf_get();

    if(dp_data.SWITCH == TRUE){
        
        PR_DEBUG("work_mode:%d",dp_data.WORK_MODE);
        
		get_light_data();
		switch(dp_data.WORK_MODE)
		{
			case WHITE_MODE:
			case COLOUR_MODE:
			case SCENE_MODE:
				light_data.LAST_RED_VAL = light_data.FIN_RED_VAL/RESO_VAL;
				light_data.LAST_GREEN_VAL = light_data.FIN_GREEN_VAL/RESO_VAL;
				light_data.LAST_BLUE_VAL = light_data.FIN_BLUE_VAL/RESO_VAL;
				light_data.LAST_WHITE_VAL = light_data.FIN_WHITE_VAL/RESO_VAL;
				light_data.LAST_WARM_VAL = light_data.FIN_WARM_VAL/RESO_VAL;
				send_light_data(light_data.FIN_RED_VAL,light_data.FIN_GREEN_VAL,light_data.FIN_BLUE_VAL,light_data.FIN_WHITE_VAL,light_data.FIN_WARM_VAL);
				break;
				break;
			default:
				light_data.LAST_RED_VAL = 0;
				light_data.LAST_GREEN_VAL = 0;
				light_data.LAST_BLUE_VAL = 0;
				light_data.LAST_WHITE_VAL = 0;
				light_data.LAST_WARM_VAL = 0;
				flash_scene_flag = TRUE;
				break;		
		}
	}
}

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

STATIC VOID reset_fsw_cnt_cb(UINT timerID,PVOID pTimerArg)
{
    PR_DEBUG("%s",__FUNCTION__);

    set_reset_cnt(0);
}

VOID dev_reset_judge(VOID)
{
    OPERATE_RET op_ret;
    UCHAR_T rst_cnt;
    UCHAR_T rst_num;

    CHAR_T *rst_inf = system_get_rst_info();
    PR_NOTICE("%s", rst_inf);
    rst_num = atoi(rst_inf+23);
    PR_DEBUG("rst_inf:%d", rst_num);

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


const uint8_t light_step_table[8] = {25, 57, 92, 125, 156, 191, 224, 255};//25, 58, 91, 124, 157, 190, 223, 255
const uint8_t temper_step_table[9] = {0, 36, 72, 108, 144, 180, 216, 255, 255};

VOID UserIrCmdDeal(IRCMD cmd,IRCODE irType){
	STATIC UCHAR last_irType = 0xff; 
	USHORT key;
//	STATIC INT cur_col = 0;
	STATIC IRCMD_new last_cmd = -1;
	STATIC UCHAR I = 0;

	uint8_t index;
	if(irType == IRCODESTART || irType == IRCODEREPEAT) {
		if(irType == IRCODESTART){
			last_cmd = cmd;
			PR_DEBUG_RAW("NEW CMD:%x\r\n",last_cmd);
			I = 0;
		}
		else{
			if(last_cmd == -1 || (last_cmd == KEY_a) || (last_cmd == KEY_b) || (last_cmd == KEY_e)) {
				return;
			}
			PR_DEBUG_RAW("REPEAT%d CMD:%x\r\n",++I,last_cmd);
		}
	}
	switch(last_cmd){
		case KEY_e:
		    {                
            //    PR_DEBUG("SWITCH:%d", dp_data.SWITCH);
                
                if(dp_data.SWITCH == TRUE) {
			        //关灯
			        dp_data.SWITCH = FALSE;
					light_data.FIN_WHITE_VAL = 0;
					light_data.FIN_WARM_VAL = 0;
					light_data.WHITE_VAL = 0;
					light_data.WARM_VAL = 0;
             		start_gra_change(NORMAL_DELAY_TIME);
                } 
				else {
			        //开灯
					dp_data.SWITCH = TRUE;				
					get_light_data();           
					start_gra_change(NORMAL_DELAY_TIME);
                }
		    }
		break;
		case KEY_f://亮
			if(dp_data.SWITCH == FALSE)
				return;
			
			if(dp_data.BRIGHT < 255) 
			{
    		    for(key = 0; key < sizeof(light_step_table) - 1; key ++) 
				{
                    if((dp_data.BRIGHT < light_step_table[key+1]) && (dp_data.BRIGHT >= light_step_table[key])) {//找出比当前亮度大的数值
                            break;
                       }
    		        }
				dp_data.BRIGHT = light_step_table[key+1];		
				get_light_data();
				start_gra_change(NORMAL_DELAY_TIME);
			}
		break;
		case KEY_g://暗
			if(dp_data.SWITCH == FALSE)
				return;
			
			if(dp_data.BRIGHT > 25) 
			{
				for(key = 0; key < sizeof(light_step_table) - 1; key ++) {
 					if((dp_data.BRIGHT > light_step_table[key]) && (dp_data.BRIGHT <= light_step_table[key + 1])) {
						break;
                        }
    		        }
					dp_data.BRIGHT = light_step_table[key];
				
					get_light_data();
					start_gra_change(NORMAL_DELAY_TIME);    				
			}
		break;
		case KEY_c://暖
			if(dp_data.SWITCH == FALSE)
				return;

			if(dp_data.COL_TEMPERATURE > 0){
				for(key = 0; key < sizeof(temper_step_table) - 1; key ++)
			    {
	                if((dp_data.COL_TEMPERATURE > temper_step_table[key]) && (dp_data.COL_TEMPERATURE <= temper_step_table[key + 1])) 
	                    break;
	                    
			    }
			    dp_data.COL_TEMPERATURE = temper_step_table[key];		                        
				get_light_data();
	    		start_gra_change(NORMAL_DELAY_TIME);
			}
		break;
		case KEY_d://冷
			if(dp_data.SWITCH == FALSE)
				return;
			
			if(dp_data.COL_TEMPERATURE < 255) {
            	for(key = 0; key < sizeof(temper_step_table) - 1; key ++) {
           			if((dp_data.COL_TEMPERATURE >= temper_step_table[key]) && (dp_data.COL_TEMPERATURE < temper_step_table[key + 1])) {
          				break;
      				}
           		}					
            	key ++;
            	dp_data.COL_TEMPERATURE = temper_step_table[key];                    
            }
			get_light_data();
            start_gra_change(NORMAL_DELAY_TIME);
		break;
		
		default:
		break;
	}
	sys_start_timer(data_save_timer,5000,TIMER_CYCLE);
		
	init_upload_proc();
}



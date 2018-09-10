/***********************************************************
*  File: tuya_device.c
*  Author: lql
*  Date: 20180104
***********************************************************/
#define _TUYA_DEVICE_GLOBAL
#include "tuya_device.h"
#include "adapter_platform.h"
#include "tuya_iot_wifi_api.h"
#include "tuya_iot_com_api.h"

#include "tuya_led.h"
#include "tuya_uart.h"
#include "tuya_gpio.h"
#include "tuya_key.h"
#include "uni_time_queue.h"
#include "gw_intf.h"
#include "uni_log.h"
#include "uni_thread.h"
#include "uni_queue.h"
#include "uni_time.h"
#include "uni_md5.h"

#include "flash_api.h"
#include "rtl8710b_ota.h"
#include <device_lock.h>
#include "mqc_media_app.h"
#include "gpio_test.h"


/***********************************************************
*************************micro define***********************
***********************************************************/
#define WF_SW_VERSION USER_SW_VER
#define KEY_TIMER_MS      	 20    //key timer inteval
#define KEY_RST_TIME       3000            //按键重置时间 ms




#define HN_RECV_BUF_MAX 1024 /*接收缓冲区*/
#define HN_RECV_MIN_NUM 7    /*帧数据最小长度*/
#define JUMP_FIRST_TIME 3    /*首包心跳定时器发送间隔*/
#define JUMP_SEND_TIME 15    /*心跳定时器发送间隔*/
#define JUMP_RECV_TIME 3	 /*心跳定时器接收超时*/
#define WF_GUARD_TIME  90    /*监控定时*/
#define MCU_RECV_TIME  5     /*MCU升级接收超时时间*/
#define TY_DP_TYPE 5
#define TY_DP_FRAME_MIN 5
#define QUEUE_MAX_EVENTS 10

// wifi info
#define PROT_VER 0x00

// for portable
#define WM_SUCCESS 0
#define WM_FAIL 1

#define NO_RET    0
#define NEED_RET  1

#define TP_NUMBER 0x00
#define TP_STRING 0x01

#define FAIL_CODE  0x00
#define SUCC_CODE  0x01

#define DP_RAW    0
#define DP_BOOL   1
#define DP_VALUE  2
#define DP_STRING 3
#define DP_ENUM   4
#define DP_BITMAP 5

#define WF_LIGHT_FAST_BLINK 0x00  //wifi灯快闪
#define WF_LIGHT_SLOW_BLINK 0x01  //wifi灯慢闪
#define WF_LIGHT_OFF        0x02  //wifi灯长暗
#define WF_LIGHT_ON         0x03  //wifi灯长亮
#define WF_MQ_ONLINE        0x04  //设备已连接云
#define WF_LOW_PWR          0x05  //设备低功耗

#define QRY_SEND_TIMR_OUT_MS 1000

//#define UART_TX_DEBUG 1


typedef enum {
    HEART_BEAT_CMD = 0x00,
    PRODUCT_INFO_CMD,
    WORK_MODE_CMD,
    WIFI_STATE_CMD,
    WIFI_RESET_CMD,
    WIFI_RESET_SEL_CMD,
    DATA_CTRL_CMD,
    DATA_RPT_CMD,
    DATA_QUERY_CMD = 0x08,
    UPDATE_START_CMD = 0x0a,
    UPDATE_TRANS_CMD = 0x0b,
    UT_TIME_CMD = 0x0c,
    LOCAL_TIME_CMD = 0x1c,
    WIFI_TEST_CMD = 0x0e,
    MEMORY_QUERY_CMD = 0x0f,
    WEATH_SERVER_CMD = 0x20,
    WEATH_DATA_CMD = 0x21,
    DATA_RPT_SYN_CMD = 0x22, //new add
    DATA_RPT_ACK_CMD = 0x23,
    GET_WF_SIGAL_CMD = 0x24,
    STOP_HEART_BEAT_CMD = 0x25,
    OPEN_FLOW_CONNECT_CMD = 0x26,
    FLOW_DATA_START_CMD =  0x27,
    FLOW_DATA_TRANS_CMD = 0x28,
    FLOW_DATA_END_CMD = 0x29
}WF_CMD_E;

typedef struct {
    UINT_T image_cnt;
    flash_t flash;
    UINT_T file_size;
    UINT_T start_addr;
    UINT_T send_len;
    UCHAR_T stat;
}DEV_UG_PROC_S;
DEV_UG_PROC_S dev_proc;

/***********************************************************
*************************micro define***********************
***********************************************************/
//STATIC VOID key_process(INT_T gpio_no,PUSH_KEY_TYPE_E type,INT_T cnt);
STATIC OPERATE_RET device_differ_init(VOID);
STATIC VOID ty_datapoint_proc(cJSON *root);
STATIC INT_T upload_wifi_state(VOID);
STATIC VOID start_guard_timer(UCHAR_T sec);
STATIC VOID del_queue_buf_by_cmd(UCHAR_T cmd);
STATIC OPERATE_RET dev_ug_process_cb(IN CONST FW_UG_S *fw, IN CONST UINT_T total_len,IN CONST UINT_T offset,\
                              IN CONST BYTE_T *data,IN CONST UINT_T len,OUT UINT_T *remain_len, IN PVOID_T pri_data);
STATIC VOID dev_ug_notify_cb(IN CONST FW_UG_S *fw, IN CONST INT_T download_result, IN PVOID_T pri_data);
STATIC VOID get_memory_cb(UINT_T timerID,PVOID pTimerArg);
STATIC VOID sync_dev_ver_cb(UINT_T timerID,PVOID pTimerArg);


#pragma pack(1)
typedef struct
{
	WORD_T len;		//数据长度
	UCHAR_T data[0];  //数据内容
}TY_SEND_DATA_S;

typedef struct
{
    WORD_T  head;    //固定为0x55aa(大端格式)
    UCHAR_T version; //版本，升级扩展用
    UCHAR_T fr_type; //具体帧类型(命令字)
    WORD_T  len;     //数据长度(大端格式)
    UCHAR_T data[0]; //数据内容
}TY_FRAME_S;

typedef struct
{
	UCHAR_T mode;      //工作模式,0 串口处理 1 模块IO
	UCHAR_T state_pin; //wifi状态指示
	UCHAR_T reset_pin; //wifi重置
}TY_WIFI_WORK_S;

typedef struct
{
    UCHAR_T dpid;
    UCHAR_T type;
    WORD_T len;
    UCHAR_T data[0];
}TY_DATAPOINT_DATA_S;

typedef struct
{
	UINT  offset;
    UINT  fw_size;
}TY_UP_FW_S;

typedef struct 
{
    UCHAR_T proc_buf[HN_RECV_BUF_MAX];
	P_QUEUE_CLASS pQueue;
    THRD_HANDLE thread;//lql
    MUTEX_HANDLE mutex; 
	TIMER_ID jump_send_timer;
	TIMER_ID queue_send_timer;
	TIMER_ID com_send_timer;
	TIMER_ID up_fw_timer;
	TIMER_ID guard_timer;
	UCHAR_T try_cnt;
	TY_WIFI_WORK_S wf_work; /*MCU工作方式*/
	BOOL init_flag;		/*设备初始化标志*/
	BOOL online_flag;	/*设备在线标志*/
	BOOL resend_flag;   /*同步指令未返回需重新发送*/
	UCHAR_T ack_stat;     /*固件升级包返回响应标志*/
	UCHAR_T recv_state;
    UCHAR_T prt_ver;
	INT_T rcount;
	TY_UP_FW_S up_st;
	BOOL_T IsInUpgrade;
	BOOL_T IsMcuUgReset;
	BOOL_T StopHartBeat;
	BOOL_T flow_sver_flag;
}TY_MSG_S;
#pragma pack()

/* Init state */
enum recv_proc{
	UART_RECV = 0,
	UART_PROC,
};

/* Upgrade state*/
enum up_stat{
	ST_NO_SUPPORT = 0,
	ST_NOT_READY,
	ST_QUERY_FAIL,
	ST_NO_UPGRADE,
	ST_WAIT_UPGRADE,
};

/*ack state*/
enum fw_ack_stat{
	FW_IDE = 0,	/*空闲态,包已发出，未返回*/
	FW_RETURN,  /*收到响应包*/
	FW_TIME_OUT,/*接收响应超时*/
	FW_FAIL,    /*失败*/
};

/*flow result*/
enum media_flow_resp{
    FLOW_SUCC_E = 0x00, /*成功*/
    FLOW_SERVER_NOT_READY_E = 0x01, /*流服务未开启*/
    FLOW_STAT_FALSE_E = 0x02, /*流服务未连接*/
    FLOW_DATA_TMOUT_E = 0x03, /*数据推送超时*/
    FLOW_DATA_LEN_ERR_E = 0x04, /*传输数据长度错误*/
};

/*wifi test result*/
enum wifi_test_result{
    WF_SCAN_FAIL = 0x00, /*扫描失败*/
    WF_NO_AUTH = 0x01,   /*设备未授权*/
};

/***********************************************************
*************************variable define********************
***********************************************************/
STATIC TY_MSG_S ty_msg;
LED_HANDLE wifi_light = NULL;
CHAR_T dev_sw_ver[SW_VER_LEN+1];
CHAR_T product_key[PRODUCT_KEY_LEN+1];
GW_WF_CFG_MTHD_SEL cfg_mode;

TIMER_ID get_memory_timer;// 倒计时定时器
TIMER_ID sync_dev_ver_timer;// 更新MCU版本定时器
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
    PR_DEBUG("%s:%s",APP_BIN_NAME,WF_SW_VERSION);
    SetLogManageAttr(LOG_LEVEL_INFO);
}
/***********************************************************
*  Function: app_init
*  Input: none
*  Output: none
*  Return: none
*  Note: called by user_main
***********************************************************/
VOID app_init(VOID) {
    return ;
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
    return gpio_test_cb(RTL_BOARD_WR3) || gpio_test_cb(RTL_BOARD_WR1) || gpio_test_cb(RTL_BOARD_WR2) || gpio_test_cb(RTL_BOARD_WR4) || gpio_test_cb(RTL_BOARD_WR7);
}


/*checksum校验*/
UCHAR_T getCheckSum(CONST UCHAR_T *pack, INT_T pack_len)
{
    UINT check_sum = 0;
    while(--pack_len >= 0)
    {
        check_sum += *pack++;
    }
    return check_sum&0xff;
}

#if 0
VOID start_jump_timer(UCHAR_T sec)
{
	sys_start_timer(ty_msg.jump_send_timer,sec*1000,TIMER_CYCLE);
}
#endif
#if 0
VOID stop_jump_timer(VOID)
{
	sys_stop_timer(ty_msg.jump_send_timer);
}
#endif

VOID close_heartbeat(VOID)
{
    if(ty_msg.init_flag) {
        ty_msg.StopHartBeat = TRUE;
    }
}

VOID open_heartbeat(VOID)
{
    ty_msg.StopHartBeat = FALSE;
}

VOID com_stop_timer(VOID)
{
    if(0 == GetCurQueNum(ty_msg.pQueue)) {
        return;
    }
    
	ty_msg.try_cnt = 0;
	
	TY_SEND_DATA_S *pSend = NULL;
	GetQueueMember(ty_msg.pQueue, 1, (UCHAR_T *)&pSend, 1);
	if( pSend != NULL )
	{
		PR_DEBUG("free buffer... ...");
		Free(pSend);
	}		
	DelQueueMember(ty_msg.pQueue, 1); //del by cmd ?
	sys_stop_timer(ty_msg.com_send_timer);
}

VOID com_stop_timer_cmd(UCHAR_T cmd)
{
    del_queue_buf_by_cmd(cmd);
    sys_stop_timer(ty_msg.com_send_timer);
}

/*串口数据发送*/
STATIC INT_T ty_uart_send(UCHAR_T fr_type, UINT len, UCHAR_T *data)
{
    UINT i = 0;
    WORD_T head = 0x55aa;
    UINT send_len = SIZEOF(TY_FRAME_S) + len + 1;
    
    UCHAR_T* SendBuf = (UCHAR_T*)Malloc(send_len);
    if(SendBuf == NULL) {
        PR_ERR("ty_uart_send SendBuf is NULL");
        return WM_FAIL;
    }
    
    memset(SendBuf, 0, send_len);
    TY_FRAME_S *hn_frame = (TY_FRAME_S *)(SendBuf);
    hn_frame->head = WORD_SWAP(head);
    hn_frame->version = PROT_VER;
    hn_frame->fr_type = fr_type;
    hn_frame->len = WORD_SWAP(len);
    if(data && len) {
        memcpy(hn_frame->data, data, len);
    }
    UCHAR_T num = getCheckSum(SendBuf, send_len - 1);
    SendBuf[send_len - 1] = num;

 	MutexLock(ty_msg.mutex);
    ty_uart_send_data(TY_UART0,SendBuf,send_len);
#ifdef UART_TX_DEBUG
    INT_T cnt = 0;
    for(i = 0;i < send_len; i++) {
        PR_DEBUG_RAW("%02X ",SendBuf[i]);
    }
    PR_DEBUG("\r\n");
#endif
    Free(SendBuf);
	MutexUnLock(ty_msg.mutex);

    return WM_SUCCESS;
}

/*发送数据入队*/
STATIC INT_T ty_msg_send(UCHAR_T fr_type, WORD_T len, UCHAR_T *data)
{
    UINT i = 0;
    WORD_T head = 0x55aa;
    UINT hn_send_len = SIZEOF(TY_SEND_DATA_S) + SIZEOF(TY_FRAME_S) + len + 1;
    
    UCHAR_T* SendBuf = (UCHAR_T*)Malloc(hn_send_len);
    if(SendBuf == NULL) {
        PR_ERR("SendBuf is NULL");
        return WM_FAIL;
    }
    
    memset(SendBuf, 0, hn_send_len);
    TY_FRAME_S *hn_frame = (TY_FRAME_S *)(SendBuf + SIZEOF(TY_SEND_DATA_S));
    hn_frame->head = WORD_SWAP(head);
    hn_frame->version = PROT_VER;
    hn_frame->fr_type = fr_type;
    hn_frame->len = WORD_SWAP(len);
    if(data && len) {
        memcpy(hn_frame->data, data, len);
    }
    UCHAR_T num = getCheckSum(SendBuf + SIZEOF(TY_SEND_DATA_S), hn_send_len - SIZEOF(TY_SEND_DATA_S) - 1);
    SendBuf[hn_send_len - 1] = num;

    TY_SEND_DATA_S *pSend = (TY_SEND_DATA_S *)SendBuf;   
    pSend->len = hn_send_len - SIZEOF(TY_SEND_DATA_S);
    
    
    
    if(InQueue(ty_msg.pQueue, (UCHAR_T *)&pSend, 1) == 0) {    
        Free(SendBuf);
        PR_ERR("InQueue err");
        return WM_FAIL;
    }
    return WM_SUCCESS;
}


STATIC VOID del_queue_buf_by_cmd(UCHAR_T cmd)
{
    TY_SEND_DATA_S *pSend = NULL;
    GetQueueMember(ty_msg.pQueue,1,(UCHAR_T *)(&pSend),1);
    if(NULL != pSend) {
        TY_FRAME_S *pFrame = (TY_FRAME_S *)pSend->data;
        if(pFrame->fr_type != cmd) {
            return;
        }
        DelQueueMember(ty_msg.pQueue,1);
        Free(pSend);
    }
}

/*发送心跳包*/
STATIC VOID send_jump_pack(VOID)
{
    PR_NOTICE("send jump_pack");
	ty_msg_send(HEART_BEAT_CMD, 0, NULL);
}


STATIC VOID query_prod_info(VOID)
{
    ty_msg_send(PRODUCT_INFO_CMD, 0, NULL);
}


/*查询模块工作模式*/
STATIC VOID query_work_mode(VOID)
{
    ty_msg_send(WORK_MODE_CMD, 0, NULL);
}

/*查询同步dp数据*/
STATIC VOID query_dp_state(VOID)
{
    ty_uart_send(DATA_QUERY_CMD, 0, NULL);
}

STATIC INT_T recv_jump_pack(TY_FRAME_S *p_frame)
{
	UCHAR_T start = 0x00;

	com_stop_timer_cmd(HEART_BEAT_CMD);
	start_guard_timer(WF_GUARD_TIME);  //重新开始GUARD TIMER
	
	start = p_frame->data[0];
	if( !ty_msg.init_flag ) //设备刚上电
	{
		ty_msg.prt_ver = p_frame->version;
		PR_DEBUG("Device power on ... ... version:%d", p_frame->version);
		query_prod_info();
	}
	else    //设备已初始化
	{
		if(start == 0x00) 
		{
            // MCU非升级成功之后重启
			PR_DEBUG("First jump back ... ...");
            if( ty_msg.wf_work.mode == 0 ) {
                upload_wifi_state();//lql
            }
			query_dp_state();
		}
		#if 0
		else
		{
			if( !ty_msg.online_flag ) //断线后重新连接
			{
                if( ty_msg.wf_work.mode == 0 ) {
                    upload_wifi_state();//lql
                }
				query_dp_state();
			}
		}
		#endif
	}
	//ty_msg.online_flag = TRUE;
	return 0;
}


/*同步指令,MCU必须返回*/
STATIC OPERATE_RET get_dev_info(IN TY_FRAME_S *p_frame)
{
	if(NULL == p_frame) {
	   PR_ERR("invalid param");
	   return OPRT_INVALID_PARM;
	}

	WORD_T len = WORD_SWAP(p_frame->len);
	
	if(ty_msg.prt_ver < 0x03) {
	#if 1
		if((len <= 0x0010) && (len > 0x018)){
			PR_ERR("prod len = %d", len);
			return OPRT_MALLOC_FAILED;
		}
		memset(product_key, 0, sizeof(product_key));
		memset(dev_sw_ver, 0, sizeof(dev_sw_ver));
		
		memcpy(product_key, p_frame->data, 16);
		memcpy(dev_sw_ver, p_frame->data + 16, len - 16); 
	#endif
	}else {
	#if 1
		UCHAR_T* p_data = (UCHAR_T *)Malloc(len+1);
		if(NULL == p_data) {
			return OPRT_MALLOC_FAILED;
		}
		memset(p_data, 0, len+1);
		memcpy(p_data, p_frame->data, len);

		PR_DEBUG("jstr:%s",p_data);
		cJSON *root = cJSON_Parse(p_data);
		if(NULL == root) {
			PR_ERR("cjson parse err");
			Free(p_data);
			return OPRT_CJSON_PARSE_ERR;
		}
        
		if( ( NULL == cJSON_GetObjectItem(root,"p") ) || \
        	( NULL == cJSON_GetObjectItem(root,"v") ) || \
        	( NULL == cJSON_GetObjectItem(root,"m") ) ){
        	PR_ERR("param is no correct");
			cJSON_Delete(root);
			Free(p_data);
        	return OPRT_INVALID_PARM;
    	}
		
		cJSON *json;
		json = cJSON_GetObjectItem(root, "p");	
		strcpy(product_key,json->valuestring);

		json = cJSON_GetObjectItem(root, "v");
		strcpy(dev_sw_ver,json->valuestring);

		json = cJSON_GetObjectItem(root, "m");
		cfg_mode = json->valueint;
		cJSON_Delete(json);
		Free(p_data);
	#endif
	}
	return OPRT_OK;
}
/*获取产品信息*/
STATIC INT_T recv_prod_info(TY_FRAME_S *p_frame)
{
	PR_DEBUG("recv prod info");
	
	//com_stop_timer();
	com_stop_timer_cmd(PRODUCT_INFO_CMD);	
    OPERATE_RET op_ret = OPRT_OK;
	op_ret = get_dev_info(p_frame);//get procuct_key  sw_ver  cfg_mode
	if(OPRT_OK != op_ret) {
		return WM_FAIL;
	}
    query_work_mode();
	return WM_SUCCESS;
}

STATIC INT_T recv_work_mode(TY_FRAME_S *p_frame)
{
	//com_stop_timer();
	com_stop_timer_cmd(WORK_MODE_CMD);

	WORD_T len = WORD_SWAP(p_frame->len);
	
	if( len == 0 )  //mcu与模块配合处理
	{
		ty_msg.wf_work.mode = 0;
		PR_DEBUG("state mcu");
	}
	else if( len == 2 )  //模块自处理
	{
		ty_msg.wf_work.mode = 1;
		ty_msg.wf_work.state_pin = p_frame->data[0];
		ty_msg.wf_work.reset_pin = p_frame->data[1];
		PR_DEBUG("state pin:%d, reset pin:%d", ty_msg.wf_work.state_pin, ty_msg.wf_work.reset_pin);
	}
	if(ty_msg.IsMcuUgReset) {
        ty_msg.IsMcuUgReset = FALSE;
        //send mcu version to tuya cloud
        if(dev_sw_ver) {
            if(0 == is_valid_version((CHAR_T *)dev_sw_ver)) {
                PR_ERR("dev_sw_ver:%s",dev_sw_ver);
                return OPRT_INVALID_PARM;
            }
            if(0 != strcasecmp(get_gw_cntl()->gw_if.dev_sw_ver,dev_sw_ver)) {
                strcpy(get_gw_cntl()->gw_if.dev_sw_ver,dev_sw_ver);
                 OPERATE_RET op_ret= OPRT_OK;
                op_ret = sys_add_timer(sync_dev_ver_cb, NULL, &sync_dev_ver_timer);
                if(OPRT_OK != op_ret) {
                    PR_ERR("sys_add_timer err");
                    return op_ret;
                }
                else{
                    PR_NOTICE("cd_timer ID:%d",sync_dev_ver_timer);
                    sys_start_timer(sync_dev_ver_timer, 1000, TIMER_ONCE);//1s
                }
                
            }
        }
	}else {
	    device_differ_init();//lql
	}
	ty_msg.init_flag = TRUE;
	return 0;
}

STATIC VOID sync_dev_ver_cb(UINT_T timerID,PVOID pTimerArg)
{
    OPERATE_RET op_ret= OPRT_OK;
    op_ret = http_gw_update_version_v41();
    if(op_ret != OPRT_OK) {
        sys_start_timer(sync_dev_ver_timer, 5000, TIMER_ONCE);//5s
    }else {
        wd_gw_desc_if_write(&(get_gw_cntl()->gw_if));
    }
}

/*WIFI工作状态*/
STATIC INT_T recv_wifi_ack(VOID)
{
	//com_stop_timer();
	com_stop_timer_cmd(WIFI_STATE_CMD);
    return 0;
}

STATIC OPERATE_RET upgrade_pre_cb(IN CONST UINT_T file_size)
{
    PR_DEBUG("file_size = %d", file_size);
    
    UCHAR_T start_upgrade = UPDATE_START_CMD;
	UCHAR_T len_buf[4];
	UINT_T  icnt = 0,len;

    if( file_size == 0 ) {
        PR_ERR("file_size = %d", file_size);
        return OPRT_COM_ERROR;
    }

    ty_msg.up_st.offset = 0;
    ty_msg.up_st.fw_size = file_size;
    
	len = file_size;
	len_buf[0] = (len>>24)&0xFF;
	len_buf[1] = (len>>16)&0xFF;
	len_buf[2] = (len>>8)&0xFF;
	len_buf[3] = (len)&0xFF;
	ty_uart_send(start_upgrade, 4, len_buf);  //启动升级

    ty_msg.ack_stat = FW_IDE;
	sys_start_timer(ty_msg.up_fw_timer,MCU_RECV_TIME*1000, TIMER_ONCE);
    while( 1 )
	{
		if( ty_msg.ack_stat == FW_IDE )
		{
			SystemSleep(200);
		}
		else if( ty_msg.ack_stat == FW_TIME_OUT ) {
            icnt ++;
			if( icnt >= 3 )
			{
			    ty_msg.ack_stat = FW_FAIL;
			    return OPRT_COM_ERROR;
			}
            PR_DEBUG("start upgrade again");
			ty_msg.ack_stat = FW_IDE;
			sys_start_timer(ty_msg.up_fw_timer,MCU_RECV_TIME*1000, TIMER_ONCE);
            ty_uart_send(start_upgrade, 4, len_buf); //超时再次启动
		}else {
            break;
		}
    }
    ty_msg.ack_stat = FW_IDE;
	return OPRT_OK;
}

STATIC OPERATE_RET upgrade_file_cb(IN CONST BYTE_T *data, IN CONST UINT_T len)
{	
	UCHAR_T head_buf[10];
	UCHAR_T sum_check = 0;
	UCHAR_T icnt = 0, i = 0;
	WORD_T  data_len = 0;
    WORD_T  head_len = 0;
	
	//PR_DEBUG("upgrade len = %d", len);
	
	if( ty_msg.ack_stat == FW_FAIL ) {
		return OPRT_COM_ERROR;
	}

    if( ty_msg.prt_ver == 0x00 ) {
        data_len = len + 2;
    }else {
        data_len = len + 4;
    }

	head_buf[0] = 0x55;
	head_buf[1] = 0xaa;
	head_buf[2] = 0x00;
	head_buf[3] = 0x0b;
	head_buf[4] = (data_len>>8)&0xFF;
	head_buf[5] = (data_len&0xFF);	
    if( ty_msg.prt_ver == 0x00 ) {
    	head_buf[6] = ((ty_msg.up_st.offset>>8)&0xFF);
    	head_buf[7] = (ty_msg.up_st.offset&0xFF);
        head_len = 8; 
    }else {
    	head_buf[6] = ((ty_msg.up_st.offset>>24)&0xFF);
    	head_buf[7] = ((ty_msg.up_st.offset>>16)&0xFF);
    	head_buf[8] = ((ty_msg.up_st.offset>>8)&0xFF);
    	head_buf[9] = (ty_msg.up_st.offset&0xFF);
        head_len = 10;    	
    }
    sum_check = getCheckSum(head_buf, head_len) + getCheckSum(data,len);
    
    MutexLock(ty_msg.mutex);
    ty_uart_send_data(TY_UART0,head_buf, head_len);
    ty_uart_send_data(TY_UART0,(UCHAR_T *)data, len);
    ty_uart_send_data(TY_UART0,&sum_check, 1);
	MutexUnLock(ty_msg.mutex);
	
	ty_msg.ack_stat = FW_IDE;
	sys_start_timer(ty_msg.up_fw_timer,MCU_RECV_TIME*1000, TIMER_ONCE);
	ty_msg.up_st.offset += len;
	
	while( 1 )
	{
		if( ty_msg.ack_stat == FW_IDE )
		{
			SystemSleep(200);
		}
		else if( ty_msg.ack_stat == FW_TIME_OUT )
		{
			icnt ++;
			if( icnt >= 3 )
			{
				ty_msg.ack_stat = FW_FAIL;
				//start_jump_timer(JUMP_SEND_TIME);
				return OPRT_COM_ERROR;
			}
			PR_DEBUG("try again");
			ty_msg.ack_stat = FW_IDE;
			sys_start_timer(ty_msg.up_fw_timer,MCU_RECV_TIME*1000, TIMER_ONCE);
			
			MutexLock(ty_msg.mutex);
            ty_uart_send_data(TY_UART0,head_buf, head_len);
		    ty_uart_send_data(TY_UART0,(UCHAR_T *)data, len);
		    ty_uart_send_data(TY_UART0,&sum_check, 1);
		    MutexUnLock(ty_msg.mutex);
		}
		else if( ty_msg.ack_stat == FW_RETURN )
		{
			break;
		}
		else
		{
			//start_jump_timer(JUMP_SEND_TIME);
			return OPRT_COM_ERROR;
		}
	}

	if( ty_msg.up_st.offset == ty_msg.up_st.fw_size)
	{
        if( ty_msg.prt_ver == 0x00 ) {
            head_buf[4] = 0x00;
		    head_buf[5] = 0x02;
    		head_buf[6] = ((ty_msg.up_st.offset>>8)&0xFF);
    		head_buf[7] = (ty_msg.up_st.offset&0xFF);
            head_len = 8;
        }else {
            head_buf[4] = 0x00;
		    head_buf[5] = 0x04;
        	head_buf[6] = ((ty_msg.up_st.offset>>24)&0xFF);
        	head_buf[7] = ((ty_msg.up_st.offset>>16)&0xFF);
        	head_buf[8] = ((ty_msg.up_st.offset>>8)&0xFF);
        	head_buf[9] = (ty_msg.up_st.offset&0xFF);
            head_len = 10;
        }
		sum_check = getCheckSum(head_buf, head_len);

        MutexLock(ty_msg.mutex);
		ty_uart_send_data(TY_UART0,head_buf, head_len);
		ty_uart_send_data(TY_UART0,&sum_check, 1);
		MutexUnLock(ty_msg.mutex);
		PR_DEBUG("last pack");

        ty_msg.init_flag = FALSE;
        ty_msg.IsMcuUgReset = TRUE;
        ty_msg.IsInUpgrade = FALSE;
        PR_DEBUG(" init_flag[%d] IsMcuUgReset[%d]  IsInUpgrade[%d]",ty_msg.init_flag,ty_msg.IsMcuUgReset,ty_msg.IsInUpgrade);
	}

	return OPRT_OK;
}

STATIC VOID long_to_byte(INT_T val, UCHAR_T *out)
{
	//PR_DEBUG("val:%d",val);
	out[0] = (UCHAR_T)((val&0xFF000000)>>24);
	out[1] = (UCHAR_T)((val&0x00FF0000)>>16);
	out[2] = (UCHAR_T)((val&0x0000FF00)>>8);
	out[3] = (UCHAR_T)((val&0x000000FF));
}

STATIC INT_T get_lklv_data(IN CHAR_T *jstr,OUT CHAR_T *data, OUT INT_T *dlen)
{
	UCHAR_T len = 0;
	UCHAR_T type = 0x00;
	INT_T i = 0;

	cJSON *root = cJSON_Parse(jstr);
	if(NULL == root) {
		return WM_FAIL;
	}
		
    cJSON *nxt = root->child;
    while(nxt) {
		len = strlen(nxt->string);
		//PR_DEBUG("len:%d string:%s",len,nxt->string);
		data[i++] = len; //L
		memcpy(&data[i],nxt->string,len); //K
		i += len;
		if(cJSON_Number == nxt->type) {
			data[i++] = TP_NUMBER;
			data[i++] = 4; //L
			long_to_byte(nxt->valueint,&data[i]); //V
			i += 4;			
		}else if(cJSON_String == nxt->type) {
			data[i++] = TP_STRING;
			len = strlen(nxt->valuestring);
			//PR_DEBUG("len:%d,val string:%s",len,nxt->valuestring);
			data[i++] = len; //L
			memcpy(&data[i],nxt->valuestring,len); //K
			i += len;
		}
		nxt = nxt->next;
    }

	*dlen = i;
	cJSON_Delete(root);
	return WM_SUCCESS;
}

STATIC VOID syn_weather_data(OPERATE_RET result,CHAR_T *param)
{
	UCHAR_T fr_type = WEATH_DATA_CMD;
	UCHAR_T data[2];
	UINT len = 0;

	if((OPRT_OK != result) || (NULL == param)) {
		goto FAIL_EXIT;
	}

	//PR_DEBUG("param:%s",param);
	UCHAR_T *buf = (UCHAR_T *)Malloc(strlen(param)+128);
	if(NULL == buf) {
		goto FAIL_EXIT;
	}
	
	buf[0] = SUCC_CODE;
	if(WM_FAIL == get_lklv_data(param,buf+1,&len)) {
		Free(buf);
		goto FAIL_EXIT;
	}

	ty_msg_send(fr_type,len+1,buf);
	Free(buf);
	return;
	
FAIL_EXIT:
	data[0] = FAIL_CODE;
	data[1] = 0x01;
	len = 2;
	ty_msg_send(fr_type,len,data);
	return;		
}


STATIC INT_T recv_weather_ack(VOID)
{
    com_stop_timer_cmd(WEATH_DATA_CMD);
	return 0;
}

//start mq media flow func
STATIC INT_T recv_connect_flow(VOID)
{
    UCHAR_T fr_type = OPEN_FLOW_CONNECT_CMD;
    UCHAR_T data = FLOW_SERVER_NOT_READY_E;
    UINT_T len = 1;

    OPERATE_RET op_ret = tuya_iot_media_init();
    PR_NOTICE("mqc_media_init op_ret:%d",op_ret);
    if(OPRT_OK != op_ret) {
        ty_msg.flow_sver_flag = FALSE;
    }else {
        data = FLOW_SUCC_E;
        ty_msg.flow_sver_flag = TRUE;
    }

    close_heartbeat();
    ty_uart_send(fr_type,len,&data);
    return 0;
}

STATIC INT_T recv_flow_start(UCHAR_T *data)
{
    OPERATE_RET op_ret;
	UCHAR_T fr_type = FLOW_DATA_START_CMD;
	UCHAR_T result = FLOW_SUCC_E;
	UINT_T len = 1;
	FLOW_BODY_ST flow_data;

    if(!ty_msg.flow_sver_flag){
        result = FLOW_SERVER_NOT_READY_E;
        goto UART_SEND;
    }

    BOOL_T media_sta = get_mqc_media_conn_stat();
    if(FALSE == media_sta){
        result = FLOW_STAT_FALSE_E;
        goto UART_SEND;
    }

    PR_DEBUG("data[0] is %02x",data[0]);
    PR_DEBUG("data[1] is %02x",data[1]);

    memset(&flow_data,0,SIZEOF(FLOW_BODY_ST));
    flow_data.id= ((data[0]<<8)|data[1]);
    flow_data.posix = uni_time_get_posix();
    flow_data.step = TS_START;
    flow_data.offset = 0;
    flow_data.len = 0;

    op_ret = tuya_iot_media_data_report(&flow_data,5);
    if(OPRT_OK != op_ret){
        result = FLOW_DATA_TMOUT_E;
        goto UART_SEND;
    }

    result = FLOW_SUCC_E;
UART_SEND:
    ty_uart_send(fr_type,len,&result);
	return 0;
}

STATIC INT_T recv_flow_end(UCHAR_T *data)
{
    OPERATE_RET op_ret;
	UCHAR fr_type = FLOW_DATA_END_CMD;
	UCHAR result = FLOW_SUCC_E;
	UINT result_len = 1;
	FLOW_BODY_ST flow_data;

    if(!ty_msg.flow_sver_flag){
        result = FLOW_SERVER_NOT_READY_E;
        goto UART_SEND;
    }

    BOOL_T media_sta = get_mqc_media_conn_stat();
    if(FALSE == media_sta){
        result = FLOW_STAT_FALSE_E;
        goto UART_SEND;
    }

    memset(&flow_data,0,SIZEOF(FLOW_BODY_ST));
    flow_data.step = TS_END;
    flow_data.id = ((data[0]<<8)|data[1]);
    PR_DEBUG("id is %d",flow_data.id);
    flow_data.posix = uni_time_get_posix();
    flow_data.offset = ((data[2]<<24)|(data[3]<<16)|(data[4]<<8)|(data[5]));
    PR_DEBUG("offset is %d",flow_data.offset);
    flow_data.len = 0;
    op_ret = tuya_iot_media_data_report(&flow_data,5);
    if(op_ret != OPRT_OK){
        result = FLOW_DATA_TMOUT_E;
        goto UART_SEND;
    }

    result = FLOW_SUCC_E;
UART_SEND:
    ty_uart_send(fr_type,result_len,&result);
    return 0;
}

STATIC INT_T recv_flow_data(WORD_T len,UCHAR_T *data)
{
    OPERATE_RET op_ret;
	UCHAR_T fr_type = FLOW_DATA_TRANS_CMD;
	UCHAR_T result = FLOW_SUCC_E;
	UINT_T result_len = 1;
	WORD_T data_len = WORD_SWAP(len);
	FLOW_BODY_ST *flow_data = NULL;

    if(!ty_msg.flow_sver_flag){
        result = FLOW_SERVER_NOT_READY_E;
        goto UART_SEND;
    }

    BOOL_T media_sta = get_mqc_media_conn_stat();
    if(FALSE == media_sta){
        result = FLOW_STAT_FALSE_E;
        goto UART_SEND;
    }

    data_len = WORD_SWAP(len);
    PR_DEBUG("recv len is %d",data_len);
    if( (data_len > (HN_RECV_BUF_MAX-13)) || (data_len < 6)  ){
        result = FLOW_DATA_LEN_ERR_E;
        goto UART_SEND;
    }

    flow_data = (FLOW_BODY_ST *)Malloc(SIZEOF(FLOW_BODY_ST) + (data_len-6));
    if(NULL == flow_data){
       PR_ERR("recv_flow_data Malloc flow_data is fail"); 
       return 0;
    }

    memset(flow_data,0,SIZEOF(FLOW_BODY_ST) + (data_len-6));

    flow_data->step = TS_TRANSFER;
    flow_data->id = ((data[0]<<8)|data[1]);
    PR_DEBUG("id is %04x",flow_data->id);
    flow_data->offset = ((data[2]<<24)|(data[3]<<16)|(data[4]<<8)|(data[5]));
    PR_DEBUG("offset is %08x",flow_data->offset);
    flow_data->len = (data_len-6);
    flow_data->posix = uni_time_get_posix();
    PR_DEBUG("data len is %d",flow_data->len);
    memcpy((UCHAR_T *)flow_data->data,data + 6,flow_data->len);

    op_ret = tuya_iot_media_data_report(flow_data,5);
    if(op_ret != OPRT_OK){
        result = FLOW_DATA_TMOUT_E;
        goto UART_SEND;
    }

    Free(flow_data);
    flow_data = NULL;
    result = FLOW_SUCC_E;
    PR_DEBUG("data send is ok..............");

UART_SEND:
    if(flow_data) {
        Free(flow_data);
    }
    ty_uart_send(fr_type,result_len,&result);
	return 0;
}


//ty_msg_upload_proc

/***********************************************************
*  Function: get_pub_param 
*  Input: p_fr: 
*         p_fr->data采用L(长度1byte)+K(请求参数名字符串)+...,
*		  如:L:0x06 K:w.temp
*		     L:0x06 K:w.pm25
*			 L:0x0a K:w.humidity
*  Output: *ppOut: cJSON数组字符串
*          如:["w.temp","w.pm25","w.humidity"]
*  Return: 参照返回值列表
*  Note:   将LKLK...数据格式转换为cJSON数组字符串
***********************************************************/
STATIC OPERATE_RET get_pub_param(IN TY_FRAME_S *p_fr, OUT CHAR_T **ppout)
{
	WORD_T data_len = WORD_SWAP(p_fr->len);
	
	if(0 == data_len) {
		PR_ERR("data_len:%d",data_len);
		return OPRT_INVALID_PARM;
	}

	CHAR_T *buf = (CHAR_T *)Malloc(256);
	if(NULL == buf) {		
		return OPRT_MALLOC_FAILED;
	}
	
	cJSON *root = cJSON_CreateArray();
	if(NULL == root) {
	    Free(buf);
		return OPRT_CR_CJSON_ERR;
	}
	
	//PR_DEBUG("data_len:%d",data_len);
	UCHAR_T len = 0;
	INT_T i = 0;
	while(i < data_len) {
		len = p_fr->data[i++];	
		//PR_DEBUG("len:%d",len);
		if((0 == len) || (len > data_len)) {
			PR_ERR("len is %d",len);
			break;
		}
		memset(buf, 0, 256);
		memcpy(buf, p_fr->data + i,len);
		i += len;
		//PR_DEBUG("i:%d,buf:%s",i,buf);
		cJSON *json = cJSON_CreateString(buf);
		if(json) {
			cJSON_AddItemToArray(root, json);
		}
	}

	Free(buf);
    CHAR_T *out = cJSON_PrintUnformatted(root);	
    cJSON_Delete(root),root = NULL;
	if(NULL == out) {
		return OPRT_MALLOC_FAILED;
	}
	*ppout = out;	
	return OPRT_OK;
}

STATIC VOID recv_weather_open(TY_FRAME_S *p_frame)
{
#if 0
	UCHAR_T fr_type = WEATH_SERVER_CMD;
	CHAR_T *pub_para = NULL;
	CHAR_T data[2];

	OPERATE_RET op_ret = get_pub_param(p_frame,&pub_para);
	if(OPRT_OK == op_ret) {
		//pub_service_open(pub_para);//lql
		Free(pub_para);
		data[0] = SUCC_CODE;
		data[1] = 0x00;
	}else {
		//PR_ERR("get_pub_param op_ret:%d",op_ret);
		data[0] = FAIL_CODE;
		if(OPRT_INVALID_PARM == op_ret) {
			data[1] = 0x01;
		}else {
			data[1] = 0x02;
		}
	}
	ty_uart_send(fr_type,2,data);
#endif
}


STATIC INT_T recv_upgrade_start(TY_FRAME_S *p_frame)
{
    //is need stop com timer 
    sys_stop_timer(ty_msg.up_fw_timer);
    PR_DEBUG("upgrade start");

    ty_msg.prt_ver = p_frame->version;
    ty_msg.ack_stat = FW_RETURN;
    return 0;
}

STATIC INT_T recv_upgrade_transfer(VOID)
{
	//PR_DEBUG("transfer ack\r\n");
	sys_stop_timer(ty_msg.up_fw_timer);
	ty_msg.ack_stat = FW_RETURN;
	return 0;
}


STATIC INT_T recv_sys_datetime(VOID)
{
	//判断时间是否正确
	INT_T ret;
	UCHAR_T *pTime = NULL;
	UCHAR_T is_succ = 0;
	POSIX_TM_S st_time;

	memset(&st_time, 0, sizeof(POSIX_TM_S));
	ret =uni_time_get(&st_time);
	if( ret == 0 )
	{
		is_succ = 0x01;
	}
	else
	{
		is_succ = 0x00;
	}

	if( st_time.tm_year <= 100 )
	{
		is_succ = 0x00;
	}
//	PR_DEBUG("year:%d mon:%d day:%d", st_time.tm_year, st_time.tm_mon, st_time.tm_mday);
//	PR_DEBUG("hour:%d min:%d sec:%d", st_time.tm_hour, st_time.tm_min, st_time.tm_sec);
	
	pTime = (UCHAR_T *)Malloc(7+1);
	if( pTime == NULL )
	{
		PR_DEBUG("malloc failed!");
		return 1;
	}

	pTime[0] = is_succ;	
	pTime[1] = (st_time.tm_year - 100)&0xFF; //year since 2000
	pTime[2] = (st_time.tm_mon + 1)&0xFF; //mon 1 - 12
	pTime[3] = (st_time.tm_mday)&0xFF;
	pTime[4] = (st_time.tm_hour)&0xFF;
	pTime[5] = (st_time.tm_min)&0xFF;
	pTime[6] = (st_time.tm_sec)&0xFF;
	
	ty_uart_send(UT_TIME_CMD, 7, pTime);
	Free(pTime);
	return 0;
}


STATIC INT_T recv_local_datetime(VOID)
{
    UCHAR_T fr_type = LOCAL_TIME_CMD;

    UCHAR_T *pTime = (UCHAR_T *)Malloc(8+1);
	if( pTime == NULL ) {
		PR_ERR("malloc err");
		return WM_FAIL;
	}
    memset(pTime, 0, 8+1);
    
    POSIX_TM_S st_time;
    memset(&st_time, 0, sizeof(POSIX_TM_S));
    OPERATE_RET op_ret = uni_local_time_get(&st_time);
    if( OPRT_OK != op_ret ) {
        PR_ERR("get_local_time op_ret:%d",op_ret);
        goto exit;
    }

    pTime[0] = 0x01;
    pTime[1] = (st_time.tm_year-100)&0xFF;
    pTime[2] = (st_time.tm_mon+1)&0xFF;
    pTime[3] = (st_time.tm_mday)&0xFF;
    pTime[4] = (st_time.tm_hour)&0xFF;
    pTime[5] = (st_time.tm_min)&0xFF;
    pTime[6] = (st_time.tm_sec)&0xFF;
    pTime[7] = (st_time.tm_wday)&0xFF;
    if( pTime[7] == 0 ) {
        pTime[7] = 7;
    }

exit:    
	ty_uart_send(fr_type, 8, pTime);
    Free(pTime);
    return WM_SUCCESS;
}

STATIC VOID recv_func_test(VOID)
{
    UCHAR_T fr_type = WIFI_TEST_CMD;
    OPERATE_RET op_ret = OPRT_OK;
    UCHAR_T buf[2];

    // gateway base info verify
    if(0 == get_gw_cntl()->gw_base.auth_key[0] || \
       0 == get_gw_cntl()->gw_base.uuid[0]) {
        PR_NOTICE("please write uuid and auth_key first");
        buf[0] = 0x00;
        buf[1] = WF_NO_AUTH;
        ty_uart_send(fr_type, 2, buf);
        return;
    }

    AP_IF_S *ap = NULL;
    op_ret = wf_assign_ap_scan("tuya_mdev_test",&ap);//lql
    if(OPRT_OK != op_ret) {
        buf[0] = 0x00;
        buf[1] = WF_SCAN_FAIL;
    }else {
        SCHAR_T result = ap->rssi;
        buf[0] = 0x01;
        if( result <= -100 ) {
            buf[1] = 0;
        }else if( (result > -100) && (result <= -80) ) {
            buf[1] = 40;
        }else if( (result > -80) && (result <= -60) ) {
            buf[1] = 60;
        }else if( (result > -60) && (result <= -40) ) {
            buf[1] = 80;
        }else{
            buf[1] = 100;
        }
    }

    wf_release_ap(ap);
    ty_uart_send(fr_type, 2, buf);
	return;
}


STATIC VOID recv_mem_check(VOID)
{
    UCHAR_T fr_type = MEMORY_QUERY_CMD;
    CHAR_T buf[5];

    memset(buf,0,sizeof(buf));
    INT_T size = SysGetHeapSize();//lql
    
    memset(buf,0,sizeof(buf));
    buf[0] = ((size>>24)&0xFF);
    buf[1] = ((size>>16)&0xFF);
    buf[2] = ((size>>8)&0xFF);
    buf[3] = (size&0xFF);

	ty_uart_send(fr_type, 4, buf);
}

STATIC VOID send_wf_signal(VOID)
{
    SCHAR_T rssi = 0x00;
    OPERATE_RET op_ret = wf_station_get_conn_ap_rssi(&rssi);
    if(OPRT_OK != op_ret) {
        ty_uart_send(GET_WF_SIGAL_CMD, 1, &rssi);
        return;
    }

    PR_DEBUG("RSSI:%d",rssi);
    ty_uart_send(GET_WF_SIGAL_CMD, 1, &rssi);
    return;
}

STATIC VOID recv_stop_heart_beat(VOID)
{
    close_heartbeat();
    ty_msg_send(STOP_HEART_BEAT_CMD, 0, NULL);
}

STATIC INT_T recv_prod_test(VOID)
{
	PR_DEBUG("MCU have enter prod test");
}

/*重置wifi*/
STATIC INT_T reset_wifi_pro(VOID)
{
	ty_uart_send(WIFI_RESET_CMD, 0, NULL);
	tuya_iot_wf_gw_unactive(WRT_AUTO);//AP  smart cfg   切换
	return 0;
}


/*重置wifi选择模式*/
STATIC INT_T reset_wifi_select(TY_FRAME_S *p_frame)
{
	WORD_T len = WORD_SWAP(p_frame->len);
	
	ty_msg_send(WIFI_RESET_SEL_CMD, 0, NULL);
	if(ty_msg.wf_work.mode == 0)
	{
        if( len > 0 ) {
            if( p_frame->data[0] == 0 ) {
                gw_wifi_reset(WRT_SMT_CFG);
            }else {
                gw_wifi_reset(WRT_AP);
            }
        }else {
            PR_ERR("cmd para err");//lql
        }
	}
	return 0;
}

/*报告WIFI工作状态*/
STATIC INT_T upload_wifi_state(VOID)
{
	UCHAR_T fr_type = WIFI_STATE_CMD;
	UCHAR_T data;
	static INT_T num=0;
	GW_WIFI_NW_STAT_E wf_stat = 0;
    OPERATE_RET op_ret = OPRT_OK;
    op_ret = get_wf_gw_nw_status(&wf_stat);
    if(OPRT_OK != op_ret) {
        PR_ERR("get_wf_gw_nw_status error:%d",op_ret);
        return 0;
    }
	switch( wf_stat )
	{
		case STAT_UNPROVISION:
			data = WF_LIGHT_FAST_BLINK;
			break;
		case STAT_AP_STA_UNCFG:
			data = WF_LIGHT_SLOW_BLINK;
			break;
        case STAT_AP_STA_DISC:
		case STAT_STA_DISC:
			data = WF_LIGHT_OFF;
			break;
        case STAT_AP_STA_CONN:
		case STAT_STA_CONN:
			data = WF_LIGHT_ON;
			break;
		case STAT_CLOUD_CONN:
		case STAT_AP_CLOUD_CONN:
		    data = WF_MQ_ONLINE;
		    break;
		case STAT_LOW_POWER:
			if(ty_msg.prt_ver < 0x03) {
				data = WF_LIGHT_OFF;
			}else {
				data = WF_LOW_PWR;
			}
			break;
		default:
			data = 0x00;
			break;
	}
	
	return ty_msg_send(fr_type, 1, &data);
}


STATIC WORD_T ty_bitmap_len_proc(UCHAR_T bit_len) 
{
    WORD_T len;

	len = bit_len/8 + ((bit_len%8 == 0) ? 0:1);
	if( len > 4 )
		len = 4;
    return len;
}

STATIC UCHAR_T ty_dp_change_proc(UCHAR_T dp)
{
	dp++;
	return dp;
}

STATIC INT_T ty_get_obj_type_len(TY_OBJ_DP_S *obj_dp, UCHAR_T *type, WORD_T *len)
{    
	//*dpid = obj_dp->dpid;
	DEV_CNTL_N_S *dev_cntl = get_gw_dev_cntl();//lql
	INT_T i;
	DP_CNTL_S *dp_cntl =  NULL;
	for(i = 0; i < dev_cntl->dp_num; i++) {
		if(dev_cntl->dp[i].dp_desc.dp_id == obj_dp->dpid) {
			dp_cntl = &dev_cntl->dp[i];
			break;
		}
	}
	
	if(NULL == dp_cntl) {
		PR_ERR("dp_cntl is NULL");
		return WM_FAIL;
	} 
	
	// find dp
	if(dp_cntl->dp_desc.mode == M_RO) {
		PR_ERR("dp_cntl->dp_desc.mode is M_RO");
		return WM_FAIL;
	}
	
	//if(dp_cntl->dp_desc.type == T_RAW) {
		//*type = 0x00;
		//*len = strlen(root->valuestring);//lql
	//}else 
	if(dp_cntl->dp_desc.type == T_OBJ) {
		switch(dp_cntl->dp_desc.prop_tp) {
			case PROP_BOOL:   *type = 0x01; *len = 0x01; break;
			case PROP_VALUE:  *type = 0x02; *len = 0x04; break;
			case PROP_STR:    *type = 0x03; *len = strlen(obj_dp->value.dp_str); break;
			case PROP_ENUM:   *type = 0x04; *len = 0x01; break;
			case PROP_BITMAP: *type = 0x05; *len = ty_bitmap_len_proc(dp_cntl->prop.prop_bitmap.max_len); break;
			default:          *type = 0x00; *len = 0; return WM_FAIL;        
		}
	}else {
        PR_DEBUG("dp type is err (not obj_data)");
        return WM_FAIL;
	}
	//PR_DEBUG("*dpid[%d], *type[%d], *len[%d]", *dpid, *type, *len);
	return WM_SUCCESS;
}


STATIC UCHAR_T *ty_get_enum_str(DP_CNTL_S *dp_cntl, UCHAR_T enum_id)
{
	if( dp_cntl == NULL ) {
		return NULL;
	}

	if( enum_id >= dp_cntl->prop.prop_enum.cnt ) {
		return NULL;
	}
	
	return dp_cntl->prop.prop_enum.pp_enum[enum_id];	
}


STATIC INT_T ty_get_enum_id(UCHAR_T dpid, UCHAR_T *enum_str)
{
	UCHAR_T i = 0;
	UCHAR_T enum_id = 0;
	DP_CNTL_S *dp_cntl =  NULL;	
	DEV_CNTL_N_S *dev_cntl = get_gw_dev_cntl();//lql

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


STATIC INT_T ty_data_proc(TY_OBJ_DP_S *obj_dp, UCHAR_T type, WORD_T len, UCHAR_T *buf)
{    
	INT_T base64_len = len;
	switch(type) {
		//case DP_RAW: {
			//base64_len = base64_decode(root->valuestring, buf);
			//PR_DEBUG("base64_len=%d",base64_len);
		//} 
		//break;
		
		case DP_BOOL: {
			if(obj_dp->value.dp_bool == cJSON_False)
				buf[0] = 0x00;
			else
				buf[0] = 0x01;
		}         
		break;
		
		case DP_VALUE: {
			UINT val_data = DWORD_SWAP(obj_dp->value.dp_value);
			// PR_DEBUG("val_data[%d], root->valueint[%d]", val_data, root->valueint);
			memcpy(buf, &val_data, len);
		}
		break;
		
		case DP_STRING: memcpy(buf, obj_dp->value.dp_str, len); break;
		case DP_ENUM: {
			buf[0] = obj_dp->value.dp_enum;//ty_get_enum_id(atoi(root->string), root->valuestring);
		}
		break;
		case DP_BITMAP: {
			if(len == 1) {
				buf[0] = obj_dp->value.dp_bitmap;
			}else if(len == 2) {
				WORD_T bit = WORD_SWAP(obj_dp->value.dp_bitmap);
				memcpy(buf, &bit, len); 
			}else {
				UINT bit = DWORD_SWAP(obj_dp->value.dp_bitmap);
				memcpy(buf, &bit, len);
			}				            
		}
		break;
		
		default: break;    
	}
	return base64_len;
}
STATIC VOID ty_raw_datapoint_proc(TY_RECV_RAW_DP_S *raw_dp)//need to improve lql
{
    WORD_T len;
    
    len = raw_dp->len;

    UCHAR* SendBuf = (UCHAR*)Malloc(4 + len + 1);
    if(SendBuf == NULL) {
        PR_ERR("SendBuf is NULL");
        return;
    }
    memset(SendBuf, 0, 4 + len + 1);
    TY_DATAPOINT_DATA_S *dp_data = (TY_DATAPOINT_DATA_S *)SendBuf;
    dp_data->dpid = raw_dp->dpid;
    dp_data->type = 0x0;//raw

    memcpy(dp_data->data, raw_dp->data, len);

    dp_data->len = WORD_SWAP(len);
    
    ty_uart_send(DATA_CTRL_CMD, 4 + len, SendBuf);

    Free(SendBuf);
}



STATIC VOID ty_obj_datapoint_proc(TY_OBJ_DP_S *obj_dp)
{
	UCHAR_T type;
	WORD_T len, rawlen;
	INT_T ret;
	
	ret = ty_get_obj_type_len(obj_dp, &type, &len);//lql
	if( ret != WM_SUCCESS)
	{
		PR_ERR("ty_type_len_proc fail");
		return;
	}
	
	UCHAR_T* SendBuf = (UCHAR_T*)Malloc(4 + len + 1);
	if(SendBuf == NULL) {
		PR_ERR("SendBuf is NULL");
		return;
	}
	memset(SendBuf, 0, 4 + len + 1);
	TY_DATAPOINT_DATA_S *dp_data = (TY_DATAPOINT_DATA_S *)SendBuf;
	dp_data->dpid = obj_dp->dpid;
	dp_data->type = type;
#if 0		
	dp_data->len = WORD_SWAP(len);
	rawlen = ty_data_proc(root, type, len, dp_data->data);
	PR_DEBUG("rawlen=%d,len=%d",rawlen,len);
	if( type == DP_RAW )
	{
		dp_data->len = WORD_SWAP(rawlen);
		ty_uart_send(DATA_CTRL_CMD, 4 + rawlen, SendBuf); 
	}
	else
	{
		ty_uart_send(DATA_CTRL_CMD, 4 + len, SendBuf); 
	}	
#else
    rawlen = ty_data_proc(obj_dp, type, len, dp_data->data);
    dp_data->len = WORD_SWAP(rawlen);
    ty_uart_send(DATA_CTRL_CMD, 4 + rawlen, SendBuf); 
#endif
	Free(SendBuf);
}

STATIC VOID ty_datapoint_upload_proc(WORD_T len, UCHAR_T *data)
{
    if( data == NULL ) {
        PR_ERR("para err");
        return;
    }

    if( !ty_msg.init_flag || ty_msg.IsInUpgrade) {
        PR_ERR("ty_mag not init");
        return;
    }

    GW_WIFI_NW_STAT_E cur_nw_stat;
    get_wf_gw_nw_status(&cur_nw_stat);
    if((cur_nw_stat < STAT_STA_CONN) && (cur_nw_stat != STAT_AP_STA_CONN)) {
        //PR_NOTICE("device not online");
        return;
    }

    //PR_NOTICE("cmd to cloud");
	#if 0
    if( UPGRADING == get_fw_ug_stat()) {//lql
		PR_DEBUG("device is updating");
		return;
	}
    #endif
    
	WORD_T offset = 0;
	INT_T i;
	DEV_CNTL_N_S *dev_cntl = get_gw_dev_cntl();//lql
	if( dev_cntl == NULL )
		return;

	DP_CNTL_S *dp_cntl =  NULL;
	CHAR_T dpid[10], dpdata[10],*out = NULL;
	cJSON *root_test = NULL;
	OPERATE_RET op_ret;
	root_test = cJSON_CreateObject();
	if(NULL == root_test) {
		return;
	}
	len = WORD_SWAP(len);
	//PR_DEBUG("total len[%d]", len);

    BOOL obj_upload = FALSE;
#if 0
	PR_DEBUG("recv:");
	for( i = 0; i < len; i ++ )
	{
		PR_DEBUG_RAW("%02X ", data[i]);
	}
	PR_DEBUG("");
#endif
//55 aa 03 07       00 07     02 00 04 00 00 00 1e       34
    TY_DATAPOINT_DATA_S *dp_data = NULL;
	while((len - offset) >= TY_DP_FRAME_MIN) {
		dp_data = (TY_DATAPOINT_DATA_S *)(data + offset);		   
	 	//PR_DEBUG("dpid[%d], type[%d]", dp_data->dpid, dp_data->type);
		if(dp_data->type > TY_DP_TYPE)
		{
			PR_ERR("dp_data->type=%d",dp_data->type);
			goto exit;
		}
		//PR_DEBUG("dev_cntl->dp_num=%d", dev_cntl->dp_num);
		for(i = 0; i < dev_cntl->dp_num; i++) {
			if(dev_cntl->dp[i].dp_desc.dp_id == dp_data->dpid) {
			    if(((dev_cntl->dp[i].dp_desc.type == T_RAW) && (dp_data->type == 0)) || \
			      ((dev_cntl->dp[i].dp_desc.type == T_OBJ) && (dp_data->type >= 1) && (dp_data->type <= 5))) {
    				dp_cntl = &dev_cntl->dp[i];
    				break;
				}else {
				    PR_ERR("dpid and type is not match");
				}
			}
		}
		if(i >= dev_cntl->dp_num) {
			PR_ERR("no find dp_num");
			goto exit;
		}
		//PR_DEBUG("dp_data->type[%d], ty_dp_change_proc(dp_cntl->dp_desc.prop_tp[%d])", dp_data->type, ty_dp_change_proc(dp_cntl->dp_desc.prop_tp));
		if((dp_data->type != ty_dp_change_proc(dp_cntl->dp_desc.prop_tp)) && (dp_data->type != 0))
		{
			PR_ERR("a...b...c...");
			goto exit;
		}
		snprintf(dpid,10,"%d",dp_data->dpid);
		dp_data->len = WORD_SWAP(dp_data->len);
		//PR_DEBUG("dp_data->dpid[%d],dp_data->len[%d],dp_data->type[%d]\r\n", dp_data->dpid, dp_data->len,dp_data->type);
		switch(dp_data->type) {
		case DP_RAW: {
		#if 0
	        CHAR_T time[32];
            memset(time, 0, sizeof(time));
            TIME_T curtime = uni_time_get_posix();
            sprintf(time,"{\"%d\":%u}",dpid,curtime);//lql
            
            cJSON_AddBoolToObject(root_test,dpid,dp_data->data[0]);//lql
            
            out=cJSON_PrintUnformatted(root_test);
        	if(NULL == out) {
        		PR_ERR("cJSON_PrintUnformatted err:");
        		cJSON_Delete(root_test);
        		return;
        	}
        #endif
            PR_NOTICE("dev_report_dp_raw_sync in");
            op_ret = dev_report_dp_raw_sync(get_gw_cntl()->dev->dev_if.id,dp_data->dpid,dp_data->data,dp_data->len,5);
			//op_ret = sf_raw_dp_report_sync(get_gw_cntl()->dev->dev_if.id,out,time,2000);PR_NOTICE("&&&out**********");
            PR_NOTICE("dev_report_dp_raw_sync out");
            if(OPRT_OK != op_ret) {
                goto exit;
            }
		}
		break;		
		case DP_BOOL: {		
            if((dp_data->len != 0x01) || (dp_data->data[0] < 0) || (dp_data->data[0] > 1))
            {
				PR_ERR("dp_data->len:%d,dp_data->data[0]:%d",dp_data->len,dp_data->data[0]);
				goto exit;
            }
            cJSON_AddBoolToObject(root_test,dpid,dp_data->data[0]);
            obj_upload = TRUE;
            //PR_NOTICE("bool__dpid:%d",dpid);
        }
		break;		
		case DP_VALUE: {		
		    INT_T val_data = 0;
		    memcpy(&val_data, dp_data->data, dp_data->len);		
			val_data = DWORD_SWAP(val_data);
		    if((dp_data->len != 0x04) || (val_data < dp_cntl->prop.prop_value.min) || \
		        (val_data > dp_cntl->prop.prop_value.max))
		    {
				PR_ERR("val_data = %d", val_data);
				PR_ERR("data[0] = %d", dp_data->data[0]);
				PR_ERR("data[1] = %d", dp_data->data[1]);
				PR_ERR("data[2] = %d", dp_data->data[2]);
				PR_ERR("data[3] = %d", dp_data->data[3]);
				PR_ERR("min:%d,max%d", dp_cntl->prop.prop_value.min, dp_cntl->prop.prop_value.max);
				PR_ERR("555");
				goto exit;
		    }
		    cJSON_AddNumberToObject(root_test,dpid,val_data);
            obj_upload = TRUE;
		}
		break;

		case DP_STRING: {
		    if(dp_data->len > dp_cntl->prop.prop_str.max_len) {
				PR_ERR("dp_data->len=%d", dp_data->len);
				PR_ERR("dp_cntl->prop.prop_str.max_len=%d", dp_cntl->prop.prop_str.max_len);
		        op_ret = OPRT_DP_TYPE_PROP_ILLEGAL;
		        goto exit;
		    }
			UCHAR_T *pData = NULL;
			pData = Malloc(dp_data->len + 1);
			if( pData == NULL )
			{
				op_ret = OPRT_BUF_NOT_ENOUGH;
		        goto exit;
			}
			memset(pData, 0, dp_data->len + 1);
			memcpy(pData,dp_data->data,dp_data->len);
		    cJSON_AddStringToObject(root_test,dpid,pData);
			Free(pData);
            obj_upload = TRUE;
			break;
		}

		case DP_ENUM: { //转为字符串传输
		    if(dp_data->len != 0x01)
		    {
				PR_ERR("DP_ENUM dp_data->len=%d",dp_data->len);
		        goto exit;
		    }
		    //snprintf(dpdata,10,"%d",dp_data->data[0]);
		    if( NULL != ty_get_enum_str(dp_cntl, dp_data->data[0]) ) {
		        cJSON_AddStringToObject(root_test,dpid,ty_get_enum_str(dp_cntl, dp_data->data[0]));
		    }else {
                PR_ERR("DP_ENUM out of range");
                goto exit;
		    }
            obj_upload = TRUE;
		}
		break;

		case DP_BITMAP: {
//			PR_DEBUG("dp_data->len[%d],(1 << dp_cntl->prop.prop_bitmap.max_len)[%d]", dp_data->len, (1 << dp_cntl->prop.prop_bitmap.max_len));
            //if((dp_data->len) > (1 << dp_cntl->prop.prop_bitmap.max_len)) {
			if((dp_data->len) > 4) {
		        op_ret = OPRT_DP_TYPE_PROP_ILLEGAL;
				PR_ERR("dp_data->len:%d,dp_cntl->prop.prop_bitmap.max_len:%d",dp_data->len,dp_cntl->prop.prop_bitmap.max_len);
		        goto exit;
		    }
			INT_T bitdata = 0;
			if(dp_data->len == 1) {
				bitdata = dp_data->data[0];
			}else if(dp_data->len == 2) {
 				bitdata = ((dp_data->data[0])<<8) +  dp_data->data[1];
			}else {
				bitdata = ((dp_data->data[0])<<24) +  ((dp_data->data[1])<<16) +  ((dp_data->data[2])<<8) +  dp_data->data[3];
			}
		    cJSON_AddNumberToObject(root_test,dpid,bitdata);
            obj_upload = TRUE;
		}
		break;

		default: break;
		}
		offset += (4 + dp_data->len);
	}

    if(FALSE == obj_upload) {
        cJSON_Delete(root_test);
        return;
    }
    
	out=cJSON_PrintUnformatted(root_test);
	cJSON_Delete(root_test);
	if(NULL == out) {
		PR_ERR("cJSON_PrintUnformatted err:");
		return;
	}
//	PR_DEBUG("out:%s", out);

	op_ret = sf_obj_dp_report_async(get_gw_cntl()->dev->dev_if.id,out,false);
	if(OPRT_OK != op_ret) {
		//PR_ERR("sf_obj_dp_report op_ret:%d,out:%s");
		Free(out);
		return;
	}
	Free(out);
	return;

	exit: {
		cJSON_Delete(root_test);
		PR_ERR("op_ret:%d,raw dp or dp attribute is abnormal,dp_id=%d", op_ret,dp_data->dpid);
		return;
	}
}


/*心跳发送*/
STATIC VOID jump_send_timer_cb(UINT timerID,PVOID pTimerArg)
{
    OPERATE_RET op_ret = OPRT_OK;
    if((!ty_msg.IsInUpgrade) && (!ty_msg.StopHartBeat)){  //处于升级状态不发心跳包
        //发心跳包
        send_jump_pack();
    }
    
    if(!ty_msg.init_flag) {  //mcu未初始化reset         || ty_msg.IsMcuUgReset  或者mcu 
        //3s 心跳
        op_ret = sys_start_timer(ty_msg.jump_send_timer,JUMP_FIRST_TIME*1000,TIMER_ONCE);
        if(op_ret != OPRT_OK) {
            PR_ERR("jump_send_timer timer 3s start err");
        }
    }else {
        //15s 心跳
        op_ret = sys_start_timer(ty_msg.jump_send_timer,JUMP_SEND_TIME*1000,TIMER_ONCE);
        if(op_ret != OPRT_OK) {
            PR_ERR("jump_send_timer timer 15s start err ");
        }
    }
}
/*心跳监控定时器*/
STATIC VOID guard_timer_cb(UINT timerID,PVOID pTimerArg)
{
	if(!ty_msg.init_flag || ty_msg.IsInUpgrade || ty_msg.IsMcuUgReset || ty_msg.StopHartBeat) {
		return;
	}
    
	PR_NOTICE("guard time out,go to reset!!! ");
	SystemReset();
}


STATIC VOID start_guard_timer(UCHAR_T sec)
{
    sys_start_timer(ty_msg.guard_timer,sec*1000,TIMER_CYCLE);
}


STATIC VOID clear_all_queue(VOID)
{
	INT_T i = 0, ret = 0;
	INT_T n = GetCurQueNum(ty_msg.pQueue);
	for(i = 0; i < n; i ++) {
		TY_SEND_DATA_S *pSend = NULL;
		ret = OutQueue(ty_msg.pQueue, (UCHAR_T *)&pSend, 1);
		if(0 != ret) {			
			DelQueueMember(ty_msg.pQueue, 1);
			Free(pSend);
		}
	}
}

/*发送超时*/
STATIC VOID com_send_timer_cb(UINT timerID,PVOID pTimerArg)
{
	PR_DEBUG("com time out");
	INT_T ret = 0;
	
	ty_msg.try_cnt ++;
	if( ty_msg.try_cnt < 2 ) {
		return;
	}
	
	if(0 == GetCurQueNum(ty_msg.pQueue)) {
		return;
	}

	TY_SEND_DATA_S *pSend = NULL;
	ret = GetQueueMember(ty_msg.pQueue, 1, (UCHAR_T *)&pSend, 1);
	if((0 == ret) || (NULL == pSend)) {
		return;
	}

	TY_FRAME_S *pFrame = (TY_FRAME_S *)pSend->data;
	if( pFrame != NULL) {
		if(pFrame->fr_type == HEART_BEAT_CMD) { //HB
			clear_all_queue();
			PR_DEBUG("clear all");
		}else {
			PR_DEBUG("clear one");
			Free(pSend);
			DelQueueMember(ty_msg.pQueue, 1);
		}			
	}
	ty_msg.try_cnt = 0;
}

STATIC VOID up_fw_timer_cb(UINT timerID,PVOID pTimerArg)
{
	PR_DEBUG("time out fw cb");
	ty_msg.ack_stat = FW_TIME_OUT;
}


/*数据发送定时器*/
STATIC VOID queue_send_timer_cb(UINT timerID,PVOID pTimerArg)
{
	BOOL is_run = FALSE;
	TY_SEND_DATA_S *pSend = NULL;
	INT_T qNum = 0;

	//上次发送数据还未返回
	is_run = IsThisSysTimerRun(ty_msg.com_send_timer);
	if( is_run ) {
        return;
	}

	qNum = GetCurQueNum(ty_msg.pQueue);	
	if( qNum > 0 )
	{
		GetQueueMember(ty_msg.pQueue, 1, (UCHAR_T *)&pSend, 1);
		//PR_DEBUG("qqq num=%d", qNum);
		if( pSend == NULL )
		{
			PR_ERR("pSend is NULL");
			return;
		}
		
		MutexLock(ty_msg.mutex);	
		ty_uart_send_data(TY_UART0,pSend->data, pSend->len);
		MutexUnLock(ty_msg.mutex);
		sys_start_timer(ty_msg.com_send_timer,QRY_SEND_TIMR_OUT_MS,TIMER_ONCE);
		
#ifdef UART_TX_DEBUG
		INT_T i = 0;
		for(i = 0; i < pSend->len; i++) {
            PR_DEBUG_RAW("%02X ",pSend->data[i]);
		}
#endif

	}
}

/*数据处理*/
STATIC INT_T ty_msg_upload_proc(UCHAR_T* data)
{
	INT_T ret = 0;
    TY_FRAME_S *ty_frame = (TY_FRAME_S *)data;
    switch(ty_frame->fr_type) {
		PR_DEBUG("recv type:%02X", ty_frame->fr_type);
		/*心跳检测*/
        case HEART_BEAT_CMD: {
			recv_jump_pack(ty_frame);
        }
        break;
			
        /*产品信息*/
        case PRODUCT_INFO_CMD: {
			ret = recv_prod_info(ty_frame);
		}
        break;
			
        /*工作模式*/
        case WORK_MODE_CMD: {
			ret = recv_work_mode(ty_frame);
			if( ret == 0 ) {
				if( ty_msg.wf_work.mode == 0 ) {
					upload_wifi_state();
				}
				query_dp_state();
			}
        }
        break; 
			
        /*WIFI工作状态*/
        case WIFI_STATE_CMD: {
            recv_wifi_ack();
        }
            break;
			
        /*重置WIFI*/
        case WIFI_RESET_CMD: {
			reset_wifi_pro();
        }
            break;
			
        /*重置WIFI选择模式*/
        case WIFI_RESET_SEL_CMD: {
			reset_wifi_select(ty_frame);
        }
            break;
			
		/*dp数据处理*//*命令下发*/
		//case 0x06:
			//break;
			
		/*状态上报*/
		case DATA_RPT_CMD:
            //PR_DEBUG("upload fr_type = %d", ty_frame->fr_type);
			//PR_NOTICE("get cmd->cloud dpid:%x",ty_frame->data[0]);
			//PR_NOTICE("DATA_RPT_CMD in");
            ty_datapoint_upload_proc(ty_frame->len, ty_frame->data);
            //PR_NOTICE("DATA_RPT_CMD out");
		    break;
			
		/*状态查询*/
		case DATA_QUERY_CMD:
		    break;
			
		//case 0x09:
			//recv_upgrade_query();
			//break;
			
		case UPDATE_START_CMD:
			recv_upgrade_start(ty_frame);
			break;
			
		case UPDATE_TRANS_CMD:
			recv_upgrade_transfer();
			break;
			
		case UT_TIME_CMD:
			recv_sys_datetime();
			break;

        case LOCAL_TIME_CMD:
            recv_local_datetime();
            break;
			
		//case 0x0d:
			//recv_prod_test();
			//break;
            
		case WIFI_TEST_CMD:
            recv_func_test();        
            break;
            
        case MEMORY_QUERY_CMD:
            recv_mem_check();
            break;

        case WEATH_SERVER_CMD:
            recv_weather_open(ty_frame);
            break;
            
        case WEATH_DATA_CMD:
            recv_weather_ack();
            break;
        //new add
        case GET_WF_SIGAL_CMD:
            send_wf_signal();
            break;
        case STOP_HEART_BEAT_CMD:
            recv_stop_heart_beat();
            break;
        case OPEN_FLOW_CONNECT_CMD:
            recv_connect_flow();
            break;
        case FLOW_DATA_START_CMD:
            recv_flow_start(ty_frame->data);
            break;
        case FLOW_DATA_TRANS_CMD:
            recv_flow_data(ty_frame->len,ty_frame->data);
            break;
        case FLOW_DATA_END_CMD:
            recv_flow_end(ty_frame->data);
            break;

        default: break;
    }

	return WM_SUCCESS;
}

/*串口接收*/
STATIC VOID ty_uart_recv_proc(VOID)
{
    UINT_T count = ty_uart_read_data(TY_UART0,ty_msg.proc_buf + ty_msg.rcount, HN_RECV_BUF_MAX - ty_msg.rcount);
	ty_msg.rcount += count;
	if( ty_msg.rcount > 0 ) { //count > 0 
	    PR_DEBUG("ty_msg.rcount:%d,count:%d",ty_msg.rcount,count);
    	ty_msg.recv_state = UART_PROC;
	}
}


/*消息处理*/
STATIC VOID ty_dp_proc(VOID)
{
	INT_T fr_len = 0, offset = 0;
	UCHAR_T check_num = 0;
    while((ty_msg.rcount - offset) >= HN_RECV_MIN_NUM) {
    	if(ty_msg.proc_buf[offset] != 0x55) {
            //PR_DEBUG("ty_msg.proc_buf[0][0x%x]", ty_msg.proc_buf[offset]);
            offset += 1;
            continue;  
        }
        if(ty_msg.proc_buf[offset + 1] != 0xaa) {
            //PR_DEBUG("ty_msg.proc_buf[1][0x%x]", ty_msg.proc_buf[offset + 1]);
            offset += 1;
            continue;
        }
        fr_len = ty_msg.proc_buf[offset + 4]*256 + ty_msg.proc_buf[offset + 5] + 6 + 1;
        //PR_DEBUG("proc fr_len=%d, (ty_msg.rcount - offset)[%d]", fr_len, (ty_msg.rcount - offset));
        if(fr_len > HN_RECV_BUF_MAX) {
            //PR_DEBUG("fr_len:%d",fr_len);
            offset += 1;
            continue;
        }
        if(ty_msg.rcount - offset < fr_len) {
            PR_DEBUG("ty_msg.rcount:%d,offset:%d,fr_len:%d",ty_msg.rcount,offset,fr_len);
            //break;
            ty_msg.recv_state = UART_RECV;
            return;
        }
        check_num = getCheckSum(ty_msg.proc_buf + offset, fr_len - 1);
        if(check_num != ty_msg.proc_buf[offset + fr_len - 1]) {
			//PR_DEBUG("check_num[%d], ty_msg.proc_buf[offset + fr_len - 1][%d]", check_num, ty_msg.proc_buf[offset + fr_len - 1]);
            //PR_DEBUG("offset:%d,fr_len:%d",offset,fr_len);
            offset += fr_len;
            continue;
        }
        //PR_DEBUG("ty_msg_upload_proc");
        ty_msg_upload_proc(ty_msg.proc_buf + offset); 
       	offset += fr_len;
    }

	//PR_DEBUG("ty_msg.rcount=%d,offset=%d\r\n", ty_msg.rcount, offset);
	if( offset > 0 ) {
        ty_msg.rcount -= offset;
        if(ty_msg.rcount > 0) {
            memmove(ty_msg.proc_buf, ty_msg.proc_buf + offset, ty_msg.rcount);
        }
	}
    ty_msg.recv_state = UART_RECV;
}

STATIC VOID ty_task_cb(PVOID pArg)
{
	ty_msg.recv_state = UART_RECV;
	ty_msg.rcount = 0;
    ty_uart_init(TY_UART0,TYU_RATE_9600,TYWL_8B,TYP_NONE,TYS_STOPBIT1,1024);//TYU_RATE_115200
	while(1) {
		SystemSleep(50);
		switch(ty_msg.recv_state) {
			case UART_RECV:
				ty_uart_recv_proc();
				break;
			case UART_PROC:
				ty_dp_proc();
				break;
			default:
			    PR_DEBUG("########ty_task_cb default");
				break;
		}
	}
}

VOID set_wifi_light(UCHAR_T stat)
{
	UCHAR_T fr_type = WIFI_STATE_CMD;
	UCHAR_T data = stat;	

	ty_msg_send(fr_type, 1, &data);
}

/***********************************************************
*************************function define********************
       status_changed_cb,\
       gw_ug_inform_cb,\
       dev_obj_dp_cb,\
       dev_raw_dp_cb,\
       dev_dp_query_cb,\

***********************************************************/
VOID status_changed_cb(IN CONST GW_STATUS_E status)
{
    PR_DEBUG("gw status changed to %d", status);
}

OPERATE_RET get_file_data_cb(IN CONST FW_UG_S *fw, IN CONST UINT total_len, IN CONST UINT offset,
                                     IN CONST BYTE_T *data, IN CONST UINT len, OUT UINT *remain_len, IN PVOID pri_data)
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

VOID dev_obj_dp_cb(IN CONST TY_RECV_OBJ_DP_S *dp)
{
    if(!ty_msg.init_flag || ty_msg.IsInUpgrade || ty_msg.IsMcuUgReset) {
        PR_ERR("mcu is not ready initflag[%d] IsInUpgrade[%d] IsMcuReset[%d]",ty_msg.init_flag,ty_msg.IsInUpgrade,ty_msg.IsMcuUgReset);
        return;
    }
    
    if( !get_gw_dev_cntl()->dev_if.bind ) {//gw_if.bind
        PR_ERR("dev is not bind");
        return;//未绑定
    }
    INT_T i = 0;
    for(i = 0;i < dp->dps_cnt;i++) {
        ty_obj_datapoint_proc(&dp->dps[i]);
        SystemSleep(40);
    }
}


VOID dev_raw_dp_cb(IN CONST TY_RECV_RAW_DP_S *dp)
{
    if(!ty_msg.init_flag || ty_msg.IsInUpgrade || ty_msg.IsMcuUgReset) {
        PR_ERR("mcu is not ready initflag[%d] IsInUpgrade[%d] IsMcuReset[%d]",ty_msg.init_flag,ty_msg.IsInUpgrade,ty_msg.IsMcuUgReset);
        return;
    }
    
    PR_DEBUG("dpid:%d recv len:%d",dp->dpid,dp->len);
    if( !get_gw_dev_cntl()->dev_if.bind ) {//gw_if.bind
        PR_ERR("dev is not bind");
        return;//未绑定
    }
    
    ty_raw_datapoint_proc(dp);
}

STATIC VOID __get_wf_status(IN CONST GW_WIFI_NW_STAT_E stat)
{
    
    PR_NOTICE("wifi status is :%d",stat);
    if(ty_msg.wf_work.mode == 0)
	{
		upload_wifi_state();
	}
	else if(ty_msg.wf_work.mode == 1)
	{
        switch(stat) {
            case STAT_UNPROVISION: {
                 tuya_set_led_light_type(wifi_light,OL_FLASH_HIGH,250,LED_TIMER_UNINIT);//250ms flash
                 break;
            }
            case STAT_AP_STA_UNCFG: {
                 tuya_set_led_light_type(wifi_light,OL_FLASH_HIGH,1500,LED_TIMER_UNINIT);//1500ms flash
                 break;
            }
            case STAT_AP_STA_DISC:
            case STAT_STA_DISC: {
                 tuya_set_led_light_type(wifi_light,OL_HIGH,0,0);//led down
                 break;
            }
            case STAT_AP_STA_CONN:
            case STAT_STA_CONN:
            case STAT_CLOUD_CONN: 
            case STAT_AP_CLOUD_CONN:{
                tuya_set_led_light_type(wifi_light,OL_LOW,0,0);//led up
                break;
            }
            default:{
                tuya_set_led_light_type(wifi_light,OL_LOW,0,0);//default:led up
                break;
            }
        }
    }
    return;
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
    
    PR_DEBUG("device_init start");

	memset(&ty_msg, 0, sizeof(TY_MSG_S));
	ty_msg.online_flag = FALSE;
	ty_msg.init_flag = FALSE;
    ty_msg.prt_ver = 0x00;
    ty_msg.IsInUpgrade = FALSE;
    ty_msg.IsMcuUgReset = FALSE;
    ty_msg.StopHartBeat = FALSE;

    ty_msg.pQueue = CreateQueueObj(QUEUE_MAX_EVENTS, sizeof(UCHAR_T *));
    if(ty_msg.pQueue == NULL) {
        PR_ERR("CreateQueueObj failed");
        return -1;
    }
    
    op_ret = sys_add_timer(jump_send_timer_cb,NULL,&ty_msg.jump_send_timer);
    if(OPRT_OK != op_ret) {
        PR_ERR("add jump_send_timer err");
        return op_ret;
    }else {
        //start_jump_timer(JUMP_FIRST_TIME);
        sys_start_timer(ty_msg.jump_send_timer,100,TIMER_ONCE); //加快启动速度
    }

    op_ret = sys_add_timer(queue_send_timer_cb,NULL,&ty_msg.queue_send_timer);
    if(OPRT_OK != op_ret) {
        PR_ERR("add queue_send_timer err");
        return op_ret;
    }else {
        sys_start_timer(ty_msg.queue_send_timer,50,TIMER_CYCLE);
    }
    
    op_ret = sys_add_timer(guard_timer_cb,NULL,&ty_msg.guard_timer);
    if(OPRT_OK != op_ret) {
        PR_ERR("add guard_timer_cb err");
        return op_ret;
    }else {
        start_guard_timer(WF_GUARD_TIME);  //在每次收到心跳返回包重置监控定时器
    }

    op_ret = sys_add_timer(com_send_timer_cb,NULL,&ty_msg.com_send_timer);
    if(OPRT_OK != op_ret) {
        PR_ERR("add com_send_timer err");
        return op_ret;
    }

    op_ret = sys_add_timer(up_fw_timer_cb,NULL,&ty_msg.up_fw_timer);
    if(OPRT_OK != op_ret) {
        PR_ERR("add up_fw_timer err");
        return op_ret;
    }   

    op_ret = CreateMutexAndInit(&ty_msg.mutex);
    if(op_ret != OPRT_OK) {
        PR_ERR("create mutex err");
        return op_ret;
    }

    #if 1
    THRD_HANDLE ty_user_handle = NULL;
    THRD_PARAM_S thrd_param;
    thrd_param.priority = TRD_PRIO_2;//2
    thrd_param.stackDepth = 1024*2;
    thrd_param.thrdname = "ty_task";
    op_ret = CreateAndStart(&ty_user_handle,NULL,NULL,\
                           ty_task_cb,NULL,&thrd_param);
    #endif 
    if(op_ret != OPRT_OK) {
        PR_ERR("start thread ty_task_cb err");
        return op_ret;
    }
	PR_NOTICE("device_init ok");
    return OPRT_OK;
}

STATIC VOID key_process(TY_GPIO_PORT_E port,PUSH_KEY_TYPE_E type,INT_T cnt){
    PR_DEBUG("port: %d",port);
    PR_DEBUG("type: %d",type);
    PR_DEBUG("cnt: %d",cnt);
    
    if(ty_msg.wf_work.reset_pin == port){
        if(LONG_KEY == type) {
             tuya_iot_wf_gw_unactive(WRT_AUTO);//AP  smart cfg   切换
        }    
    }
}

#define MCU_GU_ERR 0
#define MCU_GU_OK 1
VOID dev_ug_inform_cb(IN CONST CHAR_T *dev_id,IN CONST FW_UG_S *fw)
{
    PR_DEBUG("Rev GW Upgrade Info");
    PR_DEBUG("fw->fw_url:%s", fw->fw_url);
    PR_DEBUG("fw->fw_md5:%s", fw->fw_md5);
    PR_DEBUG("fw->sw_ver:%s", fw->sw_ver);
    PR_DEBUG("fw->file_size:%d", fw->file_size);
    OPERATE_RET op_ret = OPRT_OK;

    memset(&dev_proc,0,sizeof(DEV_UG_PROC_S));
    dev_proc.stat = MCU_GU_OK;
    ty_msg.IsInUpgrade = TRUE;
    dev_proc.file_size = fw->file_size;
    op_ret = tuya_iot_upgrade_dev(dev_id,fw,dev_ug_process_cb,dev_ug_notify_cb,NULL);
    if(OPRT_OK != op_ret) {
        PR_ERR("tuya_iot_upgrade_dev err:%d",op_ret);
        dev_proc.stat = MCU_GU_ERR;
        ty_msg.IsInUpgrade = FALSE;
        return;
    }

    op_ret = upgrade_pre_cb(dev_proc.file_size);
    if(op_ret != OPRT_OK) {
        PR_ERR("inform MCU to upgrade failed!");
        dev_proc.stat = MCU_GU_ERR;
        ty_msg.IsInUpgrade = FALSE;
        return;
    }
}

STATIC OPERATE_RET dev_ug_process_cb(IN CONST FW_UG_S *fw, IN CONST UINT_T total_len,IN CONST UINT_T offset,\
                              IN CONST BYTE_T *data,IN CONST UINT_T len,OUT UINT_T *remain_len, IN PVOID_T pri_data)
{
    if(dev_proc.stat == MCU_GU_ERR) {
        return OPRT_COM_ERROR;
    }
    #define SEND_UNIT 256
    UINT_T send_len= 0,send_cnt = 0;
    UINT_T i = 0,j = 0;
    if((len < SEND_UNIT) && (dev_proc.send_len + len < dev_proc.file_size)) {
        *remain_len = len;
         return OPRT_OK;
    }
    if(dev_proc.send_len >= dev_proc.file_size) {
        *remain_len = 0;
         return OPRT_OK;
    }
    OPERATE_RET op_ret = OPRT_OK;
    send_len = len;
    while(send_len >= SEND_UNIT) {
        op_ret = upgrade_file_cb(&data[len - send_len], SEND_UNIT);
        if(op_ret != OPRT_OK) {
            PR_ERR("MCU upgrade file failed!");
            ty_msg.IsInUpgrade = FALSE;
            return OPRT_COM_ERROR;
        }       
        
        dev_proc.send_len += SEND_UNIT;
        send_len -= SEND_UNIT;
    }
    
    if(send_len >= dev_proc.file_size - dev_proc.send_len) {
        op_ret = upgrade_file_cb(&data[len - send_len], dev_proc.file_size - dev_proc.send_len);
        if(op_ret != OPRT_OK) {
            PR_ERR("MCU upgrade file failed!");
            ty_msg.IsInUpgrade = FALSE;
            return OPRT_COM_ERROR;
        }       
        
        dev_proc.send_len += dev_proc.file_size - dev_proc.send_len;
        send_len -= dev_proc.file_size - dev_proc.send_len;
    }

    if(dev_proc.send_len >= dev_proc.file_size) {
        PR_NOTICE("recv all data of user file");   
    }
    *remain_len = send_len;

    return OPRT_OK;
}

STATIC VOID dev_ug_notify_cb(IN CONST FW_UG_S *fw, IN CONST INT_T download_result, IN PVOID_T pri_data)
{
    if(OPRT_OK == download_result) { // update success
        return;
    }else {
        PR_ERR("the gateway upgrade failed");
		ty_msg.IsInUpgrade = FALSE;
    }
}


STATIC OPERATE_RET device_differ_init(VOID)
{
	OPERATE_RET op_ret = OPRT_OK;
	#if 1
    TY_IOT_CBS_S wf_cbs = {
        status_changed_cb,\
        gw_ug_inform_cb,\
		NULL,         \
        dev_obj_dp_cb,\
        dev_raw_dp_cb,\
        dev_dp_query_cb,\
        dev_ug_inform_cb,
    };
    PR_NOTICE("cfg_mode:%d\r\n",cfg_mode);
    op_ret = tuya_iot_wf_mcu_dev_init(cfg_mode,&wf_cbs,product_key,WF_SW_VERSION,dev_sw_ver);
    if(OPRT_OK != op_ret) {
        PR_ERR("tuya_iot_wf_soc_dev_init err:%d",op_ret);
        return -1;
    }

    op_ret = tuya_iot_reg_get_wf_nw_stat_cb(__get_wf_status);
    if(OPRT_OK != op_ret) {
        PR_ERR("tuya_iot_reg_get_wf_nw_stat_cb err:%d",op_ret);
        return op_ret;
    }
#endif
	/*模块IO处理*/
	if( ty_msg.wf_work.mode == 1 )
	{
        op_ret = tuya_create_led_handle(ty_msg.wf_work.state_pin,TRUE,&wifi_light);
		if( op_ret != OPRT_OK){
			PR_ERR("tuya_create_led_handle err:%d",op_ret);
			return op_ret;
		}
        tuya_set_led_light_type(wifi_light, OL_HIGH, 0, 0);

        op_ret = key_init(NULL,0,KEY_TIMER_MS);
        if(op_ret != OPRT_OK){
            PR_ERR("key_init err:%d",op_ret);
            return op_ret;
        }
    
        KEY_USER_DEF_S rst_key = {ty_msg.wf_work.reset_pin,TRUE,LP_ONCE_TRIG,KEY_RST_TIME,400,key_process};
        op_ret = reg_proc_key(&rst_key);
    	if( op_ret != OPRT_OK){
    		PR_ERR("reg_proc_key err:%d",op_ret);
    		return;
    	}
	}
	
	#if 1
	op_ret = sys_add_timer(get_memory_cb, NULL, &get_memory_timer);
    if(OPRT_OK != op_ret) {
        return op_ret;
    }
    else{
        PR_NOTICE("cd_timer ID:%d",get_memory_timer);
        sys_start_timer(get_memory_timer, 30*1000, TIMER_CYCLE);//1 minute
    }
    #endif
    return OPRT_OK;
}

STATIC VOID get_memory_cb(UINT_T timerID,PVOID pTimerArg)
{
    INT_T size = SysGetHeapSize();//lql
	PR_NOTICE("free_mem_size:%d",size);
}





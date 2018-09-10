/***********************************************************
*  File: tuya_device.c
*  Author: lql
*  Date: 20180104
***********************************************************/
#if 0
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
#include "uni_queue.h"
#include "uni_time.h"



/***********************************************************
*************************micro define***********************
***********************************************************/
#define WF_SW_VERSION USER_SW_VER
#define KEY_TIMER_MS      	 20    //key timer inteval
#define KEY_RST_TIME       5000            //按键重置时间 ms




#define HN_RECV_BUF_MAX 512 /*接收缓冲区*/
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
}WF_CMD_E;


/***********************************************************
*************************micro define***********************
***********************************************************/
//STATIC VOID key_process(INT gpio_no,PUSH_KEY_TYPE_E type,INT cnt);
STATIC OPERATE_RET device_differ_init(VOID);
STATIC VOID ty_datapoint_proc(cJSON *root);
STATIC INT upload_wifi_state(VOID);
STATIC VOID start_guard_timer(UCHAR sec);
STATIC VOID del_queue_buf_by_cmd(UCHAR cmd);


#pragma pack(1)
typedef struct
{
	WORD len;		//数据长度
	UCHAR data[0];  //数据内容
}TY_SEND_DATA_S;

typedef struct
{
    WORD  head;    //固定为0x55aa(大端格式)
    UCHAR version; //版本，升级扩展用
    UCHAR fr_type; //具体帧类型(命令字)
    WORD  len;     //数据长度(大端格式)
    UCHAR data[0]; //数据内容
}TY_FRAME_S;

typedef struct
{
	UCHAR mode;      //工作模式,0 串口处理 1 模块IO
	UCHAR state_pin; //wifi状态指示
	UCHAR reset_pin; //wifi重置
}TY_WIFI_WORK_S;

typedef struct
{
    UCHAR dpid;
    UCHAR type;
    WORD len;
    UCHAR data[0];
}TY_DATAPOINT_DATA_S;

typedef struct
{
	UINT  offset;
    UINT  fw_size;
}TY_UP_FW_S;

typedef struct 
{
    UCHAR proc_buf[HN_RECV_BUF_MAX];
	P_QUEUE_CLASS pQueue;
    THRD_HANDLE thread;//lql
    MUTEX_HANDLE mutex; 
	TIMER_ID jump_send_timer;
	TIMER_ID queue_send_timer;
	TIMER_ID com_send_timer;
	TIMER_ID up_fw_timer;
	TIMER_ID guard_timer;
	UCHAR try_cnt;
	TY_WIFI_WORK_S wf_work; /*MCU工作方式*/
	BOOL init_flag;		/*设备初始化标志*/
	BOOL online_flag;	/*设备在线标志*/
	BOOL resend_flag;   /*同步指令未返回需重新发送*/
	UCHAR ack_stat;     /*固件升级包返回响应标志*/
	UCHAR recv_state;
    UCHAR prt_ver;
	INT rcount;
	TY_UP_FW_S up_st;
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



/***********************************************************
*************************variable define********************
***********************************************************/
STATIC TY_MSG_S ty_msg;
LED_HANDLE wifi_light = NULL;
CHAR dev_sw_ver[SW_VER_LEN+1];
CHAR product_key[PRODUCT_KEY_LEN+1];
GW_WF_CFG_MTHD_SEL cfg_mode;


/***********************************************************
*  Function: pre_device_init
*  Input: none
*  Output: none
*  Return: none
*  Note: to initialize device before device_init
***********************************************************/
VOID pre_device_init(VOID)
{
    PR_DEBUG("%s:%s",APP_BIN_NAME,WF_SW_VERSION);
    SetLogManageAttr(LOG_LEVEL_INFO);
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



/*checksum校验*/
UCHAR getCheckSum(CONST UCHAR *pack, INT pack_len)
{
    UINT check_sum = 0;
    while(--pack_len >= 0)
    {
        check_sum += *pack++;
    }
    return check_sum&0xff;
}


VOID start_jump_timer(UCHAR sec)
{
	sys_start_timer(ty_msg.jump_send_timer,sec*1000,TIMER_CYCLE);
}


VOID stop_jump_timer(VOID)
{
	sys_stop_timer(ty_msg.jump_send_timer);
}


VOID com_stop_timer(VOID)
{
    if(0 == GetCurQueNum(ty_msg.pQueue)) {
        return;
    }
    
	ty_msg.try_cnt = 0;
	
	TY_SEND_DATA_S *pSend = NULL;
	GetQueueMember(ty_msg.pQueue, 1, (UCHAR *)&pSend, 1);
	if( pSend != NULL )
	{
		PR_DEBUG("free buffer... ...");
		Free(pSend);
	}		
	DelQueueMember(ty_msg.pQueue, 1); //del by cmd ?
	sys_stop_timer(ty_msg.com_send_timer);
}

VOID com_stop_timer_cmd(UCHAR cmd)
{
    del_queue_buf_by_cmd(cmd);
    sys_stop_timer(ty_msg.com_send_timer);
}

/*串口数据发送*/
STATIC INT ty_uart_send(UCHAR fr_type, UINT len, UCHAR *data)
{
    UINT i = 0;
    WORD head = 0x55aa;
    UINT send_len = SIZEOF(TY_FRAME_S) + len + 1;
    
    UCHAR* SendBuf = (UCHAR*)Malloc(send_len);
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
    UCHAR num = getCheckSum(SendBuf, send_len - 1);
    SendBuf[send_len - 1] = num;

 	MutexLock(ty_msg.mutex);
    ty_uart_send_data(TY_UART0,SendBuf,send_len);
#ifdef UART_TX_DEBUG
    INT cnt = 0;
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
STATIC INT ty_msg_send(UCHAR fr_type, WORD len, UCHAR *data)
{
    UINT i = 0;
    WORD head = 0x55aa;
    UINT hn_send_len = SIZEOF(TY_SEND_DATA_S) + SIZEOF(TY_FRAME_S) + len + 1;
    
    UCHAR* SendBuf = (UCHAR*)Malloc(hn_send_len);
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
    UCHAR num = getCheckSum(SendBuf + SIZEOF(TY_SEND_DATA_S), hn_send_len - SIZEOF(TY_SEND_DATA_S) - 1);
    SendBuf[hn_send_len - 1] = num;

    TY_SEND_DATA_S *pSend = (TY_SEND_DATA_S *)SendBuf;   
    pSend->len = hn_send_len - SIZEOF(TY_SEND_DATA_S);

    if(InQueue(ty_msg.pQueue, (UCHAR *)&pSend, 1) == 0) {        
        PR_ERR("InQueue err");
        Free(SendBuf);
        return WM_FAIL;
    }

    return WM_SUCCESS;
}


STATIC VOID del_queue_buf_by_cmd(UCHAR cmd)
{
    TY_SEND_DATA_S *pSend = NULL;
    GetQueueMember(ty_msg.pQueue,1,(unsigned char *)(&pSend),1);
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

STATIC INT recv_jump_pack(TY_FRAME_S *p_frame)
{
	UCHAR start = 0x00;

	com_stop_timer_cmd(HEART_BEAT_CMD);
	start_guard_timer(WF_GUARD_TIME);//lql
	
	PR_DEBUG("recv jump ack");
	
	start = p_frame->data[0];
	if( !ty_msg.init_flag ) //设备刚上电
	{
		ty_msg.prt_ver = p_frame->version;
		PR_DEBUG("Device power on ... ... version:%d", p_frame->version);
		query_prod_info();
		start_jump_timer(JUMP_SEND_TIME);
	}
	else    //设备已初始化
	{
		if( start == 0x00 )
		{
			PR_DEBUG("First jump back ... ...");
            if( ty_msg.wf_work.mode == 0 ) {
                upload_wifi_state();//lql
            }
			query_dp_state();
		}
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
	}

	ty_msg.online_flag = TRUE;
	return 0;
}


/*同步指令,MCU必须返回*/
STATIC OPERATE_RET get_dev_info(IN TY_FRAME_S *p_frame)
{
	if(NULL == p_frame) {
	   PR_ERR("invalid param");
	   return OPRT_INVALID_PARM;
	}

	WORD len = WORD_SWAP(p_frame->len);
	
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
		UCHAR* p_data = (UCHAR *)Malloc(len+1);
		if(NULL == p_data) {
			return OPRT_MALLOC_FAILED;
		}
		memset(p_data, 0, len+1);
		memcpy(p_data, p_frame->data, len);

		PR_DEBUG("jstr:%s",p_data);
		cJSON *root = cJSON_Parse(p_data);
		if(NULL == root) {
			PR_ERR("cjson parse err");
			return OPRT_CJSON_PARSE_ERR;
		}

		if( ( NULL == cJSON_GetObjectItem(root,"p") ) || \
        	( NULL == cJSON_GetObjectItem(root,"v") ) || \
        	( NULL == cJSON_GetObjectItem(root,"m") ) ){
        	PR_ERR("param is no correct");
			cJSON_Delete(root);
        	return OPRT_INVALID_PARM;
    	}
		
		cJSON *json;
		json = cJSON_GetObjectItem(root, "p");	
		strcpy(product_key,json->valuestring);

		json = cJSON_GetObjectItem(root, "v");
		strcpy(dev_sw_ver,json->valuestring);

		json = cJSON_GetObjectItem(root, "m");
		INT cfg_mode = json->valueint;
		#if 0
		//set mode
		set_wf_cfg_method(mode);//lql  
        #endif
		cJSON_Delete(json);
	#endif
	}

#if 0
		INT i = 0;	
		for( i = 0; i < 16; i++ )
		{
			PR_DEBUG_RAW("%02X ", p_dev_if->product_key[i]);
		}
#endif	
	return OPRT_OK;
}
/*获取产品信息*/
STATIC INT recv_prod_info(TY_FRAME_S *p_frame)
{
	PR_DEBUG("recv prod info");
	
	//com_stop_timer();
	com_stop_timer_cmd(PRODUCT_INFO_CMD);	
    OPERATE_RET op_ret = OPRT_OK;
	op_ret = get_dev_info(p_frame);//get procuct_key  sw_ver  cfg_mode
	if(OPRT_OK != op_ret) {
		return WM_FAIL;
	}

	return WM_SUCCESS;
}

STATIC INT recv_work_mode(TY_FRAME_S *p_frame)
{
	//com_stop_timer();
	com_stop_timer_cmd(WORK_MODE_CMD);

	WORD len = WORD_SWAP(p_frame->len);
	
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
	device_differ_init();//lql
	ty_msg.init_flag = TRUE;
	return 0;
}

/*WIFI工作状态*/
STATIC INT recv_wifi_ack(VOID)
{
	//com_stop_timer();
	com_stop_timer_cmd(WIFI_STATE_CMD);
    return 0;
}


STATIC OPERATE_RET upgrade_pre_cb(IN CONST UINT file_size)
{
    PR_DEBUG("file_size = %d", file_size);
    
    UCHAR start_upgrade = UPDATE_START_CMD;
	UCHAR len_buf[4];
	UINT  icnt = 0,len;

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
				start_jump_timer(JUMP_SEND_TIME);
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

STATIC OPERATE_RET upgrade_file_cb(IN CONST BYTE *data, IN CONST UINT len)
{	
	UCHAR head_buf[10];
	UCHAR sum_check = 0;
	UCHAR icnt = 0, i = 0;
	WORD  data_len = 0;
    WORD  head_len = 0;
	
	PR_DEBUG("upgrade len = %d", len);
	PR_DEBUG("up_cb remain size:%d",SysGetHeapSize());//lql
	
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
    ty_uart_send_data(TY_UART0,(UCHAR *)data, len);
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
				start_jump_timer(JUMP_SEND_TIME);
				return OPRT_COM_ERROR;
			}
			PR_DEBUG("try again");
			ty_msg.ack_stat = FW_IDE;
			sys_start_timer(ty_msg.up_fw_timer,MCU_RECV_TIME*1000, TIMER_ONCE);
			
			MutexLock(ty_msg.mutex);
            ty_uart_send_data(TY_UART0,head_buf, head_len);
		    ty_uart_send_data(TY_UART0,(UCHAR *)data, len);
		    ty_uart_send_data(TY_UART0,&sum_check, 1);
		    MutexUnLock(ty_msg.mutex);
		}
		else if( ty_msg.ack_stat == FW_RETURN )
		{
			break;
		}
		else
		{
			start_jump_timer(JUMP_SEND_TIME);
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
		//恢复心跳
		start_jump_timer(JUMP_FIRST_TIME);
//        SystemSleep(2*1000);
//        SystemReset();
        ty_msg.init_flag = FALSE;
	}

	return OPRT_OK;
}

STATIC VOID long_to_byte(INT val, UCHAR *out)
{
	//PR_DEBUG("val:%d",val);
	out[0] = (UCHAR)((val&0xFF000000)>>24);
	out[1] = (UCHAR)((val&0x00FF0000)>>16);
	out[2] = (UCHAR)((val&0x0000FF00)>>8);
	out[3] = (UCHAR)((val&0x000000FF));
}

STATIC INT get_lklv_data(IN CHAR *jstr,OUT CHAR *data, OUT INT *dlen)
{
	UCHAR len = 0;
	UCHAR type = 0x00;
	INT i = 0;

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

STATIC VOID syn_weather_data(OPERATE_RET result,CHAR *param)
{
	UCHAR fr_type = WEATH_DATA_CMD;
	UCHAR data[2];
	UINT len = 0;

	if((OPRT_OK != result) || (NULL == param)) {
		goto FAIL_EXIT;
	}

	//PR_DEBUG("param:%s",param);
	UCHAR *buf = (UCHAR *)Malloc(strlen(param)+128);
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


STATIC INT recv_weather_ack(VOID)
{
    com_stop_timer_cmd(WEATH_DATA_CMD);
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
STATIC OPERATE_RET get_pub_param(IN TY_FRAME_S *p_fr, OUT CHAR **ppout)
{
	WORD data_len = WORD_SWAP(p_fr->len);
	
	if(0 == data_len) {
		PR_ERR("data_len:%d",data_len);
		return OPRT_INVALID_PARM;
	}

	CHAR *buf = (CHAR *)Malloc(256);
	if(NULL == buf) {		
		return OPRT_MALLOC_FAILED;
	}
	
	cJSON *root = cJSON_CreateArray();
	if(NULL == root) {
		return OPRT_CR_CJSON_ERR;
	}
	
	//PR_DEBUG("data_len:%d",data_len);
	UCHAR len = 0;
	INT i = 0;
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
    CHAR *out = cJSON_PrintUnformatted(root);	
    cJSON_Delete(root),root = NULL;
	if(NULL == out) {
		return OPRT_MALLOC_FAILED;
	}
	//PR_DEBUG("out:%s",out);
	*ppout = out;	
	return OPRT_OK;
}

STATIC VOID recv_weather_open(TY_FRAME_S *p_frame)
{
#if 0
	UCHAR fr_type = WEATH_SERVER_CMD;
	CHAR *pub_para = NULL;
	CHAR data[2];

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


STATIC INT recv_upgrade_start(TY_FRAME_S *p_frame)
{
    //停止心跳
    stop_jump_timer();// is need stop jump
    //is need stop com timer ?	
    sys_stop_timer(ty_msg.up_fw_timer);
    PR_DEBUG("up grade start");

    ty_msg.prt_ver = p_frame->version;
    ty_msg.ack_stat = FW_RETURN;
    return 0;
}

STATIC INT recv_upgrade_transfer(VOID)
{
	PR_DEBUG("transfer ack\r\n");
	sys_stop_timer(ty_msg.up_fw_timer);
	ty_msg.ack_stat = FW_RETURN;
	return 0;
}


STATIC INT recv_sys_datetime(VOID)
{
	//判断时间是否正确
	INT ret;
	UCHAR *pTime = NULL;
	UCHAR is_succ = 0;
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
	
	pTime = (UCHAR *)Malloc(7+1);
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


STATIC INT recv_local_datetime(VOID)
{
    UCHAR fr_type = LOCAL_TIME_CMD;

    UCHAR *pTime = (UCHAR *)Malloc(8+1);
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
    UCHAR fr_type = WIFI_TEST_CMD;
    UCHAR buf[2];
	#if 0
    BOOL flag = ws_exist_auzkey();//lql
    if( FALSE == flag ) {
        buf[0] = 0x00;
        buf[1] = 0x01;
    	ty_uart_send(fr_type, 2, buf);
        return;
    }
    #endif
    OPERATE_RET op_ret;
    AP_IF_S *ap = NULL;
    op_ret = wf_assign_ap_scan("tuya_mdev_test",&ap);//lql
	if(OPRT_OK != op_ret) {
		buf[0] = 0x00;
        buf[1] = 0x00;
	}else {
	    signed char result = ap->rssi;
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
	ty_uart_send(fr_type, 2, buf);
	return;
}


STATIC VOID recv_mem_check(VOID)
{
    UCHAR fr_type = MEMORY_QUERY_CMD;
    CHAR buf[5];

    memset(buf,0,sizeof(buf));
    INT size = SysGetHeapSize();//lql
    
    memset(buf,0,sizeof(buf));
    buf[0] = ((size>>24)&0xFF);
    buf[1] = ((size>>16)&0xFF);
    buf[2] = ((size>>8)&0xFF);
    buf[3] = (size&0xFF);

	ty_uart_send(fr_type, 4, buf);
}

STATIC INT recv_prod_test(VOID)
{
	PR_DEBUG("MCU have enter prod test");
}

/*重置wifi*/
STATIC INT reset_wifi_pro(VOID)
{
	ty_uart_send(WIFI_RESET_CMD, 0, NULL);
	tuya_iot_wf_gw_unactive(WRT_AUTO);//AP  smart cfg   切换
	return 0;
}


/*重置wifi选择模式*/
STATIC INT reset_wifi_select(TY_FRAME_S *p_frame)
{
	WORD len = WORD_SWAP(p_frame->len);
	
	ty_msg_send(WIFI_RESET_SEL_CMD, 0, NULL);
	if(ty_msg.wf_work.mode == 0)
	{
        if( len > 0 ) {
            if( p_frame->data[0] == 0 ) {
                tuya_iot_wf_gw_unactive(WRT_SMT_CFG);
            }else {
                tuya_iot_wf_gw_unactive(WRT_AP);
            }
        }else {
            PR_ERR("cmd para err");//lql
        }
	}
	return 0;
}

/*报告WIFI工作状态*/
STATIC INT upload_wifi_state(VOID)
{
	UCHAR fr_type = WIFI_STATE_CMD;
	UCHAR data;
	static int num=0;
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


STATIC WORD ty_bitmap_len_proc(UCHAR bit_len) 
{
    WORD len;

	len = bit_len/8 + ((bit_len%8 == 0) ? 0:1);
	if( len > 4 )
		len = 4;
    return len;
}

STATIC UCHAR ty_dp_change_proc(UCHAR dp)
{
	dp++;
	return dp;
}

STATIC INT ty_get_obj_type_len(TY_OBJ_DP_S *obj_dp, UCHAR *type, WORD *len)
{    
	//*dpid = obj_dp->dpid;
	DEV_CNTL_N_S *dev_cntl = get_gw_dev_cntl();//lql
	INT i;
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


STATIC INT ty_get_enum_id(UCHAR dpid, UCHAR *enum_str)
{
	UCHAR i = 0;
	UCHAR enum_id = 0;
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


STATIC INT ty_data_proc(TY_OBJ_DP_S *obj_dp, UCHAR type, WORD len, UCHAR *buf)
{    
	INT base64_len = len;
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
				WORD bit = WORD_SWAP(obj_dp->value.dp_bitmap);
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
	WORD len, base64_len;
	INT ret;
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

	base64_len = base64_decode(raw_dp->data, dp_data->data);//need to add '\0' at the tail of raw_dp->data??
    dp_data->len = WORD_SWAP(base64_len);
    
    ty_uart_send(DATA_CTRL_CMD, 4 + base64_len, SendBuf);
    
    Free(SendBuf);	
}


STATIC VOID ty_obj_datapoint_proc(TY_OBJ_DP_S *obj_dp)
{
	UCHAR type;
	WORD len, rawlen;
	INT ret;
	
	ret = ty_get_obj_type_len(obj_dp, &type, &len);//lql
	if( ret != WM_SUCCESS)
	{
		PR_ERR("ty_type_len_proc fail");
		return;
	}
	
	UCHAR* SendBuf = (UCHAR*)Malloc(4 + len + 1);
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


STATIC VOID ty_datapoint_upload_proc(WORD len, UCHAR *data)
{  
	if( data == NULL ) {
		return;
	}
	if( !ty_msg.init_flag ) {
		return;
	}
	if( get_gw_nw_status() != GNS_WAN_VALID ) {//lql
		return;
	}

	#if 0
    if( UPGRADING == get_fw_ug_stat()) {//lql
		PR_DEBUG("device is updating");
		return;
	}
    #endif
    
	WORD offset = 0;
	INT i;
	DEV_CNTL_N_S *dev_cntl = get_gw_dev_cntl();//lql
	if( dev_cntl == NULL )
		return;

	DP_CNTL_S *dp_cntl =  NULL;
	CHAR dpid[10], dpdata[10],*out = NULL;
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
				dp_cntl = &dev_cntl->dp[i];
				break;
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
	        char time[32];
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

			op_ret = sf_raw_dp_report_sync(get_gw_cntl()->gw_if.id,out,time,2000);
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
        }
		break;		
		case DP_VALUE: {		
		    INT val_data = 0;
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
			UCHAR *pData = NULL;
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
			INT bitdata = 0;
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
	op_ret = sf_obj_dp_report_async(get_gw_cntl()->gw_if.id,out,false);
	if(OPRT_OK != op_ret) {
		PR_ERR("sf_obj_dp_report op_ret:%d,out:%s",op_ret,out);
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
    send_jump_pack();
}


STATIC VOID clear_all_queue(VOID)
{
	INT i = 0, ret = 0;
	INT n = GetCurQueNum(ty_msg.pQueue);
	for(i = 0; i < n; i ++) {
		TY_SEND_DATA_S *pSend = NULL;
		ret = OutQueue(ty_msg.pQueue, (UCHAR *)&pSend, 1);
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
	INT ret = 0;
	
	ty_msg.try_cnt ++;
	if( ty_msg.try_cnt < 2 ) {
		return;
	}
	
	if(0 == GetCurQueNum(ty_msg.pQueue)) {
		return;
	}

	TY_SEND_DATA_S *pSend = NULL;
	ret = GetQueueMember(ty_msg.pQueue, 1, (UCHAR *)&pSend, 1);
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

/*心跳监控定时器*/
STATIC VOID guard_timer_cb(UINT timerID,PVOID pTimerArg)
{
	if( !ty_msg.init_flag ) {
		PR_ERR("guard init_flag is FALSE");
		return;
	}

	#if 0
    if( UPGRADING == get_fw_ug_stat()) {//lql
		PR_ERR("guard mcu updating");
		return;
	}
    #endif
    
	PR_DEBUG("guard time out ");
	SystemReset();
}


STATIC VOID start_guard_timer(UCHAR sec)
{
    sys_start_timer(ty_msg.guard_timer,sec*1000,TIMER_CYCLE);
}

/*数据发送定时器*/
STATIC VOID queue_send_timer_cb(UINT timerID,PVOID pTimerArg)
{
	BOOL is_run = FALSE;
	TY_SEND_DATA_S *pSend = NULL;
	INT qNum = 0;

	//上次发送数据还未返回
	is_run = IsThisSysTimerRun(ty_msg.com_send_timer);
	if( is_run ) {
        return;
	}

	qNum = GetCurQueNum(ty_msg.pQueue);	
	if( qNum > 0 )
	{
		GetQueueMember(ty_msg.pQueue, 1, (UCHAR *)&pSend, 1);
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
		INT i = 0;
		for(i = 0; i < pSend->len; i++) {
            PR_DEBUG_RAW("%02X ",pSend->data[i]);
		}
#endif

	}
}

/*数据处理*/
STATIC INT ty_msg_upload_proc(UCHAR* data)
{
	INT ret = 0;
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
			if( ret == 0 ) {
				query_work_mode();
			}
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
            ty_datapoint_upload_proc(ty_frame->len, ty_frame->data);
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
                        
        default: break;
    }

	return WM_SUCCESS;
}

/*串口接收*/
STATIC VOID ty_uart_recv_proc(VOID)
{
    UCHAR count = ty_uart_read_data(TY_UART0,ty_msg.proc_buf + ty_msg.rcount, HN_RECV_BUF_MAX - ty_msg.rcount);
	ty_msg.rcount += count;
	if( ty_msg.rcount > 0 ) {
    	ty_msg.recv_state = UART_PROC;
	}
}


/*消息处理*/
STATIC VOID ty_dp_proc(VOID)
{
	INT fr_len = 0, offset = 0;
	UCHAR check_num = 0;

    while((ty_msg.rcount - offset) >= HN_RECV_MIN_NUM) {
    	if(ty_msg.proc_buf[offset] != 0x55) {
		//	PR_DEBUG("ty_msg.proc_buf[0][0x%x]", ty_msg.proc_buf[offset]);
            offset += 1;
            continue;  
        }
        if(ty_msg.proc_buf[offset + 1] != 0xaa) {
		//	PR_DEBUG("ty_msg.proc_buf[1][0x%x]", ty_msg.proc_buf[offset + 1]);
            offset += 1;
            continue;  
        }
        fr_len = ty_msg.proc_buf[offset + 4]*256 + ty_msg.proc_buf[offset + 5] + 6 + 1;
		//PR_DEBUG("proc fr_len=%d, (ty_msg.rcount - offset)[%d]", fr_len, (ty_msg.rcount - offset));
        if(fr_len > HN_RECV_BUF_MAX) {
            offset += 1;//2 ? lql
            continue;
        }
        if(ty_msg.rcount - offset < fr_len) {
            break;
        }
        check_num = getCheckSum(ty_msg.proc_buf + offset, fr_len - 1);
        if(check_num != ty_msg.proc_buf[offset + fr_len - 1]) {
			PR_DEBUG("check_num[%d], ty_msg.proc_buf[offset + fr_len - 1][%d]", check_num, ty_msg.proc_buf[offset + fr_len - 1]);
            offset += fr_len;//2 ? lql
            continue;
        }
		//PR_DEBUG("ty_msg_upload_proc");
        ty_msg_upload_proc(ty_msg.proc_buf + offset); 
       	offset += fr_len;
    }

	//PR_DEBUG("ty_msg.rcount=%d,offset=%d", ty_msg.rcount, offset);
	if( offset > 0 ) {
    	ty_msg.rcount -= offset;
    	memmove(ty_msg.proc_buf, ty_msg.proc_buf + offset, ty_msg.rcount);
	}
    ty_msg.recv_state = UART_RECV;
}

STATIC VOID ty_task_cb(PVOID pArg)
{
	ty_msg.recv_state = UART_RECV;
	ty_msg.rcount = 0;

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
				break;
		}
	}
}

VOID set_wifi_light(UCHAR stat)
{
	UCHAR fr_type = WIFI_STATE_CMD;
	UCHAR data = stat;	

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
    if( !ty_msg.init_flag ) {
        PR_ERR("init_flag is FALSE");
        return;
    }
    #if 1
    if( !get_gw_dev_cntl()->dev_if.bind ) {//gw_if.bind
        PR_ERR("dev is not bind");
        return;//未绑定
    }
    #endif

    #if 0
    if( UPGRADING == get_fw_ug_stat()) {
        PR_ERR("mcu updating");
        return;
    }
    #endif
    INT i = 0;
    for(i = 0;i < dp->dps_cnt;i++) {
        ty_obj_datapoint_proc(&dp->dps[i]);
        SystemSleep(40);//为什么dp点要一个一个的发给mcu
    }
}


VOID dev_raw_dp_cb(IN CONST TY_RECV_RAW_DP_S *dp)
{
    #if 0
    PR_DEBUG("dpid:%d",dp->dpid);

    PR_DEBUG("recv len:%d",dp->len);
    INT i = 0;
    for(i = 0;i < dp->len;i++) {
        PR_DEBUG_RAW("%02X ",dp->data[i]);
    }
    PR_DEBUG_RAW("\n");
    PR_DEBUG("end");
    #endif
    ty_raw_datapoint_proc(dp);//lql
}

STATIC VOID __get_wf_status(IN CONST GW_WIFI_NW_STAT_E stat)
{
    PR_NOTICE("wifi status is :%d",stat);
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
            tuya_set_led_light_type(wifi_light,OL_HIGH,0,0);//led up
            break;
        }
        default:{
            tuya_set_led_light_type(wifi_light,OL_LOW,0,0);//default:led up
            break;
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

//怎么确保在配网之前从mcu获取到工作模式（模块自处理 or 联合处理）？？
OPERATE_RET device_init(VOID)
{
    OPERATE_RET op_ret = OPRT_OK;
    
    PR_DEBUG("device_init start");

	memset(&ty_msg, 0, sizeof(TY_MSG_S));
	ty_msg.online_flag = FALSE;
	ty_msg.init_flag = FALSE;
    ty_msg.prt_ver = 0x00;

    ty_msg.pQueue = CreateQueueObj(QUEUE_MAX_EVENTS, sizeof(UCHAR *));
    if(ty_msg.pQueue == NULL) {
        PR_ERR("CreateQueueObj failed");
        return -1;
    }
    
    op_ret = sys_add_timer(jump_send_timer_cb,NULL,&ty_msg.jump_send_timer);
    if(OPRT_OK != op_ret) {
        PR_ERR("add jump_send_timer err");
        return op_ret;
    }else {
        start_jump_timer(JUMP_FIRST_TIME);
    }

    op_ret = sys_add_timer(queue_send_timer_cb,NULL,&ty_msg.queue_send_timer);
    if(OPRT_OK != op_ret) {
        PR_ERR("add jump_send_timer err");
        return op_ret;
    }else {
        sys_start_timer(ty_msg.queue_send_timer,50,TIMER_CYCLE);
    }
    
    op_ret = sys_add_timer(guard_timer_cb,NULL,&ty_msg.guard_timer);
    if(OPRT_OK != op_ret) {
        PR_ERR("add guard_timer_cb err");
        return op_ret;
    }else {
        start_guard_timer(WF_GUARD_TIME);
    }

    op_ret = sys_add_timer(com_send_timer_cb,NULL,&ty_msg.com_send_timer);
    if(OPRT_OK != op_ret) {
        PR_ERR("add jump_send_timer err");
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
    
    op_ret = CreateAndStart(&ty_msg.thread,ty_task_cb,NULL,1024+1024,TRD_PRIO_2,"ty_task");
    if(op_ret != OPRT_OK) {
        PR_ERR("start thread ty_task_cb err");
        return op_ret;
    }

#if 0
    TY_IOT_CBS_S wf_cbs = {
        status_changed_cb,\
        gw_ug_inform_cb,\
        dev_obj_dp_cb,\
        dev_raw_dp_cb,\
        dev_dp_query_cb,\
        NULL,
    }; 
    op_ret = tuya_iot_wf_mcu_dev_init(TY_APP_CFG_WF,cfg_mode,&wf_cbs,product_key,WF_SW_VERSION,dev_sw_ver);
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
	PR_NOTICE("device_init ok");
    return OPRT_OK;
}

STATIC VOID key_process(TY_GPIO_PORT_E port,PUSH_KEY_TYPE_E type,INT cnt){
    PR_DEBUG("port: %d",port);
    PR_DEBUG("type: %d",type);
    PR_DEBUG("cnt: %d",cnt);

    if(ty_msg.wf_work.reset_pin == port){
        if(LONG_KEY == type) {
             tuya_iot_wf_gw_unactive(WRT_AUTO);//AP  smart cfg   切换
        }    
    }
}


STATIC OPERATE_RET device_differ_init(VOID)
{
	OPERATE_RET op_ret = OPRT_OK;
	#if 1
    TY_IOT_CBS_S wf_cbs = {
        status_changed_cb,\
        gw_ug_inform_cb,\
        dev_obj_dp_cb,\
        dev_raw_dp_cb,\
        dev_dp_query_cb,\
        NULL,
    }; 
    op_ret = tuya_iot_wf_mcu_dev_init(TY_APP_CFG_WF,cfg_mode,&wf_cbs,product_key,WF_SW_VERSION,dev_sw_ver);
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
    
        KEY_USER_DEF_S rst_key = {ty_msg.wf_work.reset_pin,FALSE,LP_ONCE_TRIG,KEY_RST_TIME,400,key_process};
        op_ret = reg_proc_key(&rst_key);
    	if( op_ret != OPRT_OK){
    		PR_ERR("reg_proc_key err:%d",op_ret);
    		return;
    	}
	}
	//ty_msg.init_flag = TRUE;
    return OPRT_OK;
}


#endif

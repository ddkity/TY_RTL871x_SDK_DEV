/*
** 
红外万能遥控器模块使用说明:
1:在device.c内添加#include    "ir_remote.h"
2:在device_differ_init()内调用ir_remote_init()初始化IR软硬件
3:在device_cb()函数内调用ir_dp_handle(root)完成红外发送
4:红外发射管脚固定为GPIO14不能更改.
5:红外发射指示灯可在device.h内修改.
6:红外发射已占用硬件定时器,请勿再其他地方再次使用.
**
*/
#include "tuya_device.h"
#include "adapter_platform.h"
#include "tuya_cloud_error_code.h"
#include "tuya_iot_wifi_api.h"
#include "tuya_cloud_types.h"
#include "uni_time_queue.h"
#include "gw_intf.h"
#include "uni_log.h"
#include "uni_thread.h"
#include "sys_api.h"
#include "timer_api.h"
#include "tuya_led.h"
#include "pwmout_api.h"

//红外相关头文件
#include "ir_remote.h"
#include "AcAlg.h"

extern LED_HANDLE wf_light;

IR_PARAM_S ir_param;
STATIC uint8_t tx_frame_count = 0;
STATIC uint16_t tx_retnum = 0;
STATIC volatile uint8_t tx_end;

#define MAX_MSG_QUEUE 5

STATIC VOID ir_remote_send(VOID);

#define PWM_3       PA_12 
#define PWM_PERIOD  1000/38   //38khz 27
int pwms =PWM_PERIOD/2;
pwmout_t pwm_led;
PinName  pwm_led_pin =  PWM_3;
gtimer_t sw_rtc;

STATIC uint32_t str2hex(uint8_t *str, uint32_t len)
{
    uint32_t i;

    for(i = 0; i < len; i ++) {
        if(str[i] >= 'A' && str[i] <= 'F')
            str[i] = str[i] - 'A' + 10;
        else if(str[i] >= 'a' && str[i] <= 'f')
            str[i] = str[i] - 'a' + 10;
        else if(str[i] >= '0' && str[i] <= '9')
            str[i] = str[i] - '0';
    }

    for(i = 0; i < len / 2; i ++) {
 		str[i] = (str[2 * i] << 4) + (str[2 * i + 1] & 0x0f);
    }

    return len / 2;
}

//uint8 irtx_flag = 0;
//uint32_t timer_cnt = 0;
/* 硬件定时器回调再次发送 */
STATIC VOID ir_tx_handler(uint32_t id)
{
    gtimer_stop(&sw_rtc);
    ir_remote_send();
}

STATIC VOID ir_tx_init(VOID)
{
    pwmout_init(&pwm_led, pwm_led_pin);
	pwmout_period_us(&pwm_led, PWM_PERIOD);
    pwmout_pulsewidth_us(&pwm_led, pwms);
    pwmout_stop(&pwm_led);
    
	gtimer_init(&sw_rtc, 4);//must be timer4 ,id = 4;
	TIMx[sw_rtc.timer_id]->PSC = 39; //fenpin
    gtimer_start_periodical(&sw_rtc, 350, (void*)ir_tx_handler, (uint32_t)&sw_rtc);
    gtimer_stop(&sw_rtc);
}

void set_ir_remote_high(uint32_t time, uint32_t feq)
{
    static uint32_t freq_old = 0xffffffff;

    if (freq_old != feq){
        pwmout_period_us(&pwm_led, 1000/(feq/1000));
    }
    pwmout_start(&pwm_led);
    gtimer_reload(&sw_rtc, time);
    gtimer_start(&sw_rtc);
}

STATIC VOID set_ir_remote_low(uint32_t time)
{
    pwmout_stop(&pwm_led);
    gtimer_reload(&sw_rtc, time);
    gtimer_start(&sw_rtc);
}

STATIC VOID ir_remote_send(VOID)
{
    uint32_t temp;
    uint16_t tx_retnum_bak;
    
    if(tx_frame_count >= ir_param.key.frameCount) {
        tx_retnum = 0;
        tx_frame_count = 0;
        
        ir_param.send_finish = IR_STATUS_SUCCESS;

        //PR_DEBUG("send ok post sem");
        PostSemaphore(ir_param.ir_send_finish_sem);
        return;
    }
    
    if(ir_param.send_mode == IR_SENDMODE_KUKONG)                    //酷控数据
        temp = (uint32_t)((ir_param.key.ir_out[tx_retnum] * 1000000) / ir_param.key.feq);
    else if(ir_param.send_mode == IR_SENDMODE_STUDY)                //自学习数据
        temp = (uint32_t)ir_param.key.ir_out[tx_retnum];

    tx_retnum_bak = tx_retnum;
    tx_retnum ++;
    if(tx_retnum >= ir_param.key.retnum) {
        tx_frame_count ++;
        tx_retnum = 0;
//        timer_cnt = 0;
    }

    if(tx_retnum_bak % 2 == 0) {
        set_ir_remote_high(temp, ir_param.key.feq);
    } else {
        set_ir_remote_low(temp);
    }
}

VOID ir_remote_data_start(VOID)
{
    ir_tx_init();
    
    ir_send_led_start();

//    irtx_flag = 1;
    tx_retnum = 0;
    tx_frame_count = 0;
    ir_param.send_finish = IR_STATUS_SENDING;
    PR_NOTICE("start send.......");
//    timer_cnt = 0;
    ir_remote_send();
}

VOID ir_remote_data_stop(VOID)
{
    pwmout_stop(&pwm_led);
    gtimer_stop(&sw_rtc);
    ir_send_led_stop();

//    irtx_flag = 0;
}


STATIC VOID ir_remote_decode(CHAR_T *ir_code, CHAR_T *key_code)
{
    double pulse;
    USHORT ir_code_len, key_code_len;
    
    ir_code_len = str2hex(ir_code, strlen(ir_code));
    key_code_len = str2hex(key_code, strlen(key_code));
    
    PR_DEBUG("ir_code_len:%d", ir_code_len);
    PR_DEBUG("key_code_len:%d", key_code_len);
    
    /* 酷控解码 */
    memset(ir_param.key.ir_out, 0, MAX_IR_OUT);
    
    //create(ir_code, ir_code_len);
    //enc(key_code, key_code_len,  ir_param.key.ir_out, MAX_IR_OUT / sizeof(uint16_t), &ir_param.key.retnum, &ir_param.key.frameCount);
       
    ir_param.key.feq = 38000;//getFrequency();
    
    PR_DEBUG("retNum=%d", ir_param.key.retnum);
    PR_NOTICE("getFrequency=%d", 38000);//getFrequency());//频率
    PR_DEBUG("frameCount=%d", ir_param.key.frameCount);
    
    //一帧的数据通过out数组返回，单位为(1000000 /频率)，重复的帧数通过frameCount返回，如果要得到完整的高低电平时间序列(单位us)使用下面代码。
    pulse = (double) 1000000 / 38000;//getFrequency();
    ir_param.key.pulse = pulse;   

#if 0    
    int t, cnt;
    for(t = 0; t < ir_param.key.frameCount; t ++) {
        for(cnt = 0; cnt < ir_param.key.retnum; cnt ++)
        PR_DEBUG_RAW("%d ", ir_param.key.ir_out[cnt]);
    }
    PR_DEBUG_RAW("\r\n");
    
    uint32_t temp;
    
    for(t = 0; t < ir_param.key.frameCount; t ++) {
        for(cnt = 0; cnt < ir_param.key.retnum; cnt ++) {
            temp = (uint32_t)(ir_param.key.ir_out[cnt] * pulse);
            PR_DEBUG_RAW("%04d ", temp);
        }
    }
    PR_DEBUG_RAW("\r\n");
#endif
}

STATIC OPERATE_RET study_remote_handler(cJSON *root)
{
    BYTE i;
    OPERATE_RET ret;
    
    const char *key_id[] = {"7", "8", "9"};
    cJSON *json_ir = NULL, *json_key = NULL;
    
    PR_NOTICE("remain size:%d",SysGetHeapSize());
    for(i = 0; i < MAX_KEY_COUNT; i ++) {
        json_key = cJSON_GetObjectItem(root, key_id[i]);
        if((json_key == NULL) || (json_key->type != cJSON_String)) {
            break;
        }

        cJSON_Minify(json_key->valuestring);
        
        PR_NOTICE("%s:%s", key_id[i], json_key->valuestring);

        ir_param.key.ir_out = (uint16_t *)Malloc(MAX_IR_OUT);
        if(NULL == ir_param.key.ir_out) {
            PR_ERR("malloc failed!");
            return OPRT_MALLOC_FAILED;
        }
        
        memset(ir_param.key.ir_out, 0, strlen(json_key->valuestring) + 1);
        ir_param.key.retnum = strlen(json_key->valuestring) + 1;
        INT_T status = base64_decode(ir_param.key.ir_out, &ir_param.key.retnum, json_key->valuestring, strlen(json_key->valuestring));
        PR_DEBUG("base64_decode:%d", status);

        ir_param.key.retnum /= 2;
        
        PR_DEBUG("retNum=%d", ir_param.key.retnum);
        ir_param.key.feq = IR_STUDY_FEQ;
        PR_NOTICE("getFrequency=%d", ir_param.key.feq);//频率
        ir_param.key.frameCount = 1;

        INT_T i;
        USHORT *pir = ir_param.key.ir_out;
        for(i = 0; i < ir_param.key.retnum; i ++) {
            PR_DEBUG_RAW("%04d ", *pir ++);
        }
        PR_DEBUG_RAW("\r\n");

        ir_param.send_mode = IR_SENDMODE_STUDY;
        ir_remote_data_start();
        
        //等待当前按键发送完成
        sys_start_timer(ir_param.ir_remote_send_timer, 500, TIMER_ONCE);

	    WaitSemaphore(ir_param.ir_send_finish_sem);

        sys_stop_timer(ir_param.ir_remote_send_timer);
        
//        PR_NOTICE("timer_cnt:%d", timer_cnt);

        ir_remote_data_stop();
        
        if(ir_param.send_finish == IR_STATUS_SUCCESS) {
            PR_NOTICE("send finish!");
        } else if(ir_param.send_finish == IR_STATUS_FAILED) {
            PR_NOTICE("send failed!");
        }

        //延时300ms
        SystemSleep(300);
    }
    
    if(NULL != ir_param.key.ir_out) {
        Free(ir_param.key.ir_out);
        ir_param.key.ir_out = NULL;
    }

    PR_NOTICE("remain size:%d",SysGetHeapSize());
    return OPRT_OK;
}

STATIC OPERATE_RET kukong_remote_handler(cJSON *root)
{
    BYTE i;
    OPERATE_RET ret;
    
    const char *key_id[] = {"4", "5", "6"};
    cJSON *json_ir = NULL, *json_key = NULL;
    
    PR_NOTICE("remain size:%d",SysGetHeapSize());
    json_ir = cJSON_GetObjectItem(root, "3");
    if((json_ir == NULL) || (json_ir->type != cJSON_String)) {
        PR_ERR("ir_code null");
         return OPRT_CJSON_GET_ERR;
    }
    
    cJSON_Minify(json_ir->valuestring);
        
    PR_NOTICE("3:%s", json_ir->valuestring);

    ir_param.key.ir_out = (uint16_t *)Malloc(MAX_IR_OUT);
    if(NULL == ir_param.key.ir_out) {
        PR_ERR("malloc failed!");
        return OPRT_MALLOC_FAILED;
    }
    
    for(i = 0; i < MAX_KEY_COUNT; i ++) {
        json_key = cJSON_GetObjectItem(root, key_id[i]);
        if((json_key == NULL) || (json_key->type != cJSON_String)) {
            break;
        }

        cJSON_Minify(json_key->valuestring);
        
        PR_NOTICE("%s:%s", key_id[i], json_key->valuestring);
        
        ir_remote_decode(json_ir->valuestring, json_key->valuestring);

        ir_param.send_mode = IR_SENDMODE_KUKONG;
        ir_remote_data_start();
        
        //等待当前按键发送完成
        sys_start_timer(ir_param.ir_remote_send_timer, 500, TIMER_ONCE);

	    WaitSemaphore(ir_param.ir_send_finish_sem);

        sys_stop_timer(ir_param.ir_remote_send_timer);
        
//        PR_NOTICE("timer_cnt:%d", timer_cnt);

        ir_remote_data_stop();
        
        if(ir_param.send_finish == IR_STATUS_SUCCESS) {
            PR_NOTICE("send finish!");
        } else if(ir_param.send_finish == IR_STATUS_FAILED) {
            PR_NOTICE("send failed!");
        }
        
        //延时300ms
        SystemSleep(300);
    }
        
    if(NULL != ir_param.key.ir_out) {
        Free(ir_param.key.ir_out);
        ir_param.key.ir_out = NULL;
    }

    PR_NOTICE("remain size:%d",SysGetHeapSize());
    return OPRT_OK;
}


void ir_remote_send_timer_cb(UINT timerID,PVOID pTimerArg)
{
    //send failed
    ir_param.send_finish = IR_STATUS_FAILED;
    PostSemaphore(ir_param.ir_send_finish_sem);
}

STATIC OPERATE_RET ir_mode_report(BYTE mode)
{
    cJSON *root = NULL;
    OPERATE_RET op_ret;
    
    root = cJSON_CreateObject();
    if(mode == MODE_IRSEND)
        cJSON_AddStringToObject(root, "1", "send_ir");
    else if(mode == MODE_IRSTUDY)
        cJSON_AddStringToObject(root, "1", "study");
    else if(mode == MODE_STUDYEXIT)
        cJSON_AddStringToObject(root, "1", "study_exit");
    else if(mode == MODE_STUDYKEY)
        cJSON_AddStringToObject(root, "1", "study_key");

    CHAR_T *buf = NULL;
    buf = cJSON_PrintUnformatted(root);
    cJSON_Delete(root), root = NULL;
    if(buf == NULL)
        return OPRT_MALLOC_FAILED;

    PR_DEBUG("buf:%s", buf);
    op_ret = sf_obj_dp_report_async(get_gw_cntl()->gw_if.id, buf, FALSE);
    if(OPRT_OK != op_ret) {
        PR_ERR("sf_obj_dp_report err:%d",op_ret);
        PR_DEBUG_RAW("%s\r\n",buf);
        Free(buf), buf = NULL;
        return op_ret;
    }
    Free(buf), buf = NULL;
    
    return OPRT_OK;
}


STATIC BYTE get_ir_remote_mode(cJSON *root)
{
    cJSON *json = NULL;
    BYTE mode;
    
    json = cJSON_GetObjectItem(root, "1");
    if(json == NULL) {
        PR_ERR("please select ir mode first!!!");
        return 0;
    }
    mode = json->valueint;
    PR_DEBUG("mode:%d", mode);
    
    if(mode == 0)
        return MODE_IRSEND;
    else if(mode == 1)
        return MODE_IRSTUDY;
    else if(mode == 2)
        return MODE_STUDYEXIT;
    else if(mode == 3)
        return MODE_STUDYKEY;
}

STATIC VOID ir_handle_thread(PVOID pArg)
{
    int i, j;
    cJSON *root = NULL;
    BYTE ir_mode;
    P_MSG_LIST msgListNode;
    OPERATE_RET op_ret;
    CHAR_T *pSend = NULL;
    
    while(1)
    {
        PR_NOTICE("remain size:%d",SysGetHeapSize());
        op_ret = WaitMessage(ir_param.msg_que,&msgListNode);
        if(op_ret != OPRT_OK) {
            if(op_ret != OPRT_MSG_LIST_EMPTY) {
                PR_ERR("WaitMessage op_ret:%d", op_ret);
            }
            continue;
        }

        switch(msgListNode->msg.msgID)
        {
            case IR_APP_CMD:
            {
                pSend = (CHAR_T *)msgListNode->msg.pMsgData;
                if(pSend == NULL)
                    continue;

//                PR_DEBUG("buf:%s", pSend);

                root = cJSON_Parse(pSend);
                if(root != NULL) {
                    ir_mode = get_ir_remote_mode(root);
                    
                    PR_NOTICE("mode:%d", ir_mode);
                    
                    ir_mode_report(ir_mode);
                    switch(ir_mode) {
                        case MODE_IRSEND:                           //酷控下发
                        {
                            ir_study_stop();
                            kukong_remote_handler(root);
                            PR_NOTICE("MODE_IRSEND END......");
                         }
                        break;

                        case MODE_IRSTUDY:                          //自学习
                        {
                            //学习模式
                            PR_NOTICE("study mode......");
                            ir_study_start();
                        } 
                        break;

                        case MODE_STUDYEXIT:                    //学习码下发
                        {
                            //学习模式退出
                            PR_NOTICE("study stop......");
                            ir_study_stop();
                        }
                        case MODE_STUDYKEY:                    //学习码下发
                        {
                            ir_study_stop();
                            study_remote_handler(root);
                            PR_NOTICE("study key send...");
                        }
                        break;
                        
                        default:
                            PR_ERR("mode error!");
                        break;
                    }
                    
                    cJSON_Delete(root);
                    root = NULL;
                }
            }
            break;

            case IR_STUDY_DATA:
            {
                BYTE *data = (BYTE *)msgListNode->msg.pMsgData;
                if(data == NULL)
                    continue;
                
                UINT len = (UINT)msgListNode->msg.msgDataLen;
                PR_DEBUG("len:%d", len);
                
                op_ret = sf_raw_dp_report_sync(get_gw_cntl()->gw_if.id, 2, data, len, 10000);
            }
            break;
        }
        
        if(msgListNode->msg.pMsgData) {
            Free(msgListNode->msg.pMsgData);
        }
        DelAndFreeMsgNodeFromQueue(ir_param.msg_que,msgListNode);
    }
}

OPERATE_RET ir_dp_handle(CHAR_T *buf)
{
    OPERATE_RET op_ret;
    INT_T msg_num = 0;

    op_ret = GetMsgNodeNum(ir_param.msg_que,&msg_num);
    if((OPRT_OK == op_ret) && (msg_num >= MAX_MSG_QUEUE)) {
        return OPRT_MSG_OUT_OF_LMT;
    }

    USHORT len = strlen(buf);
	P_MSG_DATA msg_data = NULL;
    if(len == 0)
        return OPRT_COM_ERROR;
    
    msg_data = Malloc(len+1);
    if(!msg_data) {
        return OPRT_MALLOC_FAILED;
    }       
    memset(msg_data,0,len+1);
    memcpy(msg_data,buf,len);
    
    op_ret = PostMessage(ir_param.msg_que, IR_APP_CMD,\
                         msg_data,len);
    if(OPRT_OK != op_ret) {
        Free(msg_data);
        return OPRT_MSG_LIST_EMPTY;
    }

    return OPRT_OK;
}

OPERATE_RET ir_remote_init(void)
{
    uint8_t i;
    OPERATE_RET op_ret;
        
    memset(&ir_param, 0, sizeof(ir_param));

    op_ret = CreateMsgQueAndInit(&ir_param.msg_que);
    if(OPRT_OK != op_ret) {
        return op_ret;
    }

    op_ret = CreateAndInitSemaphore(&ir_param.ir_send_finish_sem, 0, 1);
    if(OPRT_OK != op_ret) {
        return ;
    }

    op_ret = sys_add_timer(ir_remote_send_timer_cb,NULL,&ir_param.ir_remote_send_timer);
    if(OPRT_OK != op_ret) {
        goto err_exit;
    }

    THRD_PARAM_S thrd_param;
    thrd_param.stackDepth = 1024+512;
    thrd_param.priority = TRD_PRIO_2;
    thrd_param.thrdname = "ir_handle";
    op_ret = CreateAndStart(&ir_param.thread, NULL, NULL, ir_handle_thread, NULL, &thrd_param);
    if(op_ret != OPRT_OK) {
        return ;
    }

    return OPRT_OK;

err_exit:
    return op_ret;
}

/*void ir_prod_test_send(BYTE value)
{
    if(value == 0)
    {
        IR_TX_OUTPUT_LOW(IR_GPIO_OUT_NUM);
    }
    else
    {
        IR_TX_OUTPUT_HIGH(IR_GPIO_OUT_NUM);
    }
}*/



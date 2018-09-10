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
#include "gpio_irq_api.h"
#include "ir_remote.h"
#include "ir_study.h"


#define         MAX_STUDY_CODE              (300)

BYTE study_state = 0;
UINT study_num = 0;
//USHORT study_code[256];
UINT timer_count = 0;
USHORT *study_code = NULL;

STATIC gtimer_t ir_timer;
STATIC gpio_irq_t ir_irq;
STATIC BOOL ir_study_flag = FALSE;

extern IR_PARAM_S ir_param;
extern LED_HANDLE wf_light;

STATIC VOID gpio_intr_disable(VOID);

enum 
{
    STUDY_INIT = 0,
    STUDY_START,
};

STATIC VOID ir_study_data(void)
{
    switch(study_state) {
        case STUDY_INIT: {
            gtimer_stop(&ir_timer);
            
            study_state = STUDY_START;
            study_num = 0;
            //
            timer_count = 0;
            gtimer_start(&ir_timer);
            break;
        }

        case STUDY_START: {
            gtimer_stop(&ir_timer);
            if(timer_count < 100) {
                gtimer_start(&ir_timer);
                return;
            }
            
            if(study_num == 0) {
                if(timer_count < 1000) {
                    //PR_DEBUG("timer_count:%d",timer_count);
                    study_state = STUDY_INIT;
                    timer_count = 0;
                    gtimer_start(&ir_timer);
                    return;
                }
            }

            study_code[study_num] = timer_count;
            study_num ++;
        
            timer_count = 0;
            gtimer_start(&ir_timer);
        }
    }
    
}

gpio_irq_event int_flag = IRQ_RISE;
STATIC VOID gpio_interrupt(uint32_t id, gpio_irq_event event)
{
    static uint32_t val_old, val_new, dval;
    
    if(id == IR_GPIO_NUM) {
        if (int_flag == IRQ_RISE){
            int_flag = IRQ_FALL;
            gpio_irq_set_event(&ir_irq, IRQ_FALL);
        }else if (int_flag == IRQ_FALL){
            int_flag = IRQ_RISE;
            gpio_irq_set_event(&ir_irq, IRQ_RISE);
        }

        ir_study_data();
    }
}

extern BYTE prod_test_state;

STATIC VOID ir_study_timer_cb(u32 data)//(void)
{
    OPERATE_RET op_ret;

    timer_count += 10;
    
    if(timer_count > 500 * 1000)              //500ms
    {
        gpio_intr_disable();
        gtimer_stop(&ir_timer);
        study_code[study_num] = timer_count;
        study_num ++;
        if(study_num >= MAX_STUDY_CODE) {
            //数据超长,退出学习
            ir_study_stop();
        }
            
        //PR_DEBUG("study_state:%d", study_state);
        
        if(study_state == STUDY_START) {
            USHORT len = study_num * 2;

            BYTE *msg_data = NULL;
            if(len == 0) {
//                ir_study_stop();
                ir_study_start();
                return;
            }

            if ((prod_test_state) && (len<(6*8*2*2))){
                ir_study_start();
                return;
            }
            
            msg_data = (BYTE *)Malloc(len+1);
            if(!msg_data) {
                PR_ERR("OPRT_MALLOC_FAILED");
//                ir_study_stop();
                ir_study_start();
                return;
            }       
            memset(msg_data, 0, len+1);
            memcpy(msg_data, (BYTE *)study_code, len);

            UINT i;
            USHORT *pdata = (USHORT *)msg_data;
            for(i = 0; i < study_num; i ++){
                PR_DEBUG_RAW("%4d ", *pdata ++);
            }
            PR_DEBUG_RAW("\r\n");
            
//            ir_study_stop();
            ir_study_start();

            if (prod_test_state){
                    PR_DEBUG("get data test,data: len:%d",len);
                    op_ret = PostMessage(ir_param.msg_que, IR_PROC_DATA,\
                                                     msg_data,len);
            }else{
                op_ret = PostMessage(ir_param.msg_que, IR_STUDY_DATA,\
                             msg_data,len);
            }
            if(OPRT_OK != op_ret) {
                Free(msg_data);
                PR_ERR("OPRT_SND_QUE_ERR");
            }
        }else {
            //PR_DEBUG("error code");
            ir_study_start();
        }
    }   
}

STATIC void TimerInit(VOID)
{
    gtimer_init(&ir_timer, 4);
    TIMx[ir_timer.timer_id]->PSC = 0; //fenpin
    gtimer_start_periodical(&ir_timer, 10, (void*)ir_study_timer_cb, (uint32_t)&ir_timer);//50
    gtimer_stop(&ir_timer);
}

STATIC VOID gpio_intr_init(VOID)
{
    uint32_t gpio_irq_id = IR_GPIO_NUM;
    gpio_irq_init(&ir_irq,IR_GPIO_NUM,gpio_interrupt,(uint32_t)gpio_irq_id);
    int_flag = IRQ_FALL;
    gpio_irq_set(&ir_irq,IRQ_FALL,1);
    gpio_irq_pull_ctrl(&ir_irq, PullNone);
    gpio_irq_enable(&ir_irq);
}

STATIC VOID gpio_intr_disable(VOID)
{
    if (!ir_study_flag)
        return;
    
    gpio_irq_disable(&ir_irq);
}

OPERATE_RET ir_study_start(VOID)
{
    PR_DEBUG("ir_study_start");
    study_num = 0;
    study_state = STUDY_INIT;
    timer_count = 0;

    if(study_code == NULL) {
        study_code = (USHORT *)Malloc(MAX_STUDY_CODE * 2);
        if(study_code == NULL)
            return OPRT_MALLOC_FAILED;
    }
    memset(study_code, 0, sizeof(MAX_STUDY_CODE * 2));

    TimerInit();
    gpio_intr_init();
    ir_send_led_start();

    ir_study_flag = TRUE;

    return OPRT_OK;
}

VOID ir_study_stop(VOID)
{
    PR_DEBUG("ir_study_stop");

    if (ir_study_flag){
        gpio_irq_disable(&ir_irq);
        gtimer_stop(&ir_timer);
        ir_study_flag = FALSE;
    }
    
    study_num = 0;
    study_state = STUDY_INIT;
    timer_count = 0;
    ir_send_led_stop();

    if(study_code != NULL) {
        Free(study_code);
        study_code = NULL; 
    }
}


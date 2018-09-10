#include "irDecode.h"

STATIC uint16_t lastIrCnt;
STATIC uint32_t irCmd;
STATIC IRCODE last_irType;
STATIC IRCODE irType;
STATIC volatile uint32_t timer_cnt = 1;
STATIC IRCMD cur_ircmd;
STATIC int8_t cur_irType;

//STATIC TIMER_ID ir_timer;
STATIC IRDEAL irdeal;

SFT_TIMER LightGraTimer;

uint16_t ir_temp[128];
uint8_t ir_cnt = 0;

STATIC uint8_t isInRange(uint16_t n,uint16_t target,uint8_t range)
{
	if((n > (target - range)) && (n < (target + range)))
		return 1;
	else
	    return 0;
}

STATIC IRCODE IrDecodePh(uint16_t cnt)
{
	if(isInRange(cnt,108,5)){
		return IRCODESTART;
	}
	else if(isInRange(cnt,90,5)){
		return IRCODEREPEAT;
	}
	else if(isInRange(cnt,18,3)){
		return IRCODE1;
	}
	else if(isInRange(cnt,9,2)){
		return IRCODE0;
	}
	
	return IRCODEERROR;
}


STATIC void IrCmdDeal(VOID)
{
	uint8_t *p;
	UCHAR i;
	p = (uint8_t *)&irCmd;
/*	
	for(i=0;i<4;i++){
		PR_DEBUG_RAW("coed%d: 0x%x\r\n",i+1,p[i]);
		irType = IRCODEERROR;
	}
*/
	if(p[0]!=0x40 || p[2]!=0x48 || p[3]!=0xae){
		irCmd = 0;
		irType = IRCODEERROR;
		PR_DEBUG_RAW("IR CODE ERR!\r\n");
		return;
	}

	cur_ircmd = p[1];
	cur_irType = irType;
	
	PostSemaphore(irdeal.ir_cmddeal_sem);
	irType = IRCODEERROR;
	irCmd = 0;
}

STATIC void IrDecode(uint16_t cnt)
{
	IRCODE code;
    STATIC uint8_t irDecodeSecondByte;

#if 0//纯打印
	STATIC uint32_t x = 0;
	STATIC UCHAR ir_cmd_ = 0x00;
	STATIC UCHAR ir_temp_[7];

	ir_temp[ir_cnt] = cnt;
    ir_cnt ++;
    if(ir_cnt >= 32)//大于这个最少按两次
    {
    	//直接数据全局打印
        uint8_t i;
        PR_DEBUG_RAW("ir%d: ",x++);
		ir_temp_[5] = 1;
        for(i = 0; i < 48; i ++)
        {
			if(i%8 == 0)
			PR_DEBUG_RAW("\r\nnum:%d  ",i/8);

			PR_DEBUG_RAW("%04d ", ir_temp[i]);
        }
        PR_DEBUG_RAW("\r\n");
        
        ir_cnt = 0;

		/*下面为解析数据*/
		for(i=0;i<48;i++){//二进制
			if(isInRange(ir_temp[i],18,2)){
				ir_cmd_ = (ir_cmd_<<1) | 0x01;
				PR_DEBUG_RAW("1");
			}
			else if(isInRange(ir_temp[i],9,2)){
				ir_cmd_ = ir_cmd_<<1;
				PR_DEBUG_RAW("0");
			}

			else if(isInRange(ir_temp[i],108,5))
				PR_DEBUG_RAW("S");//头
			else if(isInRange(ir_temp[i],90,5))
				PR_DEBUG_RAW("P");//重复码1
			else if(isInRange(ir_temp[i],145,5))//经检验，重复码2 = CODE0
				PR_DEBUG_RAW("p");//重复码2
			
			else{
				PR_DEBUG_RAW("e");//错误码
			}

			//
			if(i%8 == 7){
				ir_temp_[i/7 - 1] = ir_cmd_;
				PR_DEBUG_RAW("   analysis num:%d \r\n",i/7 - 1);
			}
		}
		PR_DEBUG_RAW("\r\n");
		memset(ir_temp,0,48);

		for(i=0;i<6;i++){//十六进制
			PR_DEBUG_RAW("code num:%d -> 0X%02x\r\n",i,ir_temp_[i]);
		}
    }
#else//解码
//	STATIC IRCODE CmdType = IRCODEERROR;

	if(irType == IRCODESTART){//注意头码容易和重复码靠近
		switch(IrDecodePh(cnt)){
			case IRCODE1:
				irCmd = (irCmd<<1) | 0x01;
			break;
			case IRCODE0:
				irCmd = (irCmd<<1);
			break;
			default:
				irType = IRCODEERROR;
				irCmd = 0;
				ir_cnt = 0;
				return;
			break;
		}
		ir_cnt++;
		if(ir_cnt >= 32){
			IrCmdDeal();
			ir_cnt = 0;
		}
		return;
	}
	if(irType == IRCODEREPEAT){
		if(isInRange(cnt,9,2)){//重复码后面再加一个补充，不容易与头码互相干扰
			cur_irType = IRCODEREPEAT;
			PostSemaphore(irdeal.ir_cmddeal_sem);
			irType = IRCODEERROR;
			return;
		}
	}
	irType = IrDecodePh(cnt);

#endif
}


STATIC void ir_test_timer_handler(uint32_t id)
{
	timer_cnt ++;
	if(timer_cnt > TIMER_CNT_MAX){
		timer_cnt = 0;
	//	ir_cnt = 0;
		irCmd = 0;
	}
}

STATIC void gpio_interrupt(uint32_t id, gpio_irq_event event)
{
    if(id == IR_GPIO_NUM) {
        //获取计数
        IrDecode(timer_cnt);
		timer_cnt = 0;
    }
}

STATIC VOID ir_timer_cb(uint32_t id)
{
	gpio_t* gpio_led = (gpio_t *)id;
	gpio_write(gpio_led, !gpio_read(gpio_led));
	PR_DEBUG_RAW("\r\ntimer3_cd -> %d\r\n",timer_cnt);
}


//*****************************************************
//******************红外线程***************************
STATIC VOID IrCmdDealThread(PVOID pArg)
{
	static int i = 0;
	while(1) {
		WaitSemaphore(irdeal.ir_cmddeal_sem);
		UserIrCmdDeal(cur_ircmd,cur_irType);
	}
}


//********************************************************
//******************红外相关初始化************************
STATIC void gpio_intr_init(VOID)
{
#if 1//IR中断
	uint32_t gpio_irq_id = IR_GPIO_NUM;
	gpio_irq_init(&ir_irq,IR_GPIO_NUM,gpio_interrupt,(uint32_t)gpio_irq_id);
	gpio_irq_set(&ir_irq,IRQ_FALL,1);
	gpio_irq_enable(&ir_irq);
	irType = IRCODEERROR;
	last_irType = IRCODEERROR;
#else//测试使用
	gpio_init(&test_gpio_x, PA_12);
    gpio_dir(&test_gpio_x, PIN_OUTPUT);    // Direction: Output
    gpio_mode(&test_gpio_x, PullNone);     // No pull
    gpio_write(&test_gpio_x,0);//测试正常可用,接高点亮依据亮度看现象
#endif
}
STATIC void TimerInit(VOID)
{
	gtimer_init(&ir_timer, TIMER3);
    gtimer_start_periodical(&ir_timer, 100, (void*)ir_test_timer_handler, NULL);//100us
//	gtimer_stop(&ir_timer);
//	gtimer_start(&ir_timer);
}

STATIC VOID UtilInit(VOID)
{
	OPERATE_RET op_ret;
 	
	op_ret = CreateAndInitSemaphore(&irdeal.ir_cmddeal_sem, 0, 1);
	if(OPRT_OK != op_ret) {
		PR_ERR("CreateAndInitSemaphore err:%d",op_ret);
        return ;
    }
	op_ret = PostSemaphore(irdeal.ir_cmddeal_sem);
	if(OPRT_OK != op_ret) {
		PR_ERR("PostSemaphore err:%d",op_ret);
        return ;
    }
	
	THRD_PARAM_S thrd_param;
    thrd_param.stackDepth = 1024+512;
    thrd_param.priority = TRD_PRIO_2;
    thrd_param.thrdname = "ir_task";
    op_ret = CreateAndStart(&irdeal.ir_thread, NULL, NULL, IrCmdDealThread, NULL, &thrd_param);
    if(op_ret != OPRT_OK) {
		PR_ERR("CreateAndStart ir_thread err:%d",op_ret);
        return ;
    }
	
	PR_DEBUG("UtilInit Finish!");
}
VOID UserInit(VOID)
{
	gtimer_init(&test_timer, TIMER1);
    gtimer_start_periodical(&test_timer, 500000, (void*)ir_timer_cb, (uint32_t)&test_gpio_x);//500ms测试定时器
}
void Infrared_Init(VOID)
{
	gpio_intr_init();
	TimerInit();
	UtilInit();
//	UserInit();
}


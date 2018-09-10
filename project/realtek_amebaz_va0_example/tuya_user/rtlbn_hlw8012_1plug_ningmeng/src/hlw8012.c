#include "hlw8012.h"
#include "uni_semaphore.h"

#include "gpio_api.h"  
#include "timer_api.h"
#include "gpio_irq_api.h"  
#include "sys_timer.h"
#include "uni_log.h"

#define RES_VAL 1
SEM_HANDLE ele_cal_busy;
SEM_HANDLE get_ele_busy;
TIMER_ID get_ele_timer;
BOOL ele_cal_flag = FALSE;

UINT_T P_VAL;
UINT_T V_VAL;
UINT_T I_VAL;
UINT_T E_VAL;
RPT_REF rpt_ref_data;


#define GPIO_CF1_SEL        app_dltj.dltj.ivcpin.pin
#define GPIO_CF             app_dltj.dltj.epin
#define GPIO_CF1            app_dltj.dltj.ivpin

#define IRQ_CF1_NUM    1018
#define IRQ_CF_NUM     1022
#define IRQ_TIMER1_NUM  1012
gtimer_t my_timer1;
gtimer_t my_timer2;
gpio_t gpio_led1;
gpio_t gpio_led2;
gpio_irq_t gpio_btn;

//--------------------------------------------------------------------------------------------
//Time1定时器定时,时间基数 = 1ms
#define D_TIME1_20MS				20/RES_VAL		
#define D_TIME1_100MS				100/RES_VAL
#define D_TIME1_150MS				150/RES_VAL	
#define D_TIME1_200MS				200/RES_VAL	
#define D_TIME1_400MS				400/RES_VAL	
#define D_TIME1_500MS				500/RES_VAL	
#define D_TIME1_1S				1000/RES_VAL		//Time1定时器定时1S时间常数
#define D_TIME1_2S				2000/RES_VAL	
#define D_TIME1_3S				3000/RES_VAL	
#define D_TIME1_4S				4000/RES_VAL	
#define D_TIME1_5S				5000/RES_VAL
#define D_TIME1_6S				6000/RES_VAL
#define D_TIME1_8S				8000/RES_VAL
#define D_TIME1_9S				9000/RES_VAL
#define D_TIME1_10S				10000/RES_VAL
#define D_TIME1_11S				11000/RES_VAL
#define D_TIME1_20S				20000/RES_VAL


#define D_TIME1_V_OVERFLOW          500/RES_VAL        //Time1定时器,电压溢出常数设定为500mS,溢出说明脉宽周期大于500mS
#define D_TIME1_I_OVERFLOW			10000/RES_VAL	   //Time1定时器,电流溢出常数设定为10S,溢出说明脉宽周期大于10S
#define D_TIME1_P_OVERFLOW			10000/RES_VAL	   //Time1定时器,功率溢出常数设定为10S(约0.5W最小值),溢出说明脉宽周期大于10S
//校正时间，记录在此时间内的脉冲数，100W负载在用电36S时间内耗费0.001度电
#define D_TIME1_CAL_TIME			(18000000/app_dltj.dltj.p_def/RES_VAL)


//--------------------------------------------------------------------------------------------
typedef struct 
{
	DLTJ_CONFIG 			dltj;
}APP_DLTJ_S;

APP_DLTJ_S app_dltj;

VOID dltj_copy(DLTJ_CONFIG *des, DLTJ_CONFIG *src)
{
	des->epin 			= src->epin;
	des->ivpin			= src->ivpin;
	des->ivcpin.pin 	= src->ivcpin.pin;
	des->ivcpin.type 	= src->ivcpin.type;
	des->v_ref 			= src->v_ref;
	des->i_ref 			= src->i_ref;
	des->p_ref			= src->p_ref;
	des->e_ref			= src->e_ref;
    des->v_def          = src->v_def;
    des->i_def          = src->i_def;
    des->p_def          = src->p_def;
    des->edpid			= src->edpid;
	des->idpid			= src->idpid;
	des->pdpid			= src->pdpid;
	des->vdpid			= src->vdpid;
	des->if_have		= src->if_have;
}


VOID hlw8012_init(DLTJ_CONFIG *dltj){
	dltj_copy(&(app_dltj.dltj),dltj);
}
//电流电压切换口   
#define ENTER_V_MODE 			gpio_write(&gpio_led1, (app_dltj.dltj.ivcpin.type == IO_DRIVE_LEVEL_LOW ? 0 : 1))
#define ENTER_I_MODE 			gpio_write(&gpio_led1, (app_dltj.dltj.ivcpin.type == IO_DRIVE_LEVEL_LOW ? 1 : 0))
#define DEF_V                   app_dltj.dltj.v_def
#define DEF_I                   app_dltj.dltj.i_def
#define DEF_P                   app_dltj.dltj.p_def

#define my_abs(x,y) ((x)>(y) ? (x)-(y):(y)-(x))

//--------------------------------------------------------------------------------------------
u16	U16_P_TotalTimes;			//当前脉冲 功率测量总时间
u16	U16_V_TotalTimes;			//当前脉冲 电压测量总时间
u16	U16_I_TotalTimes;			//当前脉冲 电流测量总时间

u16	U16_P_OneCycleTime;			//功率测量时间参数
u16	U16_V_OneCycleTime;			//电压测量时间参数
u16	U16_I_OneCycleTime;			//电流测量时间参数

u16	U16_P_Last_OneCycleTime;		//功率测量时间参数，上一次数量值
u16	U16_V_Last_OneCycleTime;		//电压测量时间参数，上一次数量值
u16	U16_I_Last_OneCycleTime;		//电流测量时间参数，上一次数量值

u16	U16_P_CNT;				//功率测量脉冲数量
u16	U16_V_CNT;				//电压测量脉冲数量
u16	U16_I_CNT;				//电流测量脉冲数量

u16	U16_P_Last_CNT;				//功率测量脉冲数量，上一次数量值
u16	U16_V_Last_CNT;				//电压测量脉冲数量，上一次数量值
u16	U16_I_Last_CNT;				//电流测量脉冲数量，上一次数量值

BOOL	B_P_TestOneCycle_Mode;			//功率测量模式 1:单周期测量，0:1S定时测量
BOOL	B_V_TestOneCycle_Mode;
BOOL	B_I_TestOneCycle_Mode;

BOOL	B_P_Last_TestOneCycle_Mode;
BOOL	B_V_Last_TestOneCycle_Mode;
BOOL	B_I_Last_TestOneCycle_Mode;
    		
BOOL  	B_P_OVERFLOW;       			// 功率脉冲周期 溢出标志位 
BOOL  	B_V_OVERFLOW;       			// 电压脉冲周期 溢出标志位
BOOL  	B_I_OVERFLOW;       			// 电流脉冲周期 溢出标志位

BOOL	B_P_Last_OVERFLOW;       		// 功率脉冲周期 溢出标志位 
BOOL  	B_V_Last_OVERFLOW;       		// 电压脉冲周期 溢出标志位
BOOL  	B_I_Last_OVERFLOW;       		// 电流脉冲周期 溢出标志位

BOOL    B_VI_Test_Mode;				//1:电压测量模式;0:电流测量模式
u16   	U16_VI_Test_Times;				
u16   	U16_Cal_Times;	

u16   	U16_AC_P;				//功率值 1000.0W
u16   	U16_AC_V;				//电压值 220.0V
u16   	U16_AC_I;				//电流值 4.545A
UINT_T   	U32_AC_E;				//用电量   0.01度

UINT_T  	U16_REF_001_E_Pluse_CNT;        	//0.1度电脉冲总数参考值
UINT_T   	U16_E_Pluse_CNT;          	 	//脉冲个数寄存器

UINT_T   	U32_Cal_Times;                 		//校正时间

UINT_T   	U32_P_REF_PLUSEWIDTH_TIME;      	//参考功率 脉冲周期
UINT_T   	U32_V_REF_PLUSEWIDTH_TIME;      	//参考电压 脉冲周期
UINT_T   	U32_I_REF_PLUSEWIDTH_TIME;      	//参考电流 脉冲周期

UINT_T   	U32_P_CURRENT_PLUSEWIDTH_TIME;      	//当前功率 脉冲周期
UINT_T   	U32_V_CURRENT_PLUSEWIDTH_TIME;      	//当前电压 脉冲周期
UINT_T   	U32_I_CURRENT_PLUSEWIDTH_TIME;      	//当前电流 脉冲周期

u16   	U16_P_REF_Data;				//参考功率值,如以1000W校正。1000.0W
u16   	U16_V_REF_Data;				//参考电压  220.0V
u16   	U16_I_REF_Data;				//参考电流  1000W,220V条件下是4.545A

u8    	U8_CURR_WorkMode;
//--------------------------------------------------------------------------------------------

/*-------------------------------------------- 功率、电压、电流计算 -------------------------------------------*/


void gpio_interrupt(uint32_t id, gpio_irq_event event);


/*=====================================================
 * Function : void HLW8012_Measure_P(void)
 * Describe : 
 * Input    : none
 * Output   : none
 * Return   : none
 * Record   : 2014/04/14
=====================================================*/
void HLW8012_Measure_P(void)
{

    if (B_P_Last_TestOneCycle_Mode == 1)
    {
		U32_P_CURRENT_PLUSEWIDTH_TIME = U16_P_Last_OneCycleTime*1000;
    }
    else
    {
    	if(U16_P_Last_CNT < 2){
			return;
		}
		U32_P_CURRENT_PLUSEWIDTH_TIME = U16_P_Last_OneCycleTime*1000/(U16_P_Last_CNT-1);
    }

	if (U8_CURR_WorkMode == D_CAL_START_MODE)
    {
		U32_P_REF_PLUSEWIDTH_TIME = U32_P_CURRENT_PLUSEWIDTH_TIME;	   // 校正时取U32_P_CURRENT_PLUSEWIDTH_TIME参数作为参考值	 
		return;
	}
	if(U32_P_CURRENT_PLUSEWIDTH_TIME == 0){
		U16_AC_P = 0;
	}else{
		U16_AC_P = U16_P_REF_Data * U32_P_REF_PLUSEWIDTH_TIME/U32_P_CURRENT_PLUSEWIDTH_TIME;
	}
    
    
    if (U16_AC_P == 0xffff)     //开机时U32_P_CURRENT_PLUSEWIDTH_TIME = 0，计算溢出
    {
        U16_AC_P = 0; 
    }
    
    if (B_P_Last_OVERFLOW == TRUE)
    {
        U16_AC_P = 0;
    }
}
/*=====================================================
 * Function : void HLW8012_Measure_V(void)
 * Describe : 
 * Input    : none
 * Output   : none
 * Return   : none
 * Record   : 2014/04/14
=====================================================*/
void HLW8012_Measure_V(void)
{

    if (B_V_Last_TestOneCycle_Mode == 1)
    {
		U32_V_CURRENT_PLUSEWIDTH_TIME = U16_V_Last_OneCycleTime*1000;  
    }
    else
    {
    	if(U16_V_Last_CNT < 2){
			return;
		}
		U32_V_CURRENT_PLUSEWIDTH_TIME = U16_V_Last_OneCycleTime*1000/(U16_V_Last_CNT-1);  
    }
	if (U8_CURR_WorkMode == D_CAL_START_MODE)
	{
		U32_V_REF_PLUSEWIDTH_TIME = U32_V_CURRENT_PLUSEWIDTH_TIME;	   // 校正时取UINT_T_V_Period参数作为参考值	 
		return;
	}
	if(U32_V_CURRENT_PLUSEWIDTH_TIME == 0){
		U16_AC_V = 0;

	}else{
		U16_AC_V = U16_V_REF_Data * U32_V_REF_PLUSEWIDTH_TIME/U32_V_CURRENT_PLUSEWIDTH_TIME;
	}
    
    if (U16_AC_V == 0xffff)     //开机时U32_V_CURRENT_PLUSEWIDTH_TIME = 0，计算溢出
    {
        U16_AC_V = 0; 
    }
    
     if (B_V_Last_OVERFLOW == TRUE)
    {
        U16_AC_V = 0;
    }
    
 /*   if (U16_AC_P == 0)
    {
        U16_AC_V = 0;
      
    }
*/
}
/*=====================================================
 * Function : void HLW8012_Measure_I(void)
 * Describe : 
 * Input    : none
 * Output   : none
 * Return   : none
 * Record   : 2014/04/14
=====================================================*/
void HLW8012_Measure_I(void)
{
    if (B_I_Last_TestOneCycle_Mode == 1)
    {
        U32_I_CURRENT_PLUSEWIDTH_TIME = U16_I_Last_OneCycleTime*1000; 
    }
    else
    {
    	if(U16_I_Last_CNT < 2){
			return;
		}
         U32_I_CURRENT_PLUSEWIDTH_TIME = U16_I_Last_OneCycleTime*1000/(U16_I_Last_CNT-1); 
    }
	if (U8_CURR_WorkMode == D_CAL_START_MODE)
    {
		U32_I_REF_PLUSEWIDTH_TIME = U32_I_CURRENT_PLUSEWIDTH_TIME;	   // 校正时取U32_V_CURRENT_PLUSEWIDTH_TIME参数作为参考值	 
		return;
	}
	if(U32_I_CURRENT_PLUSEWIDTH_TIME == 0){
		U16_AC_I = 0;
	}else{
		U16_AC_I = U16_I_REF_Data * U32_I_REF_PLUSEWIDTH_TIME/U32_I_CURRENT_PLUSEWIDTH_TIME;
	}

    if (U16_AC_P == 0)
    {
        U16_AC_I = 0;
      
    }
  
    if (U16_AC_I == 0xffff)     //开机时U32_I_CURRENT_PLUSEWIDTH_TIME = 0，计算溢出
    {
        U16_AC_I = 0; 
    }
    
    if (B_I_Last_OVERFLOW == TRUE)
    {
        U16_AC_I = 0;  
    }
}



void gpio_interrupt(uint32_t id, gpio_irq_event event)
{
    //PR_DEBUG("!!!!!!!!!!!!!!!!!!!!irq_id:%d", id);
	if(id == IRQ_CF_NUM){
	    //PR_DEBUG("!!!!!!!!!!!!!!!!!!!!!!!!!!!!CF_INT");
		U16_P_TotalTimes = 0;		//完成一次有效的测量，溢出寄存器清零	
		U16_P_CNT++;
		if (B_P_OVERFLOW == TRUE)
		{  
			//从溢出模式转入,开始测量	  
			B_P_TestOneCycle_Mode = 0;	//初始化为计数脉冲测量模式
			U16_P_TotalTimes = 0;		//清溢出寄存器清零
			U16_P_OneCycleTime = 0; 	//清测量寄存器
			U16_P_CNT = 1;				
			B_P_OVERFLOW = FALSE;		//清溢出标志位
		}
		else
		{
			if (B_P_TestOneCycle_Mode == 1)
			{
				if (U16_P_OneCycleTime >= D_TIME1_100MS)
				{
					//单周期测量模式 
					U16_P_Last_OneCycleTime = U16_P_OneCycleTime;
					B_P_Last_TestOneCycle_Mode = B_P_TestOneCycle_Mode;
					B_P_OVERFLOW = FALSE;		//溢出标志位清零
					B_P_Last_OVERFLOW = B_P_OVERFLOW;
					 //清状态参数,重新开始测试
					B_P_TestOneCycle_Mode = 0;	//初始化为计数脉冲测量模式
					U16_P_TotalTimes = 0;		//完成一次有效的测量，溢出寄存器清零
					U16_P_OneCycleTime = 0; 	//清测量寄存器
					U16_P_CNT = 1;
				}
			}
			else
			{
				if (U16_P_OneCycleTime >= D_TIME1_1S)
				{	
					U16_P_Last_OneCycleTime = U16_P_OneCycleTime;
					U16_P_Last_CNT = U16_P_CNT;
					B_P_Last_TestOneCycle_Mode = B_P_TestOneCycle_Mode;
					B_P_OVERFLOW = FALSE;		//溢出标志位清零
					B_P_Last_OVERFLOW = B_P_OVERFLOW;
					//清状态参数,重新开始测试
					B_P_TestOneCycle_Mode = 0;	//初始化为计数脉冲测量模式
					U16_P_TotalTimes = 0;		//完成一次有效的测量，溢出寄存器清零
					U16_P_OneCycleTime = 0; 	//清测量寄存器
					U16_P_CNT = 1;
				}
			}
		}
		//校正模式
		if (U8_CURR_WorkMode == D_CAL_START_MODE)
		{
			//记录单位时间内的用电量
			U16_E_Pluse_CNT++;
		}
		
	//用电量计量，每0.1度电，用电量寄存器增加0.1度
		if (U8_CURR_WorkMode == D_NORMAL_MODE)
		{
			U16_E_Pluse_CNT++;
			if (U16_E_Pluse_CNT == 2*U16_REF_001_E_Pluse_CNT )
			{
				U16_E_Pluse_CNT = 0;
				U32_AC_E++;
			}
		}
	}
	if(id == IRQ_CF1_NUM){
	    //PR_DEBUG("!!!!!!!!!!!!!!!!!!!!!!!!!!!!CF1_INT");
		//电压测试模式
		if (B_VI_Test_Mode == 1)
		{
			U16_V_TotalTimes = 0; 
			U16_V_CNT++;
			if (B_V_OVERFLOW == TRUE)
			{				   
				//从溢出模式转入,开始测量	  
				B_V_TestOneCycle_Mode = 0;	//初始化为计数脉冲测量模式
				U16_V_TotalTimes = 0;		//清溢出寄存器清零
				U16_V_OneCycleTime = 0; 	//清测量寄存器
				U16_V_CNT = 1;				
				B_V_OVERFLOW = FALSE;		//清溢出标志位
			}
			else
			{
				if (B_V_TestOneCycle_Mode == 1)
				{
					if (U16_V_OneCycleTime >= D_TIME1_100MS)
					{
						//单周期测量模式 
						U16_V_Last_OneCycleTime = U16_V_OneCycleTime;
						B_V_Last_TestOneCycle_Mode = B_V_TestOneCycle_Mode;
						B_V_OVERFLOW = FALSE;		//溢出标志位清零
						B_V_Last_OVERFLOW = B_V_OVERFLOW;
						 //清状态参数,重新开始测试
						B_V_TestOneCycle_Mode = 0;	//初始化为计数脉冲测量模式
						U16_V_TotalTimes = 0;		//完成一次有效的测量，溢出寄存器清零
						U16_V_OneCycleTime = 0; 	//清测量寄存器
						U16_V_CNT = 1;
					}
				}
				else
				{
					if (U16_V_OneCycleTime >= D_TIME1_200MS)
					{	
						U16_V_Last_OneCycleTime = U16_V_OneCycleTime;
						U16_V_Last_CNT = U16_V_CNT;
						B_V_Last_TestOneCycle_Mode = B_V_TestOneCycle_Mode; 
						B_V_OVERFLOW = FALSE;		//溢出标志位清零
						B_V_Last_OVERFLOW = B_V_OVERFLOW;
						//清状态参数,重新开始测试
						B_V_TestOneCycle_Mode = 0;	//初始化为计数脉冲测量模式
						U16_V_TotalTimes = 0;		//完成一次有效的测量，溢出寄存器清零
						U16_V_OneCycleTime = 0; 	//清测量寄存器
						U16_V_CNT = 1;
						B_V_OVERFLOW = FALSE;		//溢出标志位清零
					}
				}
			}
		 }

	//电流测试模式
		if (B_VI_Test_Mode == 0)
		{
			U16_I_TotalTimes = 0; 
			U16_I_CNT++;
			if (B_I_OVERFLOW == TRUE)
			{
				//从溢出模式转入,开始测量	  
				B_I_TestOneCycle_Mode = 0;	//初始化为计数脉冲测量模式
				U16_I_TotalTimes = 0;		//清溢出寄存器清零
				U16_I_OneCycleTime = 0; 	//清测量寄存器
				U16_I_CNT = 1;				
				B_I_OVERFLOW = FALSE;		//清溢出标志位
			}
			else
			{
				if (B_I_TestOneCycle_Mode == 1)
				{
					if (U16_I_OneCycleTime >= D_TIME1_100MS)
					{
						//单周期测量模式 
						U16_I_Last_OneCycleTime = U16_I_OneCycleTime;
						B_I_Last_TestOneCycle_Mode = B_I_TestOneCycle_Mode;
						B_I_OVERFLOW = FALSE;		//溢出标志位清零
						B_I_Last_OVERFLOW = B_I_OVERFLOW;
						 //清状态参数,重新开始测试
						B_I_TestOneCycle_Mode = 0;	//初始化为计数脉冲测量模式
						U16_I_TotalTimes = 0;		//完成一次有效的测量，溢出寄存器清零
						U16_I_OneCycleTime = 0; 	//清测量寄存器
						U16_I_CNT = 1;
					}
				}
				else
				{
					if (U16_I_OneCycleTime >= D_TIME1_1S)
					{	
						U16_I_Last_OneCycleTime = U16_I_OneCycleTime;
						U16_I_Last_CNT = U16_I_CNT;
						B_I_Last_TestOneCycle_Mode = B_I_TestOneCycle_Mode;  
						B_I_OVERFLOW = FALSE;		//溢出标志位清零
						B_I_Last_OVERFLOW = B_I_OVERFLOW;
						//清状态参数,重新开始测试
						B_I_TestOneCycle_Mode = 0;	//初始化为计数脉冲测量模式
						U16_I_TotalTimes = 0;		//完成一次有效的测量，溢出寄存器清零
						U16_I_OneCycleTime = 0; 	//清测量寄存器
						U16_I_CNT = 1;
					}
				}
			}
		}
	}
}

STATIC VOID hw_test_timer_cb(uint32_t id)
{

	if (U8_CURR_WorkMode == D_CAL_START_MODE)
    {
        U32_Cal_Times++;//校正时间21S，1000W负载21S时间消耗0.1/200度电
        if (U32_Cal_Times == D_TIME1_CAL_TIME)
        {
            U16_REF_001_E_Pluse_CNT = U16_E_Pluse_CNT;		//记录36S时间内的脉冲数，此脉冲数表示0.01度用电量
            PostSemaphore(ele_cal_busy);
        }
    }
    
//功率测量
    if (U16_P_CNT != 0)
    {
        U16_P_OneCycleTime++;
        U16_P_TotalTimes++;
    }  
    if (U16_P_TotalTimes >= D_TIME1_P_OVERFLOW)
    {
        B_P_OVERFLOW = TRUE; 		//溢出，
        B_P_Last_OVERFLOW = B_P_OVERFLOW;
        //清状态参数,重新开始测试
        U16_P_TotalTimes = 0;       //清溢出寄存器
        U16_P_OneCycleTime = 0;
        U16_P_CNT = 0;              //等待下一次中断开始计数
        B_P_TestOneCycle_Mode = 0;   //初始化为计数脉冲测量模式      
    }
    else if (U16_P_OneCycleTime == D_TIME1_100MS)
    {
		if (U16_P_CNT < 2)
		{
			// 100ms内只有一次中断，说明周期>100ms,采用单周期测量模式 
			B_P_TestOneCycle_Mode = 1;
		}
		else
		{
			 // 100ms内有2次或以上数量脉冲，说明周期<100ms，采用计数脉冲测量模式
			 B_P_TestOneCycle_Mode = 0;   
		}
    }
    
//电压、电流测量
    if (B_VI_Test_Mode == 1)
    {
        //电压测量 
        if (U16_V_CNT != 0)
    	{
	        U16_V_OneCycleTime++;
	        U16_V_TotalTimes++;
        }

        if (U16_V_TotalTimes >= D_TIME1_V_OVERFLOW)
        {
            B_V_OVERFLOW = TRUE; 
            B_V_Last_OVERFLOW = B_V_OVERFLOW;
            //清状态参数,重新开始测试
            U16_V_TotalTimes = 0;       //清溢出寄存器
            U16_V_OneCycleTime = 0;
            U16_V_CNT = 0;              
            B_V_TestOneCycle_Mode = 0;   //初始化为计数脉冲测量模式  
        }
        else if (U16_V_OneCycleTime == D_TIME1_100MS)
        {
			if (U16_V_CNT < 2)
			{
				// 100ms内只有一次中断，说明周期>100ms,采用单周期测量模式 
				B_V_TestOneCycle_Mode = 1;
			}
			else
			{
				// 100ms内有2次或以上数量脉冲，说明周期<100ms，采用计数脉冲测量模式
				B_V_TestOneCycle_Mode = 0;   
			}
        }
    }
    else
    {
        //电流测量   
        if (U16_I_CNT != 0)
    	{
			U16_I_OneCycleTime++;
			U16_I_TotalTimes++;
        }

        if (U16_I_TotalTimes >= D_TIME1_I_OVERFLOW)
        {
            B_I_OVERFLOW = TRUE; 
            B_I_Last_OVERFLOW = B_I_OVERFLOW;
            //清状态参数,重新开始测试
            U16_I_TotalTimes = 0;       //清溢出寄存器
            U16_I_OneCycleTime = 0;
            U16_I_CNT = 0;
            B_I_TestOneCycle_Mode = 0;   //初始化为计数脉冲测量模式      
        }
        else if (U16_I_OneCycleTime == D_TIME1_100MS)
        {
			if (U16_I_CNT < 2)
			{
			// 100ms内只有一次中断，说明周期>100ms,采用单周期测量模式 
			B_I_TestOneCycle_Mode = 1;
			}
			else
			{
			 // 100ms内有2次或以上数量脉冲，说明周期<100ms，采用计数脉冲测量模式
			 B_I_TestOneCycle_Mode = 0;   
			}
        }
    }
      

//电压、电流测量模式切换  B_VI_Test_Mode:(1:电压测量模式) (0:电流测试模式) 
    U16_VI_Test_Times--;
	if(B_VI_Test_Mode == 1){
		if(U16_VI_Test_Times == 1000/RES_VAL){
			ele_cal_flag = TRUE;
		}
	}else{
		if(U16_VI_Test_Times == 8000/RES_VAL){
			ele_cal_flag = TRUE;
		}
	}
	
    if (U16_VI_Test_Times == 0)
    {

    	ele_cal_flag = FALSE;
        if (B_VI_Test_Mode == 1)
        {
            //转为电流测量模式
            B_VI_Test_Mode = 0;
            //PR_DEBUG("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!ENTER_I_MODE");
			ENTER_I_MODE;
            U16_VI_Test_Times = D_TIME1_10S;
            
            //清状态参数
            U16_I_TotalTimes = 0;
            U16_I_OneCycleTime = 0;
            U16_I_CNT = 0;
            B_I_OVERFLOW = FALSE; 
        }
        else
        {
            //转为电压测量模式
            B_VI_Test_Mode = 1;
            //PR_DEBUG("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!ENTER_V_MODE");
			ENTER_V_MODE;
            U16_VI_Test_Times = D_TIME1_1S;
            //清状态参数
            U16_V_TotalTimes = 0;
            U16_V_OneCycleTime = 0;
            U16_V_CNT = 0;
            B_V_OVERFLOW = FALSE; 
        }
    }
}

STATIC VOID Parameter_Init(void)
{

	U16_AC_P = 0;
	U16_AC_V = 0;
	U16_AC_I = 0;
	P_VAL = 0;
	V_VAL = 0;
	I_VAL = 0;
	E_VAL = 0;

	U16_P_TotalTimes = 0;
	U16_V_TotalTimes = 0;
	U16_I_TotalTimes = 0;

	U16_P_OneCycleTime = 0;
	U16_V_OneCycleTime = 0;
	U16_I_OneCycleTime = 0;
	U16_P_Last_OneCycleTime = 0;
	U16_V_Last_OneCycleTime = 0;
	U16_I_Last_OneCycleTime = 0;

	U16_P_CNT = 0;
	U16_V_CNT = 0;
	U16_I_CNT = 0;
	U16_P_Last_CNT = 0;
	U16_V_Last_CNT = 0;
	U16_I_Last_CNT = 0;

	//初始化单周期测量模式
	B_P_TestOneCycle_Mode = 1;
	B_V_TestOneCycle_Mode = 1;
	B_I_TestOneCycle_Mode = 1;
	B_P_Last_TestOneCycle_Mode = 1;
	B_V_Last_TestOneCycle_Mode = 1;
	B_I_Last_TestOneCycle_Mode = 1;

	//开始测量，置溢出标志位为1  
	B_P_OVERFLOW = 1;
	B_V_OVERFLOW = 1;
	B_I_OVERFLOW = 1;

	B_P_Last_OVERFLOW = 1;
	B_V_Last_OVERFLOW = 1;
	B_I_Last_OVERFLOW = 1;

	//上电初始化为电压测试模式 
	B_VI_Test_Mode = 1;
	ENTER_V_MODE;
	U16_VI_Test_Times = D_TIME1_2S;

	U32_AC_E = 0;
	E_VAL = 0;
	U16_E_Pluse_CNT = 0;


	//设置默认值
	U16_P_REF_Data = DEF_P;     // 857 =  85.7W
	U16_V_REF_Data = DEF_V;     // 2202 = 220.2V
	U16_I_REF_Data = DEF_I;     // 386 =  386mA

    U32_Cal_Times = 0;
}

STATIC OPERATE_RET set_coefficient(VOID)
{
	OPERATE_RET op_ret;
    cJSON *root = NULL;
	UCHAR *buf = NULL;

    root = cJSON_CreateObject();
    if(NULL == root) {
		PR_ERR("cJSON_CreateObject error");
		return OPRT_CJSON_GET_ERR;
	}
    
    cJSON_AddNumberToObject(root, "Kp", U32_P_REF_PLUSEWIDTH_TIME);
	cJSON_AddNumberToObject(root, "Kv", U32_V_REF_PLUSEWIDTH_TIME);
	cJSON_AddNumberToObject(root, "Ki", U32_I_REF_PLUSEWIDTH_TIME);
	cJSON_AddNumberToObject(root, "Ke", U16_REF_001_E_Pluse_CNT);
    buf = cJSON_PrintUnformatted(root);
    if(NULL == buf) {
        PR_ERR("buf is NULL");
        cJSON_Delete(root);
        return OPRT_COM_ERROR;
    }    
	PR_DEBUG("msf_set ele coe:%s",buf);
    cJSON_Delete(root);
    
	op_ret = flash_self_if_write(COE_SAVE_KEY,buf);
	if(OPRT_OK != op_ret) {
		PR_DEBUG("msf_set_single err:%02x",op_ret);
        Free(buf);
		return op_ret;
	}
    Free(buf);
    return OPRT_OK;    
}

STATIC VOID set_coe_init(VOID)
{
	U32_V_REF_PLUSEWIDTH_TIME = app_dltj.dltj.v_ref;
	U32_I_REF_PLUSEWIDTH_TIME = app_dltj.dltj.i_ref;
	U32_P_REF_PLUSEWIDTH_TIME = app_dltj.dltj.p_ref;
	U16_REF_001_E_Pluse_CNT = app_dltj.dltj.e_ref;
	PR_DEBUG("coe have been set to ref value!");
}

STATIC OPERATE_RET get_coefficient(VOID)
{
	OPERATE_RET op_ret;
    cJSON *root = NULL, *json = NULL;
	UCHAR *buf;
    op_ret = get_prod_test_data(&rpt_ref_data.prod_rslt);
	if(OPRT_OK != op_ret) {
		PR_ERR("get prod test result err:%02x",op_ret);
		rpt_ref_data.prod_rslt = 2;//读不到产测结果，说明未进行产测，prod_rslt = 2
	}
    PR_DEBUG("get prod test result!!!:%d",rpt_ref_data.prod_rslt);
    buf = (UCHAR *)Malloc(256);
	if(NULL == buf) {
		PR_ERR("malloc error");
		set_coe_init();
		return OPRT_MALLOC_FAILED;
	}
    UINT_T buf_len;
	op_ret = kvs_read(COE_SAVE_KEY, &buf, &buf_len);
	if(OPRT_OK != op_ret) {
		PR_DEBUG("msf_get_single err:%02x",op_ret);
		set_coe_init();
        Free(buf);
		return op_ret;
	}
    PR_DEBUG("msf_get ele coe:%s",buf);

	root = cJSON_Parse(buf);
	if(NULL == root) {
		PR_ERR("cjson parse err");
        goto JSON_PARSE_ERR;
	}

    json = cJSON_GetObjectItem(root,"Kv");
    if(NULL == json) {
        goto JSON_PARSE_ERR;
	}else{
		rpt_ref_data.v_ref = json->valueint;
		
	}
	json = cJSON_GetObjectItem(root,"Ki");
    if(NULL == json) {
        goto JSON_PARSE_ERR;
	}else{
		rpt_ref_data.i_ref = json->valueint;
		
	}
	json = cJSON_GetObjectItem(root,"Kp");
    if(NULL == json) {
        goto JSON_PARSE_ERR;
	}else{
		rpt_ref_data.p_ref = json->valueint;
	}
	json = cJSON_GetObjectItem(root,"Ke");
	if(NULL == json) {
        goto JSON_PARSE_ERR;
	}else{
		rpt_ref_data.e_ref = json->valueint;
	}

    cJSON_Delete(root);
    Free(buf);
    if(1 == rpt_ref_data.prod_rslt || (2 == rpt_ref_data.prod_rslt && rpt_ref_data.v_ref)){//兼容老版本
        U32_V_REF_PLUSEWIDTH_TIME = rpt_ref_data.v_ref;
        U32_I_REF_PLUSEWIDTH_TIME = rpt_ref_data.i_ref;
        U32_P_REF_PLUSEWIDTH_TIME = rpt_ref_data.p_ref;
        U16_REF_001_E_Pluse_CNT = rpt_ref_data.e_ref;
    }else{
        set_coe_init();
    }
    return OPRT_OK;

JSON_PARSE_ERR:
	set_coe_init();
    cJSON_Delete(root);
    Free(buf);
    return OPRT_COM_ERROR;
}

STATIC VOID timer_init(UINT_T delay_times)
{
#if 0
	hw_timer_init(1,hw_test_timer_cb);
	//1ms 中断一次
	hw_timer_arm(delay_times);
    hw_timer_enable();
#else
//todo
    gtimer_init(&my_timer1, TIMER2);
    gtimer_start_periodical(&my_timer1, 1000, \
    (void*)hw_test_timer_cb, IRQ_TIMER1_NUM);
#endif
}

STATIC VOID gpio_intr_init(VOID)
{

/*
    GPIO_ConfigTypeDef gpio_test;
	gpio_test.GPIO_Pin=1<<(Get_ELE_PORT )|( 1<<Get_CUR_VOL);
	gpio_test.GPIO_Mode=GPIO_Mode_Input;
	gpio_test.GPIO_IntrType=GPIO_PIN_INTR_NEGEDGE;
	gpio_test.GPIO_Pullup=GPIO_PullUp_EN;
    gpio_config(&gpio_test);
   
	gpio_intr_handler_register(gpio_interrupt);
	_xt_isr_unmask(1<<ETS_GPIO_INUM);//Enable GPIO

	GPIO_ConfigTypeDef config = {BIT(CUR_VOL_SWITCH), GPIO_Mode_Output, GPIO_PullUp_EN, GPIO_PIN_INTR_DISABLE};
	gpio_config(&config);
	
*/
    gpio_init(&gpio_led1, GPIO_CF1_SEL);
    gpio_dir(&gpio_led1, PIN_OUTPUT);    // Direction: Output
    gpio_mode(&gpio_led1, PullNone);     // No pull
    gpio_write(&gpio_led1,!0);

    /*
    gpio_init(&gpio_led2, GPIO_LED_PIN2);
    gpio_dir(&gpio_led2, PIN_OUTPUT);    // Direction: Output
    gpio_mode(&gpio_led2, PullNone);     // No pull
    gpio_write(&gpio_led2,!0);
    */
    if(gpio_irq_init(&gpio_btn, GPIO_CF, gpio_interrupt, IRQ_CF_NUM)){
        PR_ERR("gpio cf init err!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }
    gpio_irq_set(&gpio_btn, IRQ_FALL, 1);   // Falling Edge Trigger
    gpio_irq_enable(&gpio_btn);
    if(gpio_irq_init(&gpio_btn, GPIO_CF1, gpio_interrupt, IRQ_CF1_NUM)){
        PR_ERR("gpio cf1 init err!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }
    gpio_irq_set(&gpio_btn, IRQ_FALL, 1);   // Falling Edge Trigger
    gpio_irq_enable(&gpio_btn);

}

STATIC VOID get_ele_timer_cb(UINT_T timerID,PVOID pTimerArg)
{
	if(ele_cal_flag == TRUE){
		HLW8012_Measure_P();
		HLW8012_Measure_V();
		HLW8012_Measure_I();
		xSemaphoreTake(get_ele_busy,portMAX_DELAY);
		P_VAL = U16_AC_P;
		V_VAL = U16_AC_V;
		I_VAL = U16_AC_I;
		E_VAL += U32_AC_E;
		U32_AC_E = 0;
		xSemaphoreGive(get_ele_busy);
	}
}

STATIC BOOL cal_data_judge(VOID)
{
    static char pdata[100];
    BOOL opt = TRUE;
  
	PR_DEBUG("v:%d I:%d P:%d E:%d",\
		U32_V_REF_PLUSEWIDTH_TIME,\
		U32_I_REF_PLUSEWIDTH_TIME,\
		U32_P_REF_PLUSEWIDTH_TIME,\
		U16_REF_001_E_Pluse_CNT);


	if(my_abs(U32_V_REF_PLUSEWIDTH_TIME,app_dltj.dltj.v_ref) > app_dltj.dltj.v_ref/2)
        opt = FALSE;
	if(my_abs(U32_I_REF_PLUSEWIDTH_TIME,app_dltj.dltj.i_ref) > app_dltj.dltj.i_ref/2)
		opt = FALSE;
	if(my_abs(U32_P_REF_PLUSEWIDTH_TIME,app_dltj.dltj.p_ref) > app_dltj.dltj.p_ref/2)
		opt = FALSE;
	if(my_abs(U16_REF_001_E_Pluse_CNT,app_dltj.dltj.e_ref) > app_dltj.dltj.e_ref/2)
		opt = FALSE;

    sprintf(pdata,"(%s) --> NEW={v:%d I:%d P:%d E:%d}; REF={v:%d I:%d P:%d E:%d}",
            (opt == TRUE ? "TRUE":"FALSE"),
            U32_V_REF_PLUSEWIDTH_TIME,U32_I_REF_PLUSEWIDTH_TIME,U32_P_REF_PLUSEWIDTH_TIME,U16_REF_001_E_Pluse_CNT,
            app_dltj.dltj.v_ref,app_dltj.dltj.i_ref,app_dltj.dltj.p_ref,app_dltj.dltj.e_ref);

    return opt;
}



OPERATE_RET ele_cnt_init(INT_T mode)
{
	OPERATE_RET op_ret;

	U8_CURR_WorkMode = mode;

	if(U8_CURR_WorkMode != D_CAL_START_MODE){
		op_ret = get_coefficient();
		if(op_ret != OPRT_OK ){
			PR_ERR("coe get err: %d",op_ret);
		}
	}else{
		U32_V_REF_PLUSEWIDTH_TIME = 0;
		U32_I_REF_PLUSEWIDTH_TIME = 0;
		U32_P_REF_PLUSEWIDTH_TIME = 0;
		U16_REF_001_E_Pluse_CNT = 0;
		PR_DEBUG("enter ele cal mode");
	}
	
	Parameter_Init();
	timer_init(1000*RES_VAL);
	gpio_intr_init();
	
	vSemaphoreCreateBinary(get_ele_busy);//创造信号量
	op_ret = sys_add_timer(get_ele_timer_cb,NULL,&get_ele_timer);
    if(OPRT_OK != op_ret) {
        gtimer_stop(&my_timer1);
        gpio_irq_disable(&gpio_btn);
		PR_ERR("add get_ele_timer err,ele cnt is disable");
    	return op_ret;
    }else {
    	sys_start_timer(get_ele_timer,500,TIMER_CYCLE);
    }
	if(U8_CURR_WorkMode == D_CAL_START_MODE){
		op_ret = CreateAndInitSemaphore(&ele_cal_busy,0,1);
		if(op_ret != OPRT_OK){
            gtimer_stop(&my_timer1);
            gpio_irq_disable(&gpio_btn);
	        sys_delete_timer(get_ele_timer); 
	        PR_ERR("creat ele sem err!!!!!!!!!!!!");
			return OPRT_COM_ERROR;
		}
		WaitSemaphore(ele_cal_busy);
 
        gtimer_stop(&my_timer1);
        gpio_irq_disable(&gpio_btn);
	    sys_delete_timer(get_ele_timer); 
	    ReleaseSemaphore(ele_cal_busy); 

		if(FALSE == cal_data_judge()){
		    op_ret = set_coefficient();
		    save_prod_test_data(0);
			return OPRT_COM_ERROR;
		}
		op_ret = set_coefficient();
		save_prod_test_data(1);;
		if(OPRT_OK != op_ret){
			return op_ret;
		}
	}

	return OPRT_OK;
}

VOID get_ele_par(OUT UINT_T *P,OUT UINT_T *V,OUT UINT_T *I)
{
	xSemaphoreTake(get_ele_busy,portMAX_DELAY);
	*P = P_VAL;
	*V = V_VAL;
	*I = I_VAL;
	xSemaphoreGive(get_ele_busy);
}

VOID get_ele(OUT UINT_T *E)
{
	xSemaphoreTake(get_ele_busy,portMAX_DELAY);
	*E = E_VAL;
	E_VAL = 0;
	xSemaphoreGive(get_ele_busy);
}

VOID get_dltj_ref_data(OUT UINT_T *PROD_RSLT,OUT UINT_T *V_REF,OUT UINT_T *I_REF,OUT UINT_T *P_REF, OUT UINT_T *E_REF)
{
	xSemaphoreTake(get_ele_busy,portMAX_DELAY);
	*PROD_RSLT = rpt_ref_data.prod_rslt;
	*V_REF = rpt_ref_data.v_ref;
	*I_REF = rpt_ref_data.i_ref;
	*P_REF = rpt_ref_data.p_ref;
	*E_REF = rpt_ref_data.e_ref;
	
	xSemaphoreGive(get_ele_busy);
}




 OPERATE_RET save_prod_test_data(INT_T state)
{
	OPERATE_RET op_ret;
    cJSON *root = NULL;
	UCHAR *buf = NULL;
    PR_DEBUG("prod test has last for %d mS",D_TIME1_CAL_TIME);
    root = cJSON_CreateObject();
    if(NULL == root) {
		PR_ERR("cJSON_CreateObject error");
		return OPRT_CJSON_GET_ERR;
	}
    
    cJSON_AddNumberToObject(root, "prod_rslt", state);
    buf = cJSON_PrintUnformatted(root);
    if(NULL == buf) {
        PR_ERR("buf is NULL");
        cJSON_Delete(root);
        return OPRT_COM_ERROR;
    }    
    cJSON_Delete(root);
    
	op_ret = flash_self_if_write(PROD_RSLT_KEY,buf);
	if(OPRT_OK != op_ret) {
		PR_DEBUG("msf_get_single err:%02x",op_ret);
        Free(buf);
		return op_ret;
	}
    
    Free(buf);
    return OPRT_OK;    
}

 OPERATE_RET get_prod_test_data(INT_T *state)
{
	OPERATE_RET op_ret;
    cJSON *root = NULL, *json = NULL;
	UCHAR *buf;

    buf = (UCHAR *)Malloc(256);
	if(NULL == buf) {
		PR_ERR("malloc error");
		return OPRT_MALLOC_FAILED;
	}
    UINT_T buf_len;
	op_ret = kvs_read(PROD_RSLT_KEY, &buf, &buf_len);
	if(OPRT_OK != op_ret) {
		PR_DEBUG("msf_get_single err:%02x",op_ret);
        Free(buf);
		return op_ret;
	}

	root = cJSON_Parse(buf);
	if(NULL == root) {
		PR_ERR("cjson parse");
        goto JSON_PARSE_ERR;
	}

    json = cJSON_GetObjectItem(root,"prod_rslt");
    if(NULL == json) {
        PR_ERR("cjson get ");
        goto JSON_PARSE_ERR;
	}

    *state = json->valueint;
    cJSON_Delete(root);
    Free(buf);
    return OPRT_OK;

JSON_PARSE_ERR:
    cJSON_Delete(root);
    Free(buf);
    return OPRT_COM_ERROR;
}


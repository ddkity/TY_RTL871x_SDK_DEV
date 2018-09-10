#ifndef __IR_REMOTE_H__
#define __IR_REMOTE_H__

#include "ir_proc.h"

#define IR_GPIO_OUT_MUX 	//PERIPHS_IO_MUX_MTMS_U //MTCK PIN ACT AS I2S CLK OUT
#define IR_GPIO_OUT_NUM 	//14
#define IR_GPIO_OUT_FUNC  	//FUNC_GPIO14


#define IR_NEC_TX_IO_MUX 		IR_GPIO_OUT_MUX
#define IR_NEC_TX_GPIO_NUM 		IR_GPIO_OUT_NUM

#define         IR_STUDY_FEQ                38000                   //学习红外遥控接收头频率

#define IR_TX_OUTPUT_LOW(ir_out_gpio_num)  \
    //gpio_output_conf(0, 1<<ir_out_gpio_num, 1<<ir_out_gpio_num, 0)
#define IR_TX_OUTPUT_HIGH(ir_out_gpio_num) \
	//gpio_output_conf(1<<ir_out_gpio_num, 0, 1<<ir_out_gpio_num, 0)

#define IR_TX_SET_INACTIVE(io_num)   IR_TX_OUTPUT_LOW(io_num)

#define TIMER_INTERVAL_US  63
#define I2C_BASE                0x60000D00
#define I2S_BCK_DIV_NUM 		0x0000003F
#define I2S_BCK_DIV_NUM_S 		22
#define I2S_CLKM_DIV_NUM 		0x0000003F
#define I2S_CLKM_DIV_NUM_S 		16
#define I2S_BITS_MOD 			0x0000000F
#define I2S_BITS_MOD_S 			12
#define I2SCONF					(DR_REG_I2S_BASE + 0x0008)
#define DR_REG_I2S_BASE 		(0x60000e00)

#define U32 uint32_t
#define BYTE uint8_t
#define i2c_bbpll         						0x67
#define i2c_bbpll_en_audio_clock_out        	4
#define i2c_bbpll_en_audio_clock_out_msb    	7
#define i2c_bbpll_en_audio_clock_out_lsb    	7
#define i2c_bbpll_hostid       					4



#define         MAX_IR_OUT              (1024 * 2)              //unsigned short
#define         MAX_KEY_COUNT           3

#define         MODE_IRSEND                 0x01
#define         MODE_IRSTUDY                0x02
#define         MODE_STUDYEXIT              0x03
#define         MODE_STUDYKEY               0x04

#define         IR_APP_CMD              0x01
#define         IR_STUDY_DATA           0x02
#define         IR_PROC_DATA            0x03

//extern uint32_t timer_cnt;

typedef struct {
    uint16_t retnum;                 //脉冲个数数组，元素个数
    uint8_t frameCount;
    uint32_t feq;
    double pulse;
    uint16_t *ir_out;
} IR_KEY_PARAM_S;

typedef struct {
    THRD_HANDLE thread;
    SEM_HANDLE ir_send_finish_sem;
    //
    uint8_t send_finish;
    uint8_t send_mode;
    //
	MSG_QUE_HANDLE msg_que;
    //
    TIMER_ID ir_remote_send_timer;
    IR_KEY_PARAM_S key;
    IR_STA ir_status;
    IR_BUF_S ir_tx_buf;
    IR_BUF_S ir_rx_buf;
} IR_PARAM_S;
extern IR_PARAM_S ir_param;

enum
{
    IR_STATUS_NONE = 0,
    IR_STATUS_SENDING,
    IR_STATUS_SUCCESS,
    IR_STATUS_FAILED,    
};

enum
{
    IR_SENDMODE_KUKONG = 0,
    IR_SENDMODE_STUDY,
};

OPERATE_RET ir_remote_init(void);

OPERATE_RET ir_dp_handle(CHAR_T *buf);

void ir_demo_test(void);

void ir_prod_test_send(BYTE value);

void ir_remote_send_timer_cb(UINT timerID,PVOID pTimerArg);

VOID ir_remote_data_start(VOID);

VOID ir_remote_data_stop(VOID);

#endif


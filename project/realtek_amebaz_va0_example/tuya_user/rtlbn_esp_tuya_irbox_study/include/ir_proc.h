/***********************************************************
*  File: ir_proc.h 
*  Author: hjh
*  Date: 20180614
***********************************************************/
#ifndef _IR_PROC_TEST_H
#define _IR_PROC_TEST_H


typedef enum
{
	IR_STANDBY = 0,
	IR_MASTER,
	IR_PCBA,
}IR_STA;//红外测试状态，待机，主机，从机

typedef struct{
    UCHAR ir_buf[10];
    UINT  ir_len;
}IR_BUF_S;//红外发射接收buf结构体


/****************************************************************************
功能描述: 
输入参数: 无 
输出参数: 无  
返 回 值: 
备	  注: 
****************************************************************************/
VOID prod_test_init(VOID);
IR_STA get_ir_test_status(VOID);
VOID set_ir_test_status(IR_STA status);
VOID ir_send_mac(uint8_t *ir_buf, uint16_t ir_len);

#endif


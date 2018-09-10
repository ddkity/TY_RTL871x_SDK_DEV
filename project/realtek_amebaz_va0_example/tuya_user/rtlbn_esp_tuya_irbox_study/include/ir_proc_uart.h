/***********************************************************
*  File: ir_proc_uart.h 
*  Author: anby
*  Date: 20151218
***********************************************************/
#ifndef _IR_PROC_UART_H
#define _IR_PROC_UART_H

extern TIMER_ID button_timeout_dir;
extern uint8 button_flag;

VOID button_cb(void);
VOID button_timeout_func(VOID);
OPERATE_RET ir_modu_chip_test(VOID);
VOID ir_recv_mac_func(uint8 *ir_buf, uint16 ir_len);
VOID ir_send_suc_func(uint8 send_status);


#endif

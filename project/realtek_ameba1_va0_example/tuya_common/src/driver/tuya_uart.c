/***********************************************************
*  File: tuya_uart.c
*  Author: nzy
*  Date: 20171106
***********************************************************/
#define __TUYA_UART_GLOBALS
#include "tuya_uart.h"
#include "objects.h"
#include "serial_api.h"
#include "uni_log.h"


/***********************************************************
*************************micro define***********************
***********************************************************/
#define UART_TX PA_23 //UART0 TX
#define UART_RX PA_18 //UART0 RX 

typedef struct {
    serial_t sobj;

    BYTE_T buf[256];
    USHORT in;
    USHORT out;
}TUYA_UART_S;

/***********************************************************
*************************variable define********************
***********************************************************/
STATIC TUYA_UART_S ty_uart[TY_UART_NUM];

/***********************************************************
*************************function define********************
***********************************************************/
static void __uart_irq(uint32_t id, SerialIrq event);

/***********************************************************
*  Function: ty_uart_init 
*  Input: port badu bits parity stop
*  Output: none
*  Return: OPERATE_RET
***********************************************************/
OPERATE_RET ty_uart_init(IN CONST TY_UART_PORT_E port,IN CONST TY_UART_BAUD_E badu,\
                               IN CONST TY_DATA_BIT_E bits,IN CONST TY_PARITY_E parity,\
                               IN CONST TY_STOPBITS_E stop)
{
    if(port >= TY_UART_NUM) {
        return OPRT_INVALID_PARM;
    }

    serial_init(&ty_uart[port].sobj,UART_TX,UART_RX);
    serial_baud(&ty_uart[port].sobj,badu);

    int data_bit = 0;
    if(bits == TYWL_5B) {
        data_bit = 5;
    }else if(bits == TYWL_6B) {
        data_bit = 6;
    }else if(bits == TYWL_7B) {
        data_bit = 7;
    }else {
        data_bit = 8;
    }
    serial_format(&ty_uart[port].sobj,data_bit,parity,stop);
    
    serial_irq_handler(&ty_uart[port].sobj, __uart_irq, (uint32_t)&ty_uart[port].sobj);
    serial_irq_set(&ty_uart[port].sobj, RxIrq, 1);

    ty_uart[port].in = 0;
    ty_uart[port].out = 0;

    return OPRT_OK;
}

/***********************************************************
*  Function: ty_uart_send_data 
*  Input: port data len
*  Output: none
*  Return: none
***********************************************************/
VOID ty_uart_send_data(IN CONST TY_UART_PORT_E port,IN CONST BYTE_T *data,IN CONST UINT len)
{
    if(port >= TY_UART_NUM) {
        return;
    }

    UINT i = 0;
    for(i = 0;i < len;i++) {
       serial_putc(&ty_uart[port].sobj, *(data+i));
    }
}

STATIC UINT __ty_uart_read_data_size(IN CONST TY_UART_PORT_E port)
{
    UINT remain_buf_size = 0;

    if(ty_uart[port].in >= ty_uart[port].out) {
        remain_buf_size = ty_uart[port].in-ty_uart[port].out;
    }else {
        remain_buf_size = ty_uart[port].in + SIZEOF(ty_uart[port].buf) - ty_uart[port].out;
    }

    return remain_buf_size;
}


static void __uart_irq(uint32_t id, SerialIrq event)
{
    serial_t *sobj = (void*)id;
    int rc = 0;
    
    int i = 0;
    for(i = 0;i < TY_UART_NUM;i++) {
        if(&ty_uart[i].sobj == sobj) {
            break;
        }
    }
    
    if(event == RxIrq) {
        rc = serial_getc(sobj);
        //PR_NOTICE("rc = %d", rc);
        if(__ty_uart_read_data_size(i) < (SIZEOF(ty_uart[i].buf)-1)) {
            ty_uart[i].buf[ty_uart[i].in++] = rc;
            if(ty_uart[i].in >= SIZEOF(ty_uart[i].buf)){
                ty_uart[i].in = 0;
            }
        }
    }
}


/***********************************************************
*  Function: ty_uart_send_data 
*  Input: len->data buf len
*  Output: buf->read data buf
*  Return: actual read data size
***********************************************************/
UINT ty_uart_read_data(IN CONST TY_UART_PORT_E port,OUT BYTE_T *buf,IN CONST UINT len)
{
     if(NULL == buf || 0 == len) {
        return 0;
    }

    UINT actual_size = 0;
    UINT cur_num = __ty_uart_read_data_size(port);

    if(len > cur_num) {
        actual_size = cur_num;
    }else {
        actual_size = len;
    }
    //PR_NOTICE("uart_num = %d", cur_num);
    UINT i = 0;
    for(i = 0;i < actual_size;i++) {
        *buf++ = ty_uart[port].buf[ty_uart[port].out++];
        if(ty_uart[port].out >= SIZEOF(ty_uart[port].buf)) {
            ty_uart[port].out = 0;
        }
    }

    return actual_size;
}




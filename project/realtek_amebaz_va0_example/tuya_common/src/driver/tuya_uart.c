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
    UINT_T buf_len;
    BYTE_T *buf;
    USHORT_T in;
    USHORT_T out;
}TUYA_UART_S;

/***********************************************************
*************************variable define********************
***********************************************************/
STATIC TUYA_UART_S ty_uart[TY_UART_NUM];

/***********************************************************
*************************function define********************
***********************************************************/
STATIC VOID __uart_irq(UINT_T id, SerialIrq event);

/***********************************************************
*  Function: ty_uart_init 
*  Input: port badu bits parity stop
*  Output: none
*  Return: OPERATE_RET
***********************************************************/
OPERATE_RET ty_uart_init(IN CONST TY_UART_PORT_E port,IN CONST TY_UART_BAUD_E badu,\
                               IN CONST TY_DATA_BIT_E bits,IN CONST TY_PARITY_E parity,\
                               IN CONST TY_STOPBITS_E stop,IN CONST UINT_T bufsz)
{
    if((port >= TY_UART_NUM) || (bufsz == 0)) {
        return OPRT_INVALID_PARM;
    }
    
    if(ty_uart[port].buf == NULL) {
        memset(&ty_uart[port], 0, sizeof(TUYA_UART_S));
        ty_uart[port].buf = Malloc(bufsz);
        if(ty_uart[port].buf == NULL) {
            return OPRT_MALLOC_FAILED;
        }
        ty_uart[port].buf_len = bufsz;
        PR_DEBUG("uart buf size : %d",bufsz);
    }else {
        return OPRT_COM_ERROR;
    }
    
    serial_init(&ty_uart[port].sobj,UART_TX,UART_RX);
    serial_baud(&ty_uart[port].sobj,badu);

    INT_T data_bit = 0;
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
*  Function: ty_uart_free 
*  Input:free uart
*  Output: none
*  Return: OPERATE_RET
***********************************************************/
OPERATE_RET ty_uart_free(IN CONST TY_UART_PORT_E port)

{
    if(port >= TY_UART_NUM) {
       return OPRT_INVALID_PARM;
    }
    serial_free(&ty_uart[port].sobj);
    if(ty_uart[port].buf != NULL) {
        Free(ty_uart[port].buf);
        ty_uart[port].buf = NULL;
    }
    ty_uart[port].buf_len = 0;

   return OPRT_OK;
}

/***********************************************************
*  Function: ty_uart_send_data 
*  Input: port data len
*  Output: none
*  Return: none
***********************************************************/
VOID ty_uart_send_data(IN CONST TY_UART_PORT_E port,IN CONST BYTE_T *data,IN CONST UINT_T len)
{
    if(port >= TY_UART_NUM) {
        return;
    }

    UINT_T i = 0;
    for(i = 0;i < len;i++) {
       serial_putc(&ty_uart[port].sobj, *(data+i));
    }
}

STATIC UINT __ty_uart_read_data_size(IN CONST TY_UART_PORT_E port)
{
    UINT_T remain_buf_size = 0;

    if(ty_uart[port].in >= ty_uart[port].out) {
        remain_buf_size = ty_uart[port].in-ty_uart[port].out;
    }else {
        remain_buf_size = ty_uart[port].in + ty_uart[port].buf_len - ty_uart[port].out;
    }

    return remain_buf_size;
}


STATIC VOID __uart_irq(UINT_T id, SerialIrq event)
{
    serial_t *sobj = (void*)id;
    INT_T rc = 0;
    
    INT_T i = 0;
    for(i = 0;i < TY_UART_NUM;i++) {
        if(&ty_uart[i].sobj == sobj) {
            break;
        }
    }
    
    if(event == RxIrq) {
        rc = serial_getc(sobj);
        //PR_NOTICE("rc = %d", rc);
        if(__ty_uart_read_data_size(i) < ty_uart[i].buf_len - 1) {
            ty_uart[i].buf[ty_uart[i].in++] = rc;
            if(ty_uart[i].in >= ty_uart[i].buf_len){
                ty_uart[i].in = 0;
            }
        }else {
            //PR_ERR("uart fifo is overflow! buf_zize:%d  data_get:%d",ty_uart[i].buf_len,__ty_uart_read_data_size(i));
        }
    }
}


/***********************************************************
*  Function: ty_uart_send_data 
*  Input: len->data buf len
*  Output: buf->read data buf
*  Return: actual read data size
***********************************************************/
UINT_T ty_uart_read_data(IN CONST TY_UART_PORT_E port,OUT BYTE_T *buf,IN CONST UINT_T len)
{
     if(NULL == buf || 0 == len) {
        return 0;
    }

    UINT_T actual_size = 0;
    UINT_T cur_num = __ty_uart_read_data_size(port);
    if(cur_num > ty_uart[port].buf_len - 1) {
        PR_DEBUG("uart fifo is full! buf_zize:%d  len:%d",cur_num,len);
    }

    if(len > cur_num) {
        actual_size = cur_num;
    }else {
        actual_size = len;
    }
    //PR_NOTICE("uart_num = %d", cur_num);
    UINT_T i = 0;
    for(i = 0;i < actual_size;i++) {
        *buf++ = ty_uart[port].buf[ty_uart[port].out++];
        if(ty_uart[port].out >= ty_uart[port].buf_len) {
            ty_uart[port].out = 0;
        }
    }

    return actual_size;
}




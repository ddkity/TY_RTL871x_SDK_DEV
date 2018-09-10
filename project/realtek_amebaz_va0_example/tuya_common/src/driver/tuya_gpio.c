/***********************************************************
*  File: tuya_gpio.c
*  Author: nzy
*  Date: 20171106
***********************************************************/
#define __TUYA_GPIO_GLOBALS
#include "tuya_gpio.h"
#include "gpio_api.h"
#include "gpio_irq_api.h"

#include "mem_pool.h"
#include "uni_log.h"

/***********************************************************
*************************micro define***********************
***********************************************************/
typedef struct {
    PinName pf_port; // platform port
    gpio_t *pf_obj;
}TY_GPIO_MAP_S;

/***********************************************************
*************************variable define********************
***********************************************************/
TY_GPIO_MAP_S gpio_map_list[] = {
    {PA_0,NULL},
    {PA_1,NULL}, 
    {PA_2,NULL}, 
    {PA_3,NULL}, 
    {PA_4,NULL}, 
    {PA_5,NULL}, 
    {PA_6,NULL}, 
    {PA_7,NULL}, 
    {PA_8,NULL}, 
    {PA_9,NULL}, 
    {PA_10,NULL},
    {PA_11,NULL},
    {PA_12,NULL},
    {PA_13,NULL},
    {PA_14,NULL},
    {PA_15,NULL},
    {PA_16,NULL},
    {PA_17,NULL},
    {PA_18,NULL},
    {PA_19,NULL},
    {PA_20,NULL},
    {PA_21,NULL},
    {PA_22,NULL},
    {PA_23,NULL},
    {PA_24,NULL},
    {PA_25,NULL},
    {PA_26,NULL},
    {PA_27,NULL},
    {PA_28,NULL},
    {PA_29,NULL},
    {PA_30,NULL},
    {PA_31,NULL},
    {PB_0,NULL},
    {PB_1,NULL},
    {PB_2,NULL},
    {PB_3,NULL},
    {PB_4,NULL},
    {PB_5,NULL},
    {PB_6,NULL},
    {PB_7,NULL},
    {PB_8,NULL},
};

/***********************************************************
*************************function define********************
***********************************************************/
/***********************************************************
*  Function: tuya_gpio_inout_set
*  Input: port
*         in->TRUE:in
*             FALSE:out
*  Output: none
*  Return: OPERATE_RET
***********************************************************/
OPERATE_RET tuya_gpio_inout_set(IN CONST TY_GPIO_PORT_E port,IN CONST BOOL_T in)
{
    if(NULL == gpio_map_list[port].pf_obj) {
        gpio_map_list[port].pf_obj = Malloc(SIZEOF(gpio_t));
        if(NULL == gpio_map_list[port].pf_obj) {
            return OPRT_MALLOC_FAILED;
        }
    }
    if(TRUE == in) {
        gpio_init(gpio_map_list[port].pf_obj, gpio_map_list[port].pf_port);
        gpio_dir(gpio_map_list[port].pf_obj, PIN_INPUT);     // Direction: Input
        gpio_mode(gpio_map_list[port].pf_obj, PullUp);
    }else {
        gpio_init(gpio_map_list[port].pf_obj, gpio_map_list[port].pf_port);
        gpio_dir(gpio_map_list[port].pf_obj, PIN_OUTPUT);
        gpio_mode(gpio_map_list[port].pf_obj, PullNone);//PullUp
    }

    return OPRT_OK;
}

/***********************************************************
*  Function: tuya_gpio_inout_set
*  Input: port
*         in->TRUE:in
*             FALSE:out
*  Output: none
*  Return: OPERATE_RET
***********************************************************/
INT_T tuya_gpio_read(IN CONST TY_GPIO_PORT_E port)
{
    if(NULL == gpio_map_list[port].pf_obj) {
        return -1;
    }

    return gpio_read(gpio_map_list[port].pf_obj);
}

/***********************************************************
*  Function: tuya_gpio_write
*  Input: port
*         high->TRUE:high
*               FALSE:low
*  Output: none
*  Return: OPERATE_RET
***********************************************************/
OPERATE_RET tuya_gpio_write(IN CONST TY_GPIO_PORT_E port,IN CONST BOOL_T high)
{  
    if(NULL == gpio_map_list[port].pf_obj) {
        PR_ERR("tuya_gpio_write err");
        return OPRT_COM_ERROR;
    }

    gpio_write(gpio_map_list[port].pf_obj,high);

    return OPRT_OK;
}

OPERATE_RET tuya_gpio_irq_init(IN CONST TY_GPIO_PORT_E port,IN CONST gpio_irq_handler gpio_irq_handler_cb, UINT_T id)
{  
    if(NULL == gpio_map_list[port].pf_obj) {
        gpio_map_list[port].pf_obj = Malloc(SIZEOF(gpio_t));
        if(NULL == gpio_map_list[port].pf_obj) {
            return OPRT_MALLOC_FAILED;
        }
    }
    gpio_irq_init(gpio_map_list[port].pf_obj, gpio_map_list[port].pf_port, gpio_irq_handler_cb, (uint32_t)(&id));
    gpio_irq_set(gpio_map_list[port].pf_obj, IRQ_FALL, 1);   // Falling Edge Trigger
    gpio_irq_enable(gpio_map_list[port].pf_obj);
    return OPRT_OK;
}






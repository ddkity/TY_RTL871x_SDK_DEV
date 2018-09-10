/***********************************************************
*  File: PL7211.h
*  Author: wym
*  Date: 20180503
***********************************************************/
#ifndef  __PL7211_H__
#define  __PL7211_H__

#include "tuya_cloud_error_code.h"
#include "tuya_cloud_types.h"

#ifdef __cplusplus
extern "C" {
#endif




typedef enum{
    PL_INIT_NORMAL,
    PL_INIT_PRODTEST
}PL_INIT_MODE;

OPERATE_RET pl7211_init(PL_INIT_MODE pl_init_mode);
VOID get_ele_par(OUT UINT *P,OUT UINT *V,OUT UINT *I);
VOID get_ele(OUT UINT *add_energy);
VOID uart_ctrl_relay(UCHAR relay);
VOID set_upgrading_state(BOOL state);



#endif













































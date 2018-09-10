/***********************************************************
*  File: app_dltj.h
*  Author: litao
*  Date: 170704
***********************************************************/
#ifndef  __APP_DLTJ_H__
#define  __APP_DLTJ_H__

	#include "pl7211.h"
#ifdef __cplusplus
extern "C" {
#endif
#ifdef _APP_DLTJ_GLOBAL
    #define _APP_DLTJ_EXT
#else
    #define _APP_DLTJ_EXT extern
#endif




/***********************************************************
*************************micro define***********************
***********************************************************/
#define _APP_DLTJ_DEBUG 0

#define DP_ELE  3//增加电量上报
#define DP_CURRENT  4//电流上报
#define DP_POWER  5//功率上报
#define DP_VOLTAGE  6//电压上报

/***********************************************************
*************************variable define********************
***********************************************************/
typedef enum{
    ELE_NOT_ACTIVE,//上电后未配网激活
    ELE_NORMAL,//已配网成功且当前mqtt连接成功
    ELE_UNCONNECT,//已配网成功当当前mqtt连接失败
    ELE_SYS_OTA//系统正在进行在线升级
}ELE_THREAD_STATE;

typedef enum{
    APP_DLTJ_NORMAL,
    APP_DLTJ_PRODTEST
}APP_DLTJ_MODE;

/***********************************************************
*************************function define********************
***********************************************************/
_APP_DLTJ_EXT \
OPERATE_RET app_dltj_init(APP_DLTJ_MODE mode);

_APP_DLTJ_EXT \
VOID wf_nw_stat_inform(GW_WIFI_NW_STAT_E stat);

_APP_DLTJ_EXT \
VOID set_ele_thread_state(ELE_THREAD_STATE instate);



#ifdef __cplusplus
}
#endif
#endif

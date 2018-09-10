/***********************************************************
*  File: app_dltj.h
*  Author: litao
*  Date: 170704
***********************************************************/
#ifndef  __APP_DLTJ_H__
#define  __APP_DLTJ_H__

	#include "hlw8012.h"
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
#define _APP_DLTJ_DEBUG 1

#define DP_ELE  3//增加电量上报
#define DP_CURRENT  4//电流上报
#define DP_POWER  5//功率上报
#define DP_VOLTAGE  6//电压上报
//以下可选择上报或不上报
#define DP_PTRSLT  7//产测结果，0为失败，1为成功，2为未产测
#define DP_VREF  8//电压系数
#define DP_IREF  9//电流系数
#define DP_PREF  10//功率系数
#define DP_EREF  11//电量系数

/***********************************************************
*************************variable define********************
***********************************************************/
typedef enum{
    ELE_NOT_ACTIVE,//上电后未配网激活
    ELE_NORMAL,//已配网成功且当前mqtt连接成功
    ELE_UNCONNECT,//已配网成功当当前mqtt连接失败
    ELE_SYS_OTA//系统正在进行在线升级
}ELE_THREAD_STATE;


/***********************************************************
*************************function define********************
***********************************************************/
_APP_DLTJ_EXT \
OPERATE_RET app_dltj_init(DLTJ_CONFIG *dltj);

_APP_DLTJ_EXT \
VOID wf_nw_stat_inform(GW_WIFI_NW_STAT_E stat);

_APP_DLTJ_EXT \
VOID set_ele_thread_state(ELE_THREAD_STATE instate);

_APP_DLTJ_EXT \
OPERATE_RET dltj_config_init(int mode);



#ifdef __cplusplus
}
#endif
#endif

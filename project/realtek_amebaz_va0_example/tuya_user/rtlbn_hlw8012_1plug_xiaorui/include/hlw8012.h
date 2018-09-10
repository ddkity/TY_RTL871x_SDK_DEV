#ifndef _HLW8012_H
#define _HLW8012_H


#include "cJSON.h"
#include "tuya_cloud_types.h"
#include "tuya_cloud_error_code.h"
#include "flash_self.h"
#ifdef __cplusplus
	extern "C" {
#endif



enum gpio_level{
	GPIO_LOW = 0,
	GPIO_HIGH,
};


typedef struct{
    UINT_T v_ref;
    UINT_T i_ref;
    UINT_T p_ref;
    UINT_T e_ref;
    UINT_T prod_rslt;
}RPT_REF;

#ifndef __IO_CONFIG__
#define __IO_CONFIG__
typedef enum 
{
	IO_DRIVE_LEVEL_HIGH,		// 高电平有效
	IO_DRIVE_LEVEL_LOW,			// 低电平有效
	IO_DRIVE_LEVEL_NOT_EXIST	// 该IO不存在
}IO_DRIVE_TYPE;

typedef struct
{
	IO_DRIVE_TYPE type;	// 有效电平类型
	unsigned char pin;	// 引脚号
}IO_CONFIG;
#endif

typedef struct{
	unsigned char epin;
	unsigned char ivpin;
	IO_CONFIG	  ivcpin;
	unsigned int  v_ref;
	unsigned int  i_ref;
	unsigned int  p_ref;
	unsigned int  e_ref;
    unsigned int  v_def;
    unsigned int  i_def;
    unsigned int  p_def;
    unsigned char edpid;
	unsigned char idpid;
	unsigned char pdpid;
	unsigned char vdpid;
	BOOL		  if_have;
	unsigned char prdpid;
	unsigned char vrefdpid;
	unsigned char irefdpid;
	unsigned char prefdpid;
    unsigned char erefdpid;
}DLTJ_CONFIG;

//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------
#define D_ERR_MODE                	0x00        //错误提示模式
#define D_NORMAL_MODE		      	0x10	    //正常工作模式
#define D_CAL_START_MODE		    0x21	    //校正模式，启动
#define D_CAL_END_MODE		        0x23	    //校正模式，完成
//--------------------------------------------------------------------------------------------
#define DEVICE_MOD "device_mod"
#define COE_SAVE_KEY "coe_save_key"
#define PROD_RSLT_KEY "prod_rslt_key"
#define SAVE_CAL_RSLT_DATA
OPERATE_RET save_prod_test_data(INT_T state);
OPERATE_RET get_prod_test_data(INT_T *state);

OPERATE_RET ele_cnt_init(INT_T mode);
VOID get_ele_par(OUT UINT_T *P,OUT UINT_T *V,OUT UINT_T *I);
VOID get_ele(OUT UINT_T *E);
VOID get_dltj_ref_data(OUT UINT_T *PROD_RSLT,OUT UINT_T *V_REF,OUT UINT_T *I_REF,OUT UINT_T *P_REF, OUT UINT_T *E_REF);







#ifdef __cplusplus
}
#endif
#endif



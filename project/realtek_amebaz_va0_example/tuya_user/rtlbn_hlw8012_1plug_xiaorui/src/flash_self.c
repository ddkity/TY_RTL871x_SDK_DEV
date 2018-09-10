/***********************************************************
*  File: tuya_ws_db.c
*  Author: wym
*  Date: 20180413
***********************************************************/
#define __TUYA_WS_DB_GLOBALS
#include "flash_self.h"
#include "kv_storge.h"
#include "cJSON.h"
#include "uni_log.h"

/***********************************************************
*************************micro define***********************
***********************************************************/
#define USER_PARAM_KEY "user_param_key"




/***********************************************************
*  Function: flash_self_if_write
*  Input: ai
*  Output: none
*  Return: OPERATE_RET
***********************************************************/
OPERATE_RET flash_self_if_write(IN char* addr_str, IN char *ai)
{
    if(NULL == ai) {
        PR_ERR("input is null");
        return OPRT_INVALID_PARM;
    }

    OPERATE_RET op_ret = OPRT_OK;
    UINT_T buf_len = strlen(ai);
    PR_DEBUG("to write:%s",ai);
    PR_DEBUG("write buf lenth:%d.............................",buf_len);
	op_ret = kvs_write(addr_str, ai, buf_len);
	if(OPRT_OK == op_ret){
        PR_DEBUG("%s write success",addr_str);
	}

    return op_ret;
}

/***********************************************************
*  Function: flash_self_if_read
*  Input: none
*  Output: ai
*  Return: OPERATE_RET
***********************************************************/
OPERATE_RET flash_self_if_read(IN char* addr_str,OUT char* ai)//to do
{
    if(NULL == ai) {
        PR_ERR("input is null");
        return OPRT_INVALID_PARM;
    }

    OPERATE_RET op_ret = OPRT_OK;
    UINT_T buf_len;

	op_ret = kvs_read(addr_str,&ai,&buf_len);
	
	PR_DEBUG("read end:%s..................................",ai);
	PR_DEBUG("read buf lenth:%d.............................",buf_len);
    if(OPRT_OK == op_ret){
        PR_DEBUG("%s read success",addr_str);
    }
    return op_ret;
}





#include "tuya_device.h"
#include "adapter_platform.h"
#include "tuya_cloud_error_code.h"
#include "tuya_iot_wifi_api.h"
#include "tuya_cloud_types.h"
#include "tuya_led.h"
#include "tuya_uart.h"
#include "tuya_gpio.h"
#include "tuya_key.h"
#include "uni_time_queue.h"
#include "gw_intf.h"
#include "uni_log.h"
#include "uni_thread.h"
#include "sys_api.h"
#include "gpio_test.h"
#include "pwmout_api.h"
#include "timer_api.h"

//
#include "light_driver.h"
#include "light_handler.h"
#include "light_config.h"
#include "light_prod_test.h"

TEST_DEF test_handle;

volatile BOOL flash_scene_flag = TRUE;

STATIC OPERATE_RET get_light_test_flag(VOID)
{
	OPERATE_RET op_ret;
    UINT_T buf_len;

	UCHAR *buf;
	buf = Malloc(32);
	if(NULL == buf) {
		PR_ERR("Malloc failed");
		goto ERR_EXT;
	}

	op_ret = kvs_read(LIGHT_TEST_KEY, &buf, &buf_len);
	if(OPRT_OK != op_ret){
		PR_ERR("msf_get_single failed");
		Free(buf);
		goto ERR_EXT;
	}
	PR_DEBUG("buf:%s",buf);
	
	cJSON *root = NULL;
	root = cJSON_Parse(buf);
	if(NULL == root) {
		Free(buf);
		test_handle.test_mode = FUC_TEST1;
		return OPRT_CJSON_PARSE_ERR;
	}
	Free(buf);

	cJSON *json;
	json = cJSON_GetObjectItem(root,"test_mode");
	if(NULL == json) {
		test_handle.test_mode = FUC_TEST1;
	}else{
		test_handle.test_mode = json->valueint;
	}
	
	cJSON_Delete(root);
	return OPRT_OK;
		
ERR_EXT:	
	test_handle.test_mode = FUC_TEST1;
	return OPRT_COM_ERROR;

}

STATIC OPERATE_RET set_light_test_flag(VOID)
{
	OPERATE_RET op_ret;
    INT_T i = 0;
	CHAR_T *out = NULL;

	cJSON *root_test = NULL;
	root_test = cJSON_CreateObject();
	if(NULL == root_test) {
		PR_ERR("json creat failed");
		goto ERR_EXT;
	}
	
	cJSON_AddNumberToObject(root_test, "test_mode", test_handle.test_mode);
	
	out=cJSON_PrintUnformatted(root_test);
	cJSON_Delete(root_test);
	if(NULL == out) {
		PR_ERR("cJSON_PrintUnformatted err:");
		Free(out);
		goto ERR_EXT;
	}
	//PR_DEBUG("out[%s]", out);
	op_ret = kvs_write(LIGHT_TEST_KEY, out, strlen(out));
	if(OPRT_OK != op_ret) {
		PR_ERR("data write psm err: %d",op_ret);
		Free(out);
		goto ERR_EXT;
	}
	Free(out);
	return OPRT_OK;
ERR_EXT:	
	return OPRT_COM_ERROR;

}


STATIC OPERATE_RET get_aging_tested_time(VOID)
{
	OPERATE_RET op_ret;
    UINT_T buf_len;

	UCHAR *buf;
	buf = Malloc(64);
	if(NULL == buf) {
		PR_ERR("Malloc failed");
		goto ERR_EXT;
	}

	op_ret = kvs_read(AGING_TESTED_TIME, &buf, &buf_len);
	if(OPRT_OK != op_ret){
		PR_ERR("msf_get_single failed");
		Free(buf);
		goto ERR_EXT;
	}
	PR_DEBUG("buf:%s",buf);
	
	cJSON *root = NULL;
	root = cJSON_Parse(buf);
	if(NULL == root) {
		Free(buf);
		test_handle.aging_tested_time = 0;
		return OPRT_CJSON_PARSE_ERR;
	}
	Free(buf);

	cJSON *json;
	json = cJSON_GetObjectItem(root,"aging_tested_time");
	if(NULL == json) {
		test_handle.aging_tested_time = 0;
	}else{
		test_handle.aging_tested_time = json->valueint;
	}

    if (test_handle.aging_tested_time > AGING_TEST_TIME){
        PR_DEBUG("get aging time error > default time");
        test_handle.aging_tested_time = 0;
    }

	cJSON_Delete(root);
	return OPRT_OK;
		
ERR_EXT:
	test_handle.aging_tested_time = 0;
	return OPRT_COM_ERROR;

}

STATIC OPERATE_RET set_aging_tested_time(VOID)
{
	OPERATE_RET op_ret;
    INT_T i = 0;
	CHAR_T *out = NULL;

	cJSON *root_test = NULL;
	root_test = cJSON_CreateObject();
	if(NULL == root_test) {
		PR_ERR("json creat failed");
		goto ERR_EXT;
	}
	
	cJSON_AddNumberToObject(root_test, "aging_tested_time", test_handle.aging_tested_time);
	
	out=cJSON_PrintUnformatted(root_test);
	cJSON_Delete(root_test);
	if(NULL == out) {
		PR_ERR("cJSON_PrintUnformatted err:");
		Free(out);
		goto ERR_EXT;
	}
	PR_DEBUG("out[%s]", out);
	op_ret = kvs_write(AGING_TESTED_TIME, out, strlen(out));
	if(OPRT_OK != op_ret) {
		PR_ERR("data write psm err: %d",op_ret);
		Free(out);
		goto ERR_EXT;
	}
	Free(out);
	return OPRT_OK;
ERR_EXT:	
	return OPRT_COM_ERROR;

}

STATIC VOID aging_test_timer_cb(UINT timerID,PVOID pTimerArg)
{
	test_handle.aging_tested_time ++;
	if(OPRT_OK != set_aging_tested_time()){
		send_light_data(0x00, 0x00, 0xff, 0x00, 0x00);
		sys_stop_timer(test_handle.aging_test_timer);
	}
	
	if(test_handle.aging_tested_time >= AGING_TEST_TIME){
		sys_stop_timer(test_handle.aging_test_timer);
		send_light_data(0, 0xff, 0, 0, 0);
		test_handle.test_mode = FUC_TEST2;
		test_handle.aging_tested_time = 0;
		if(OPRT_OK != set_light_test_flag()){
			send_light_data(0x00, 0x00, 0xff, 0x00, 0x00);
		}
		
		if(OPRT_OK != set_aging_tested_time()){
			send_light_data(0x00, 0x00, 0xff, 0x00, 0x00);
		}
	}else{
        if(test_handle.aging_tested_time >= (AGING_TEST_W_TIME+AGING_TEST_C_TIME) && \
             (test_handle.aging_tested_time < AGING_TEST_TIME)) {
            send_light_data(0xff, 0xff, 0xff, 0x00, 0x00);
        } else if (test_handle.aging_tested_time >= AGING_TEST_C_TIME && \
             (test_handle.aging_tested_time < (AGING_TEST_W_TIME+AGING_TEST_C_TIME))) {
            send_light_data(0x00, 0x00, 0x00, 0x00, 0xff);
        }
        else{
            send_light_data(0x00, 0x00, 0x00, 0xff, 0x00);
        }
	}
}

STATIC VOID fuc_test_timer_cb(UINT timerID,PVOID pTimerArg)
{
	test_handle.pmd_times ++;


	if(test_handle.test_mode == FUC_TEST1){
		 switch(test_handle.pmd_times % 6){
			case 1: send_light_data(TEST_R_BRGHT, 0, 0, 0, 0); break;
			case 2: send_light_data(0, TEST_G_BRGHT, 0, 0, 0); break;
			case 3: send_light_data(0, 0, TEST_B_BRGHT, 0, 0); break;
			case 4:	send_light_data(0, 0, 0, 0xff, 0); break;
            case 5: 
			case 0:	send_light_data(0, 0, 0, 0, 0XFF); break;
			default:break;
		}
	 }
	 else{
		switch(test_handle.pmd_times % 11){
		 	case 1: send_light_data(TEST_R_BRGHT, 0, 0, 0, 0); break;
			case 2: send_light_data(0, 0, 0, 0, 0); break;
			case 3: send_light_data(0, TEST_G_BRGHT, 0, 0, 0); break;
			case 4: send_light_data(0, 0, 0, 0, 0); break;
			case 5: send_light_data(0, 0, TEST_B_BRGHT, 0, 0); break;
			case 6: send_light_data(0, 0, 0, 0, 0); break;
			case 7: send_light_data(0, 0, 0, 0xff, 0); break;	
			case 8: send_light_data(0, 0, 0, 0, 0); break;
			case 9: 
			case 10:send_light_data(0, 0, 0, 0, 0xff); break;
			case 0: send_light_data(0, 0, 0, 0, 0); break;
			default:break;
		 }
	 }
	 
	if(test_handle.pmd_times == 120){
		if(test_handle.test_mode == FUC_TEST1){
			test_handle.test_mode = AGING_TEST;
			if(OPRT_OK != set_light_test_flag()) { 
				sys_stop_timer(test_handle.fuc_test_timer);
				send_light_data(0x00, 0x00, 0x00, 0x00, 0x00);
			}else{
				SystemReset();
			}
		}
	}
}


VOID prod_test(BOOL_T flag, CHAR_T rssi)
{
    OPERATE_RET op_ret;
	flash_scene_flag = FALSE;
	PR_DEBUG("rssi:%d", rssi);
	set_reset_cnt(0);
	//prod thread create and start

	if(OPRT_OK != get_light_test_flag()){
		PR_ERR("get_light_test_flag err.......");
	}
	
	if(test_handle.test_mode == AGING_TEST){
		get_aging_tested_time();
        
		op_ret = sys_add_timer(aging_test_timer_cb,NULL,&test_handle.aging_test_timer);
	    if(OPRT_OK != op_ret) {
			send_light_data(0x00, 0x00, 0x00, 0x00, 0x00);
	        return;
	    }
	    
		if(test_handle.aging_tested_time >= (AGING_TEST_W_TIME+AGING_TEST_C_TIME) && \
             (test_handle.aging_tested_time < AGING_TEST_TIME)) {
            send_light_data(0xff, 0xff, 0xff, 0x00, 0x00);
        } else if (test_handle.aging_tested_time >= AGING_TEST_C_TIME && \
             (test_handle.aging_tested_time < (AGING_TEST_W_TIME+AGING_TEST_C_TIME))) {
            send_light_data(0x00, 0x00, 0x00, 0x00, 0xff);
        }
        else{
            send_light_data(0x00, 0x00, 0x00, 0xff, 0x00);
        }
    	
        sys_start_timer(test_handle.aging_test_timer, 60000, TIMER_CYCLE);
	} else {
		if(rssi < -60 || flag == FALSE) {
			send_light_data(TEST_R_BRGHT, 0, 0, 0, 0);
			return;
	    }
		
		op_ret = sys_add_timer(fuc_test_timer_cb,NULL,&test_handle.fuc_test_timer);
	    if(OPRT_OK != op_ret) {  
			send_light_data(0x00, 0x00, 0x00, 0x00, 0x00);
	        return;
	    }
		test_handle.pmd_times = 0;
		
		/*if(test_handle.test_mode == FUC_TEST1){
			sys_start_timer(test_handle.fuc_test_timer, 1000, TIMER_CYCLE);
		}else{
			sys_start_timer(test_handle.fuc_test_timer, 500, TIMER_CYCLE);
		}*/
		sys_start_timer(test_handle.fuc_test_timer, 1000, TIMER_CYCLE);
	}
	return;	
}


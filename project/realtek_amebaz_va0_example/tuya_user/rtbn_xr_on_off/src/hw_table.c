/***********************************************************
File: hw_table.c 
Author: qiuping.wang 
Date: 2018-05-31
Description:
    硬件结构抽象表，表达通用硬件配置。
    描述内容：
        1.wifi指示灯[数量(0~1)]：
            驱动方式：高、低驱动
        2.开关、插座控制通道(ctrl_channel)[1~n]
            继电器(relay): 高、低驱动
            按钮：高、低驱动或不存在(目前只能配置是否存在)
            LED指示灯：高、低驱动或不存在
       	3.App支持自由配置上电状态：通电、断电和断电记忆
       	4.(关于断电记忆)任意通道均可以自由选择是否断电记忆
            
***********************************************************/
#include <mem_pool.h> // 编译器非标准，在使用下面方法初始化结构体时依赖此头文件
#include "hw_table.h"
#include "uni_log.h"
#include "tuya_device.h"

// =============================== 需配置部分 ===============================
// 开关、插座通道初始化
// 该部分代码中的通道数计算方法导致该变量应慎重移动到其他位置
// 目前按钮的有效电平配置无法设定 仅可控制存在与否

#define PRESS_HOLD_TIME 3000 // 长按时间 单位ms
#define KEY_TIMER_MS      	 20    //key timer inteval
#define KEY_RST_TIME       3000            //按键重置时间 ms

#define NUM 1  //通道数

//app控制上电状态，默认下次重启生效
#define INIT_OFF  0  //上电关
#define INIT_ON   1  //上电开
#define INIT_MEM  2  //断电记忆
extern TIMER_ID save_init_timer;  //上电状态	
extern UCHAR_T init_stat;

extern OPERATE_RET dp_upload_proc(CHAR_T *jstr);
extern VOID Start_boot_up(VOID);
STATIC VOID key_process(TY_GPIO_PORT_E port,PUSH_KEY_TYPE_E type,INT_T cnt);
STATIC VOID Protest_key_process(TY_GPIO_PORT_E port,PUSH_KEY_TYPE_E type,INT_T cnt);
extern OPERATE_RET write_def_mode(UCHAR_T def_stat);
//断电记忆
extern volatile BOOL is_save_stat ;
extern TIMER_ID save_stat_timer;
STATIC BOOL_T IS_CONNECTED = TRUE; //目的：联网状态下，继电器led为灭，连上网之后wifi指示灯为灭
//wifi状态实时更新
extern GW_WIFI_NW_STAT_E wifi_stat ;

static CTRL_CHANNEL channels[] = 
{
    // channel 0
    {
        // 继电器
        .relay  = {.type = IO_DRIVE_LEVEL_HIGH, .pin = TY_GPIOA_14},
        // 按钮
        .button = {TY_GPIOA_5,TRUE,LP_INVALID,0,50,NULL},
        // 通道状态指示灯
        .led    = {.type = IO_DRIVE_LEVEL_NOT_EXIST },
        // 绑定dpid
        .dpid   = 1,
        // 绑定倒计时dpid DPID_NOT_EXIST表示不存在倒计时
        .cd_dpid = 102,
        //绑定设备上电状态dpid DPID_NOT_EXIST表示不存在默认断电模式
        .prtest_swti1_count = 0
    }
};

// 硬件配置表全局变量
HW_TABLE g_hw_table = 
{
	.mode_dpid = 101,
    // wifi状态指示灯 有效对应wifi连接 无效对应wifi未连接
    .wifi_stat_led = {.type = IO_DRIVE_LEVEL_LOW, .pin = TY_GPIOA_0},
    .rst_button = {TY_GPIOA_5,TRUE,LP_ONCE_TRIG,KEY_RST_TIME,50,key_process},
    .channels = channels
};

// ============================= 需配置部分结束 =============================

// 初始化hw_table
// 并注册所需硬件
// 返回注册成功与否
OPERATE_RET init_hw(HW_TABLE *hw_table)
{
    OPERATE_RET op_ret = OPRT_OK;
	UCHAR_T i;	
	if(hw_table->wifi_stat_led.type == IO_DRIVE_LEVEL_HIGH) {  //初始化 点亮led
	    op_ret = tuya_create_led_handle(hw_table->wifi_stat_led.pin,FALSE,&hw_table->wifi_stat_led_handle);
	}else if(hw_table->wifi_stat_led.type == IO_DRIVE_LEVEL_LOW) {
        op_ret = tuya_create_led_handle(hw_table->wifi_stat_led.pin,TRUE,&hw_table->wifi_stat_led_handle);
	}
	if( op_ret != OPRT_OK) {
		PR_ERR("tuya_create_led_handle err:%d",op_ret);
		return op_ret;
	}

	op_ret = key_init(NULL,0,KEY_TIMER_MS);
    if(op_ret != OPRT_OK) {
        PR_ERR("key_init err:%d",op_ret);
        return op_ret;
    }
    if(hw_table->rst_button.call_back != NULL) {
        op_ret = reg_proc_key(&hw_table->rst_button);//初始化reset按键
    	if( op_ret != OPRT_OK){
    		PR_ERR("reg_proc_key err:%d",op_ret);
    		return;
    	}
    }
    hw_table->channel_num = sizeof(channels) / sizeof(CTRL_CHANNEL);
    PR_DEBUG("channel num: %d", hw_table->channel_num);

	for(i = 0; i < hw_table->channel_num; i++) {
		if(hw_table->channels[i].led.type == IO_DRIVE_LEVEL_HIGH){//指示灯
	    	op_ret = tuya_create_led_handle(hw_table->channels[i].led.pin,FALSE,&hw_table->channels[i].led_handle);
    	}else if(hw_table->channels[i].led.type == IO_DRIVE_LEVEL_LOW) {
	    	op_ret = tuya_create_led_handle(hw_table->channels[i].led.pin,TRUE,&hw_table->channels[i].led_handle);
    	}
    	if( op_ret != OPRT_OK) {
    		PR_ERR("tuya_create_led_handle err:%d",op_ret);
    		return op_ret;
    	}

        if(hw_table->channels[i].button.call_back != NULL) {//按键
        	op_ret = reg_proc_key(&hw_table->channels[i].button);
        	if( op_ret != OPRT_OK) {
        		PR_ERR("reg_proc_key err:%d",op_ret);
        		return;
        	}
        }

        if(hw_table->channels[i].relay.type != IO_DRIVE_LEVEL_NOT_EXIST) {//普通输出io
        	op_ret = tuya_gpio_inout_set(hw_table->channels[i].relay.pin,FALSE);
            if(OPRT_OK != op_ret) {
                return op_ret;
            }
        }

        hw_table->channels[i].cd_sec = -1;
        if((init_stat == INIT_OFF)||(init_stat == INIT_MEM)) {
        	hw_set_channel(&g_hw_table,i,false);
		}
		else if(init_stat == INIT_ON) {
			hw_set_channel(&g_hw_table,i, true);
		}
    	
	}

	return OPRT_OK;
}

OPERATE_RET prod_test_init_hw(HW_TABLE *hw_table)
{
    OPERATE_RET op_ret = OPRT_OK;
	UCHAR_T i;	
	if(hw_table->wifi_stat_led.type == IO_DRIVE_LEVEL_HIGH) {  //初始化 点亮led
	    op_ret = tuya_create_led_handle(hw_table->wifi_stat_led.pin,TRUE,&hw_table->wifi_stat_led_handle);
	}else if(hw_table->wifi_stat_led.type == IO_DRIVE_LEVEL_LOW) {
        op_ret = tuya_create_led_handle(hw_table->wifi_stat_led.pin,FALSE,&hw_table->wifi_stat_led_handle);
	}
	if( op_ret != OPRT_OK) {
		PR_ERR("tuya_create_led_handle err:%d",op_ret);
		return op_ret;
	}

	tuya_set_led_light_type(hw_table->wifi_stat_led_handle,OL_FLASH_HIGH,250,LED_TIMER_UNINIT);//250ms flash
    
	op_ret = key_init(NULL,0,KEY_TIMER_MS);
    if(op_ret != OPRT_OK) {
        PR_ERR("key_init err:%d",op_ret);
        return op_ret;
    }

    hw_table->rst_button.call_back = Protest_key_process;
    if(hw_table->rst_button.call_back != NULL) {
        op_ret = reg_proc_key(&hw_table->rst_button);//初始化reset按键
    	if( op_ret != OPRT_OK){
    		PR_ERR("reg_proc_key err:%d",op_ret);
    		return;
    	}
    }
    hw_table->channel_num = sizeof(channels) / sizeof(CTRL_CHANNEL);
    PR_DEBUG("channel num: %d", hw_table->channel_num);
    
	for(i = 0; i < hw_table->channel_num; i++) {//-----------产测电源指示灯动作修改------------
		if(hw_table->channels[i].led.type == IO_DRIVE_LEVEL_HIGH) {//指示灯
	        op_ret = tuya_create_led_handle(hw_table->channels[i].led.pin,FALSE,&hw_table->channels[i].led_handle);
    	}else if(hw_table->channels[i].led.type == IO_DRIVE_LEVEL_LOW) {
            op_ret = tuya_create_led_handle(hw_table->channels[i].led.pin,TRUE,&hw_table->channels[i].led_handle);
    	}
    	if( op_ret != OPRT_OK) {
    		PR_ERR("tuya_create_led_handle err:%d",op_ret);
    		return op_ret;
    	}
        #if 0
        if(hw_table->channels[i].button.call_back != NULL) {//按键
        	op_ret = reg_proc_key(&hw_table->channels[i].button);
        	if( op_ret != OPRT_OK){
        		PR_ERR("reg_proc_key err:%d",op_ret);
        		return;
        	}
        }
        #endif
        if(hw_table->channels[i].relay.type != IO_DRIVE_LEVEL_NOT_EXIST) {//普通输出io
        	op_ret = tuya_gpio_inout_set(hw_table->channels[i].relay.pin,FALSE);
            if(OPRT_OK != op_ret) {
                return op_ret;
            }
        }

        hw_table->channels[i].cd_sec = -1;
        hw_table->channels[i].stat = FALSE;
    	
	}
	return OPRT_OK;
}

void sync_led_and_relay_connected(HW_TABLE *hw_table)
{
    INT_T i = 0;
    for(i = 0; i < hw_table->channel_num; i++) {
         if(hw_table->channels[i].stat) {
            switch(hw_table->channels[i].led.type) {
                case IO_DRIVE_LEVEL_HIGH:
                    // 高电平有效 置高
                    tuya_set_led_light_type(hw_table->channels[i].led_handle, OL_HIGH, 0,0);
                    break;
                case IO_DRIVE_LEVEL_LOW:
                    // 低电平有效 置低
                    tuya_set_led_light_type(hw_table->channels[i].led_handle, OL_LOW, 0,0);
                    break;
                case IO_DRIVE_LEVEL_NOT_EXIST:
                    // IO不存在 跳过
                    break;
                default:
                    break;
            }
         }
    }
}

void sync_led_and_relay_unconnected(HW_TABLE *hw_table)
{
    INT_T i = 0;
    for(i = 0; i < hw_table->channel_num; i++) {
       
        //hw_set_channel(hw_table, i, hw_table->channels[i].stat);
        switch(hw_table->channels[i].led.type) {
            case IO_DRIVE_LEVEL_HIGH:
                // 高电平有效 置高
                tuya_set_led_light_type(hw_table->channels[i].led_handle, OL_LOW, 0,0);
                break;
            case IO_DRIVE_LEVEL_LOW:
                // 低电平有效 置低
                tuya_set_led_light_type(hw_table->channels[i].led_handle, OL_HIGH, 0,0);
                break;
            case IO_DRIVE_LEVEL_NOT_EXIST:
                // IO不存在 跳过
                break;
            default:
                break;
        }
    }
}

// 控制通道
// hw_table:    硬件对象
// channel_no:  被控制的通道号 范围[1, 通道数]
// is_active:   通道是否有效 - 有效对应TURE - 对应该插座、开关通电
void hw_set_channel(HW_TABLE *hw_table, INT_T channel_no, BOOL_T is_active)
{
    if(channel_no < 0 || channel_no >= hw_table->channel_num) {
        PR_ERR("channel_no error: %d", channel_no);
    }
    if(is_active) {
        // 通道有效
        PR_DEBUG("channel: %d true", channel_no);
        hw_table->channels[channel_no].stat = true;
        // 根据继电器 有效电平定义 控制继电器
        switch(hw_table->channels[channel_no].relay.type) {
            case IO_DRIVE_LEVEL_HIGH:
                // 高电平有效 置高
                tuya_gpio_write(hw_table->channels[channel_no].relay.pin,true);
                break;
            case IO_DRIVE_LEVEL_LOW:
                // 低电平有效 置低
                tuya_gpio_write(hw_table->channels[channel_no].relay.pin,FALSE);
                break;
            case IO_DRIVE_LEVEL_NOT_EXIST:
                // IO不存在 跳过
                break;
            default:
                break;
        }

        // 根据状态指示灯 有效电平定义 控制状态指示灯
        switch(hw_table->channels[channel_no].led.type) {
            case IO_DRIVE_LEVEL_HIGH:
                // 高电平有效 置高
                tuya_set_led_light_type(hw_table->channels[channel_no].led_handle, OL_HIGH, 0,0);
                break;
            case IO_DRIVE_LEVEL_LOW:
                // 低电平有效 置低
                tuya_set_led_light_type(hw_table->channels[channel_no].led_handle, OL_LOW, 0,0);
                break;
            case IO_DRIVE_LEVEL_NOT_EXIST:
                // IO不存在 跳过
                break;
            default:
                break;
        }
    }
    else 
	{
        // 通道无效
        PR_DEBUG("channel: %d false", channel_no);
        hw_table->channels[channel_no].stat = false;
        // 根据继电器 无效电平定义 控制继电器
        switch(hw_table->channels[channel_no].relay.type) {
            case IO_DRIVE_LEVEL_HIGH:
                // 高电平有效 置低
                tuya_gpio_write(hw_table->channels[channel_no].relay.pin,FALSE);
                break;
            case IO_DRIVE_LEVEL_LOW:
                // 低电平有效 置高
                tuya_gpio_write(hw_table->channels[channel_no].relay.pin,TRUE);
                break;
            case IO_DRIVE_LEVEL_NOT_EXIST:
                // IO不存在 跳过
                break;
            default:
                break;
        }

        // 根据状态指示灯 无效电平定义 控制继电器
        switch(hw_table->channels[channel_no].led.type) {
            case IO_DRIVE_LEVEL_HIGH:
                // 高电平有效 置低
                tuya_set_led_light_type(hw_table->channels[channel_no].led_handle, OL_LOW, 0,0);
                break;
            case IO_DRIVE_LEVEL_LOW:
                // 低电平有效 置高
                tuya_set_led_light_type(hw_table->channels[channel_no].led_handle, OL_HIGH, 0,0);
                break;
            case IO_DRIVE_LEVEL_NOT_EXIST:
                // IO不存在 跳过
                break;
            default:
                break;
        }
    }

	INT_T is_any_active = false;
	INT_T i;
	for(i=0; i<hw_table->channel_num;++i) {
		is_any_active = is_any_active || hw_table->channels[i].stat; 
	}
	if(IS_CONNECTED) {
		if(is_any_active) {
	        //sync_led_and_relay_unconnected(&g_hw_table);
	        if(hw_table->wifi_stat_led.type == IO_DRIVE_LEVEL_LOW) {
	            tuya_set_led_light_type(hw_table->wifi_stat_led_handle,OL_LOW,0,0);
	        }else {
	            tuya_set_led_light_type(hw_table->wifi_stat_led_handle,OL_HIGH,0,0);
	        }
		}else{
			if(hw_table->wifi_stat_led.type == IO_DRIVE_LEVEL_LOW) {
	            tuya_set_led_light_type(hw_table->wifi_stat_led_handle,OL_HIGH,0,0);
	        }else {
	            tuya_set_led_light_type(hw_table->wifi_stat_led_handle,OL_LOW,0,0);
	        }
		}
	}
}

void hw_trig_channel(HW_TABLE *hw_table, INT_T channel_no)
{
    // 测试下标是否越界 下标范围[0, 通道数-1]
    if(channel_no < 0 || channel_no >= hw_table->channel_num) {
        PR_ERR("channel_no error: %d", channel_no);
    }

    // 按目前通道状态反置通道状态
    if(hw_table->channels[channel_no].stat == TRUE) {
        hw_set_channel(hw_table, channel_no, FALSE);
    }else {
        hw_set_channel(hw_table, channel_no, TRUE);
    }
}

INT_T hw_set_channel_by_dpid(HW_TABLE *hw_table, INT_T dpid, BOOL_T is_active)
{
    INT_T i;
    // 遍历通道列表
    for(i=0; i<hw_table->channel_num; ++i) {
        // 判断dpid是否一致
        if(dpid == hw_table->channels[i].dpid) {
            hw_set_channel(hw_table, i, is_active);
            return i;
        }
    }
    // 至此未返回说明未找到通道
    return -1;
}

INT_T hw_find_channel_by_cd_dpid(HW_TABLE *hw_table, INT_T dpid)
{
    INT_T i;
    // 遍历通道列表
    for(i=0; i<hw_table->channel_num; ++i) {
        // 判断dpid是否一致
        if(dpid == hw_table->channels[i].cd_dpid) {
            return i;
        }
    }
    return -1;
}
INT_T hw_find_channel_by_button(HW_TABLE *hw_table, TY_GPIO_PORT_E gpio_num)
{
    INT_T i;
    // 遍历通道列表
    for(i=0; i<hw_table->channel_num; ++i) {
        // 判断gpio_num是否一致
        if(gpio_num == hw_table->channels[i].button.port) {
            return i;
        }
    }
    return -1;
}
void hw_set_wifi_led_stat(HW_TABLE *hw_table, GW_WIFI_NW_STAT_E wifi_stat)
{
    GW_WIFI_NW_STAT_E wf_stat;
    wf_stat = wifi_stat;
    PR_NOTICE("wifi status is :%d",wifi_stat);
	INT_T is_any_active = false;
	INT_T i;

	OPERATE_RET op_ret;
	for(i=0; i<hw_table->channel_num;++i) {
		is_any_active = is_any_active || hw_table->channels[i].stat; 
	}
		
    switch(wf_stat) {
		
            case STAT_UNPROVISION: {
                IS_CONNECTED = FALSE;
                //sync_led_and_relay_unconnected(&g_hw_table);
                tuya_set_led_light_type(hw_table->wifi_stat_led_handle,OL_FLASH_LOW,250,LED_TIMER_UNINIT);//250ms flash
                 break;
            }
            case STAT_AP_STA_UNCFG: {
                IS_CONNECTED = FALSE;
                //sync_led_and_relay_unconnected(&g_hw_table);
                tuya_set_led_light_type(hw_table->wifi_stat_led_handle,OL_FLASH_LOW,1500,LED_TIMER_UNINIT);//1500ms flash
                 break;
            }
            case STAT_AP_STA_DISC:
            case STAT_STA_DISC: {  //turn on
                //IS_CONNECTED = FALSE;
                IS_CONNECTED = TRUE;
				if(is_any_active) {
	                //sync_led_and_relay_unconnected(&g_hw_table);
	                if(hw_table->wifi_stat_led.type == IO_DRIVE_LEVEL_LOW) {
	                    tuya_set_led_light_type(hw_table->wifi_stat_led_handle,OL_LOW,0,0);
	                }else{
	                    tuya_set_led_light_type(hw_table->wifi_stat_led_handle,OL_HIGH,0,0);
	                }
				}else{
					if(hw_table->wifi_stat_led.type == IO_DRIVE_LEVEL_LOW) {
	                    tuya_set_led_light_type(hw_table->wifi_stat_led_handle,OL_HIGH,0,0);
	                }else{
	                    tuya_set_led_light_type(hw_table->wifi_stat_led_handle,OL_LOW,0,0);
	                }
				}
                break;
            }
            case STAT_AP_STA_CONN:
            case STAT_STA_CONN: { //turn off
                IS_CONNECTED = TRUE;
				TY_OBJ_DP_S mode_dp;
                mode_dp.dpid = DP_POWER_MODE;
                mode_dp.type = PROP_ENUM;
                mode_dp.time_stamp = 0;
				mode_dp.value.dp_enum = init_stat;
			    op_ret = dev_report_dp_json_async(get_gw_cntl()->gw_if.id,&mode_dp,1);
				
			    if(OPRT_OK != op_ret) {
			        PR_ERR("sf_obj_dp_report op_ret:%d",op_ret);
			        return op_ret;
			    }
                if(is_any_active) {
	                if(hw_table->wifi_stat_led.type == IO_DRIVE_LEVEL_LOW) {
	                    tuya_set_led_light_type(hw_table->wifi_stat_led_handle,OL_LOW,0,0);
	                }else {
	                    tuya_set_led_light_type(hw_table->wifi_stat_led_handle,OL_HIGH,0,0);
	                }
				}else { 
					if(hw_table->wifi_stat_led.type == IO_DRIVE_LEVEL_LOW) {
	                    tuya_set_led_light_type(hw_table->wifi_stat_led_handle,OL_HIGH,0,0);
	                }else {
	                    tuya_set_led_light_type(hw_table->wifi_stat_led_handle,OL_LOW,0,0);
	                }
				}
               break;
            }
            case STAT_CLOUD_CONN: 
            case STAT_AP_CLOUD_CONN: {  //turn off
                IS_CONNECTED = TRUE;
				TY_OBJ_DP_S mode_dp;
				mode_dp.dpid = DP_POWER_MODE;
                mode_dp.type = PROP_ENUM;
                mode_dp.time_stamp = 0;
				mode_dp.value.dp_enum = init_stat;
			    op_ret = dev_report_dp_json_async(get_gw_cntl()->gw_if.id,&mode_dp,1);
			    if(OPRT_OK != op_ret) {
			        PR_ERR("upload_all_dp_stat op_ret:%d",op_ret);
				}
                if(is_any_active) {
	                if(hw_table->wifi_stat_led.type == IO_DRIVE_LEVEL_LOW) {
	                    tuya_set_led_light_type(hw_table->wifi_stat_led_handle,OL_LOW,0,0);
	                }else {
	                    tuya_set_led_light_type(hw_table->wifi_stat_led_handle,OL_HIGH,0,0);
	                }
				}else{
					if(hw_table->wifi_stat_led.type == IO_DRIVE_LEVEL_LOW) {
	                    tuya_set_led_light_type(hw_table->wifi_stat_led_handle,OL_HIGH,0,0);
	                }else {
	                    tuya_set_led_light_type(hw_table->wifi_stat_led_handle,OL_LOW,0,0);
	                }
				}
                Start_boot_up();
                break;
            }
            default: {
                if(hw_table->wifi_stat_led.type == IO_DRIVE_LEVEL_LOW) {
                    tuya_set_led_light_type(hw_table->wifi_stat_led_handle,OL_HIGH,0,0);
                }else {
                    tuya_set_led_light_type(hw_table->wifi_stat_led_handle,OL_LOW,0,0);
                }
               break;
            }
        }
        return;
}

STATIC VOID key_process(TY_GPIO_PORT_E port,PUSH_KEY_TYPE_E type,INT_T cnt) {
    char buff[30];
    PR_DEBUG("port: %d",port);
    PR_DEBUG("type: %d",type);
    PR_DEBUG("cnt: %d",cnt);
	INT_T  i;
	OPERATE_RET op_ret;

        if(LONG_KEY == type) {
			if(g_hw_table.rst_button.port== port) {
				write_def_mode(INIT_OFF);
	            gw_wifi_reset(WRT_AUTO);	 
			}
        }else if(NORMAL_KEY == type) {       
	        INT_T ch_idx = hw_find_channel_by_button(&g_hw_table,port);	
	        if(ch_idx < 0) {
				PR_NOTICE("port = %d",port);
				return;
			} 
			
			hw_trig_channel(&g_hw_table, ch_idx);
            if( g_hw_table.channels[ch_idx].cd_sec != -1) {
                 g_hw_table.channels[i].cd_sec = -1; // 关闭通道定时
            }
            upload_channel_stat(ch_idx);
			
			if(is_save_stat) {
		        sys_start_timer(save_stat_timer, 5000, TIMER_ONCE);    // 启动断电记忆存储定时器
		    }
		}
}

STATIC VOID Protest_key_process(TY_GPIO_PORT_E port,PUSH_KEY_TYPE_E type,INT_T cnt)
{
    PR_DEBUG("port: %d",port);
    PR_DEBUG("type: %d",type);
    PR_DEBUG("cnt: %d",cnt);
    if(g_hw_table.rst_button.port== port){
        if(LONG_KEY == type) {
            SystemReset();
        }else if(NORMAL_KEY == type) {
            if(IsThisSysTimerRun(g_hw_table.switch_timer)==false) {  
              sys_start_timer(g_hw_table.switch_timer,500,TIMER_ONCE);
            }   
        }
     }
}




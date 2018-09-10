/***********************************************************
File: hw_table.c 
Author: 徐靖航 JingHang Xu
Date: 2017-06-01
Description:
    硬件结构抽象表，表达通用硬件配置。
    描述内容：
        1.wifi指示灯[数量(0~1)]：
            驱动方式：高、低驱动
        2.开关、插座控制通道(ctrl_channel)[1~n]
            继电器(relay): 高、低驱动
            按钮：高、低驱动或不存在(目前只能配置是否存在)
            LED指示灯：高、低驱动或不存在
            
***********************************************************/
#include <mem_pool.h> // 编译器非标准，在使用下面方法初始化结构体时依赖此头文件
#include "cJSON.h"
#include "hw_table.h"
#include "uni_log.h"

#define KEY_TIMER_MS      	 20    //key timer inteval

HW_TABLE g_hw_table;

extern void update_all_stat(VOID);


/***********************************************************
*  Function:  IO配置信息打印
*  Input:
*  Output:
*  Return: 
***********************************************************/
void io_config_debug(IO_DRIVE_TYPE io_type,TY_GPIO_PORT_E io_pin, char *name)
{
    if((io_type == IO_DRIVE_LEVEL_NOT_EXIST) || (name == NULL))
    {
        // 错误返回
        return;
    }
    PR_NOTICE("IO - %s:\tpin-%d\ttype %s", name, io_pin,
             (io_type == IO_DRIVE_LEVEL_HIGH)?("high"):("low"));
}

/***********************************************************
*  Function: LED管脚注册
*  Input:    output    输出管脚及响应电平
*  Output:
*  Return: 
***********************************************************/
static OPERATE_RET led_pin_reg(OUTPUT_PIN *output)
{
    OPERATE_RET op_ret;

    if(output == NULL)
    {
        PR_ERR("NULL pointer");
        return OPRT_INVALID_PARM;
    }

    if(!IS_IO_TYPE_ALL_PERIPH(output->io_cfg.type)){
        PR_ERR("IO type not define");
        return OPRT_INVALID_PARM;
    }
    
    // 高电平有效 或者低电平有效时注册该输出管脚
    if(output->io_cfg.type != IO_DRIVE_LEVEL_NOT_EXIST){
       op_ret = tuya_create_led_handle(output->io_cfg.pin,\
                                      IO_ACTIVE_TYPE(output->io_cfg.type),\
                                      &(output->io_handle));
       if(OPRT_OK != op_ret) {
           return op_ret;
       }

    }

    return OPRT_OK;
}

/***********************************************************
*  Function: 输出管脚初始化
*  Input:    output    输出管脚及响应电平
*  Output:
*  Return: 
***********************************************************/
static OPERATE_RET out_pin_init(OUTPUT_PIN *output)
{
    OPERATE_RET op_ret;

    if(output == NULL)
    {
        PR_ERR("NULL pointer");
        return OPRT_INVALID_PARM;
    }

    if(!IS_IO_TYPE_ALL_PERIPH(output->io_cfg.type)){
        PR_ERR("IO type not define");
        return OPRT_INVALID_PARM;
    }

    //普通输出io
    if(output->io_cfg.type != IO_DRIVE_LEVEL_NOT_EXIST){
        op_ret = tuya_gpio_inout_set(output->io_cfg.pin,FALSE);
        if(OPRT_OK != op_ret) {
            return op_ret;
        }
    }

    return OPRT_OK;
}

/***********************************************************
*  Function:  按键引脚初始化
*  Input:     pbutton 按键引脚信息
              hw_key_process  按键处理回调函数
*  Output:
*  Return: 
***********************************************************/
static OPERATE_RET button_pin_init(BUTTON_PIN *pbutton,
                                       void (*hw_key_process)(TY_GPIO_PORT_E , PUSH_KEY_TYPE_E, INT_T))
{
    OPERATE_RET ret;
    
    if((NULL == pbutton) || (NULL == hw_key_process))
    {
        PR_ERR("NULL pointer");
        return OPRT_INVALID_PARM;
    }

    if(IO_DRIVE_LEVEL_NOT_EXIST == pbutton->type)
    {
        return OPRT_OK;
    }
    
    pbutton->key_cfg.low_level_detect = !IO_ACTIVE_TYPE(pbutton->type);

    pbutton->key_cfg.call_back = hw_key_process;
    pbutton->key_cfg.long_key_time = g_hw_table.press_hold_time * 1000u;
    pbutton->key_cfg.lp_tp  = LP_ONCE_TRIG;
    pbutton->key_cfg.seq_key_detect_time = 400;

    ret = reg_proc_key(&(pbutton->key_cfg));

    return ret;
}


/***********************************************************
*  Function: 设置输出IO的状态
*  Input:    output     输出管脚及响应电平
             is_active: 状态
*  Output:
*  Return: 
***********************************************************/
void out_pin_set_stat(IO_CONFIG io_cfg, BOOL is_active)
{
    if(!IS_IO_TYPE_ALL_PERIPH(io_cfg.type)){
        PR_ERR("IO type not define");
        return;
    }

    // IO不存在则跳过
    if(io_cfg.type != IO_DRIVE_LEVEL_NOT_EXIST){
        tuya_gpio_write(io_cfg.pin, DRV_STATE_TYPE(is_active, io_cfg.type));
    }
}


/***********************************************************
*  Function: 设置LED的亮灭
*  Input:    output       led引脚及有效电平
             is_active:    亮灭状态
*  Output:
*  Return: 
***********************************************************/
void led_pin_set_stat(OUTPUT_PIN *output, BOOL is_active)
{
    // 检查LED_HANDLE防止崩溃
    if(output == NULL && output->io_handle != NULL)
    {
        PR_ERR("NULL pointer");
        return;
    }

    if(!IS_IO_TYPE_ALL_PERIPH(output->io_cfg.type)){
        PR_ERR("IO type not define");
        return OPRT_INVALID_PARM;
    }

    // IO不存在则跳过
    if(output->io_cfg.type != IO_DRIVE_LEVEL_NOT_EXIST){
        tuya_set_led_light_type(output->io_handle,\
                                DRV_STATE_TYPE(is_active, output->io_cfg.type),\
                                0, 0);
    }
}

/***********************************************************
*  Function: 设置io闪烁
*  Input:    output      被控管教及有效电平
             flash_type: 闪烁相位
             half_period 半个闪烁周期的时间(单位: ms)
*  Output:
*  Return: 
***********************************************************/
void led_pin_set_flash(OUTPUT_PIN *output, LED_LT_E flash_type, unsigned short half_period)
{   
    if(output == NULL)
    {
        PR_ERR("NULL pointer");
        return;
    }
    // IO不存在时跳过
    if(output->io_cfg.type != IO_DRIVE_LEVEL_NOT_EXIST){
        tuya_set_led_light_type(output->io_handle, flash_type, half_period,LED_TIMER_UNINIT);
    }
}

/***********************************************************
*  Function: 获得引脚驱动类型
*  Input:    json    json格式的参数
*  Output:   lv_type 驱动类型
*  Return:   执行结果
***********************************************************/
static INT_T get_io_driver_level_type(cJSON *json, IO_DRIVE_TYPE *lv_type)
{
    cJSON *js_item;
    cJSON *js_value;

    if((NULL == json)  || (NULL == lv_type)){
        PR_ERR("PARAM ERR !");
        return OPRT_INVALID_PARM;
    }

    js_item = cJSON_GetObjectItem(json, "lv");
    if(NULL == js_item){
        *lv_type = IO_DRIVE_LEVEL_NOT_EXIST;
        return OPRT_OK;
    }

    js_value = cJSON_GetObjectItem(js_item, "value");
    if(NULL == js_value){
        *lv_type = IO_DRIVE_LEVEL_NOT_EXIST;
        return OPRT_OK;
    }
    
    if(js_value->type == cJSON_True){
        *lv_type = IO_DRIVE_LEVEL_HIGH;
        return OPRT_OK;
    }
    else if(js_value->type == cJSON_False){
        *lv_type = IO_DRIVE_LEVEL_LOW;
        return OPRT_OK;
    }
    else{
        PR_ERR("lv type error");
        return OPRT_COM_ERROR;
    }
}

/***********************************************************
*  Function: 获得引脚号
*  Input:    json    json格式的参数
*  Output:   
*  Return:    引脚号
             -1    获取失败
***********************************************************/
static OPERATE_RET get_io_pin(cJSON *json, TY_GPIO_PORT_E *io_pin)
{
    cJSON *js_item;
    cJSON *js_value;

    if((NULL == json)  || (NULL == io_pin)){
        PR_ERR("PARAM ERR !");
        return OPRT_INVALID_PARM;
    }

    js_item = cJSON_GetObjectItem(json, "pin");
    if(js_item == NULL){
        PR_ERR("pin JSON is null");
        return OPRT_COM_ERROR;
    }

    js_value = cJSON_GetObjectItem(js_item, "value");
    if(js_value == NULL){
        PR_ERR("pin JSON value is null");
        return OPRT_COM_ERROR;
    }

    if((js_value->valueint < TY_GPIOA_0) ||( js_value->valueint > TY_GPIOB_8))
    {
        PR_ERR(" get pin err !");
        return OPRT_COM_ERROR;
    }

    *io_pin =  (UCHAR_T)(js_value->valueint);

    return OPRT_OK;

}

/***********************************************************
*  Function: 根据数据初始化输出管脚
*  Input:    json    json格式的通道参数
*  Output:   io_cfg      待配置通道指针
*  Return: 
***********************************************************/
static OPERATE_RET io_cfg_set_by_json(IO_DRIVE_TYPE *ptype,
                                           TY_GPIO_PORT_E *ppin, cJSON *json)
{
     OPERATE_RET ret = OPRT_OK;

    if(NULL == json){
        PR_ERR("PARAM ERR !");
        return OPRT_INVALID_PARM;
    }

    // 配置管脚类型lv
    ret = get_io_driver_level_type(json, ptype);
    if( ret != OPRT_OK ){
        return ret;
    }

    // 管脚不存在时无需关系管脚号 直接返回配置成功
    if( IO_DRIVE_LEVEL_NOT_EXIST == *ptype ){
        return OPRT_OK;
    }

    // 配置管脚号pin
    ret =  get_io_pin(json, ppin);
    if( ret != OPRT_OK ){
        return ret;
    }
    
    return OPRT_OK;
}

/***********************************************************
*  Function: 根据数据配置单个通道参数
*  Input:    js_ch    json格式的通道参数
*  Output:   pch      待配置通道指针
*  Return: 
***********************************************************/
static OPERATE_RET config_ch_by_param(OUT CTRL_CHANNEL *pch, IN cJSON *js_ch)
{
    OPERATE_RET ret;
    cJSON *js_item = NULL, *js_value = NULL;
    
    if((NULL == pch) || (NULL == js_ch)){
        PR_ERR("PARAM ERR !");
        return OPRT_INVALID_PARM;
    }
    
    //配置继电器
    js_item = cJSON_GetObjectItem(js_ch, "rl");
    ret = io_cfg_set_by_json(&(pch->relay.io_cfg.type),\
                             &(pch->relay.io_cfg.pin), js_item);
    if(ret != OPRT_OK){
        PR_ERR("relay cfg fail !");
        return OPRT_COM_ERROR;
    }
    
    // 配置通道的按钮
    js_item = cJSON_GetObjectItem(js_ch, "bt");
    ret = io_cfg_set_by_json(&(pch->button.type),\
                             &(pch->button.key_cfg.port), js_item);
    if(ret != OPRT_OK){
        PR_ERR("button cfg fail !");
        return OPRT_COM_ERROR;
    }

    // 配置通道的状态指示灯
    js_item = cJSON_GetObjectItem(js_ch, "led");
    ret = io_cfg_set_by_json(&(pch->led.io_cfg.type),\
                             &(pch->led.io_cfg.pin), js_item);
    if( ret != OPRT_OK){
        PR_ERR("led cfg fail !");
        return OPRT_COM_ERROR;
    }

    // 配置通道的默认状态
    pch->default_stat = FALSE;                //初始状态同一设置为关
    js_item = cJSON_GetObjectItem(js_ch, "df");
    if(js_item != NULL)
    {
        js_value = cJSON_GetObjectItem(js_item, "value");
        if(js_value != NULL){
            pch->default_stat = (BOOL)js_value->valueint;
        }
    }

    // 配置通道的dpid, 一定会存在的
    js_item = cJSON_GetObjectItem(js_ch, "dpid");
    if(js_item == NULL){
        PR_ERR("dpid is null");
        return OPRT_COM_ERROR;
    }
    js_value = cJSON_GetObjectItem(js_item, "value");
    if(js_value == NULL){
        PR_ERR("dpid value is null");
        return OPRT_COM_ERROR;
    }
    
    if(js_value->type != cJSON_Number){
        PR_ERR("dpid value type error");
        return OPRT_COM_ERROR;
    }
    
    pch->dpid = js_value->valueint;

    // 配置通道的倒计时dpid，可缺省
    pch->cd_dpid = DPID_NOT_EXIST;
    js_item = cJSON_GetObjectItem(js_ch, "cddpid");
    if(js_item != NULL)
    {
        js_value = cJSON_GetObjectItem(js_item, "value");
        if(js_value != NULL)
        {
            if(js_value->type == cJSON_Number){
                pch->cd_dpid = js_value->valueint;
            }else{
                PR_ERR("cddpid value type error");
                return OPRT_COM_ERROR;
            }

        }
    }

    return OPRT_OK;
}


/***********************************************************
*  Function: 使用JSON配置通道列表
*  Input:    js_sw 待解析的JSON格式数据
*  Output:   hw_table 解析后的数据存放表
*  Return: 
***********************************************************/
OPERATE_RET channels_set_by_json(OUT HW_TABLE *hw_table, IN cJSON *js_sw)
{
    OPERATE_RET ret = OPRT_OK;
    INT_T ch_num = 0, i;
    cJSON *js_item = NULL, *js_channels = NULL;
    
    if((NULL == hw_table) || (NULL == js_sw)){
        PR_ERR("PARAM ERR !");
        return OPRT_INVALID_PARM;
    }

    // 获取通道类
    js_channels = cJSON_GetObjectItem(js_sw, "ch");
    if(js_channels == NULL)
    {
        PR_ERR("ch is null");
        return OPRT_MALLOC_FAILED;
    }

    //计算总通道数
    ch_num = cJSON_GetArraySize(js_channels);
    hw_table->channel_num = ch_num;
    PR_DEBUG("channel number: %d", ch_num);
    
    // 申请存放通道配置的空间
    CTRL_CHANNEL *channels = (CTRL_CHANNEL *)Malloc(sizeof(CTRL_CHANNEL) * ch_num);
    if(channels == NULL)
    {
        PR_ERR("Malloc channels error");
        return OPRT_MALLOC_FAILED;
    }

    //配置通道列表
    for(i=0; i<ch_num; ++i)
    {
        // 读取第i个元素
        PR_DEBUG("ch_num[%d] initializing...", i);
        js_item = cJSON_GetArrayItem(js_channels, i);
        if(js_item == NULL){
            PR_ERR("ch[%d] is null", i);
            return OPRT_COM_ERROR;
        }

        //配置通道参数
        ret = config_ch_by_param( &channels[i], js_item);
        if(OPRT_OK != ret){
          PR_ERR("ch[%d] config failed! ", i);
          return OPRT_COM_ERROR;
        }
    }

    hw_table->channels = channels;

    return OPRT_OK;
}

/***********************************************************
*  Function: 使用JSON配置总控通道
*  Input:    js_sw 待解析的JSON格式数据
*  Output:   hw_table 解析后的数据存放表
*  Return: 
***********************************************************/
OPERATE_RET master_channel_set_by_json(OUT HW_TABLE *hw_table, IN cJSON *js_sw)
{
    OPERATE_RET ret;
    cJSON *js_tch = NULL, *js_item = NULL;

    if((NULL == hw_table) || (NULL == js_sw)){
        PR_ERR("PARAM ERR !");
        return OPRT_INVALID_PARM;
    }

    js_tch = cJSON_GetObjectItem(js_sw, "tch");

	//总控按键
	js_item = cJSON_GetObjectItem(js_tch, "tbt");
	if(NULL == js_item){
		 PR_DEBUG("tbt is null");
         hw_table->power_button.type = IO_DRIVE_LEVEL_NOT_EXIST;
	}else{
		io_cfg_set_by_json(&(hw_table->power_button.type),\
		                   &(hw_table->power_button.key_cfg.port), js_item);
	}

    //总继电器指示灯
    js_item = cJSON_GetObjectItem(js_tch, "tled");
	if(NULL == js_item){
		PR_DEBUG("tled is null");
		hw_table->power_stat_led.io_cfg.type = IO_DRIVE_LEVEL_NOT_EXIST;
	}else{
		ret = io_cfg_set_by_json(&(hw_table->power_stat_led.io_cfg.type),\
		                         &(hw_table->power_stat_led.io_cfg.pin), js_item);
		if(ret != OPRT_OK){
		    PR_DEBUG("power led cfg fail !");
		    return OPRT_COM_ERROR;
	    }
	}

	return OPRT_OK;
}

/***********************************************************
*  Function: wifi灯，上电类型等其他配置
*  Input:    js_sw 待解析的JSON格式数据
*  Output:   hw_table 解析后的数据存放表
*  Return: 
***********************************************************/
OPERATE_RET other_set_by_json(OUT HW_TABLE *hw_table, IN cJSON *js_sw)
{
    OPERATE_RET ret;
    cJSON *js_net = NULL;
    cJSON *js_item = NULL, *js_value;

    if((NULL == hw_table) || (NULL == js_sw)){
        PR_ERR("PARAM ERR !");
        return OPRT_INVALID_PARM;
    }

    js_net = cJSON_GetObjectItem(js_sw, "net");

    //读取上电wifi指示灯的状态配置：低功耗or上电快闪
    js_item = cJSON_GetObjectItem(js_net,"owm");
    if(NULL == js_item){
        PR_DEBUG("js_owm is null");
        hw_table->wf_mode = LOW_POWER;
    }else{
        js_value = cJSON_GetObjectItem(js_item,"value");
        if(NULL == js_value){
            PR_DEBUG("ERROR!"); 
            return OPRT_COM_ERROR;
        }
        
        if(js_value->type == cJSON_True)
            hw_table->wf_mode = LOW_POWER;
        else
            hw_table->wf_mode = FLASH_ON;
    }
    
    // 读取wifi状态指示灯管脚配置
    js_item = cJSON_GetObjectItem(js_net, "wfst");
    if(NULL == js_item){
       PR_ERR("wfst is null");
       return OPRT_COM_ERROR;
    }else{
       ret = io_cfg_set_by_json(&(hw_table->wifi_stat_led.wfl.io_cfg.type),\
                                &(hw_table->wifi_stat_led.wfl.io_cfg.pin), js_item);
       if(ret != OPRT_OK){
		    PR_DEBUG("wifi led cfg fail !");
		    return OPRT_COM_ERROR;
	    }
    }

   //wifi未连接下的状态
   js_item = cJSON_GetObjectItem(js_net, "netn");

   if(NULL == js_item){
        hw_table->wifi_stat_led.wfl_not_connect = WFL_OFF;
    }else{
        js_value = cJSON_GetObjectItem(js_item, "value");
        if(NULL != js_value )
        {
            if(js_value->type == cJSON_True){
                hw_table->wifi_stat_led.wfl_not_connect = WFL_ON;
            }else if(js_value->type == cJSON_False){
                hw_table->wifi_stat_led.wfl_not_connect = WFL_OFF;
            }else if(js_value->type == cJSON_NULL){
                hw_table->wifi_stat_led.wfl_not_connect = WFL_DIR_RL;
            }
        }
    }

    //wifi连接下的状态
   js_item = cJSON_GetObjectItem(js_net, "nety");

   if(NULL == js_item){
        hw_table->wifi_stat_led.wfl_connected = WFL_OFF;
    }else{
        js_value = cJSON_GetObjectItem(js_item, "value");
        if(NULL != js_value )
        {
            if(js_value->type == cJSON_True){
                hw_table->wifi_stat_led.wfl_connected = WFL_ON;
            }else if(js_value->type == cJSON_False){
                hw_table->wifi_stat_led.wfl_connected = WFL_OFF;
            }else if(js_value->type == cJSON_NULL){
                hw_table->wifi_stat_led.wfl_connected = WFL_DIR_RL;
            }
        }
    }
    
    // 读取长按时间阈值
   js_item = cJSON_GetObjectItem(js_net, "rsthold");
   if(NULL == js_item){
       PR_ERR("rsthold is null");
       return OPRT_COM_ERROR;
   }
   js_value = cJSON_GetObjectItem(js_item, "value");

   if(NULL == js_value){
        PR_DEBUG("ERROR!");
        return OPRT_COM_ERROR;
    }else{
       if(js_value->type != cJSON_Number){
           PR_ERR("rsthold value type error");
           return OPRT_COM_ERROR;
       }else{
            hw_table->press_hold_time = js_value->valueint;
       }
    }


    return OPRT_OK;
}



/***********************************************************
*  Function: 使用JSON配置hw_table信息
*  Input:    root 待解析的JSON格式数据
*  Output:   hw_table 解析后的数据存放表
*  Return: 
***********************************************************/
OPERATE_RET hw_set_by_json(OUT HW_TABLE *hw_table, IN cJSON *root)
{
    OPERATE_RET ret = OPRT_OK;
    cJSON *js_switch;

    if((NULL == hw_table) || (NULL == root)){
        PR_ERR("PARAM ERR !");
        return OPRT_INVALID_PARM;
    }

    // 读取开关大类
    js_switch = cJSON_GetObjectItem(root, "sw");
    if(js_switch == NULL)
    {
        PR_ERR("sw node is null");
        return OPRT_MALLOC_FAILED;
    }

    //解析通道列表配置
    ret = channels_set_by_json(hw_table, js_switch);
    if(OPRT_OK != ret){
        PR_ERR("channels config failed!");
        return ret;
    }

    //解析主控通道配置
    ret = master_channel_set_by_json(hw_table, js_switch);
    if(OPRT_OK != ret){
        PR_ERR("the master channel config failed!");
        return ret;
    }

    //解析配网相关的配置
    ret = other_set_by_json(hw_table, js_switch);
    if(OPRT_OK != ret){
        PR_ERR("the master channel config failed!");
        return ret;
    }

    return OPRT_OK;
}

/***********************************************************
*  Function: 硬件初始化,并进行相应的注册
*  Input:    hw_table 硬件抽象表
             hw_key_process  按键回调
*  Output:
*  Return:   操作结果是否成功
***********************************************************/
OPERATE_RET hw_init(HW_TABLE *hw_table,
    void (*hw_key_process)(TY_GPIO_PORT_E , PUSH_KEY_TYPE_E, INT_T)
    )
{
    int i;
    OPERATE_RET op_ret = OPRT_OK;// 返回状态

    if(hw_table == NULL || hw_key_process == NULL)
    {
        PR_ERR("NULL pointer");
        return OPRT_INVALID_PARM;
    }

    PR_DEBUG("initialize hardware...");

	op_ret = key_init(NULL,0,KEY_TIMER_MS);
    if(op_ret != OPRT_OK){
        PR_ERR("key_init err:%d",op_ret);
        return op_ret;
    }

    // io_cfg debug用字符串
    char name_buff[20];
    // 初始化控制通道
    for(i=0; i<hw_table->channel_num; ++i)
    {
        hw_table->channels[i].stat = FALSE; // 初始状态
        
        // 1.初始化通道中的继电器
        op_ret = out_pin_init(&(hw_table->channels[i].relay));
        if(op_ret != OPRT_OK){
            PR_ERR("ch%d relay init failed!", i);
            return op_ret;
        }
        sprintf(name_buff, "relay[%d]", i);
        io_config_debug(hw_table->channels[i].relay.io_cfg.type,\
                        hw_table->channels[i].relay.io_cfg.pin, name_buff);

        // 2.初始化通道中的按键
        op_ret = button_pin_init(&(hw_table->channels[i].button), hw_key_process);
        if(op_ret != OPRT_OK){
            PR_ERR("ch%d button init failed!", i);
            return op_ret;
        }
        sprintf(name_buff, "button[%d]", i);
        io_config_debug(hw_table->channels[i].button.type,\
                        hw_table->channels[i].button.key_cfg.port, name_buff);

        // 3.初始化通道中的状态指示灯
        op_ret = led_pin_reg(&(hw_table->channels[i].led));
        if(op_ret != OPRT_OK){
            PR_ERR("ch%d relay init failed!", i);
            return op_ret;
        }
        sprintf(name_buff, "led[%d]", i);
        io_config_debug(hw_table->channels[i].led.io_cfg.type,\
                        hw_table->channels[i].led.io_cfg.pin,name_buff);

        // 4.初始倒计时为停止
        hw_table->channels[i].cd_sec = -1;

        PR_NOTICE("CH[%d],    DPID[%d]", i, hw_table->channels[i].dpid);
        PR_NOTICE("CH[%d],    CDDPID[%d]", i, hw_table->channels[i].cd_dpid);
    }

    
    // 初始化wifi状态指示灯
    op_ret = led_pin_reg(&(hw_table->wifi_stat_led.wfl));
    if(op_ret != OPRT_OK){
        PR_ERR("wifi led init failed!");
        return op_ret;
    }
    io_config_debug(hw_table->wifi_stat_led.wfl.io_cfg.type ,\
                    hw_table->wifi_stat_led.wfl.io_cfg.pin, "wifi_stat");

    // 初始化电源状态指示灯
    op_ret = led_pin_reg(&(hw_table->power_stat_led));
    if(op_ret != OPRT_OK){
        PR_ERR("power led init failed!");
        return op_ret;
    }
    io_config_debug(hw_table->power_stat_led.io_cfg.type,\
                    hw_table->power_stat_led.io_cfg.pin,"power_stat");

    // 3.初始化总电源按键
    op_ret = button_pin_init(&(hw_table->power_button), hw_key_process);
    if(op_ret != OPRT_OK){
        PR_ERR("power button init failed!", i);
        return op_ret;
    }
    io_config_debug(hw_table->power_button.type,\
                    hw_table->power_button.key_cfg.port, "power_button");

    return op_ret;
}

/***********************************************************
*  Function: 根据wifi指示灯状态,设置指示灯状态
*  Input:    wfl_stat  wifi指示灯状态
*  Output:
*  Return:
***********************************************************/
void hw_set_wfl_state(WFL_STAT      wfl_state)
{
    BOOL is_any_active = FALSE;
    INT_T i;

    if(WFL_OFF == wfl_state)
    {
        led_pin_set_stat(&(g_hw_table.wifi_stat_led.wfl ), FALSE);
    }
    else if(WFL_ON == wfl_state)
    {
        led_pin_set_stat(&(g_hw_table.wifi_stat_led.wfl), TRUE);
    }
    else if(WFL_DIR_RL == wfl_state)
    {
        for(i=0; i<g_hw_table.channel_num; ++i)
        {
            is_any_active = is_any_active || g_hw_table.channels[i].stat;
        }
        led_pin_set_stat(&(g_hw_table.wifi_stat_led.wfl), is_any_active);
    }
}


/***********************************************************
*  Function: 根据wifi指示灯状态,设置指示灯状态
*  Input:    wfl_stat  wifi指示灯状态
*  Output:
*  Return:
***********************************************************/
void hw_set_power_led_state(VOID)
{
    BOOL is_any_active = FALSE;
    INT_T i;

    for(i=0; i<g_hw_table.channel_num; ++i)
    {
        is_any_active = is_any_active || g_hw_table.channels[i].stat;
    }

    //无电源指示灯
    if(IO_DRIVE_LEVEL_NOT_EXIST == g_hw_table.power_stat_led.io_cfg.type)
    {
        if((NOT_CONNECT == g_hw_table.wifi_conn_stat) &&\
           (WFL_DIR_RL == g_hw_table.wifi_stat_led.wfl_not_connect))
        {
            led_pin_set_stat(&(g_hw_table.wifi_stat_led.wfl), is_any_active);
        }
        else if( (CONNECTED == g_hw_table.wifi_conn_stat) &&\
                 (WFL_DIR_RL == g_hw_table.wifi_stat_led.wfl_connected))
        {
            led_pin_set_stat(&(g_hw_table.wifi_stat_led.wfl), is_any_active);
        }
        else
        {
            ;
        }
    }
    else
    {
        led_pin_set_stat(&(g_hw_table.power_stat_led), is_any_active);
    }
}

/***********************************************************
*  Function: 控制通道
*  Input:    hw_table:    硬件对象
             channel_no:  被控制的通道号 范围[0, 通道数-1]
             is_active:   通道是否有效 
*  Output:
*  Return:
***********************************************************/
void hw_set_channel(HW_TABLE *hw_table, int channel_no, BOOL is_active)
{

    if(hw_table == NULL)
    {
        PR_ERR("NULL pointer");
    }
    // 测试下标是否越界 下标范围[0, 通道数-1]
    if(channel_no < 0 || channel_no >= hw_table->channel_num)
    {
        PR_ERR("channel_no error: %d", channel_no);
        return;
    }
    // 控制继电器和状态指示灯
    out_pin_set_stat(hw_table->channels[channel_no].relay.io_cfg, is_active);
    led_pin_set_stat(&(hw_table->channels[channel_no].led), is_active);
    hw_table->channels[channel_no].stat = is_active;

    //停止倒计时
    hw_table->channels[channel_no].cd_sec = -1;
	
	// 控制电源指示灯
    hw_set_power_led_state();

}
/***********************************************************
*  Function: 翻转通道状态
*  Input:    hw_table:    硬件对象
             channel_no:  被控制的通道号 范围[0, 通道数-1]
*  Output:
*  Return:
***********************************************************/
void hw_trig_channel(HW_TABLE *hw_table, int channel_no)
{
    if(hw_table == NULL)
    {
        PR_ERR("NULL pointer");
    }
    // 按目前通道状态反置通道状态
    if(hw_table->channels[channel_no].stat == TRUE)
    {
        hw_set_channel(hw_table, channel_no, FALSE);
    }
    else
    {
        hw_set_channel(hw_table, channel_no, TRUE);
    }
}

/***********************************************************
*  Function: 设置wifi灯状态
*  Input:    hw_table:    硬件对象
             wifi_stat:  wifi目前联网状态
*  Output:
*  Return:
***********************************************************/
void hw_set_wifi_led_stat(HW_TABLE *hw_table, GW_WIFI_NW_STAT_E wifi_stat)
{
    if(hw_table == NULL)
    {
        PR_ERR("NULL pointer");
    }
    switch(wifi_stat)
    {
    case STAT_UNPROVISION: 
        // 智能配网 快闪
        g_hw_table.wifi_conn_stat = CONNECTING;
        led_pin_set_flash(&(hw_table->wifi_stat_led.wfl), OL_FLASH_HIGH, 250);
        break;
        
    case STAT_AP_STA_UNCFG:
        // AP配网 慢闪
        g_hw_table.wifi_conn_stat = CONNECTING;
        led_pin_set_flash(&(hw_table->wifi_stat_led.wfl), OL_FLASH_HIGH, 1500);
        break;
        
    case STAT_LOW_POWER:
    case STAT_AP_STA_DISC:
    case STAT_STA_DISC:
        // 低功耗及智能配网未连接
        g_hw_table.wifi_conn_stat = NOT_CONNECT;
        hw_set_wfl_state(hw_table->wifi_stat_led.wfl_not_connect);
        break;
        
    case STAT_AP_STA_CONN:
    case STAT_STA_CONN:
        g_hw_table.wifi_conn_stat = CONNECTED;
        hw_set_wfl_state(hw_table->wifi_stat_led.wfl_connected);
        break;
        
    case STAT_CLOUD_CONN: 
    case STAT_AP_CLOUD_CONN: {
        g_hw_table.wifi_conn_stat = CONNECTED;
        hw_set_wfl_state(hw_table->wifi_stat_led.wfl_connected);
        update_all_stat();
        break;
    }
    default:{
       break;
    }
    }
}

/***********************************************************
*  Function: 通过pid设置相应通道状态
*  Input:    hw_table:    硬件对象
             dpid:        通道pid
             is_active    控制状态
*  Output:
*  Return:
***********************************************************/
int hw_set_channel_by_dpid(HW_TABLE *hw_table, int dpid, BOOL is_active)
{
    if(hw_table == NULL)
    {
        PR_ERR("NULL pointer");
        return -1;
    }
    int i;
    // 遍历通道列表
    for(i=0; i<hw_table->channel_num; ++i)
    {
        // 判断dpid是否一致
        if(dpid == hw_table->channels[i].dpid)
        {
            hw_set_channel(hw_table, i, is_active);
            return i;
        }
    }
    // 至此未返回说明未找到通道
    return -1;
}

/***********************************************************
*  Function: 通过dpid寻找对应的通道号
*  Input:    hw_table:    硬件对象
             dpid:        通道pid
*  Output:
*  Return:    对应的通道号        范围[0, 通道数-1]
             -1 为未找到通道
***********************************************************/
int hw_find_channel_by_cd_dpid(HW_TABLE *hw_table, int dpid)
{
    if(hw_table == NULL)
    {
        PR_ERR("NULL pointer");
        return -1;
    }
    int i;
    // 遍历通道列表
    for(i=0; i<hw_table->channel_num; ++i)
    {
        // 判断dpid是否一致
        if(dpid == hw_table->channels[i].cd_dpid)
        {
            return i;
        }
    }
    return -1;
}





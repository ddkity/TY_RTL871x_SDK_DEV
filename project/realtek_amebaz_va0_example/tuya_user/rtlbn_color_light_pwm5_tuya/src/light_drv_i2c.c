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

STATIC MUTEX_HANDLE mutex = NULL;


#define         DEVICE_ADDR             0xA0

#define         DRV_I2C_ERROR           0
#define         DRV_I2C_SUCCESS         1

gpio_t gpio_sda0;
gpio_t gpio_scl0;

static void light_drv_i2c_scl_set(unsigned char GPIO_NUM)
{
    gpio_write(&gpio_scl0, 1);
}
static void light_drv_i2c_scl_reset(unsigned char GPIO_NUM)
{
    gpio_write(&gpio_scl0, 0);
}

static void light_drv_i2c_sda_set(unsigned char GPIO_NUM)
{
    gpio_write(&gpio_sda0, 1);
}

static void light_drv_i2c_sda_reset(unsigned char GPIO_NUM)
{
    gpio_write(&gpio_sda0, 0);
}

#if 0
static unsigned char light_drv_i2c_scl_read(unsigned char GPIO_NUM)
{
    gpio_init(&gpio_scl0, GPIO_NUM);
    gpio_dir(&gpio_scl0, PIN_INPUT);    // Direction: Output
    gpio_mode(&gpio_scl0, PullUp);     // No pull
    
    return gpio_read(&gpio_scl0);
//    (((gpio_input_get() & BIT(GPIO_NUM)) >> GPIO_NUM) & 0x01)
}

static unsigned char light_drv_i2c_sda_read(unsigned char GPIO_NUM)
{
    gpio_init(&gpio_sda0, GPIO_NUM);
    gpio_dir(&gpio_sda0, PIN_INPUT);    // Direction: Output
    gpio_mode(&gpio_sda0, PullUp);     // No pull
    
    return gpio_read(&gpio_sda0);

//    (((gpio_input_get() & BIT(GPIO_NUM)) >> GPIO_NUM) & 0x01)
}
#endif

static void light_drv_i2c_delay(void)
{       
	volatile unsigned short j = 80; 
	while(j)
	{
		j --;
	}
}

#if 0
static unsigned char light_drv_i2c_start(unsigned char gpio_sda, unsigned char gpio_scl)
{
	light_drv_i2c_sda_set(gpio_sda);
	light_drv_i2c_scl_set(gpio_scl);

	light_drv_i2c_delay();

	if(!light_drv_i2c_sda_read(gpio_sda))
		return DRV_I2C_ERROR;                                                               

	light_drv_i2c_sda_reset(gpio_sda);
	light_drv_i2c_delay();

	if(light_drv_i2c_sda_read(gpio_sda)) 
		return DRV_I2C_ERROR;                                                               

	light_drv_i2c_sda_reset(gpio_sda);
	light_drv_i2c_delay();

	return DRV_I2C_SUCCESS;
}

static void light_drv_i2c_stop(unsigned char gpio_sda, unsigned char gpio_scl)
{
	light_drv_i2c_scl_reset(gpio_scl);
	light_drv_i2c_delay();
	
	light_drv_i2c_sda_reset(gpio_sda);
	light_drv_i2c_delay();
	
	light_drv_i2c_scl_set(gpio_scl);
	light_drv_i2c_delay();
	
	light_drv_i2c_sda_set(gpio_sda);
	light_drv_i2c_delay();
}

static void light_drv_i2c_ack(unsigned char gpio_sda, unsigned char gpio_scl)
{       
	light_drv_i2c_scl_reset(gpio_scl);
	light_drv_i2c_delay();
	
	light_drv_i2c_sda_reset(gpio_sda);
	light_drv_i2c_delay();
	
	light_drv_i2c_scl_set(gpio_scl);
	light_drv_i2c_delay();
	
	light_drv_i2c_scl_reset(gpio_scl);	
	light_drv_i2c_delay();
}

static void light_drv_i2c_noack(unsigned char gpio_sda, unsigned char gpio_scl)
{       
	light_drv_i2c_scl_reset(gpio_scl);
	light_drv_i2c_delay();
	
	light_drv_i2c_sda_set(gpio_sda);
	light_drv_i2c_delay();
	
	light_drv_i2c_scl_set(gpio_scl);
	light_drv_i2c_delay();
	
	light_drv_i2c_scl_reset(gpio_scl);
	light_drv_i2c_delay();
}

static unsigned char light_drv_i2c_waitack(unsigned char gpio_sda, unsigned char gpio_scl)
{
    light_drv_i2c_sda_set(gpio_sda);                       
    light_drv_i2c_delay();
    
	light_drv_i2c_scl_reset(gpio_scl);
	light_drv_i2c_delay();
	
	light_drv_i2c_scl_set(gpio_scl);
	light_drv_i2c_delay();
	
	if(light_drv_i2c_sda_read(gpio_sda)) {
		light_drv_i2c_scl_reset(gpio_scl);
		return DRV_I2C_ERROR;
	}
	light_drv_i2c_scl_reset(gpio_scl);
	return DRV_I2C_SUCCESS;
}
#endif

static void light_drv_i2c_send(unsigned char gpio_sda, unsigned char gpio_scl, unsigned char value)
{
	unsigned char i = 8;

	while(i --) {	
		light_drv_i2c_scl_reset(gpio_scl);
		light_drv_i2c_delay();

		if(value & 0x80)
			light_drv_i2c_sda_set(gpio_sda);  
		else
		    light_drv_i2c_sda_reset(gpio_sda);  

		value <<= 1;
		light_drv_i2c_delay();

		light_drv_i2c_scl_set(gpio_scl);
		light_drv_i2c_delay();  
	}
    
	light_drv_i2c_scl_reset(gpio_scl);
	light_drv_i2c_sda_reset(gpio_sda);
}

#if 0
static unsigned char light_drv_i2c_recv(unsigned char gpio_sda, unsigned char gpio_scl)
{
	unsigned char i = 8;
	unsigned char recv = 0;

	light_drv_i2c_sda_set(gpio_sda);                                                                                            

	while(i --) {
		recv <<= 1;  

		light_drv_i2c_scl_reset(gpio_scl);
		light_drv_i2c_delay();
		
		light_drv_i2c_scl_set(gpio_scl);
		light_drv_i2c_delay();  

		if(light_drv_i2c_sda_read(gpio_sda))
		{
		  recv |= 0x01;
		}
	}

	light_drv_i2c_scl_reset(gpio_scl);                                                             

	return recv;
}
#endif
static void light_drv_i2c_delayms(unsigned int time)
{
	volatile unsigned short i;  
	while(time --)
	{
		i = 200;
		while(i --);
	}  
}

OPERATE_RET light_drv_i2c_init(unsigned char gpio_sda, unsigned char gpio_scl)
{
    OPERATE_RET op_ret;
    STATIC UINT8_T init = 0;


    // Init LED control pin
    gpio_init(&gpio_sda0, gpio_sda);
    gpio_dir(&gpio_sda0, PIN_OUTPUT);    // Direction: Output
    gpio_mode(&gpio_sda0, PullUp);     // No pull

    
    gpio_init(&gpio_scl0, gpio_scl);
    gpio_dir(&gpio_scl0, PIN_OUTPUT);    // Direction: Output
    gpio_mode(&gpio_scl0, PullUp);     // No pull
    
    if(init == 0) {
        op_ret = CreateMutexAndInit(mutex);
        if(op_ret != OPRT_OK) {
            return op_ret;
        }

        init = 1;
    }
    
    return OPRT_OK;
}

void light_sm16726b_send_data(unsigned char gpio_sda, unsigned char gpio_scl,unsigned char *ptr,uint16_t number)
{
    MutexLock(mutex);
    
    unsigned char start_bit_cnt = 6;

    while(start_bit_cnt--)
        light_drv_i2c_send(gpio_sda, gpio_scl, 0x00);

    light_drv_i2c_send(gpio_sda, gpio_scl, 0x01);

    while(number --)
         light_drv_i2c_send(gpio_sda, gpio_scl, *ptr ++);    
    
    light_drv_i2c_sda_set(gpio_sda);
    light_drv_i2c_delay();
    light_drv_i2c_scl_reset(gpio_scl);
    light_drv_i2c_delay();
    light_drv_i2c_scl_set(gpio_scl);
    light_drv_i2c_delay();  
    light_drv_i2c_scl_reset(gpio_scl);
    
    light_drv_i2c_delayms(10);
    
    MutexUnLock(mutex);
    
    return SUCCESS;
    
}


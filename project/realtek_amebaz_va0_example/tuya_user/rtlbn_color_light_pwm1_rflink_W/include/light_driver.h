#ifndef __LIGHT_DRIVER_H__
#define __LIGHT_DRIVER_H__

OPERATE_RET light_drv_pwm_init(VOID);
VOID light_pwm_send_data(unsigned char *ptr,uint16_t number);

#endif


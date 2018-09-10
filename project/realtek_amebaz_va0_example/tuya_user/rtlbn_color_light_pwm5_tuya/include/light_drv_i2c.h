OPERATE_RET light_drv_i2c_init(unsigned char gpio_sda, unsigned char gpio_scl);

void light_sm16726b_send_data(unsigned char gpio_sda, unsigned char gpio_scl,unsigned char *ptr,uint16_t number);



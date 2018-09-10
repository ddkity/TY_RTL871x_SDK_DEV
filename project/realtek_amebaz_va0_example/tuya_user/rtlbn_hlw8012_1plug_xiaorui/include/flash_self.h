#ifndef __FLASH_SELF_H__
#define __FLASH_SELF_H__
#include "tuya_cloud_error_code.h"
#include "tuya_cloud_types.h"


OPERATE_RET flash_self_if_write(IN char* addr_str, IN char *ai);
OPERATE_RET flash_self_if_read(IN char* addr_str,OUT char *ai);

#endif

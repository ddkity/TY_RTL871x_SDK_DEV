/***********************************************************
*  File: tuya_gpio_test.h
*  Author: lql
*  Date: 20180502
***********************************************************/

typedef BYTE_T BOARD_TYPE; 
#define RTL_BOARD_WR1 0
#define RTL_BOARD_WR2 1
#define RTL_BOARD_WR3 2
#define RTL_BOARD_WR4 3
#define RTL_BOARD_WR5 4
#define RTL_BOARD_WR6 5
#define RTL_BOARD_WR7 6


BOOL_T gpio_test_cb(BOARD_TYPE type);
BOOL_T gpio_test_all(VOID) ;
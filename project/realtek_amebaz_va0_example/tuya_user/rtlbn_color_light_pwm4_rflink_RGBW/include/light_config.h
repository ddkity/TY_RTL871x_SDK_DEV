#define DEVICE_MOD "device_mod"
#define DEVICE_PART "device_part"
#define DP_DATA_KEY   "dp_data_key"
#define FASE_SW_CNT_KEY "fsw_cnt_key"
#define LIGHT_TEST_KEY   "light_test_key"
#define AGING_TESTED_TIME  "aging_tested_time"

#define BRIGHT_INIT_VALUE 255
//#define COL_TEMP_INIT_VALUE 255
//#define TEMP_FULL_VALUE 255
#define NORMAL_DELAY_TIME 3
#define RESO_VAL 4

#define AGING_TEST_TIME 30
#define AGING_TEST_C_TIME 10
#define AGING_TEST_W_TIME 10
#define AGING_TEST_RGB_TIME		(AGING_TEST_TIME - AGING_TEST_W_TIME)
#define TIME_SAVE_INTERVAL 1

STATIC UCHAR TEST_R_BRGHT = 10;
STATIC UCHAR TEST_G_BRGHT = 10;
STATIC UCHAR TEST_B_BRGHT = 10;
STATIC UCHAR TEST_W_BRGHT = 1;

#define     COLOUR_MODE_DEFAULT         "ff00000000ffff"

typedef enum {
    FUC_TEST1 = 0,
	AGING_TEST,
	FUC_TEST2,
}TEST_MODE_DEF;

typedef enum {
	FUN_SUC = 0,
    NO_KEY,
	WIFI_TEST_ERR,
}FUN_TEST_RET;

typedef struct
{
	UINT pmd_times;
	UINT aging_tested_time;
	BOOL wf_test_ret;
	FUN_TEST_RET fun_test_ret;
	TEST_MODE_DEF test_mode;
	TIMER_ID fuc_test_timer;
	TIMER_ID aging_test_timer;
}TEST_DEF;


#define CHAN_NUM    4
#define PWM_0_OUT_IO_NUM PA_5  //0channel--R
#define PWM_1_OUT_IO_NUM PA_15  //1channel--G
#define PWM_2_OUT_IO_NUM PA_14  //5channel--B
#define PWM_3_OUT_IO_NUM PA_12   //4channel--WC




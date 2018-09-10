#ifndef __LIGHT_HANDLER_H__
#define __LIGHT_HANDLER_H__
VOID dp_upload_proc(IN CONST TY_RECV_OBJ_DP_S *dp);

VOID light_wf_gw_status(IN CONST GW_WIFI_NW_STAT_E stat);

OPERATE_RET app_light_init(VOID);
void power_up_count_judge_smcfg (void);

void pre_light_init (void);
INT_T get_reset_cnt(VOID);

#endif


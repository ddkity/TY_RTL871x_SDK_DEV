/***********************************************************
*  File: tuya_main.c
*  Author: nzy
*  Date: 20171012
***********************************************************/
#include "adapter_platform.h"
#include "tuya_iot_wifi_api.h"
#include "flash_api.h"
#include <device_lock.h>
#include "rtl8710b_ota.h"
#include "mf_test.h"
#include "tuya_uart.h"
#include "tuya_gpio.h"
#include "gw_intf.h"
#include "wf_basic_intf.h"



/***********************************************************
*************************micro define***********************
***********************************************************/
#define TEST_SSID "tuya_mdev_test1"

/***********************************************************
*************************variable define********************
***********************************************************/

typedef enum {
    UGS_RECV_HEADER = 0,
    UGS_RECV_OTA_HD,
    UGS_SEARCH_SIGN,
    UGS_RECV_SIGN,
    UGS_RECV_IMG_DATA,
    // UGS_RECV_RDP_DATA,
    UGS_FINISH
}UG_STAT_E;

typedef struct {
    UG_STAT_E stat;
    update_ota_target_hdr hdr;
    u32 ota_index;

    u32 image_cnt;
    u8 signature[8];
    update_dw_info DownloadInfo[2];
    u32 cur_image_cnt[2];

    flash_t flash;
    u32 new_addr;
    u32 recv_data_cnt;
}UG_PROC_S;

STATIC UG_PROC_S *ug_proc = NULL;
extern VOID app_init(VOID);
typedef VOID (*APP_PROD_CB)(BOOL_T flag, CHAR_T rssi);//lql
STATIC APP_PROD_CB app_prod_test = NULL;//lql
STATIC GW_WF_CFG_MTHD_SEL gwcm_mode = GWCM_OLD;//lql



/***********************************************************
*************************function define********************
***********************************************************/
extern VOID pre_device_init(VOID);
extern OPERATE_RET device_init(VOID);
extern BOOL_T gpio_test(VOID);
extern void sys_jtag_off(void);
extern TY_GPIO_PORT_E swith_ctl_port;
STATIC VOID __gw_ug_inform_cb(IN CONST FW_UG_S *fw);
STATIC OPERATE_RET __gw_upgrage_process_cb(IN CONST FW_UG_S *fw, IN CONST UINT total_len,IN CONST UINT offset,\
                                                      IN CONST BYTE_T *data,IN CONST UINT len,OUT UINT *remain_len, IN PVOID pri_data);
STATIC VOID __gw_upgrade_notify_cb(IN CONST FW_UG_S *fw, IN CONST INT_T download_result, IN PVOID pri_data);

STATIC OPERATE_RET __mf_gw_upgrade_notify_cb(VOID);
STATIC VOID __mf_gw_ug_inform_cb(VOID);

STATIC VOID __tuya_mf_uart_init(UINT_T baud,UINT_T bufsz)
{
    ty_uart_init(TY_UART0,baud,TYWL_8B,TYP_NONE,TYS_STOPBIT1,bufsz);
}
STATIC VOID __tuya_mf_uart_free(VOID)
{
    ty_uart_free(TY_UART0);
}

STATIC VOID __tuya_mf_send(IN CONST BYTE_T *data,IN CONST UINT len)
{
    ty_uart_send_data(TY_UART0,data,len);
}

STATIC UINT __tuya_mf_recv(OUT BYTE_T *buf,IN CONST UINT len)
{
    return ty_uart_read_data(TY_UART0,buf,len);
}
STATIC BOOL_T scan_test_ssid(VOID)
{
	//if(FALSE == get_new_prod_mode()) {
		//return false;
	//}

	OPERATE_RET op_ret = OPRT_OK;
	//special for GWCM_OLD_PROD.......only do prodtesting when in smartlink or ap mode
	if(gwcm_mode == GWCM_OLD_PROD ) {
        op_ret = wd_gw_wsm_read(&(get_gw_cntl()->gw_wsm));
        if(get_gw_cntl()->gw_wsm.nc_tp >= GWNS_TY_SMARTCFG) {
            return false;
        }
	}

	wf_wk_mode_set(WWM_STATION);
	AP_IF_S *ap = NULL;
	BOOL_T flag = TRUE;
	op_ret = wf_assign_ap_scan(TEST_SSID, &ap);//lql
	wf_station_disconnect();
	if(OPRT_OK != op_ret) {
	    PR_NOTICE("wf_assign_ap_scan failed(%d)",op_ret);
		return FALSE;
	}
    //check if has authorized
    op_ret = wd_gw_base_if_read(&(get_gw_cntl()->gw_base));
    if(OPRT_OK != op_ret) {
        PR_DEBUG("read flash err");
        flag = FALSE;
    }
    // gateway base info verify
    if(0 == get_gw_cntl()->gw_base.auth_key[0] || \
       0 == get_gw_cntl()->gw_base.uuid[0]) {
        PR_DEBUG("please write uuid and auth_key first");
        flag = FALSE;
    }

	if(app_prod_test) {
		app_prod_test(flag, ap->rssi);
	}
	return TRUE;
}

STATIC BOOL_T is_cfg_or_ap_mode (void)
{
	OPERATE_RET op_ret = OPRT_OK;
    op_ret = wd_gw_wsm_read(&(get_gw_cntl()->gw_wsm));
    PR_DEBUG("get_gw_cntl()->gw_wsm.nc_tp:%d", get_gw_cntl()->gw_wsm.nc_tp);

    if(get_gw_cntl()->gw_wsm.nc_tp == GWNS_UNCFG_SMC || get_gw_cntl()->gw_wsm.nc_tp == GWNS_UNCFG_AP)
    {
        return TRUE;
    }
    return FALSE;
}

void app_cfg_set(IN CONST GW_WF_CFG_MTHD_SEL mthd, APP_PROD_CB callback)
{
	app_prod_test = callback;
	gwcm_mode = mthd;
}

/***********************************************************
*  Function: user_main
*  Input: none
*  Output: none
*  Return: none
***********************************************************/
void user_main(void)
{
    OPERATE_RET op_ret = OPRT_OK;
    op_ret = tuya_iot_init(NULL);
    if(OPRT_OK != op_ret) {
        PR_ERR("tuya_iot_init err:%d",op_ret);
        return;
    }
    sys_jtag_off();
    pre_device_init();

#if 1
    mf_reg_gw_ug_cb(__mf_gw_ug_inform_cb, __gw_upgrage_process_cb, __mf_gw_upgrade_notify_cb);
    MF_IMPORT_INTF_S intf = {
        __tuya_mf_uart_init,
        __tuya_mf_uart_free,
        __tuya_mf_send,
        __tuya_mf_recv,
        gpio_test,
    };
    op_ret = mf_init(&intf,APP_BIN_NAME,USER_SW_VER,TRUE);
    if(OPRT_OK != op_ret) {
        PR_ERR("mf_init err:%d",op_ret);
        return;
    }
    ty_uart_free(TY_UART0);
    PR_NOTICE("mf_init succ");
#endif
    // register gw upgrade inform callback
    tuya_iot_force_reg_gw_ug_cb(__gw_ug_inform_cb);

#if 0
    WF_GW_PROD_INFO_S wf_prod_info = {
        "003tuyatestf7f149189","NeA8Wc7srpAZHEMuru867oblOLN2QCC5",NULL,NULL
    };
    op_ret = tuya_iot_set_wf_gw_prod_info(&wf_prod_info);
    if(OPRT_OK != op_ret) {
        PR_ERR("tuya_iot_set_wf_gw_prod_info err:%d",op_ret);
        return;
    }
#endif

    app_init();
    PR_DEBUG("gwcm_mode %d",gwcm_mode);
    if(gwcm_mode != GWCM_OLD) {
        if (is_soft_reset_or_cnt_3() && (is_cfg_or_ap_mode() == FALSE))
        {
            PR_DEBUG("scan_test_ssid()");
            if(true == scan_test_ssid()) {
                PR_DEBUG("prodtest");
                return;
            }
            PR_DEBUG("no tuya_mdev_test1!");
        }
        else
        {
            PR_DEBUG("SKIP scan_test_ssid()");
        }
        op_ret = device_init();
        if(OPRT_OK != op_ret) {
            PR_ERR("device_init error:%d",op_ret);
            return;
        }
    }else {
        PR_DEBUG("device_init in");
        op_ret = device_init();
        if(OPRT_OK != op_ret) {
            PR_ERR("device_init err:%d",op_ret);
            return;
        }
    }
}
// mf gateway upgrade start
VOID __mf_gw_ug_inform_cb(VOID)
{
    ug_proc = Malloc(SIZEOF(UG_PROC_S));
    if(NULL == ug_proc) {
        PR_ERR("malloc err");
        return;
    }
    memset(ug_proc,0,SIZEOF(UG_PROC_S));

    if (ota_get_cur_index() == OTA_INDEX_1) {
        ug_proc->ota_index = OTA_INDEX_2;
        PR_DEBUG("OTA2 address space will be upgraded\n");
    } else {
        ug_proc->ota_index = OTA_INDEX_1;
        PR_DEBUG("OTA1 address space will be upgraded\n");
    }
}

// gateway upgrade start
STATIC VOID __gw_ug_inform_cb(IN CONST FW_UG_S *fw)
{
    ug_proc = Malloc(SIZEOF(UG_PROC_S));
    if(NULL == ug_proc) {
        PR_ERR("malloc err");
        return;
    }
    memset(ug_proc,0,SIZEOF(UG_PROC_S));
    OPERATE_RET op_ret = OPRT_OK;
    op_ret = tuya_iot_upgrade_gw(fw,__gw_upgrage_process_cb,__gw_upgrade_notify_cb,NULL);
    if(OPRT_OK != op_ret) {
        PR_ERR("tuya_iot_upgrade_gw err:%d",op_ret);
        return;
    }

    memset(ug_proc,0,SIZEOF(UG_PROC_S));
    if (ota_get_cur_index() == OTA_INDEX_1) {
        ug_proc->ota_index = OTA_INDEX_2;
        PR_DEBUG("OTA2 address space will be upgraded\n");
    } else {
        ug_proc->ota_index = OTA_INDEX_1;
        PR_DEBUG("OTA1 address space will be upgraded\n");
    }
}
// mf gateway upgrade result notify
OPERATE_RET __mf_gw_upgrade_notify_cb(VOID)
{
    // verify
    u32 ret = 0;
    ret = verify_ota_checksum(ug_proc->new_addr,ug_proc->DownloadInfo[0].ImageLen, \
                              ug_proc->signature,&ug_proc->hdr);
    if(1 != ret) {
        PR_ERR("verify_ota_checksum err");

        device_mutex_lock(RT_DEV_LOCK_FLASH);
        flash_erase_sector(&ug_proc->flash, ug_proc->new_addr - SPI_FLASH_BASE);
        device_mutex_unlock(RT_DEV_LOCK_FLASH);
        return OPRT_COM_ERROR;
    }

    if(!change_ota_signature(ug_proc->new_addr, ug_proc->signature, ug_proc->ota_index)) {
        PR_ERR("change signature failed\n");
        return OPRT_COM_ERROR;
    }

    PR_NOTICE("the gateway upgrade success");
    return OPRT_OK;
}

// gateway upgrade result notify
STATIC VOID __gw_upgrade_notify_cb(IN CONST FW_UG_S *fw, IN CONST INT_T download_result, IN PVOID pri_data)
{
    if(OPRT_OK == download_result) { // update success
        // verify
        u32 ret = 0;
        ret = verify_ota_checksum(ug_proc->new_addr,ug_proc->DownloadInfo[0].ImageLen, \
                                  ug_proc->signature,&ug_proc->hdr);
        if(1 != ret) {
            PR_ERR("verify_ota_checksum err");
            #if 0
            device_mutex_lock(RT_DEV_LOCK_FLASH);
            flash_erase_sector(&ug_proc->flash, ug_proc->new_addr - SPI_FLASH_BASE);
            device_mutex_unlock(RT_DEV_LOCK_FLASH);
            #endif
            return;
        }

        if(!change_ota_signature(ug_proc->new_addr, ug_proc->signature, ug_proc->ota_index)) {
            PR_ERR("change signature failed\n");
            return;
        }

        PR_NOTICE("the gateway upgrade success");
        ota_platform_reset();
        return;
    }else {
        PR_ERR("the gateway upgrade failed");
    }
}

// gateway upgrade process
STATIC OPERATE_RET __gw_upgrage_process_cb(IN CONST FW_UG_S *fw, IN CONST UINT total_len,IN CONST UINT offset,\
                                                      IN CONST BYTE_T *data,IN CONST UINT len,OUT UINT *remain_len, IN PVOID pri_data)
{
    STATIC UINT ota_hd_len = 0;

    switch(ug_proc->stat) {
        case UGS_RECV_HEADER: {
            if(len < SIZEOF(update_file_hdr)) {
                *remain_len = len;
                break;
            }

            memcpy(&ug_proc->hdr.FileHdr,data,SIZEOF(update_file_hdr));
            ug_proc->stat = UGS_RECV_OTA_HD;
            ota_hd_len = (ug_proc->hdr.FileHdr.HdrNum) * SIZEOF(update_file_img_hdr);
            *remain_len = len;

            PR_DEBUG("ota_hd_len:%d",ota_hd_len);
        }
        //break;

        case UGS_RECV_OTA_HD: {
            if(len < ota_hd_len+SIZEOF(update_file_hdr)) {
                *remain_len = len;
                break;
            }

            char * pImgId = NULL;
            if(OTA_INDEX_1 == ug_proc->ota_index) {
                pImgId = "OTA1";
            }else {
                pImgId = "OTA2";
            }

            u32 ret = 0;
            ret = get_ota_tartget_header(data,(ota_hd_len+SIZEOF(update_file_hdr)), \
                                         &(ug_proc->hdr), pImgId);
            if(0 == ret) {
                PR_ERR("get_ota_tartget_header err");
                return OPRT_COM_ERROR;
            }

            // get new image addr and check new address validity
            if(!get_ota_address(ug_proc->ota_index, &ug_proc->new_addr, &(ug_proc->hdr))) {
                PR_ERR("get_ota_address err");
                return OPRT_COM_ERROR;
            }

            u32 new_img_len = ug_proc->hdr.FileImgHdr.ImgLen;

            PR_NOTICE("ug_proc->new_addr:0x%x",ug_proc->new_addr);
            PR_NOTICE("new_img_len:%d",new_img_len);
            if(new_img_len > (0x80000 - 0xB000)) {
                PR_ERR("image length is too big");
                return OPRT_COM_ERROR;
            }
            erase_ota_target_flash(ug_proc->new_addr, new_img_len);
            if(ug_proc->hdr.RdpStatus == ENABLE) {
                device_mutex_lock(RT_DEV_LOCK_FLASH);
                flash_erase_sector(&ug_proc->flash, RDP_FLASH_ADDR - SPI_FLASH_BASE);
                device_mutex_unlock(RT_DEV_LOCK_FLASH);
            }

            // 暂时仅支持image的升级
            if(ug_proc->hdr.RdpStatus == ENABLE) {
                ug_proc->image_cnt = 2;
                if(ug_proc->hdr.FileImgHdr.Offset < ug_proc->hdr.FileRdpHdr.Offset) {
                    ug_proc->DownloadInfo[0].ImgId = OTA_IMAG;
                    /* get OTA image and Write New Image to flash, skip the signature,
                    not write signature first for power down protection*/
                    ug_proc->DownloadInfo[0].FlashAddr = ug_proc->new_addr -SPI_FLASH_BASE + 8;
                    ug_proc->DownloadInfo[0].ImageLen = ug_proc->hdr.FileImgHdr.ImgLen - 8;/*skip the signature*/
                    ug_proc->DownloadInfo[0].ImgOffset = ug_proc->hdr.FileImgHdr.Offset;
                    #if 0
                    ug_proc->DownloadInfo[1].ImgId = RDP_IMAG;
                    ug_proc->DownloadInfo[1].FlashAddr = RDP_FLASH_ADDR - SPI_FLASH_BASE;
                    ug_proc->DownloadInfo[1].ImageLen = ug_proc->hdr.FileRdpHdr.ImgLen;
                    ug_proc->DownloadInfo[1].ImgOffset = ug_proc->hdr.FileRdpHdr.Offset;
                    #endif
                } else {
                    #if 0
                    ug_proc->DownloadInfo[0].ImgId = RDP_IMAG;
                    ug_proc->DownloadInfo[0].FlashAddr = RDP_FLASH_ADDR - SPI_FLASH_BASE;
                    ug_proc->DownloadInfo[0].ImageLen = ug_proc->hdr.FileRdpHdr.ImgLen;
                    ug_proc->DownloadInfo[0].ImgOffset = ug_proc->hdr.FileRdpHdr.Offset;
                    ug_proc->DownloadInfo[1].ImgId = OTA_IMAG;
                    /* get OTA image and Write New Image to flash, skip the signature,
                    not write signature first for power down protection*/
                    ug_proc->DownloadInfo[1].FlashAddr = ug_proc->new_addr -SPI_FLASH_BASE + 8;
                    ug_proc->DownloadInfo[1].ImageLen = ug_proc->hdr.FileImgHdr.ImgLen - 8;/*skip the signature*/
                    ug_proc->DownloadInfo[1].ImgOffset = ug_proc->hdr.FileImgHdr.Offset;
                    #else
                    ug_proc->DownloadInfo[0].ImgId = OTA_IMAG;
                    /* get OTA image and Write New Image to flash, skip the signature,
                    not write signature first for power down protection*/
                    ug_proc->DownloadInfo[0].FlashAddr = ug_proc->new_addr -SPI_FLASH_BASE + 8;
                    ug_proc->DownloadInfo[0].ImageLen = ug_proc->hdr.FileImgHdr.ImgLen - 8;/*skip the signature*/
                    ug_proc->DownloadInfo[0].ImgOffset = ug_proc->hdr.FileImgHdr.Offset;
                    #endif
                }
            }else {
                ug_proc->image_cnt = 1;
                ug_proc->DownloadInfo[0].ImgId = OTA_IMAG;
                /* get OTA image and Write New Image to flash, skip the signature,
                not write signature first for power down protection*/
                ug_proc->DownloadInfo[0].FlashAddr = ug_proc->new_addr -SPI_FLASH_BASE + 8;
                ug_proc->DownloadInfo[0].ImageLen = ug_proc->hdr.FileImgHdr.ImgLen - 8;/*skip the signature*/
                ug_proc->DownloadInfo[0].ImgOffset = ug_proc->hdr.FileImgHdr.Offset;
            }
            PR_NOTICE("FlashAddr = 0x%x", ug_proc->DownloadInfo[0].FlashAddr);
            PR_NOTICE("ImageLen = %d", ug_proc->DownloadInfo[0].ImageLen);
            PR_NOTICE("ImgOffset = %d", ug_proc->DownloadInfo[0].ImgOffset);
            PR_NOTICE("OTA Image Address = 0x%x", ug_proc->new_addr);
            if(ug_proc->hdr.RdpStatus == ENABLE) {
                PR_NOTICE("RDP Image Address = %x", RDP_FLASH_ADDR);
            }

            ug_proc->recv_data_cnt = ota_hd_len + sizeof(update_file_hdr); // (ug_proc->hdr.FileHdr.HdrNum * ug_proc->hdr.FileImgHdr.ImgHdrLen)
            *remain_len = len - (ota_hd_len+SIZEOF(update_file_hdr));
            ug_proc->stat = UGS_SEARCH_SIGN;
        }
        break;

        case UGS_SEARCH_SIGN: {
            if(ug_proc->recv_data_cnt + len < ug_proc->DownloadInfo[0].ImgOffset) {
                ug_proc->recv_data_cnt += len;
                *remain_len = 0;
                break;
            }
            PR_NOTICE("ug_proc->recv_data_cnt:%d",ug_proc->recv_data_cnt);
            PR_NOTICE("len:%d",len);

            INT_T offset = (ug_proc->DownloadInfo[0].ImgOffset - ug_proc->recv_data_cnt);
            *remain_len = len - offset;
            if(*remain_len < SIZEOF(ug_proc->signature)) {
                ug_proc->recv_data_cnt += offset;
                ug_proc->stat = UGS_RECV_SIGN;
            }else {
                mempcpy(ug_proc->signature,data+offset,SIZEOF(ug_proc->signature));
                *remain_len -= SIZEOF(ug_proc->signature);
                ug_proc->stat = UGS_RECV_IMG_DATA;
            }
        }
        break;

        case UGS_RECV_SIGN: {
            if(len < SIZEOF(ug_proc->signature)) { // 8 is signature
                *remain_len = len;
                break;
            }

            mempcpy(ug_proc->signature,data,SIZEOF(ug_proc->signature));

            #if 0
            INT_T i = 0;
            PR_DEBUG_RAW("ug_proc->signature:");
            for(i = 0;i < SIZEOF(ug_proc->signature);i++) {
                PR_DEBUG_RAW("%02x",ug_proc->signature[i]);
            }
            PR_DEBUG_RAW("\n");
            #endif

            ug_proc->recv_data_cnt += 8;
            *remain_len = len-8;
            ug_proc->stat = UGS_RECV_IMG_DATA;
        }
        break;

        case UGS_RECV_IMG_DATA: {
            #define RT_IMG_WR_UNIT 1024
            if(ug_proc->cur_image_cnt[0] >= ug_proc->DownloadInfo[0].ImageLen) {
                ug_proc->stat = UGS_FINISH;
                *remain_len = len;
                break;
            }

            if((ug_proc->cur_image_cnt[0] + len < ug_proc->DownloadInfo[0].ImageLen) && \
               (len < RT_IMG_WR_UNIT)) {
                *remain_len = len;
                break;
            }

            u32 write_len = 0;
            if(ug_proc->cur_image_cnt[0] + len < ug_proc->DownloadInfo[0].ImageLen) {
                write_len = RT_IMG_WR_UNIT;
            }else {
                write_len = (ug_proc->DownloadInfo[0].ImageLen - ug_proc->cur_image_cnt[0]);
            }
            device_mutex_lock(RT_DEV_LOCK_FLASH);
            #if 0
            {
                PR_DEBUG_RAW("wirite data:");
                INT_T i = 0;
                for(i = 0;i < 16;i++) {
                    PR_DEBUG_RAW("%02x ",data[i]);
                }
                PR_DEBUG_RAW("\n");
            }
            #endif
            if(flash_stream_write(&ug_proc->flash, ug_proc->DownloadInfo[0].FlashAddr + ug_proc->cur_image_cnt[0], \
                                  write_len, data) < 0) {
                device_mutex_unlock(RT_DEV_LOCK_FLASH);
                PR_ERR("[%s] Write sector failed");
                return OPRT_WR_FLASH_ERROR;
            }
            device_mutex_unlock(RT_DEV_LOCK_FLASH);
            ug_proc->cur_image_cnt[0] += write_len;
            ug_proc->recv_data_cnt += write_len;
            *remain_len = len - write_len;

            if(ug_proc->cur_image_cnt[0] >= ug_proc->DownloadInfo[0].ImageLen) {
                ug_proc->stat = UGS_FINISH;
                break;
            }
        }
        break;

        case UGS_FINISH: {
            *remain_len = 0;
        }
        break;
    }
    return OPRT_OK;
}











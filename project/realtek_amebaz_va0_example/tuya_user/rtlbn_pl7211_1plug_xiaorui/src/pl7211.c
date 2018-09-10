#include "pl7211.h"
#include "adapter_platform.h"
#include "tuya_iot_wifi_api.h"
#include "tuya_led.h"
#include "tuya_uart.h"
#include "tuya_gpio.h"
#include "tuya_key.h"
#include "uni_time_queue.h"
#include "gw_intf.h"
#include "uni_log.h"
#include "uni_thread.h"
#include "uni_time.h"

#include "hw_table.h"
#include "tuya_ws_db.h"


#define  PL_RESET_PIN                TY_GPIOA_12
#define  MODU_RELAY_PIN              TY_GPIOA_15//CHG
#define  UART_TX_DEBUG               0
#define  DATA_BUF_SZ                 256
#define  MIN_RECV_LENTH              6

#define  UART_PREAMBLE_H			0x55
#define  UART_PREAMBLE_L            0xaa
#define  UART_SID					0xff			
#define  UART_ACK					0x5a
#define  UART_NAK					0xa5
#define  UART_CMD_R_CRC				1
#define  UART_CMD_W_CRC				2	
#define  UART_CMD_R					3
#define  UART_CMD_W					4	
#define  ADR_V                      0x3078
#define  ADR_I                      0x3084
#define  ADR_P                      0x3090
#define  ADR_E                      0X312C
#define  ADR_RELAY                  0x3803
#define  ADR_SET_ZERO_EN            0x3804
#define  ADR_RW_LONG_LEN            0x3805
#define  ADR_LOCK_BAUD              0x380d
#define  ADR_TIMEOUT_PERIOD         0x380e
#define  ADR_ZERO_ON_BUF            0X304C
#define  ADR_ZERO_OFF_BUF           0X3052
#define  ADR_ZCC_CNT                0X3018
#define  ADR_ZCC_START              0X301E
#define  ADR_ZCC_STOP               0X3024
#define  ADR_SAM_CNT                0X3809
#define  ADR_ZERO_ON_RO             0X4026
#define  ADR_ZERO_OFF_RO            0X402C
#define  ADR_ZERO_SELF_LEARN_RO     0X4074
#define  WRITE_DELAY_TIME_MS        20
#define  READ_DELAY_TIME_MS         10
#define  OPRT_RELAY_DELAY_TIME_MS   500


#define ZERO_DATA_SAVE_KEY "zero_data_key"

typedef struct RuningInf_s{
    unsigned int	voltage;//当前电压值，单位为0.1V
    unsigned int	electricity;//当前电流值,单位为0.01A
    unsigned int	power;//当前功率值,单位为0.1W
    unsigned int	addEnergy;//增加的电量
    unsigned int    last_allEner;
}RuningInf_t;


typedef enum
{
    V_DEAL = 0,
    I_DEAL,
    P_DEAL,
    E_DEAL,
}DATA_DEAL;


typedef struct {
    MUTEX_HANDLE    write_mutex;
    MUTEX_HANDLE    receive_mutex;
    THRD_HANDLE 	uart_data_thread;
}UART_S;

typedef struct
{
    UCHAR		pream_h;
    UCHAR		pream_l;
    UCHAR		sid;
    union{
        UCHAR		i;
        struct{
            UCHAR		len    : 4;	//d[3:0]
            UCHAR		rw     : 3;	//d[6:4]
            UCHAR		incAdr : 1;	//d[7]		
        }b;	
    }cmd;
    UCHAR		adr_h;
    UCHAR		adr_l;
}uart_read_s;

typedef struct
{
    UCHAR		pream_h;
    UCHAR		pream_l;
    UCHAR		sid;
    union{
        UCHAR		i;
        struct{
            UCHAR		len    : 4;	//d[3:0]
            UCHAR		rw     : 3;	//d[6:4]
            UCHAR		incAdr : 1;	//d[7]		
        }b;	
    }cmd;
    UCHAR		adr_h;
    UCHAR		adr_l;
    UCHAR       data;
}uart_write_s;


typedef enum{
    ZERO_MODE_FIRST_ON,
    ZERO_MODE_LEARN,
    ZERO_MODE_NORMAL,
    ZERO_MODE_WRITE
}ZERO_MODE;




RuningInf_t runingInf={0};
LED_HANDLE pl_reset_handle = NULL;
LED_HANDLE modu_relay_handle = NULL;

MUTEX_HANDLE     piv_mutex;   //信号量
MUTEX_HANDLE     E_mutex;   //信号量
STATIC UCHAR proc_buf[36] = {0};
STATIC UCHAR wr_buf[6] = {0};
STATIC UCHAR r_err = 0;
STATIC UART_S uart_proc = {0};

ZERO_MODE zero_mode = ZERO_MODE_FIRST_ON;
INT_T zero_data = 0;
UINT_T zero_frq = 0;
UINT_T cur_frq = 0;


#define my_abs(x,y) ((x)>(y) ? (x)-(y):(y)-(x))

UCHAR CRC8_Calculation(UCHAR* ptr_crc, UCHAR len);
STATIC VOID  read_register_deal(UCHAR *pbuf, UCHAR sta       );
STATIC OPERATE_RET ur_send_data(UCHAR *data,UCHAR len);
STATIC VOID uart_data_deal(PVOID pArg);
STATIC bool uart_recieve_data_proc(IN UCHAR read_len,OUT UCHAR* out_buf,IN USHORT_T timeout_ms);
STATIC VOID uart_send_read_cmd(IN USHORT adr,IN UCHAR read_len);
STATIC VOID uart_send_write_cmd(IN USHORT adr,IN UCHAR write_data);
STATIC OPERATE_RET set_zero_data_val();
STATIC OPERATE_RET get_zero_data_val();
STATIC VOID uart_send_write_array(IN USHORT adr,IN UCHAR* pwrite,IN UCHAR write_len);
STATIC BOOL get_cur_frequence();
STATIC VOID pl7211_reset_init();





OPERATE_RET pl7211_init(PL_INIT_MODE pl_init_mode)
{
    OPERATE_RET op_ret;
    if(PL_INIT_NORMAL == pl_init_mode){
        if(OPRT_OK != get_zero_data_val()){
            zero_mode = ZERO_MODE_LEARN;
        }else{
            zero_mode = ZERO_MODE_FIRST_ON;
        }
    }else{
        zero_mode = ZERO_MODE_LEARN;
    }
    ty_uart_init(TY_UART0,TYU_RATE_9600,TYWL_8B,TYP_NONE,TYS_STOPBIT1,256);

    op_ret = CreateMutexAndInit(&uart_proc.write_mutex);
    if(OPRT_OK != op_ret) {
        PR_ERR("uart_proc.write_mutex init is err");
        return op_ret;
    }
    op_ret = CreateMutexAndInit(&uart_proc.receive_mutex);
    if(OPRT_OK != op_ret) {
        PR_ERR("uart_proc.write_mutex init is err");
        return op_ret;
    }

    op_ret = CreateMutexAndInit(&piv_mutex);
    if(OPRT_OK != op_ret) {
        PR_ERR("piv_mutex init is err");
        return op_ret;
    }

    op_ret = CreateMutexAndInit(&E_mutex);
    if(OPRT_OK != op_ret) {
        PR_ERR("E_mutex init is err");
        return op_ret;
    }
    
    op_ret = tuya_create_led_handle(PL_RESET_PIN,false,&pl_reset_handle);
    if(OPRT_OK  != op_ret) {
        return op_ret;
    }
    
    op_ret = tuya_create_led_handle(MODU_RELAY_PIN,false,&modu_relay_handle);
    if(OPRT_OK  != op_ret) {
        return op_ret;
    }

    op_ret = CreateAndStart(&uart_proc.uart_data_thread,uart_data_deal,NULL,1024,TRD_PRIO_3,"uart_data_deal");
    if(op_ret != OPRT_OK) {
        return op_ret;
    }

}

STATIC BOOL pl_connect_confirm()
{
    uart_send_read_cmd(ADR_TIMEOUT_PERIOD,1);
    if(uart_recieve_data_proc(1, proc_buf, 200)){
        return true;
    }else{
        SystemSleep(50);
        uart_send_read_cmd(ADR_TIMEOUT_PERIOD,1);
        if(uart_recieve_data_proc(1, proc_buf, 200)){
            return true;
        }else{
            return false;
        }
    }
}

STATIC VOID pl_hardware_reset()
{
    PR_DEBUG("pl reset!!!");
    tuya_set_led_light_type(pl_reset_handle, OL_LOW, 0,0);
    SystemSleep(300);
    tuya_set_led_light_type(pl_reset_handle, OL_HIGH, 0,0);
    SystemSleep(100);
    while(!pl_connect_confirm()){
        PR_DEBUG("pl reset!!!");
        tuya_set_led_light_type(pl_reset_handle, OL_LOW, 0,0);
        SystemSleep(700);
        tuya_set_led_light_type(pl_reset_handle, OL_HIGH, 0,0);
        SystemSleep(300);
    }
}


STATIC UCHAR zero_on_l,zero_on_m;
STATIC UCHAR zero_off_l,zero_off_m;
STATIC VOID pl7211_reset_init()
{
    if(g_hw_table.channels[0].stat){
        tuya_set_led_light_type(modu_relay_handle, OL_HIGH, 0,0);
    }else{
        tuya_set_led_light_type(modu_relay_handle, OL_LOW, 0,0);
    }
    pl_hardware_reset();
    uart_send_write_cmd(ADR_LOCK_BAUD,0xd3);
    BOOL read_ret = false;
    while(zero_mode != ZERO_MODE_NORMAL){
        switch(zero_mode){
        case ZERO_MODE_LEARN:
            uart_send_write_cmd(ADR_SET_ZERO_EN,0x10);
            uart_send_write_cmd(ADR_SET_ZERO_EN,0x00);
            uart_send_write_cmd(ADR_RELAY,0x7a);
            SystemSleep(OPRT_RELAY_DELAY_TIME_MS);
            UCHAR i=0;
            for(i=0;i<3;i++){
                uart_send_write_cmd(ADR_RELAY,0x7e);
                SystemSleep(OPRT_RELAY_DELAY_TIME_MS);
                uart_send_write_cmd(ADR_RELAY,0x7a);
                SystemSleep(OPRT_RELAY_DELAY_TIME_MS);
            }
            uart_send_write_cmd(ADR_RELAY,0x78);
            
            uart_send_read_cmd(ADR_ZERO_ON_BUF,2);
            read_ret = uart_recieve_data_proc(2, proc_buf, 200);
            if(read_ret){
                zero_on_l = proc_buf[0];
                zero_on_m = proc_buf[1];
                uart_send_read_cmd(ADR_ZERO_OFF_BUF,2);
                read_ret = uart_recieve_data_proc(2, proc_buf, 200);
                if(read_ret){
                    zero_off_l = proc_buf[0];
                    zero_off_m = proc_buf[1];
                    read_ret = get_cur_frequence();
                    if(read_ret){
                        if(OPRT_OK != set_zero_data_val()){
                            PR_ERR("set_zero_data err!!!");
                        }else{
                            zero_mode = ZERO_MODE_NORMAL;
                        }
                    }
                }
            }
            if(!read_ret){
                pl_hardware_reset();
            }
            break;
        case ZERO_MODE_FIRST_ON:
            if(get_cur_frequence()){
                if( my_abs(cur_frq,zero_frq) > 20){
                    zero_mode = ZERO_MODE_LEARN;
                    PR_DEBUG("cur_frq:%d,zero frq:%d,unit:0.1hz,now relearn",cur_frq,zero_frq);
                }else{
                    PR_DEBUG("frequence not change,no need to learn!!!");
                    zero_mode = ZERO_MODE_WRITE;
                }
            }else{
                r_err++;
                if(r_err>=3){
                    r_err = 0;
                    pl_hardware_reset();
                }
                PR_DEBUG("get frequence err!!!");
            }
            break;
        case ZERO_MODE_WRITE:
            wr_buf[0] = zero_on_l;
            wr_buf[1] = zero_on_m;
            uart_send_write_array(ADR_ZERO_ON_RO, wr_buf, 2);
            wr_buf[0] = zero_off_l;
            wr_buf[1] = zero_off_m;
            uart_send_write_array(ADR_ZERO_OFF_RO, wr_buf, 2);
            wr_buf[0] = 0;
            wr_buf[1] = 0;
            wr_buf[2] = 0;
            wr_buf[3] = 0;
            uart_send_write_array(ADR_ZERO_SELF_LEARN_RO, wr_buf, 4);
            zero_mode = ZERO_MODE_NORMAL;
            break;
        }
        SystemSleep(50);
    }
    uart_send_write_cmd(ADR_RW_LONG_LEN,36);
    if(g_hw_table.channels[0].stat){
        uart_send_write_cmd(ADR_RELAY, 0x7C);
    }else{
        uart_send_write_cmd(ADR_RELAY, 0x78);
    }

}

STATIC BOOL is_upgrading = false;
VOID set_upgrading_state(BOOL state)
{
    is_upgrading = state;
}

STATIC VOID uart_data_deal(PVOID pArg)
{

    SystemSleep(1000);
    pl7211_reset_init();

    UCHAR i;
    UINT_T e_cnt=0;
    BOOL read_ret = false;
    while(1) {
        if(!is_upgrading){
            if(zero_mode == ZERO_MODE_NORMAL){
                MutexLock(uart_proc.receive_mutex);
                uart_send_read_cmd(ADR_V, 36);
                read_ret = uart_recieve_data_proc(36, proc_buf, 200);
                SystemSleep(20);
                MutexUnLock(uart_proc.receive_mutex);
                if(read_ret){
                    r_err = 0;
                    read_register_deal(proc_buf,V_DEAL);
                    read_register_deal(proc_buf + 2*MIN_RECV_LENTH,I_DEAL);
                    read_register_deal(proc_buf + 4*MIN_RECV_LENTH,P_DEAL);
                }else{
                    r_err++;
                    if(r_err >=3){
                        r_err = 0;
                        pl7211_reset_init();
                    }
                }
                e_cnt++;
                if(e_cnt >= 10){
                    e_cnt = 0;
                    SystemSleep(2000);
                    MutexLock(uart_proc.receive_mutex);
                    uart_send_read_cmd(ADR_E, 6);
                    read_ret = uart_recieve_data_proc(6, proc_buf, 200);
                    MutexUnLock(uart_proc.receive_mutex);
                    if(read_ret){
                        r_err = 0;
                        read_register_deal(proc_buf,E_DEAL);
                    }else{
                        r_err++;
                        if(r_err >=3){
                            r_err = 0;
                            pl7211_reset_init();
                        }
                    }
                }
            }
        }
        
        SystemSleep(1000);
    }
}

STATIC bool uart_recieve_data_proc(IN UCHAR read_len,OUT UCHAR* out_buf,IN USHORT_T timeout_ms)
{
    UCHAR tmp_rv_buf[DATA_BUF_SZ] = {0};
    UINT_T tmp_rv_buf_len = 0;
    USHORT count = 0;
    USHORT read_time = 0;
    while(tmp_rv_buf_len < read_len && read_time < timeout_ms){
        SystemSleep(10);
        read_time += 10;
        count = ty_uart_read_data(TY_UART0, tmp_rv_buf + \
        tmp_rv_buf_len,DATA_BUF_SZ - tmp_rv_buf_len);
        tmp_rv_buf_len += count;
        //PR_DEBUG("To read len:%d! this time, read count :%d,curr lenth:%d",read_len,count,tmp_rv_buf_len);
    }
    //PR_DEBUG("To read len:%d,Finally, real read lenth:%d ,read time:%d ms",read_len,tmp_rv_buf_len,read_time);
    /*
       if(tmp_rv_buf_len==MIN_RECV_LENTH+1){
           crc=CRC8_Calculation(tmp_rv_buf,MIN_RECV_LENTH);
           if(crc==tmp_rv_buf[MIN_RECV_LENTH]){
           tmp_rv_buf_len=0;
           memcpy(buf,tmp_rv_buf,MIN_RECV_LENTH);
           memset(tmp_rv_buf,0,DATA_BUF_SZ);
           return true; 
           }else{
           PR_DEBUG("crc is err: %d",crc);
           PR_DEBUG("rv_buf[5] :%d",tmp_rv_buf[MIN_RECV_LENTH]);
           tmp_rv_buf_len=0;
           memset(tmp_rv_buf,0,DATA_BUF_SZ);
           return false;
           }
       }*/
    UCHAR i=0;
    for(i=0;i<tmp_rv_buf_len;i++)
    {
        PR_DEBUG("rv_buf[%d]=%d",i,tmp_rv_buf[i]);
    }
    if(tmp_rv_buf_len== read_len){
        memcpy(out_buf,tmp_rv_buf,read_len);
        return true;
    }else if(tmp_rv_buf_len){
        ty_uart_read_data(TY_UART0, tmp_rv_buf + \
        tmp_rv_buf_len,DATA_BUF_SZ - tmp_rv_buf_len);
        PR_DEBUG("!!!!!!!!!!current user uart buffer:%d",count);
        uart_send_write_cmd(ADR_RW_LONG_LEN,36);
        return false;
    }else{
        return false;
    }
}





STATIC VOID  read_register_deal(UCHAR *pbuf, UCHAR sta       )
{
    UINT_T    buf_value=0;
    double  d_buf=0; 
    UINT_T    v_buf=0;
    switch(sta){
        case V_DEAL: 
            buf_value=0;
            buf_value= pbuf[5];
            buf_value=(buf_value<<8)+pbuf[4];
            buf_value=(buf_value<<8)+pbuf[3];
            buf_value=(buf_value<<8)+pbuf[2];
            d_buf =(double)buf_value / (double)(256.0);  
            v_buf = d_buf*10.00;
            if((v_buf>=0)&&(v_buf<=2500)){
                MutexLock(piv_mutex);
                runingInf.voltage= v_buf;
                MutexUnLock(piv_mutex);
            }
            PR_DEBUG(" voltage is %d", runingInf.voltage);
            memset(pbuf,0,6);
            break;
        case I_DEAL:
            buf_value=0;
            buf_value= pbuf[5];
            buf_value=(buf_value<<8)+pbuf[4];
            buf_value=(buf_value<<8)+pbuf[3];
            buf_value=(buf_value<<8)+pbuf[2];
            d_buf =(double)buf_value / (double)(16384.0); 
            v_buf = d_buf*1000.00;
            if((v_buf>=0)&&(v_buf<=30000)){
                MutexLock(piv_mutex);
                runingInf.electricity= v_buf;
                MutexUnLock(piv_mutex);
            }
            PR_DEBUG(" current is %d",  runingInf.electricity);
            memset(pbuf,0,6);
            break;
        case P_DEAL:
            buf_value=0;
            buf_value= pbuf[5];
            buf_value=(buf_value<<8)+pbuf[4];
            buf_value=(buf_value<<8)+pbuf[3];
            buf_value=(buf_value<<8)+pbuf[2];
            d_buf =(double)buf_value / (double)(256.0); 
            v_buf = d_buf*10.00;
            if((v_buf>=0)&&(v_buf<=50000)){
                MutexLock(piv_mutex);
                runingInf.power= v_buf;
                MutexUnLock(piv_mutex);
            }
            PR_DEBUG(" power is %d",  runingInf.power);
            memset(pbuf,0,6);
            break;
        case E_DEAL:
            buf_value=0;
            buf_value= pbuf[3];
            buf_value=(buf_value<<8)+pbuf[2];
            buf_value=(buf_value<<8)+pbuf[1];
            buf_value=(buf_value<<8)+pbuf[0];
            d_buf =(double)buf_value*(double)(0.3125);
            PR_DEBUG(" d_buf is %d",(unsigned int)d_buf);
            MutexLock(E_mutex);
            if(runingInf.addEnergy==0){
                //if((unsigned int)d_buf>=0xfffffff0){
                //   PR_DEBUG("rest 7211");
                //   if(runingInf.last_allEner<=(unsigned int)d_buf){
                //      runingInf.addEnergy=((unsigned int)d_buf-runingInf.last_allEner);
                //  }
                //   runingInf.last_allEner=0;
                //}
                if(runingInf.last_allEner<=(unsigned int)d_buf){
                    runingInf.addEnergy=( (unsigned int)d_buf - runingInf.last_allEner );
                    runingInf.last_allEner=(unsigned int)d_buf;
                }
                PR_DEBUG(" E is %d",runingInf.addEnergy);
                PR_DEBUG(" E last is   %d",runingInf.last_allEner);
                memset(pbuf,0,6);
            }
            MutexUnLock(E_mutex);
            break;
                    
        default :
            PR_ERR(" !!!read_register_deal is err %d",sta);
            break;
    }

}


BOOL get_frq_base_data(USHORT adr,UCHAR len,UINT_T *data)
{
    BOOL read_ret = false;
    UCHAR i=0;
    UINT_T  d_buf=0; 
    uart_send_read_cmd(adr,len);
    read_ret = uart_recieve_data_proc(len, proc_buf, 200);
    if(read_ret){
        d_buf = 0;
        for(i = 0;i <= len;i++){
            d_buf=(d_buf<<8)+proc_buf[len-i];
        }
        *data = (UINT_T)d_buf;
        PR_DEBUG("adr:0x%x,val:%d",adr,d_buf);
        return true;
    }else{
        return false;
    }

}


STATIC BOOL get_cur_frequence()
{
    double f_buf = 0;
    UINT_T zcc_cnt,zcc_start,zcc_stop,zcc_sam;
    BOOL get_ret;
    if(get_frq_base_data(ADR_ZCC_CNT,2,&zcc_cnt)){
        if(get_frq_base_data(ADR_ZCC_START,4,&zcc_start)){
            if(get_frq_base_data(ADR_ZCC_STOP,4,&zcc_stop)){
                if(get_frq_base_data(ADR_SAM_CNT,2,&zcc_sam)){
                    if(zcc_stop-zcc_start&&zcc_sam){
                        PR_DEBUG("cnt:%x,start:%x,stop:%x,sample:%x",zcc_cnt,\
                        zcc_start,zcc_stop,zcc_sam);
                        f_buf = ((double)(zcc_cnt - 1))/2*zcc_sam*10/(zcc_stop-zcc_start);//频率的单位为0.1hz
                        cur_frq = (UINT_T)f_buf;
                        PR_DEBUG("current frequence:%d",cur_frq);
                        return true;
                    }
                }
            }
        }
    }
    return false;
}



STATIC VOID uart_send_read_cmd(IN USHORT adr,IN UCHAR read_len)
{
    UCHAR wait_time = 0;
    UCHAR tmp_rv_buf[DATA_BUF_SZ] = {0};
    UINT_T tmp_rv_buf_len = 0;
    while(ty_uart_read_data(TY_UART0, tmp_rv_buf + \
        tmp_rv_buf_len,DATA_BUF_SZ - tmp_rv_buf_len)\
        && wait_time < 50 ){
        SystemSleep(10);
        wait_time += 10;

    }
    if(wait_time){
        PR_DEBUG("!!!!!!!!!!ready to read 0X%4x,curr len:%d,wait time:%d ms",\
            adr,tmp_rv_buf_len,wait_time);
    }

    UCHAR crc_=0;
    uart_read_s uart_pack={0};
    uart_pack.pream_h=UART_PREAMBLE_H;
    uart_pack.pream_l=UART_PREAMBLE_L;
    uart_pack.sid=UART_SID;
    uart_pack.cmd.b.rw=UART_CMD_R;
    if(read_len <=15){
        uart_pack.cmd.b.len = read_len;
    }else{
        uart_pack.cmd.b.len = 0;
    }
    uart_pack.adr_h=(adr>>8);
    uart_pack.adr_l=(UCHAR)adr;
    ur_send_data((UCHAR*)&uart_pack,SIZEOF(uart_read_s));
    SystemSleep(READ_DELAY_TIME_MS);
}


STATIC VOID uart_send_write_cmd(IN USHORT adr,IN UCHAR write_data)
{
    uart_write_s write_word={0};
    write_word.pream_h = UART_PREAMBLE_H;
    write_word.pream_l = UART_PREAMBLE_L;
    write_word.sid = UART_SID;
    write_word.cmd.b.rw = UART_CMD_W;
    write_word.cmd.b.len = 1;
    write_word.adr_h = (adr>>8);
    write_word.adr_l = (UCHAR)adr;
    write_word.data = write_data;
    ur_send_data((UCHAR*)&write_word,SIZEOF(uart_write_s));
    SystemSleep(WRITE_DELAY_TIME_MS);
    if(adr == ADR_RW_LONG_LEN){
        SystemSleep(180);
    }
}

STATIC VOID uart_send_write_array(IN USHORT adr,IN UCHAR* pwrite,IN UCHAR write_len)
{
    UCHAR write_array[15+6] = {0};
    UCHAR len = 0;
    write_array[len++] = UART_PREAMBLE_H;
    write_array[len++] = UART_PREAMBLE_L;
    write_array[len++] = UART_SID;
    write_array[len++] = 0x40 | (write_len<=15 ? write_len : 0);
    write_array[len++] = (adr>>8);
    write_array[len++] = (UCHAR)adr;
    UCHAR i;
    for(i=0;i<write_len;i++){
        write_array[len++] = pwrite[i];
    }
    ur_send_data(write_array,len);
    SystemSleep(WRITE_DELAY_TIME_MS);
}



STATIC UCHAR *p_relay_state;
VOID uart_ctrl_relay(UCHAR relay)
{

    if(relay){
        if(zero_mode == ZERO_MODE_NORMAL){
            uart_send_write_cmd(ADR_RELAY, 0x7C);
        }else{
            tuya_set_led_light_type(modu_relay_handle, OL_HIGH, 0,0);
        }
    }else{
        if(zero_mode == ZERO_MODE_NORMAL){
            uart_send_write_cmd(ADR_RELAY, 0x78);
        }else{
            tuya_set_led_light_type(modu_relay_handle, OL_LOW, 0,0);
        }
    }


#if 0
    MutexLock(uart_proc.receive_mutex);
    
    uart_send_read_cmd(ADR_RELAY, 1);
    BOOL read_ret = uart_recieve_data_proc(1, p_relay_state, 50);
    MutexUnLock(uart_proc.receive_mutex);
    if(read_ret){
        PR_DEBUG("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!read relay state:%2x",*p_relay_state);
    }else{
        reset_uart_proc_buf();
        PR_ERR("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!read relay state fail");
    }
#endif

}





STATIC OPERATE_RET ur_send_data(UCHAR *data,UCHAR len)
{
    if(((NULL == data) && len != 0) || \
            ((data) && len == 0)) {
        return OPRT_INVALID_PARM;
    }

#if UART_TX_DEBUG
    {
        PR_DEBUG_RAW("TX:\n");
        int i;
        for(i = 0;i < len;i++) {
            PR_DEBUG_RAW("%02X ",data[i]);
            if(i && ((i % 20) == 0)) {
                PR_DEBUG_RAW("\n");
            }
        }
        PR_DEBUG_RAW("\n");
    }
#endif
    MutexLock(uart_proc.write_mutex);
    ty_uart_send_data(TY_UART0,(UCHAR_T *)data,len);
    MutexUnLock(uart_proc.write_mutex);

    return OPRT_OK;


}


VOID get_ele_par(OUT UINT *P,OUT UINT *V,OUT UINT *I)
{
    MutexLock(piv_mutex);
    *P = runingInf.power;
    *V = runingInf.voltage;
    *I = runingInf.electricity;
    MutexUnLock(piv_mutex);
}

VOID get_ele(OUT UINT *add_energy)
{
    MutexLock(E_mutex);
    *add_energy = runingInf.addEnergy;
    PR_DEBUG(" get adde is %d  ", runingInf.addEnergy);
    runingInf.addEnergy = 0;
    MutexUnLock(E_mutex);
}

UCHAR CRC8_Calculation(UCHAR* ptr_crc, UCHAR len)
{
    UCHAR i;
    UCHAR crc=0;

    crc=0;
    while(len--){

        for(i=0x80;i!=0;i>>=1){
            if((crc&0x80)!=0){
                crc <<=1;
                crc^=0x07;
            }else
                crc <<=1;
            if((*ptr_crc&i)!=0)
                crc^=0x07;
        }
        ptr_crc++;
    }
    return(crc);
}



STATIC OPERATE_RET set_zero_data_val()
{
	OPERATE_RET op_ret;
    cJSON *root = NULL;
	UCHAR *buf = NULL;

    root = cJSON_CreateObject();
    if(NULL == root) {
		PR_ERR("cJSON_CreateObject error");
		return OPRT_CJSON_GET_ERR;
	}
    zero_data = ((int)zero_on_l<<24) | ((int)zero_on_m<<16) |\
    ((int)zero_off_l<<8) | ((int)zero_off_m);
    PR_DEBUG("frq:%d,onl:0x%x,onm:0x%x,offl:0x%x,offm:0x%x,zero:0x%8x",\
    zero_frq,zero_on_l,zero_on_m,zero_off_l,zero_off_m,zero_data);
    PR_DEBUG("zero data:%8x",zero_data);
    cJSON_AddNumberToObject(root, "zero", zero_data);
    cJSON_AddNumberToObject(root, "frq", cur_frq);
    buf = cJSON_PrintUnformatted(root);
    if(NULL == buf) {
        PR_ERR("buf is NULL");
        cJSON_Delete(root);
        return OPRT_COM_ERROR;
    }    
    cJSON_Delete(root);
    
    op_ret = wd_common_write(ZERO_DATA_SAVE_KEY,buf, strlen(buf));
	if(OPRT_OK != op_ret) {
		PR_DEBUG("msf_set_zero_data err:%d",op_ret);
        Free(buf);
		return op_ret;
	}
    
    Free(buf);
    return OPRT_OK;    
}

STATIC OPERATE_RET get_zero_data_val()
{
	OPERATE_RET op_ret;
    cJSON *root = NULL, *json = NULL;
	UCHAR *buf = NULL;

    UINT_T buf_len;
	op_ret = wd_common_read(ZERO_DATA_SAVE_KEY,&buf,&buf_len);
	if(OPRT_OK != op_ret) {
		PR_DEBUG("msf_get_zero_data err:%d",op_ret);
        Free(buf);
		return op_ret;
	}
	PR_DEBUG("get zero_data buf:%s",buf);
	root = cJSON_Parse(buf);
	if(NULL == root) {
		PR_ERR("cjson parse");
        goto JSON_PARSE_ERR;
	}

    json = cJSON_GetObjectItem(root,"zero");
    if(NULL == json) {
        PR_ERR("cjson get zero_data err");
        goto JSON_PARSE_ERR;
	}else if(json->type == cJSON_Number){
        zero_data = json->valueint;
	}
    json = cJSON_GetObjectItem(root,"frq");
    if(NULL == json) {
        PR_ERR("cjson get zero_frq err");
        goto JSON_PARSE_ERR;
	}else if(json->type == cJSON_Number){
        zero_frq = json->valueint;
	}


    zero_off_m = (UCHAR)(zero_data);
    zero_off_l = (UCHAR)(zero_data>>8);
    zero_on_m = (UCHAR)(zero_data>>16);
    zero_on_l = (UCHAR)(zero_data>>24);
    PR_DEBUG("frq:%d,onl:0x%x,onm:0x%x,offl:0x%x,offm:0x%x,zero:0x%8x",\
    zero_frq,zero_on_l,zero_on_m,zero_off_l,zero_off_m,zero_data);
    cJSON_Delete(root);
    Free(buf);
    return OPRT_OK;

JSON_PARSE_ERR:
    cJSON_Delete(root);
    Free(buf);
    return OPRT_COM_ERROR;
}



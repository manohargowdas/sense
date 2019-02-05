#include "C:\Work\Sensorise\Product\SenseTise\FW\STM32\Code\Build\CFD\MDK-ARM\define.h"

//extern void MX_USART1_UART_Init(void);

#include "math.h"
//0x0800F000
#define FLASH_USER_START_ADDR  ((uint32_t)0x0801f000) //ADDR_FLASH_PAGE_16   /* Start @ of user Flash area */
static FLASH_EraseInitTypeDef EraseInitStruct;
uint32_t Address = 0, PageError = 0;
//#define FLASH_USER_START_ADDR_KM ((uint32_t)0x0801ec00)

unsigned char sms_aunthentication,latch_0_counter,inactive_button;
unsigned char send_fix_tiemr,which_summary;
unsigned char mode_value,latch_op1;
unsigned int wake_up_hour_counter;
unsigned int autotest_deactivate_time=0,su_deactivate_time=0;
unsigned int gsm_reset_counter;
unsigned long ten_minute_counter,batdata_minute_counter;
unsigned char get_iccid_bit,bypass_wdt,power_on_gsm;
unsigned char	send_command,eng_read;
unsigned char restart_gsm,qintz_bit;
unsigned char idea_apn[20];
unsigned char voda_apn[20];
unsigned char sense_apn[20];
unsigned char bsnl_apn[20];
unsigned char mtnl_apn[20];
unsigned char airtel_apn[20];
unsigned char aircel_apn[20];
unsigned char custom_apn[20];
unsigned char useCustomAPN=0;
unsigned char hap1 = 0,ok1 = 0,sad1= 0;
unsigned int  but_press_dly = 0;
unsigned char operator_name[15];
unsigned char	fota_update_bit,get_iccid_value;
unsigned long wakeup_hours;
unsigned char firm_ver[8];
unsigned char ctr = 0;
unsigned char eol =0;
unsigned char read_clock_bit,connect_http,power_ip;
unsigned char iccid_no[25];
unsigned char get_ip_local;
unsigned char local_ip_address[30];
unsigned char loacal_ip_calculated[30];
unsigned char local_ip_error;
unsigned char error_bit;
unsigned char get_operator_bit;
unsigned long currentdatetime=0;
unsigned char mcc[10];
unsigned char mnc[10];
unsigned char lac[10];
unsigned char cell_id[10];

unsigned char gsm_signal[5];
unsigned long temp_long,current_date_sequence_button,current_date_sequence_helth,current_date_sequence_summarycurrent_date_sequence_monitering=0;
unsigned int time_delay;
unsigned char gsm_tx_buf[500];
unsigned int inc_ctr;
unsigned char apn[20];
unsigned char apn_mem[20];

unsigned char phone_no_admin[12];
unsigned char rcvd_data2[2];

unsigned char time_ist[8];
unsigned char cell_id[10];
unsigned char date_ist[10];//="556677,";
unsigned char imei[18];
unsigned char gsm_rx_dt[2500];
unsigned char get_imei_bit;//,temp_char;
unsigned int temp_char;
unsigned char call_ready,sms_ready;
unsigned int one_sec_counter,time_out_ctr;
unsigned char restart_counter;
unsigned char ok_gsm,UartReady;
unsigned int rcv_ctr2;
unsigned char sms_recieved,network,time_read;
unsigned char ascii_number[18]={0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,'A','B','C','D','E','F'};//inc 16 to 18

unsigned char read_network_bit,message_update_counter;
unsigned char read_signal_counter;
unsigned int one_sec_counter1,check_alert_ctr;

unsigned char cpin_check;
unsigned char gsm_blink_counter,booting;
unsigned int halt_ms;
unsigned int loc_counter;

unsigned char cell_info_count,get_iccid_counter,read_network_counter;

//-------------------------------------------
unsigned int time_out;
unsigned int main_reset_counter;
//-------------------------------------------
extern UART_HandleTypeDef huart3;
unsigned char load_default_bit;


//-----------------------------manage_in memory-----------------------
unsigned char no_of_power_outage,ldr_time,su_mode_act_cnt;
unsigned int good_button_count,ok_button_count,bad_button_count,heart_beat;

unsigned char no_of_power_outage_o,ldr_time_o,su_mode_act_cnt_o;
unsigned int good_button_count_o,ok_button_count_o,bad_button_count_o,heart_beat_o;

unsigned char last_power_outage_time[4];
unsigned char last_su_mode_time[4];
unsigned char last_ldr_time[4];
unsigned char last_heart_beat_time[4];
unsigned char last_button_press_record_time[6];

unsigned char last_power_outage_time_o[4];
unsigned char last_su_mode_time_o[4];
unsigned char last_ldr_time_o[4];
unsigned char last_heart_beat_time_o[4];
unsigned char last_button_press_record_time_o[6];

//-----------------------------------manage_in memory

// Find Operator
void find_operator()
{
    if(power_on_gsm==0) return;

     operator_name[0]='N';
     operator_name[1]='o';
     operator_name[2]='n';
     operator_name[3]='e';
     operator_name[4]='\0';
     operator_name[5]='\0';
     operator_name[6]='\0';
     operator_name[7]='\0';
     cell_id[0] = '0';
     cell_id[1] = '0';
     cell_id[2] = '0';
     cell_id[3] = '0';
     cell_id[4] = '\0';
     lac[0] = '0';
     lac[1] = '0';
     lac[2] = '0';
     lac[3] = '0';
     lac[4] = '\0';
     mcc[0] = '0';
     mcc[1] = '0';
     mcc[2] = '0';
     mcc[3] = '0';
     mcc[4] = '\0';
     mcc[0] = '0';
     mnc[1] = '0';
     mnc[2] = '0';
     mnc[3] = '0';
     mnc[4] = '\0';

    send_command=0;

    send_gsm("at\r");ok_check(50); //  5 Sec
    send_gsm("AT+COPS=3,0\r");ok_check(750);  // 75 Sec

    send_gsm("at\r");ok_check(50); //  5 Sec
    get_operator_bit = 1;
    send_gsm("AT+COPS?\r");ok_check(750);  // 75 Sec
    if ((get_operator_bit != 0)&&(error_bit==0)) {
        time_delay = 30;
        while ((time_delay>0) && (get_operator_bit !=0)) {HAL_Delay(100); --time_delay;}
        get_operator_bit = 0;
    }
    mno = 0;
    for (time_delay = 0; (time_delay < 12) && (operator_name[time_delay] != '\0'); time_delay++) {
        if (((operator_name[time_delay]=='I') || (operator_name[time_delay]=='i')) &&
            ((operator_name[time_delay+1]=='D') || (operator_name[time_delay+1]=='d')) &&
            ((operator_name[time_delay+2]=='E') || (operator_name[time_delay+2]=='e')) &&
            ((operator_name[time_delay+3]=='A') || (operator_name[time_delay+3]=='a'))) mno = 1 ; // Idea
        if (((operator_name[time_delay]=='V') || (operator_name[time_delay]=='v')) &&
            ((operator_name[time_delay+1]=='O') || (operator_name[time_delay+1]=='o')) &&
            ((operator_name[time_delay+2]=='D') || (operator_name[time_delay+2]=='d')) &&
            ((operator_name[time_delay+3]=='A') || (operator_name[time_delay+3]=='a'))) mno = 2 ; // Vodafone
        if (((operator_name[time_delay]=='A') || (operator_name[time_delay]=='a')) &&
            ((operator_name[time_delay+1]=='I') || (operator_name[time_delay+1]=='i')) &&
            ((operator_name[time_delay+2]=='R') || (operator_name[time_delay+2]=='r')) &&
            ((operator_name[time_delay+3]=='T') || (operator_name[time_delay+3]=='t'))) mno = 3 ; // Airtel
        if (((operator_name[time_delay]=='A') || (operator_name[time_delay]=='a')) &&
            ((operator_name[time_delay+1]=='I') || (operator_name[time_delay+1]=='i')) &&
            ((operator_name[time_delay+2]=='R') || (operator_name[time_delay+2]=='r')) &&
            ((operator_name[time_delay+3]=='C') || (operator_name[time_delay+3]=='c'))) mno = 4 ; // Aircel
        if (((operator_name[time_delay]=='B') || (operator_name[time_delay]=='b')) &&
            ((operator_name[time_delay+1]=='S') || (operator_name[time_delay+1]=='s')) &&
            ((operator_name[time_delay+2]=='N') || (operator_name[time_delay+2]=='n')) &&
            ((operator_name[time_delay+3]=='L') || (operator_name[time_delay+3]=='l'))) mno = 5 ; // BSNL
        if (((operator_name[time_delay]=='M') || (operator_name[time_delay]=='m')) &&
            ((operator_name[time_delay+1]=='T') || (operator_name[time_delay+1]=='t')) &&
            ((operator_name[time_delay+2]=='N') || (operator_name[time_delay+2]=='n')) &&
            ((operator_name[time_delay+3]=='L') || (operator_name[time_delay+3]=='l'))) mno = 6 ; // MTNL
        if (mno != 0) break;
    } // for
}


void select_apn()
{

    for (temp_char = 0 ; temp_char < 20; temp_char++) {
        if (useCustomAPN ==1) apn[temp_char] = custom_apn[temp_char];
        else if (qosim == 1) apn[temp_char] = sense_apn[temp_char];
        else if (mno == 1) apn[temp_char] = idea_apn[temp_char];
        else if (mno == 2) apn[temp_char] = voda_apn[temp_char];
        else if (mno == 3) apn[temp_char] = airtel_apn[temp_char];
        else if (mno == 4) apn[temp_char] = aircel_apn[temp_char];
        else if (mno == 5) apn[temp_char] = bsnl_apn[temp_char];
        else if (mno == 6) apn[temp_char] = mtnl_apn[temp_char];
        else apn[temp_char] = sense_apn[temp_char];

        if (apn_mem[temp_char] != apn[temp_char]) done = 2;
        apn_mem[temp_char] = apn[temp_char];
    }

}


/* ----------------------
 * Sends the command to the GSM module
 *
*/
void send_gsm(unsigned char *q)
{
    UartReady= RESET;
    for(inc_ctr=0;*q!='\0';q++,inc_ctr++)
    {
        gsm_tx_buf[inc_ctr]=*q;
        gsm_tx_buf[inc_ctr+1]='\0';
    }
    ok_gsm = 0;  // Set to 1 in RxCallback handler of UART
    error_bit = 0;
    HAL_UART_Transmit_IT(&huart1,gsm_tx_buf,inc_ctr);
    time_out_ctr=0;
    while((UartReady!=SET)&&(time_out_ctr<1000))  // Waiting for USART to become free again
    {delay_ms(2);++time_out_ctr;}
}

/* -----------------------------------------------
Checks if GSM Modulehas returned ok status
Wait time is maximum of 2.50 seconds.
*/
void ok_check(unsigned int tout)
{
    time_delay=tout;
    while((time_delay>0)&&(ok_gsm==0) && (error_bit==0))
    {
        HAL_Delay(100);--time_delay;
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    }

}

/* -----------------------------------------------
Checks if GSM Modulehas returned ok status for the
HTTP request. Wait time is maximum of 10 seconds.
*/
void ok_check_http()
{
    time_delay=250;ok_gsm=0;error_bit=0;
    while((time_delay)&&(ok_gsm==0) && (error_bit==0))  
    {
        HAL_Delay(100);--time_delay;
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    }
	
}


// Get IMEI Number
void get_imei()
{
    get_imei_bit=1;
    // Send IMEI request
    send_gsm("at+gsn\r");ok_check(30);
    if (get_imei_bit != 0) {
        // Wait for IMEI requet to be completed
        while((time_delay>0)&&(get_imei_bit!=0))
        {HAL_Delay(10);--time_delay;}
        get_imei_bit = 0;
    }
}


// Fires every millisecond
void HAL_SYSTICK_Callback(void)
{
    if(bypass_wdt==0) HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);  // Do not refresh . This will result in reboot
    if((booting==0)&&(test_bit < 2))
    {
        if((inactive_button==0) && (latch_op1 ==0)) // There is no already pressed button
        {
            if ((hap1==0)&&(HAL_GPIO_ReadPin(ip2_port,ip2_pin)==SET)) {
                hap1 = 1;
                if (but_press_dly ==0) but_press_dly =1;
            }
            if ((ok1==0)&&(HAL_GPIO_ReadPin(ip1_port,ip1_pin)==SET)) {
                ok1 = 1;
                if (but_press_dly ==0) but_press_dly =1;
            }
            if ((sad1==0)&&(HAL_GPIO_ReadPin(ip3_port,ip3_pin)==SET)) {
                sad1 = 1;
                if (but_press_dly ==0) but_press_dly =1;
            }
        }

    }

    if (but_press_dly >= 500) {
        inactive_button=1;
        // 500 Mil Seconds since first press so the press should be complete
        if ((hap1==1) && (ok1==0) && (sad1==0)) {
            // Happy button press
            op1_1;latch_op1=1;++happy;hap1 = 0; ok1 = 0; sad1= 0;
            latch_0_counter=latch_0_counter_value;
            if (su_mode_bit <= 2) su_mode_bit = 0;
        }
        if ((hap1==0) && (ok1==1) && (sad1==0)) {
            // OK button press
            ++ok; op1_1;latch_op1=1;hap1 = 0; ok1 = 0; sad1= 0;
            latch_0_counter=latch_0_counter_value;
            if (su_mode_bit <= 2) su_mode_bit = 0;
        }
        if ((hap1==0) && (ok1==0) && (sad1==1)) {
            //Sad button press
            ++sad; op1_1;latch_op1=1;hap1 = 0; ok1 = 0; sad1= 0;
            latch_0_counter=latch_0_counter_value;
            if (su_mode_bit <= 2) su_mode_bit = 0;
        }
        if ((hap1==1) && (ok1==1) && (sad1==1)) {
            //Test mode enabled
            test_bit = 1;
            autotest_deactivate_time=auto_test_deact_time;
        }
        if ((su_mode_bit==2) && (hap1==1) && (ok1==0) && (sad1==1)) {
            //SU Mode activated
            su_mode_bit = 3;
            su_deactivate_time = 1800;  // Auto exit time of 30 minutes
            mode_value = 10;  // SU Activated
        }
        if ((hap1==0) && (ok1==1) && (sad1==1)&& ((su_mode_bit==0) || (su_mode_bit==4))) {
            // SU mode enabled
            if (su_mode_bit==0)
                su_mode_bit = 1;
            else {
                su_mode_bit=5;  // exit SU mode
                su_deactivate_time = 0;
                mode_value = 11;  // Manual exist
            }
        }
        if((HAL_GPIO_ReadPin(ip3_port,ip3_pin)==RESET)&&(HAL_GPIO_ReadPin(ip2_port,ip2_pin)==RESET)&&(HAL_GPIO_ReadPin(ip1_port,ip1_pin)==RESET))
        {   inactive_button=0;
            but_press_dly= 0;
            hap1 = 0; ok1 = 0; sad1= 0;
            if (su_mode_bit == 3) su_mode_bit = 4;
            if (su_mode_bit == 1) su_mode_bit = 2;
            if (test_bit == 1) test_bit = 2;
        }
    }

    if (but_press_dly > 0) ++but_press_dly;
    if(latch_op1==0) {op1_0;}


    ++one_sec_counter;  // Increment millcecond timer

    // Tasks to be done every 0.5 sec (500 millisecond)
    if(one_sec_counter==500)
    {
        if(((network==1)||(network==5))&&(gsm_blink_counter==1))
                led_gsm_0;// sigle blink if gsm ok
        if((local_ip_error!=0)&&(gsm_blink_counter==2)&&((network==1)||(network==5)))
                led_gsm_0;//double blink if gprs not ok

    } // End of 500 millisecond processing

    if(one_sec_counter>=1000) //--------------------------------------------one sec
    {
        ++main_reset_counter;
        if(main_reset_counter>180)
        {   bypass_wdt=1;
            RebootDevice();
        }

        ++currentdatetime;
        ++wake_up_hour_counter;
        if(wake_up_hour_counter>3600) {++wakeup_hours;wake_up_hour_counter=0;}

        // Sending Health data counters and bit
        if(send_fix_tiemr>1) --send_fix_tiemr;


        if(latch_0_counter>0) --latch_0_counter;
        if(latch_0_counter==0) latch_op1=0; // Turn off LED

        if((gsm_reset_counter>0) && (power_on_gsm==1)) --gsm_reset_counter;  // 2 hour RESET GSM and try ending data again

        if(ten_minute_counter) --ten_minute_counter;   // Send the data after 10 minutes
        if(batdata_minute_counter) --batdata_minute_counter;   // Send the data after 30 minutes on Battery

        // Automatically come out of test mode
        if(autotest_deactivate_time > 0) --autotest_deactivate_time;
        if(autotest_deactivate_time==1) {test_bit=0; mode_value=9;} // Exit test mode and Send Alert that Test Mode exited automatically

        // Automatically come out of SU mode
        if(su_deactivate_time > 0) --su_deactivate_time;
        if(su_deactivate_time==1) {su_mode_bit=5; mode_value=12;} // Exit test mode and Send Alert that SU Mode exited automatically

        if(device_restart_counter) --device_restart_counter;  // Reboot device every 6 hours

        // Get cell info every 90 seconds
        ++cell_info_count;
        if ((power_ip==1) && (cell_info_count>=90)) {cell_info_count=0;send_command=1;}
        if ((power_ip==0) && (cell_info_count>=180)) {cell_info_count=0;send_command=1;}

        // Get SIM detail evey 3 minutes
        ++get_iccid_counter;
        if ((power_ip==1) && (get_iccid_counter>=180)) {get_iccid_value=1;get_iccid_counter=0;}
        if ((power_ip==0) && (get_iccid_counter>=300)) {get_iccid_value=1;get_iccid_counter=0;}

        led_power_0;
        if(booting==0)
        {
            ++gsm_blink_counter;
            led_gsm_1;
            if(gsm_blink_counter>=4)gsm_blink_counter=0;
        }


        // Read Network every 20 seconds
        ++read_network_counter;
        if((power_ip==1) && (read_network_counter>=20)) {read_network_counter=0;read_network_bit=1; read_clock_bit=1;}
        if((power_ip==0) && (read_network_counter>=60)) {read_network_counter=0;read_network_bit=1; read_clock_bit=1;}

        // Read clock and Battery level every 10 seconds
        if((power_ip==1) && (read_network_counter==10)) {read_clock_bit=1;}
        if((power_ip==0) && (read_network_counter==30)) {read_clock_bit=1;}

        //Read battery level & LDRs
        read_adc_bit=1;

        // LDR2 Counter
        if ((box_open_bit > 0) && (ldr_event_gap >0)) ++box_open_bit;
        if ((box_open_bit > ldr_event_gap) && (ldr_event_gap >0)) box_open_bit = 0;

        // LDR1 Counter
        if ((box_removed_bit > 0) && (ldr_event_gap >0)) ++box_removed_bit;
        if ((box_removed_bit > ldr_event_gap) && (ldr_event_gap >0)) box_removed_bit = 0;

        // Read Signal every 30 seconds
        ++read_signal_counter;
        if ((power_ip==1) && (read_signal_counter>=30)){read_signal_counter=0; read_signal_bit=1;}
        if ((power_ip==0) && (read_signal_counter>=90)){read_signal_counter=0; read_signal_bit=1;}

        if (monitoring_send_data_counter > 0) --monitoring_send_data_counter;

        if (health_Data_Timer > 0) --health_Data_Timer;

        if (summary_wait_time > 1) --summary_wait_time;
        if (write_flash_timer > 0) --write_flash_timer; // Write into flash

        one_sec_counter=0;
    } //-------------------------------------one sec
}
	

void read_local_ip()
{
    if (power_on_gsm == 0) return;
    send_gsm("AT\r"); ok_check(30); // Init
    get_ip_local = 1;
    local_ip_address[0]='x';
    send_gsm("AT+QILOCIP\r"); ok_check(50); // Get Local IP Address
    // Wait
    if (get_ip_local != 0) {
        time_delay = 30;
        while((time_delay>0)&&(get_ip_local != 0))
        {HAL_Delay(100);--time_delay;}
    }

    if((local_ip_address[0]=='x')||(local_ip_address[0]<0x30)||(local_ip_address[0]>0x39))
    {local_ip_error=1;}
    else
    {local_ip_error=0;}

    if(local_ip_error==0)
    {

        for(temp_char=0;((local_ip_address[temp_char]!='\0')&&(local_ip_address[temp_char]!='.'));++temp_char)
        {}
        //------------------------------------------------------
        loacal_ip_calculated[2]=local_ip_address[temp_char-1];
        //------------------------------------------------------
        if((local_ip_address[temp_char-2]>=0x30)&&(local_ip_address[temp_char-2]<=0x39))
                loacal_ip_calculated[1]=local_ip_address[temp_char-2];
        else
                loacal_ip_calculated[1]='0';
            //------------------------------------------------------
        if((local_ip_address[temp_char-3]>=0x30)&&(local_ip_address[temp_char-3]<=0x39))
                loacal_ip_calculated[0]=local_ip_address[temp_char-3];
        else
                loacal_ip_calculated[0]='0';
                    //------------------------------------------------------

        ++temp_char;
        for(;((local_ip_address[temp_char]!='\0')&&(local_ip_address[temp_char]!='.'));++temp_char)
        {}

        loacal_ip_calculated[5]=local_ip_address[temp_char-1];
        //------------------------------------------------------
        if((local_ip_address[temp_char-2]>=0x30)&&(local_ip_address[temp_char-2]<=0x39))
                loacal_ip_calculated[4]=local_ip_address[temp_char-2];
        else
                loacal_ip_calculated[4]='0';
            //------------------------------------------------------
        if((local_ip_address[temp_char-3]>=0x30)&&(local_ip_address[temp_char-3]<=0x39))
                loacal_ip_calculated[3]=local_ip_address[temp_char-3];
        else
                loacal_ip_calculated[3]='0';
                    //------------------------------------------------------
                                                                            //------------------------------------------------------

        ++temp_char;
        for(;((local_ip_address[temp_char]!='\0')&&(local_ip_address[temp_char]!='.'));++temp_char)
        {}

        loacal_ip_calculated[8]=local_ip_address[temp_char-1];
        //------------------------------------------------------
        if((local_ip_address[temp_char-2]>=0x30)&&(local_ip_address[temp_char-2]<=0x39))
                loacal_ip_calculated[7]=local_ip_address[temp_char-2];
        else
                loacal_ip_calculated[7]='0';
            //------------------------------------------------------
        if((local_ip_address[temp_char-3]>=0x30)&&(local_ip_address[temp_char-3]<=0x39))
                loacal_ip_calculated[6]=local_ip_address[temp_char-3];
        else
                loacal_ip_calculated[6]='0';
                    //------------------------------------------------------

        ++temp_char;
        for(;((local_ip_address[temp_char]!='\0')&&(local_ip_address[temp_char]!='.'));++temp_char)
        {}

        loacal_ip_calculated[11]=local_ip_address[temp_char-1];
        //------------------------------------------------------
        if((local_ip_address[temp_char-2]>=0x30)&&(local_ip_address[temp_char-2]<=0x39))
                loacal_ip_calculated[10]=local_ip_address[temp_char-2];
        else
                loacal_ip_calculated[10]='0';
            //------------------------------------------------------
        if((local_ip_address[temp_char-3]>=0x30)&&(local_ip_address[temp_char-3]<=0x39))
                loacal_ip_calculated[9]=local_ip_address[temp_char-3];
        else
                loacal_ip_calculated[9]='0';
                    //------------------------------------------------------
    }

}


// This does basic initialization of GS module
void init_gsm()
{

    call_ready=0,sms_ready=0; temp_char=0;
    hw_gsm_power_off; HAL_Delay(500);
    hw_gsm_power_on; HAL_Delay(500);
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    power_on_gsm=1;
    gsm_power_key_0; HAL_Delay(500);
    gsm_power_key_0; HAL_Delay(500);
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

    send_gsm("at\r"); ok_check(4);
    time_delay=40;
    send_gsm("ate1\r");ok_check(40);

    network=0;
    cpin_check=1;                           // allow call ready check
    send_gsm("at+cpin?\r");ok_check(60);    // max 6  seconds wait
    if (cpin_check==1){
        time_delay=50;
        while((cpin_check==1)&&(time_delay>0))
        {
            HAL_Delay(100);--time_delay;
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
        }
    }

    qintz_bit=0;
    send_gsm("at+qnitz=1\r"); ok_check(10);
    if (qintz_bit == 0) {
        time_delay=30;
        while((qintz_bit==0)&&(time_delay>0))
        {
            HAL_Delay(10);--time_delay;
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
        }
    }

    send_gsm("at+ctzu=3\r");ok_check(30);
    send_gsm("at+cmgf=1\r");ok_check(30);//rpt
    send_gsm("at+clip=1\r");ok_check(30);
    send_gsm("at+cnmi=2,1,0,0,0\r");ok_check(30);
    send_gsm("AT+QIMUX=0\r");ok_check(30);
    send_gsm("AT+QIMODE=0\r");ok_check(30);
    send_gsm("AT+QSTK=0\r\n");ok_check(30); //-----sim tool kit activation
    main_reset_counter=0;
    ok_gsm = 0;
    error_bit = 0;
    HAL_Delay(100);
    time_delay = 100;
    while((call_ready==0) &&(time_delay >0)){
        --time_delay; HAL_Delay(100);
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    }
    time_delay = 100;
    while((sms_ready==0) &&(time_delay >0)){
        --time_delay; HAL_Delay(100);
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    }
    send_gsm("at+csmp=49,167,0,241\r");ok_check(30);
    send_gsm("at&w\r");ok_check(30);
    gsm_reset_counter=gsm_reset_counter_value;
}

//-------------------------------------------------------------------

// Update teh configuration variables received over SMS
void update_values()
{ 
    unsigned int location;

    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);
    loc_counter=6;temp_char=0;sms_aunthentication=0;
    for(temp_char=0;((temp_char<250)&&(gsm_rx_dt[temp_char]!=','));++temp_char)
    {}
    temp_char+=4;
    loc_counter=temp_char;

    // Is SMS from Valid number
    if((gsm_rx_dt[loc_counter]=='-')&&(gsm_rx_dt[loc_counter+1]=='D')&&(gsm_rx_dt[loc_counter+2]=='I')&&(gsm_rx_dt[loc_counter+3]=='G')&&(gsm_rx_dt[loc_counter+4]=='I')&&(gsm_rx_dt[loc_counter+5]=='T')&&(gsm_rx_dt[loc_counter+6]=='L'))
         sms_aunthentication=1;

    if((gsm_rx_dt[loc_counter]=='-')&&(gsm_rx_dt[loc_counter+1]=='1')&&(gsm_rx_dt[loc_counter+2]=='2')&&(gsm_rx_dt[loc_counter+3]=='3')&&(gsm_rx_dt[loc_counter+4]=='4')&&(gsm_rx_dt[loc_counter+5]=='5'))
        sms_aunthentication=1;

    if((gsm_rx_dt[loc_counter+1]==phone_no_admin[0])&&(gsm_rx_dt[loc_counter+2]==phone_no_admin[1])&&(gsm_rx_dt[loc_counter+3]==phone_no_admin[2])&&(gsm_rx_dt[loc_counter+4]==phone_no_admin[3])&&(gsm_rx_dt[loc_counter+5]==phone_no_admin[4])&&(gsm_rx_dt[loc_counter+6]==phone_no_admin[5])&&(gsm_rx_dt[loc_counter+7]==phone_no_admin[6])&&(gsm_rx_dt[loc_counter+8]==phone_no_admin[7])&&(gsm_rx_dt[loc_counter+9]==phone_no_admin[8])&&(gsm_rx_dt[loc_counter+10]==phone_no_admin[9]))
        sms_aunthentication=1;

    if(sms_aunthentication==1)
    {

        temp_char=0;//done=0;
      //  loc_counter=0;

        while(loc_counter<=120)//
        {
            //------------------------------------------------------------------------------------------------------
            // Read the content
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);
            if(gsm_rx_dt[loc_counter]=='#'){} // Skip it

            // #apn <APN>
            if((gsm_rx_dt[loc_counter]=='a')&&(gsm_rx_dt[loc_counter+1]=='p')&&(gsm_rx_dt[loc_counter+2]=='n'))
            {
                location=0;loc_counter+=4;
                for(;((gsm_rx_dt[loc_counter]!=' ')&&(location<20)&&(gsm_rx_dt[loc_counter]!=0x0d));++loc_counter)
                {
                    custom_apn[location]=gsm_rx_dt[loc_counter];
                    custom_apn[location+1]='\0';++location;
                }
                HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);
                useCustomAPN = 1;
                done = 2;
                select_apn();
                send_gsm("at+qiclose\r");ok_check(20);
                send_gsm("at+qideact\r");ok_check(100);
                activate_gprs();
                read_local_ip();
                return;
            }

            location=0;		HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);

            //#ipf  xxx.xxx.xxx.xxx xxxx
            if((gsm_rx_dt[loc_counter]=='i')&&(gsm_rx_dt[loc_counter+1]=='p')&&(gsm_rx_dt[loc_counter+2]=='f'))
            {
                    loc_counter+=3;
                    for(;((gsm_rx_dt[loc_counter]!=' ')&&(location<20)&&(gsm_rx_dt[loc_counter]!=0x0d));++loc_counter)
                    {
                        fota_ip[location]=gsm_rx_dt[loc_counter];
                        fota_ip[location+1]='\0';++location;
                    }
                    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);
                    ++loc_counter;location=0;
                    done =2;//send_sms();
                    return;
            }

            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);

            // #url1 <url>  -- DataURL
            if((gsm_rx_dt[loc_counter]=='u')&&(gsm_rx_dt[loc_counter+1]=='r')&&(gsm_rx_dt[loc_counter+2]=='l')&&(gsm_rx_dt[loc_counter+3]=='d')&&(gsm_rx_dt[loc_counter+4]=='a')&&(gsm_rx_dt[loc_counter+5]=='t')&&(gsm_rx_dt[loc_counter+6]=='a'))
            {
                    loc_counter+=8;
                    for(;((gsm_rx_dt[loc_counter]!=' ')&&(location<50)&&(gsm_rx_dt[loc_counter]!=0x0d));++loc_counter)
                    {dataurl[location]=gsm_rx_dt[loc_counter];dataurl[location+1]='\0';++location;}
                    ++loc_counter;location=0;
                    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);
                    done =2;//send_sms();
                    return;
            }

            // #url2 <url>  -- alertURL
            if((gsm_rx_dt[loc_counter]=='u')&&(gsm_rx_dt[loc_counter+1]=='r')&&(gsm_rx_dt[loc_counter+2]=='l')&&(gsm_rx_dt[loc_counter+3]=='a')&&(gsm_rx_dt[loc_counter+4]=='l')&&(gsm_rx_dt[loc_counter+5]=='e')&&(gsm_rx_dt[loc_counter+6]=='r')&&(gsm_rx_dt[loc_counter+7]=='t'))
            {
                    loc_counter+=9;
                    for(;((gsm_rx_dt[loc_counter]!=' ')&&(location<50)&&(gsm_rx_dt[loc_counter]!=0x0d));++loc_counter)
                    {alerturl[location]=gsm_rx_dt[loc_counter];alerturl[location+1]='\0';++location;}
                    ++loc_counter;location=0;
                    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);
                    done =2;//send_sms();
                    return;
            }

            // Factory Setting
            // default
            if((gsm_rx_dt[loc_counter]=='d')&&(gsm_rx_dt[loc_counter+1]=='e')&&(gsm_rx_dt[loc_counter+2]=='f')&&(gsm_rx_dt[loc_counter+3]=='a')&&(gsm_rx_dt[loc_counter+4]=='u')&&(gsm_rx_dt[loc_counter+5]=='l')&&(gsm_rx_dt[loc_counter+6]=='t'))//)//{				for(loc_counter=6;(loc_counter<16);++loc_counter)
            {//#default
                load_default_bit=1;
                done =2;//send_sms();
                return;
            }

            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);

            // #ftdnl
            if((gsm_rx_dt[loc_counter]=='f')&&(gsm_rx_dt[loc_counter+1]=='t')&&(gsm_rx_dt[loc_counter+2]=='d')&&(gsm_rx_dt[loc_counter+3]=='n')&&(gsm_rx_dt[loc_counter+4]=='l'))//&&(gsm_rx_dt[loc_counter+6]=='p')&&(gsm_rx_dt[loc_counter+7]=='t'))//)//{				for(loc_counter=6;(loc_counter<16);++loc_counter)
            {//#fotaupt #ftdnl
                readed_prog_update_value=0xca;
                done = 2;
                bypass_wdt=1;
                RebootDevice();
            }

            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);

            // #restadv
            if((gsm_rx_dt[loc_counter]=='r')&&(gsm_rx_dt[loc_counter+1]=='e')&&(gsm_rx_dt[loc_counter+2]=='s')&&(gsm_rx_dt[loc_counter+3]=='t')&&(gsm_rx_dt[loc_counter+4]=='a')&&(gsm_rx_dt[loc_counter+5]=='d')&&(gsm_rx_dt[loc_counter+6]=='v'))//)//{				for(loc_counter=6;(loc_counter<16);++loc_counter)
            {//#restadv
                bypass_wdt=1;
                RebootDevice();
            }
            // #summary 1
            if((gsm_rx_dt[loc_counter]=='s')&&(gsm_rx_dt[loc_counter+1]=='u')&&(gsm_rx_dt[loc_counter+2]=='m')&&(gsm_rx_dt[loc_counter+3]=='m')&&(gsm_rx_dt[loc_counter+4]=='a')&&(gsm_rx_dt[loc_counter+5]=='r')&&(gsm_rx_dt[loc_counter+6]=='y')&&(gsm_rx_dt[loc_counter+7]==' ')&&(gsm_rx_dt[loc_counter+8]=='1'))
            {//#Send sumary
                summary_wait_time = 10;
                return;
            }
            // #summary 0
            if((gsm_rx_dt[loc_counter]=='s')&&(gsm_rx_dt[loc_counter+1]=='u')&&(gsm_rx_dt[loc_counter+2]=='m')&&(gsm_rx_dt[loc_counter+3]=='m')&&(gsm_rx_dt[loc_counter+4]=='a')&&(gsm_rx_dt[loc_counter+5]=='r')&&(gsm_rx_dt[loc_counter+6]=='y')&&(gsm_rx_dt[loc_counter+7]==' ')&&(gsm_rx_dt[loc_counter+8]=='0'))
            {//#Send sumary
                which_summary = 0;
                summary_wait_time = 10;
                return;
            }
            // #Health Packet
            if((gsm_rx_dt[loc_counter]=='h')&&(gsm_rx_dt[loc_counter+1]=='e')&&(gsm_rx_dt[loc_counter+2]=='a')&&(gsm_rx_dt[loc_counter+3]=='l')&&(gsm_rx_dt[loc_counter+4]=='t')&&(gsm_rx_dt[loc_counter+5]=='h'))
            {//#Send sumary
                send_fix_tiemr = 10;
                return;
            }

						// Admin Phone
						if((gsm_rx_dt[loc_counter]=='a')&&(gsm_rx_dt[loc_counter+1]=='d')&&(gsm_rx_dt[loc_counter+2]=='p')&&(gsm_rx_dt[loc_counter+3]=='h'))
						{
                loc_counter+=5;
                done =2;
							  location = 0;
								for(;((gsm_rx_dt[loc_counter]!=' ')&&(location<10)&&(gsm_rx_dt[loc_counter]!=0x0d));++loc_counter,++location)
									phone_no_admin[location] = gsm_rx_dt[loc_counter];
						}
						
            // APDU
            if((gsm_rx_dt[loc_counter]=='a')&&(gsm_rx_dt[loc_counter+1]=='p')&&(gsm_rx_dt[loc_counter+2]=='d')&&(gsm_rx_dt[loc_counter+3]=='u'))
            {
                loc_counter+=5;
                done =2;
                //Utility Data Count
                location = 0;
                for(;location == 0;++loc_counter)
                {
                    if ((gsm_rx_dt[loc_counter] < 0x30) || (gsm_rx_dt[loc_counter]> 0x39)) return;
                    location = (gsm_rx_dt[loc_counter]-0x30)*10; loc_counter++;
                    if ((gsm_rx_dt[loc_counter] < 0x30) || (gsm_rx_dt[loc_counter]> 0x39)) return;
                    location = location + (gsm_rx_dt[loc_counter]-0x30);
                    if (location == 0) {
                        ten_minute_counter_value = 600;
                        batdata_minute_counter_value = 5400;
                        util_dataCount = 10;
                        bat_dataCount = 0;
                        health_Timer_Value = 0;
                        device_reset_counter_value = 14 * 60 *60;
                        lowBatLevel = 361;
                        criticalBatLevel = 355;
                        ldr_event_gap = 900;
                        resend_data_freq = 10 * 60;
                    } else if (location == 1) {
                        ++loc_counter;
                        if ((gsm_rx_dt[loc_counter] < 0x30) || (gsm_rx_dt[loc_counter]> 0x39)) return;
                        util_dataCount = (gsm_rx_dt[loc_counter]-0x30)*10; loc_counter++;
                        if ((gsm_rx_dt[loc_counter] < 0x30) || (gsm_rx_dt[loc_counter]> 0x39)) return;
                        util_dataCount = util_dataCount + (gsm_rx_dt[loc_counter]-0x30);
                        if (util_dataCount == 0) util_dataCount = 1;
                        location  = 0;
                    } else if (location == 2) {
                        //Utility wait time
                        ++loc_counter;
                        if ((gsm_rx_dt[loc_counter] < 0x30) || (gsm_rx_dt[loc_counter]> 0x39)) return;
                        temp_char = (gsm_rx_dt[loc_counter]-0x30); loc_counter++;
                        if ((gsm_rx_dt[loc_counter] < 0x30) || (gsm_rx_dt[loc_counter]> 0x39)) return;
                        ten_minute_counter_value = (gsm_rx_dt[loc_counter]-0x30)*10; loc_counter++;
                        if ((gsm_rx_dt[loc_counter] < 0x30) || (gsm_rx_dt[loc_counter]> 0x39)) return;
                        ten_minute_counter_value = ten_minute_counter_value + (gsm_rx_dt[loc_counter]-0x30);
                        ten_minute_counter_value = ten_minute_counter_value * 60;
                        if (temp_char == 1) ten_minute_counter_value = ten_minute_counter_value * 60;
                        location  = 0;
                    } else if (location == 3) {
                        ++loc_counter;
                        // Battery Configuration
                        if ((gsm_rx_dt[loc_counter] < 0x30) || (gsm_rx_dt[loc_counter]> 0x39)) return;
                        bat_dataCount = (gsm_rx_dt[loc_counter]-0x30)*10; loc_counter++;
                        if ((gsm_rx_dt[loc_counter] < 0x30) || (gsm_rx_dt[loc_counter]> 0x39)) return;
                        bat_dataCount = bat_dataCount + (gsm_rx_dt[loc_counter]-0x30);
                        location  = 0;
                    } else if (location == 4) {
                        ++loc_counter;
                        if ((gsm_rx_dt[loc_counter] < 0x30) || (gsm_rx_dt[loc_counter]> 0x39)) return;
                        temp_char = (gsm_rx_dt[loc_counter]-0x30); loc_counter++;
                        if ((gsm_rx_dt[loc_counter] < 0x30) || (gsm_rx_dt[loc_counter]> 0x39)) return;
                        batdata_minute_counter_value = (gsm_rx_dt[loc_counter]-0x30)*10; loc_counter++;
                        if ((gsm_rx_dt[loc_counter] < 0x30) || (gsm_rx_dt[loc_counter]> 0x39)) return;
                        batdata_minute_counter_value = batdata_minute_counter_value + (gsm_rx_dt[loc_counter]-0x30);
                        batdata_minute_counter_value = batdata_minute_counter_value * 60;
                        if (temp_char == 1) batdata_minute_counter_value = batdata_minute_counter_value * 60;
                        location  = 0;
                    } else if (location == 5) {
                        ++loc_counter;
                        // Health Data Frequency
                        if ((gsm_rx_dt[loc_counter] < 0x30) || (gsm_rx_dt[loc_counter]> 0x39)) return;
                        health_Timer_Value = (gsm_rx_dt[loc_counter]-0x30);
                        health_Timer_Value = health_Timer_Value * 60 * 60;
                        location  = 0;
                    } else if (location == 6) {
                        ++loc_counter;
                        // Reboot Frequency
                        if ((gsm_rx_dt[loc_counter] < 0x30) || (gsm_rx_dt[loc_counter]> 0x39)) return;
                        temp_int = (gsm_rx_dt[loc_counter]-0x30); loc_counter++;
                        if ((gsm_rx_dt[loc_counter] < 0x30) || (gsm_rx_dt[loc_counter]> 0x39)) return;
                        device_reset_counter_value = (gsm_rx_dt[loc_counter]-0x30)*10; loc_counter++;
                        if ((gsm_rx_dt[loc_counter] < 0x30) || (gsm_rx_dt[loc_counter]> 0x39)) return;
                        device_reset_counter_value = device_reset_counter_value + (gsm_rx_dt[loc_counter]-0x30);
                        device_reset_counter_value = device_reset_counter_value * 60 * 60;
                        if (temp_int == 1) device_reset_counter_value = device_reset_counter_value * 24;
                        location  = 0;
                    } else if (location == 7) {
                        ++loc_counter;
                        if ((gsm_rx_dt[loc_counter] < 0x30) || (gsm_rx_dt[loc_counter]> 0x39)) return;
                        lowBatLevel = (gsm_rx_dt[loc_counter]-0x30)*100; loc_counter++;
                        if ((gsm_rx_dt[loc_counter] < 0x30) || (gsm_rx_dt[loc_counter]> 0x39)) return;
                        lowBatLevel = lowBatLevel+ (gsm_rx_dt[loc_counter]-0x30)*10; loc_counter++;
                        if ((gsm_rx_dt[loc_counter] < 0x30) || (gsm_rx_dt[loc_counter]> 0x39)) return;
                        lowBatLevel = lowBatLevel+ (gsm_rx_dt[loc_counter]-0x30);
                        location  = 0;
                    } else if (location == 8) {
                        ++loc_counter;
                        if ((gsm_rx_dt[loc_counter] < 0x30) || (gsm_rx_dt[loc_counter]> 0x39)) return;
                        criticalBatLevel = (gsm_rx_dt[loc_counter]-0x30)*100; loc_counter++;
                        if ((gsm_rx_dt[loc_counter] < 0x30) || (gsm_rx_dt[loc_counter]> 0x39)) return;
                        criticalBatLevel = criticalBatLevel+ (gsm_rx_dt[loc_counter]-0x30)*10; loc_counter++;
                        if ((gsm_rx_dt[loc_counter] < 0x30) || (gsm_rx_dt[loc_counter]> 0x39)) return;
                        criticalBatLevel = criticalBatLevel+ (gsm_rx_dt[loc_counter]-0x30);
                        if (criticalBatLevel > lowBatLevel) criticalBatLevel = 0;
                        location  = 0;
                    } else if (location == 9) {
                        ++loc_counter;
                        if ((gsm_rx_dt[loc_counter] < 0x30) || (gsm_rx_dt[loc_counter]> 0x39)) return;
                        ldr_event_gap =  (gsm_rx_dt[loc_counter]-0x30)*100; loc_counter++;
                        if ((gsm_rx_dt[loc_counter] < 0x30) || (gsm_rx_dt[loc_counter]> 0x39)) return;
                        ldr_event_gap =  ldr_event_gap+(gsm_rx_dt[loc_counter]-0x30)*10; loc_counter++;
                        if ((gsm_rx_dt[loc_counter] < 0x30) || (gsm_rx_dt[loc_counter]> 0x39)) return;
                        ldr_event_gap =  ldr_event_gap+(gsm_rx_dt[loc_counter]-0x30);
                        ldr_event_gap = ldr_event_gap * 60;
                        location  = 0;
                    } else if (location == 10) {
                        ++loc_counter;
                        if ((gsm_rx_dt[loc_counter] < 0x30) || (gsm_rx_dt[loc_counter]> 0x39)) return;
                        resend_data_freq = (gsm_rx_dt[loc_counter]-0x30)*10; loc_counter++;
                        if ((gsm_rx_dt[loc_counter] < 0x30) || (gsm_rx_dt[loc_counter]> 0x39)) return;
                        resend_data_freq =  resend_data_freq+(gsm_rx_dt[loc_counter]-0x30);
                        resend_data_freq = resend_data_freq * 60;
                        location  = 0;
                    } else {
                        location = 100;
                    }
                }


                HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);
                ++loc_counter;location=0;
                done = 2;
                return;
            }

            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);
            ++loc_counter;
        } // While Loop
    } // if authenticated
} // End of function


//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
//{
//  if(huart->Instance==USART1)
//	{HAL_UART_DeInit(&huart1);
//	MX_USART1_UART_Init();
//	HAL_UART_Receive_IT(&huart1,rcvd_data2,1);
//	}
//}


//-----------------------------------------------------------------

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance==USART1)
    {
        if(rcv_ctr2>=2445) rcv_ctr2=0;
        ++rcv_ctr2;
        gsm_rx_dt[rcv_ctr2]=rcvd_data2[0];
        //HAL_UART_Receive_IT(&huart1,rcvd_data2,1);

        gsm_reset_counter = gsm_reset_counter_value ;
        // Set EOL Variable
        eol = 0;
        if((rcv_ctr2>1) && (gsm_rx_dt[rcv_ctr2]==0x0a)&&(gsm_rx_dt[rcv_ctr2-1]==0x0d)) eol=1;
        if (gsm_rx_dt[rcv_ctr2]==0x0d) eol=1;
				if (gsm_rx_dt[rcv_ctr2]==0x0a) eol=1;
        //-------------errorhandler--------------------------------------------------
        if((rcv_ctr2 > 5) && (gsm_rx_dt[rcv_ctr2]=='3')&&(gsm_rx_dt[rcv_ctr2-1]=='1')&&(gsm_rx_dt[rcv_ctr2-2]=='8')&&(gsm_rx_dt[rcv_ctr2-3]=='3')&&(gsm_rx_dt[rcv_ctr2-4]==' ')&&(gsm_rx_dt[rcv_ctr2-5]==':'))
        {restart_gsm=1;rcv_ctr2= 0;}//dns error 3813

        if((rcv_ctr2 > 5) && (gsm_rx_dt[rcv_ctr2]=='7')&&(gsm_rx_dt[rcv_ctr2-1]=='2')&&(gsm_rx_dt[rcv_ctr2-2]=='8')&&(gsm_rx_dt[rcv_ctr2-3]=='3')&&(gsm_rx_dt[rcv_ctr2-4]==' ')&&(gsm_rx_dt[rcv_ctr2-5]==':'))
        {restart_gsm=1;rcv_ctr2= 0;}//http time out 3827

        if((rcv_ctr2 > 3) && (gsm_rx_dt[rcv_ctr2]=='Z')&&(gsm_rx_dt[rcv_ctr2-1]=='T')&&(gsm_rx_dt[rcv_ctr2-2]=='I')&&(gsm_rx_dt[rcv_ctr2-3]=='N'))
        {qintz_bit=1;rcv_ctr2 =0;}

        if((rcv_ctr2 > 8) && (gsm_rx_dt[rcv_ctr2]=='y')&&(gsm_rx_dt[rcv_ctr2-1]=='d')&&(gsm_rx_dt[rcv_ctr2-2]=='a')&&(gsm_rx_dt[rcv_ctr2-3]=='e')&&(gsm_rx_dt[rcv_ctr2-4]=='R')&&(gsm_rx_dt[rcv_ctr2-5]==' ')&&(gsm_rx_dt[rcv_ctr2-6]=='l')&&(gsm_rx_dt[rcv_ctr2-7]=='l')&&(gsm_rx_dt[rcv_ctr2-8]=='a'))
        {call_ready=1;rcv_ctr2= 0;}//Call Ready:

        if((rcv_ctr2 > 8) && (gsm_rx_dt[rcv_ctr2]=='y')&&(gsm_rx_dt[rcv_ctr2-1]=='d')&&(gsm_rx_dt[rcv_ctr2-2]=='a')&&(gsm_rx_dt[rcv_ctr2-3]=='e')&&(gsm_rx_dt[rcv_ctr2-4]=='R')&&(gsm_rx_dt[rcv_ctr2-5]==' ')&&(gsm_rx_dt[rcv_ctr2-6]=='S')&&(gsm_rx_dt[rcv_ctr2-7]=='M')&&(gsm_rx_dt[rcv_ctr2-8]=='S'))
        {sms_ready=1;rcv_ctr2= 0;}//SMS Ready:

        if((rcv_ctr2 > 7) && (gsm_rx_dt[rcv_ctr2]=='1')&&(gsm_rx_dt[rcv_ctr2-1]=='0')&&(gsm_rx_dt[rcv_ctr2-2]=='0')&&(gsm_rx_dt[rcv_ctr2-3]=='_')&&(gsm_rx_dt[rcv_ctr2-4]=='C')&&(gsm_rx_dt[rcv_ctr2-5]=='S')&&(gsm_rx_dt[rcv_ctr2-6]=='D')&&(gsm_rx_dt[rcv_ctr2-7]=='S'))
        {sdsc_ok=1;rcv_ctr2 = 0;}
        if((rcv_ctr2 > 7) && (gsm_rx_dt[rcv_ctr2-3]=='_')&&(gsm_rx_dt[rcv_ctr2-4]=='C')&&(gsm_rx_dt[rcv_ctr2-5]=='S')&&(gsm_rx_dt[rcv_ctr2-6]=='D')&&(gsm_rx_dt[rcv_ctr2-7]=='S'))
        {sdsc_ok=1;rcv_ctr2 = 0;}

        // Get Operator
        if ((get_operator_bit ==4) && (gsm_rx_dt[rcv_ctr2]!=0x22) && (gsm_rx_dt[rcv_ctr2]!=0x0d) && (ctr<15)) {
            operator_name[ctr] = gsm_rx_dt[rcv_ctr2]; ctr++;
            operator_name[ctr] = '\0';
        }
        if ((get_operator_bit ==4) && (eol==1)) {get_operator_bit = 0;}
        if ((get_operator_bit ==3) && (gsm_rx_dt[rcv_ctr2]==',')) {get_operator_bit++; ctr =0; }// Skip second comma
        if ((get_operator_bit ==2) && (gsm_rx_dt[rcv_ctr2]==',')) {get_operator_bit++;}// Skip first comma
        if ((get_operator_bit ==1) && (rcv_ctr2>6) && (gsm_rx_dt[rcv_ctr2-6]=='+')&& (gsm_rx_dt[rcv_ctr2-5]=='C')&&(gsm_rx_dt[rcv_ctr2-4]=='O')&&(gsm_rx_dt[rcv_ctr2-3]=='P')&&(gsm_rx_dt[rcv_ctr2-2]=='S')&&(gsm_rx_dt[rcv_ctr2-1]==':')&&(gsm_rx_dt[rcv_ctr2]==' '))
            {get_operator_bit=2;}

        //---------------------------get local ip--------------------------------
        if ((get_ip_local==2) && (eol==1) && (ctr>1)) {get_ip_local=0;}
        if ((get_ip_local==2) && (eol==0) && (ctr<19)) {
            local_ip_address[ctr]=gsm_rx_dt[rcv_ctr2];ctr++;
            local_ip_address[ctr]='\0';
        }
        if ((get_ip_local==1) && (rcv_ctr2>8) && (gsm_rx_dt[rcv_ctr2]==0x0a)&&(gsm_rx_dt[rcv_ctr2-1]==0x0d)&&(gsm_rx_dt[rcv_ctr2-2]==0x0d)&&(gsm_rx_dt[rcv_ctr2-3]=='P')&&(gsm_rx_dt[rcv_ctr2-4]=='I')&&(gsm_rx_dt[rcv_ctr2-5]=='C')&&(gsm_rx_dt[rcv_ctr2-6]=='O')&&(gsm_rx_dt[rcv_ctr2-7]=='L')&&(gsm_rx_dt[rcv_ctr2-8]=='I'))
            {get_ip_local=2;ctr = 0;}


        // ----------------------------------- waiting after send_gsm command  ---- ok_gsm flag reset
        if((rcv_ctr2>3) && (gsm_rx_dt[rcv_ctr2]==0x0a)&&(gsm_rx_dt[rcv_ctr2-1]==0x0d)&&(gsm_rx_dt[rcv_ctr2-2]=='K')&&(gsm_rx_dt[rcv_ctr2-3]=='O'))
        {ok_gsm=1;error_bit= 0;rcv_ctr2=0;}	//ok_gsm

        if((rcv_ctr2>4) && (gsm_rx_dt[rcv_ctr2]=='R')&&(gsm_rx_dt[rcv_ctr2-1]=='O')&&(gsm_rx_dt[rcv_ctr2-2]=='R')&&(gsm_rx_dt[rcv_ctr2-3]=='R')&&(gsm_rx_dt[rcv_ctr2-4]=='E'))
        {error_bit=1;ok_gsm=0;rcv_ctr2 = 0;}//error set

        if((cpin_check==1) && (rcv_ctr2 > 8) && (gsm_rx_dt[rcv_ctr2]=='Y')&&(gsm_rx_dt[rcv_ctr2-1]=='D')&&(gsm_rx_dt[rcv_ctr2-2]=='A')&&(gsm_rx_dt[rcv_ctr2-3]=='E')&&(gsm_rx_dt[rcv_ctr2-4]=='R')&&(gsm_rx_dt[rcv_ctr2-5]==' ')&&(gsm_rx_dt[rcv_ctr2-6]==':')&&(gsm_rx_dt[rcv_ctr2-7]=='N')&&(gsm_rx_dt[rcv_ctr2-8]=='I'))
        {cpin_check=2;rcv_ctr2 = 0;}//cpinready:

        // ------------------- read Network ------------------------------------------------
        if((rcv_ctr2 > 8) && (gsm_rx_dt[rcv_ctr2-4]==':')&&(gsm_rx_dt[rcv_ctr2-5]=='G')&&(gsm_rx_dt[rcv_ctr2-6]=='E')&&(gsm_rx_dt[rcv_ctr2-7]=='R')&&(gsm_rx_dt[rcv_ctr2-8]=='C'))
        {
            network=0;
            if(gsm_rx_dt[rcv_ctr2]=='1') network=1;
            if(gsm_rx_dt[rcv_ctr2]=='5') network=5;
            rcv_ctr2 = 0;
        }

        // Cell ID , MNC, MCC, LAC 17 commas
        // ------------------- Engineering mode to get cell details----------------------------------
        //cell_id
        // ignore other fields
        if ((eng_read > 10) && (eol==1)){eng_read=0;}
        if ((eng_read>5) && (gsm_rx_dt[rcv_ctr2]==',')) {eng_read++;}
        if (eng_read == 5) {
            if ((gsm_rx_dt[rcv_ctr2]!=',') && (ctr<5)) {
                cell_id[ctr] = gsm_rx_dt[rcv_ctr2];ctr++;
                cell_id[ctr] = '\0';
            }
            if (gsm_rx_dt[rcv_ctr2]==',') {eng_read++;ctr=0;}
        }
        //lac
        if (eng_read == 4) {
            if ((gsm_rx_dt[rcv_ctr2]!=',') && (ctr<5)) {
                lac[ctr] = gsm_rx_dt[rcv_ctr2];ctr++;
                lac[ctr] = '\0';
            }
            if (gsm_rx_dt[rcv_ctr2]==',') {eng_read++;ctr=0;}
        }
        //mnc
        if (eng_read == 3) {
            if ((gsm_rx_dt[rcv_ctr2]!=',') && (ctr<5)) {
                mnc[ctr] = gsm_rx_dt[rcv_ctr2]; ctr++;
                mnc[ctr] = '\0';
            }
            if (gsm_rx_dt[rcv_ctr2]==',') {eng_read++;ctr=1;}
        }
        // mcc
        if (eng_read == 2) {
            if ((gsm_rx_dt[rcv_ctr2]!=',') && (ctr<5)) {
                mcc[ctr] = gsm_rx_dt[rcv_ctr2];ctr++;
                mcc[ctr] = '\0';
            }
            if (gsm_rx_dt[rcv_ctr2]==',') {eng_read++;ctr=0;}
        }
        if ((eng_read == 1) && (rcv_ctr2>7) && (gsm_rx_dt[rcv_ctr2-7]=='Q')&&(gsm_rx_dt[rcv_ctr2-6]=='E')&&(gsm_rx_dt[rcv_ctr2-5]=='N')&&(gsm_rx_dt[rcv_ctr2-4]=='G')&&(gsm_rx_dt[rcv_ctr2-3]==':')&&(gsm_rx_dt[rcv_ctr2-2]==' ')&&(gsm_rx_dt[rcv_ctr2-1]=='0')&&(gsm_rx_dt[rcv_ctr2]==','))
        {ctr = 0; ++eng_read;}


        // IMEI request has been initiated
        // IMEI data started coming
        if ((get_imei_bit==2) && (ctr<15) && (eol==0)) {
            imei[ctr] = gsm_rx_dt[rcv_ctr2];ctr++;
            imei[ctr] = ',';
            imei[ctr+1] = '\0';
        }
        if ((get_imei_bit==2) && (ctr>1) && (eol==1)) {get_imei_bit=0;}  // IMEI completed
        if((get_imei_bit==1) && (eol==1))
        {get_imei_bit=2;ctr=0;}


        //-------------------------------------iccid no-------------------
        if((get_iccid_bit==3) && (ctr<23) && (eol==0))
        {
            iccid_no[ctr]=gsm_rx_dt[rcv_ctr2];ctr++;
            iccid_no[ctr]='\0';
        }
        if((get_iccid_bit==3) && (eol==1) && (ctr>1)) {get_iccid_bit=0;}
        if((get_iccid_bit==2) && (eol==1)) {get_iccid_bit=3;ctr=0;}
        if((get_iccid_bit==1) && (rcv_ctr2>4) &&(gsm_rx_dt[rcv_ctr2-4]=='q') && (gsm_rx_dt[rcv_ctr2-3]=='c') && (gsm_rx_dt[rcv_ctr2-2]=='c') && (gsm_rx_dt[rcv_ctr2-1]=='i') && (gsm_rx_dt[rcv_ctr2]=='d')) {get_iccid_bit=2;ctr=0;}

        // These checks are done only on the current received character
        // ------------------- start read Time ------------------------------------------------
        if ((time_read==2) && (rcv_ctr2 >= 18))  {
            date_ist[0]=gsm_rx_dt[8];
            date_ist[1]=gsm_rx_dt[9];
            date_ist[2]=gsm_rx_dt[5];
            date_ist[3]=gsm_rx_dt[6];
            date_ist[4]=gsm_rx_dt[2];
            date_ist[5]=gsm_rx_dt[3];
            date_ist[6]='\0';
            time_ist[0]=gsm_rx_dt[11];
            time_ist[1]=gsm_rx_dt[12];
            time_ist[2]=gsm_rx_dt[14];
            time_ist[3]=gsm_rx_dt[15];
            time_ist[4]=gsm_rx_dt[17];
            time_ist[5]=gsm_rx_dt[18];
            time_ist[6]='\0';
            time_read= 0;
        }
        if((time_read==1) && (rcv_ctr2>5) && (gsm_rx_dt[rcv_ctr2]==' ') && (gsm_rx_dt[rcv_ctr2-1]==':')&&(gsm_rx_dt[rcv_ctr2-2]=='K')&&(gsm_rx_dt[rcv_ctr2-3]=='L')&&(gsm_rx_dt[rcv_ctr2-4]=='C')&&(gsm_rx_dt[rcv_ctr2-5]=='C'))
        {time_read=2;rcv_ctr2=0;}


        // ------------------- read GSM signal strength ------------------------------------------------
        if((rcv_ctr2 > 7) && (gsm_rx_dt[rcv_ctr2-4]==':')&&(gsm_rx_dt[rcv_ctr2-5]=='Q')&&(gsm_rx_dt[rcv_ctr2-6]=='S')&&(gsm_rx_dt[rcv_ctr2-7]=='C'))
        {
                if(gsm_rx_dt[rcv_ctr2-1]!=',')
                    gsm_signal[1]=gsm_rx_dt[rcv_ctr2-1];
                else
                {   gsm_signal[1]=gsm_rx_dt[rcv_ctr2-2]; gsm_signal[0]='0';}

                if(gsm_rx_dt[rcv_ctr2-1]!=',') gsm_signal[0]=gsm_rx_dt[rcv_ctr2-2];

                if((gsm_signal[0]=='9')&&(gsm_signal[1]=='9'))
                {gsm_signal[0]='0';gsm_signal[1]='0';}

                if((gsm_signal[0]>=0x34)&&(gsm_signal[1]!=0x39))
                {gsm_signal[1]=gsm_signal[0];gsm_signal[0]='0';}

                gsm_signal[2]='\0';
                rcv_ctr2 = 0;
        }
        if((rcv_ctr2 > 6) && (gsm_rx_dt[rcv_ctr2]=='T')&&(gsm_rx_dt[rcv_ctr2-1]=='C')&&(gsm_rx_dt[rcv_ctr2-2]=='E')&&(gsm_rx_dt[rcv_ctr2-3]=='N')&&(gsm_rx_dt[rcv_ctr2-4]=='N')&&(gsm_rx_dt[rcv_ctr2-5]=='O')&&(gsm_rx_dt[rcv_ctr2-6]=='C'))
        {connect_http=1;rcv_ctr2= 0;}//connect_http


        //-----------------------------------sms confirmation
        if((rcv_ctr2 > 4) && (gsm_rx_dt[rcv_ctr2]==':')&&(gsm_rx_dt[rcv_ctr2-1]=='S')&&(gsm_rx_dt[rcv_ctr2-2]=='G')&&(gsm_rx_dt[rcv_ctr2-3]=='M')&&(gsm_rx_dt[rcv_ctr2-4]=='C'))
        {sms_delevered_bit=1;rcv_ctr2= 0;}

        // ------------------------------ sms received command
        if((rcv_ctr2>1) && (sms_recieved==1)&&(eol==1))
        { sms_recieved=2;rcv_ctr2=0;}

        if((rcv_ctr2 > 4) && (gsm_rx_dt[rcv_ctr2]==':')&&(gsm_rx_dt[rcv_ctr2-1]=='I')&&(gsm_rx_dt[rcv_ctr2-2]=='T')&&(gsm_rx_dt[rcv_ctr2-3]=='M')&&(gsm_rx_dt[rcv_ctr2-4]=='C'))
        {sms_recieved=1;rcv_ctr2=0;}	//cmti:

        if((gsm_rx_dt[rcv_ctr2]=='+')&&(get_operator_bit==0) && (sms_recieved==0)&&(time_read==0)&&(eng_read==0)) rcv_ctr2=0;

        HAL_UART_Receive_IT(&huart1,rcvd_data2,1);
    }

}


void delay_ms(unsigned int dms)
{
	while(dms)
	{--dms;HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);
		for(halt_ms=0;halt_ms<=1000;++halt_ms)
		{HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);}
	}
}




void read_network()
{
        if(power_on_gsm==0) return;
        read_clock();
        network=0;
        send_gsm("AT\r");ok_check(20);
        send_gsm("AT+CREG?\r");ok_check(20);
}

void read_signal()
{
    if(power_on_gsm==0) return;
    gsm_signal[0] = '0';
    gsm_signal[1] = '0';
    gsm_signal[2] = '\0';
    send_gsm("AT\r");ok_check(30);
    send_gsm("AT+CSQ\r");ok_check(30);
}



/* --------------------------------------------------------------
 * Reads the clock value from the module
*/
void read_clock()
{	
        send_gsm("AT\r");ok_check(40);
        time_read=1;time_out=40;
        send_gsm("AT+CCLK?\r");ok_check(40);
        if (time_read!=0){
            // Wait for the response
            while((time_read != 0) && (time_out>0))
            {HAL_Delay(10);--time_out;}
        }
        // gsm_rx_dt has the date time in ' "yy/MM/dd,hh:mm:ss,+-zz"' format
        // since time is in local format we will ignore zz and assume century to be 2000
        if (time_read != 0) {
            epoch_to_date_time(date_ist,time_ist,currentdatetime);
            time_read=0;
        } else {
            currentdatetime = date_time_to_epoch(date_ist,time_ist);
        }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */ 
	message_update_counter=150;
}






void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_TxCpltCallback can be implemented in the user file
   */ 
//}  /* Set transmission flag: trasfer complete*/
	if(huart->Instance==USART1)
	{
	 UartReady = SET;
	}

}

// Load default values of the APN, URLs and FOTA
void load_default()
{ 
    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);// wdi pin
    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);//wdt_wdi=!wdt_wdi;
    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);

    // QoSIM
    sense_apn[0] = 's'; sense_apn[1] = 'e'; sense_apn[2] = 'n'; sense_apn[3] = 's'; sense_apn[4] = 'e';
    sense_apn[5] = 'm'; sense_apn[6] = '2'; sense_apn[7] = 'm'; sense_apn[8] = '\0'; sense_apn[9] = '\0';
    sense_apn[10] = '\0'; sense_apn[11] = '\0'; sense_apn[12] = '\0'; sense_apn[13] = '\0'; sense_apn[14] = '\0';
    sense_apn[15] = '\0';sense_apn[16] = '\0';sense_apn[17] = '\0';sense_apn[18] = '\0';sense_apn[19] = '\0';

    // idea apn
    idea_apn[0] = 'i'; idea_apn[1] = 'n'; idea_apn[2] = 't'; idea_apn[3] = 'e'; idea_apn[4] = 'r';
    idea_apn[5] = 'n'; idea_apn[6] = 'e'; idea_apn[7] = 't'; idea_apn[8] = '\0'; idea_apn[9] = '\0';
    idea_apn[10] = '\0'; idea_apn[11] = '\0'; idea_apn[12] = '\0'; idea_apn[13] = '\0'; idea_apn[14] = '\0';
    idea_apn[15] = '\0';idea_apn[16] = '\0';idea_apn[17] = '\0';idea_apn[18] = '\0';idea_apn[19] = '\0';

    // Voda apn
    voda_apn[0] = 'I'; voda_apn[1] = 'O'; voda_apn[2] = 'T'; voda_apn[3] = '.'; voda_apn[4] = 'C';
    voda_apn[5] = 'O'; voda_apn[6] = 'M'; voda_apn[7] = '\0'; voda_apn[8] = '\0'; voda_apn[9] = '\0';
    voda_apn[10] = '\0'; voda_apn[11] = '\0'; voda_apn[12] = '\0'; voda_apn[13] = '\0'; voda_apn[14] = '\0';
    voda_apn[15] = '\0';voda_apn[16] = '\0';voda_apn[17] = '\0';voda_apn[18] = '\0';voda_apn[19] = '\0';

    // BSNL apn
    bsnl_apn[0] = 'b'; bsnl_apn[1] = 's'; bsnl_apn[2] = 'n'; bsnl_apn[3] = 'l'; bsnl_apn[4] = 'n';
    bsnl_apn[5] = 'e'; bsnl_apn[6] = 't'; bsnl_apn[7] = '\0'; bsnl_apn[8] = '\0'; bsnl_apn[9] = '\0';
    bsnl_apn[10] = '\0'; bsnl_apn[11] = '\0'; bsnl_apn[12] = '\0'; bsnl_apn[13] = '\0'; bsnl_apn[14] = '\0';
    bsnl_apn[15] = '\0';bsnl_apn[16] = '\0';bsnl_apn[17] = '\0';bsnl_apn[18] = '\0';bsnl_apn[19] = '\0';

    // MTNL
    mtnl_apn[0] = 'm'; mtnl_apn[1] = 't'; mtnl_apn[2] = 'n'; mtnl_apn[3] = 'l'; mtnl_apn[4] = '.';
    mtnl_apn[5] = 'n'; mtnl_apn[6] = 'e'; mtnl_apn[7] = 't'; mtnl_apn[8] = '\0'; mtnl_apn[9] = '\0';
    mtnl_apn[10] = '\0'; mtnl_apn[11] = '\0'; mtnl_apn[12] = '\0'; mtnl_apn[13] = '\0'; mtnl_apn[14] = '\0';
    mtnl_apn[15] = '\0';mtnl_apn[16] = '\0';mtnl_apn[17] = '\0';mtnl_apn[18] = '\0';mtnl_apn[19] = '\0';

    // airtel apn
    airtel_apn[0] = 'a'; airtel_apn[1] = 'i'; airtel_apn[2] = 'r'; airtel_apn[3] = 't'; airtel_apn[4] = 'e';
    airtel_apn[5] = 'l'; airtel_apn[6] = 'g'; airtel_apn[7] = 'p'; airtel_apn[8] = 'r'; airtel_apn[9] = 's';
    airtel_apn[10] = '.'; airtel_apn[11] = 'c'; airtel_apn[12] = 'o'; airtel_apn[13] = 'm'; airtel_apn[14] = '\0';
    airtel_apn[15] = '\0';airtel_apn[16] = '\0';airtel_apn[17] = '\0';airtel_apn[18] = '\0';airtel_apn[19] = '\0';

    // aircel apn
    aircel_apn[0] = 'a'; aircel_apn[1] = 'i'; aircel_apn[2] = 'r'; aircel_apn[3] = 'c'; aircel_apn[4] = 'e';
    aircel_apn[5] = 'l'; aircel_apn[6] = 'g'; aircel_apn[7] = 'p'; aircel_apn[8] = 'r'; aircel_apn[9] = 's';
    aircel_apn[10] = '.'; aircel_apn[11] = 'n'; aircel_apn[12] = 'e'; aircel_apn[13] = 't'; aircel_apn[14] = '\0';
    aircel_apn[15] = '\0';aircel_apn[16] = '\0';aircel_apn[17] = '\0';aircel_apn[18] = '\0';aircel_apn[19] = '\0';

    phone_no_admin[0]='9';    phone_no_admin[1]='4';    phone_no_admin[2]='4';    phone_no_admin[3]='8';
    phone_no_admin[4]='0';    phone_no_admin[5]='7';    phone_no_admin[6]='2';    phone_no_admin[7]='9';
    phone_no_admin[8]='0';    phone_no_admin[9]='7';    phone_no_admin[10]='\0';
	

    useCustomAPN = 0;

    dataurl[0]='h';
    dataurl[1]='t';
    dataurl[2]='t';
    dataurl[3]='p';
    dataurl[4]=':';
    dataurl[5]='/';
    dataurl[6]='/';
    dataurl[7]='i';
    dataurl[8]='t';
    dataurl[9]='i';
    dataurl[10]='s';
    dataurl[11]='m';
    dataurl[12]='a';
    dataurl[13]='r';
    dataurl[14]='t';
    dataurl[15]='c';
    dataurl[16]='a';
    dataurl[17]='m';
    dataurl[18]='p';
    dataurl[19]='u';
    dataurl[20]='s';
    dataurl[21]='.';
    dataurl[22]='s';
    dataurl[23]='e';
    dataurl[24]='n';
    dataurl[25]='s';
    dataurl[26]='o';
    dataurl[27]='r';
    dataurl[28]='i';
    dataurl[29]='s';
    dataurl[30]='e';
    dataurl[31]='.';
    dataurl[32]='n';
    dataurl[33]='e';
    dataurl[34]='t';

    dataurl[35]=':';
    dataurl[36]='8';
    dataurl[37]='0';
    dataurl[38]='8';
    dataurl[39]='0';
    dataurl[40] = '\0';

    alerturl[0]='h';
    alerturl[1]='t';
    alerturl[2]='t';
    alerturl[3]='p';
    alerturl[4]=':';
    alerturl[5]='/';
    alerturl[6]='/';
    alerturl[7]='i';
    alerturl[8]='t';
    alerturl[9]='i';
    alerturl[10]='s';
    alerturl[11]='m';
    alerturl[12]='a';
    alerturl[13]='r';
    alerturl[14]='t';
    alerturl[15]='c';
    alerturl[16]='a';
    alerturl[17]='m';
    alerturl[18]='p';
    alerturl[19]='u';
    alerturl[20]='s';
    alerturl[21]='.';
    alerturl[22]='s';
    alerturl[23]='e';
    alerturl[24]='n';
    alerturl[25]='s';
    alerturl[26]='o';
    alerturl[27]='r';
    alerturl[28]='i';
    alerturl[29]='s';
    alerturl[30]='e';
    alerturl[31]='.';
    alerturl[32]='n';
    alerturl[33]='e';
    alerturl[34]='t';
    alerturl[35]=':';
    alerturl[36]='8';
    alerturl[37]='0';
    alerturl[38]='8';
    alerturl[39]='0';
    alerturl[40] = '\0';

    url_cfd[0]='/';
    url_cfd[1]='s';
    url_cfd[2]='b';
    url_cfd[3]='m';
    url_cfd[4]='/';
    url_cfd[5]='m';
    url_cfd[6]='a';
    url_cfd[7]='i';
    url_cfd[8]='d';
    url_cfd[9]='D';
    url_cfd[10]='a';
    url_cfd[11]='t';
    url_cfd[12]='a';
    url_cfd[13]='/';
    url_cfd[14]='1';
    url_cfd[15]='/';
    url_cfd[16]='\0';
	
    url_helth[0]='/';
    url_helth[1]='s';
    url_helth[2]='b';
    url_helth[3]='m';
    url_helth[4]='/';
    url_helth[5]='m';
    url_helth[6]='a';
    url_helth[7]='i';
    url_helth[8]='d';
    url_helth[9]='H';
    url_helth[10]='e';
    url_helth[11]='a';
    url_helth[12]='l';
    url_helth[13]='t';
    url_helth[14]='h';
    url_helth[15]='/';
    url_helth[16]='1';
    url_helth[17]='/';
    url_helth[18]='\0';
	
    url_monitoring[0]='/';
    url_monitoring[1]='s';
    url_monitoring[2]='b';
    url_monitoring[3]='m';
    url_monitoring[4]='/';
    url_monitoring[5]='m';
    url_monitoring[6]='a';
    url_monitoring[7]='i';
    url_monitoring[8]='d';
    url_monitoring[9]='A';
    url_monitoring[10]='l';
    url_monitoring[11]='e';
    url_monitoring[12]='r';
    url_monitoring[13]='t';
    url_monitoring[14]='/';
    url_monitoring[15]='1';
    url_monitoring[16]='/';
    url_monitoring[17]='\0';

    url_summary[0]='/';
    url_summary[1]='s';
    url_summary[2]='b';
    url_summary[3]='m';
    url_summary[4]='/';
    url_summary[5]='m';
    url_summary[6]='a';
    url_summary[7]='i';
    url_summary[8]='d';
    url_summary[9]='S';
    url_summary[10]='u';
    url_summary[11]='m';
    url_summary[12]='m';
    url_summary[13]='a';
    url_summary[14]='r';
    url_summary[15]='y';
    url_summary[16]='/';
    url_summary[17]='1';
    url_summary[18]='/';
    url_summary[19]='\0';

    fota_ip[0]='1';
    fota_ip[1]='0';
    fota_ip[2]='3';
    fota_ip[3]='.';
    fota_ip[4]='7';
    fota_ip[5]='.';
    fota_ip[6]='6';
    fota_ip[7]='4';
    fota_ip[8]='.';
    fota_ip[9]='2';
    fota_ip[10]='1';
    fota_ip[11]='\0';
    fota_ip[12]='\0';
    fota_ip[13]='\0';
    fota_ip[14]='\0';
    fota_ip[15]='\0';

    fota_port[0]='2';
    fota_port[1]='1';
    fota_port[2]='\0';

    fota_user[0]='f';
    fota_user[1]='t';
    fota_user[2]='p';
    fota_user[3]='u';
    fota_user[4]='s';
    fota_user[5]='e';
    fota_user[6]='r';
    fota_user[7]='\0';

    fota_pass[0]='F';
    fota_pass[1]='T';
    fota_pass[2]='P';
    fota_pass[3]='u';
    fota_pass[4]='s';
    fota_pass[5]='e';
    fota_pass[6]='r';
    fota_pass[7]='@';
    fota_pass[8]='1';
    fota_pass[9]='2';
    fota_pass[10]='3';
    fota_pass[11]='\0';

    fota_file[0]='c';
    fota_file[1]='u';
    fota_file[2]='r';
    fota_file[3]='r';
    fota_file[4]='e';
    fota_file[5]='n';
    fota_file[6]='t';
    fota_file[7]='.';
    fota_file[8]='h';
    fota_file[9]='e';
    fota_file[10]='x';
    fota_file[11]='\0';

    fota_folder[0]='h';
    fota_folder[1]='o';
    fota_folder[2]='m';
    fota_folder[3]='e';
    fota_folder[4]='/';
    fota_folder[5]='f';
    fota_folder[6]='t';
    fota_folder[7]='p';
    fota_folder[8]='u';
    fota_folder[9]='s';
    fota_folder[10]='e';
    fota_folder[11]='r';
    fota_folder[12]='/';
    fota_folder[13]='S';
    fota_folder[14]='B';
    fota_folder[15]='M';
    fota_folder[16]='_';
    fota_folder[17]='C';
    fota_folder[18]='F';
    fota_folder[19]='D';
    fota_folder[20]='/';
    fota_folder[21]='S';
    fota_folder[22]='T';
    fota_folder[23]='M';
    fota_folder[24]='3';
    fota_folder[25]='2';
    fota_folder[26]='A';
    fota_folder[27]='\0';
    fota_folder[28]='\0';

    // APDU Config Parameters
    ten_minute_counter_value = 600;
    batdata_minute_counter_value = 5400;
    util_dataCount = 10;
    bat_dataCount = 0;
    health_Timer_Value = 0;
    device_reset_counter_value = 14 * 60 *60;
    lowBatLevel = 361;
    criticalBatLevel = 355;
    ldr_event_gap = 900;
    resend_data_freq = 10 * 60;
    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);

}


//Initializes the GSM module and keep it ready
void initialize_gsm()
{
    Refresh_IWDG();
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);
    led_gsm_1;// off the led at gsm initialization
    led_power_0;
    init_gsm();
    get_imei();
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);
}



void make_button_data(unsigned char *dt, unsigned char *tm, unsigned char cfd )
{
	temp_long=current_date_sequence_button;
	
        post_string[3]=ascii_number[temp_long%10];
	temp_long/=10;
	post_string[2]=ascii_number[temp_long%10];
	temp_long/=10;
	post_string[1]=ascii_number[temp_long%10];
	temp_long/=10;
	post_string[0]=ascii_number[temp_long%10];
	
	mem_ctr=0;
        post_string[4]=dt[0];
        post_string[5]=dt[1];
        post_string[6]=dt[2];
        post_string[7]=dt[3];
        post_string[8]=dt[4];
        post_string[9]=dt[5];
        post_string[10]=tm[0];
        post_string[11]=tm[1];
        post_string[12]=tm[2];
        post_string[13]=tm[3];
        post_string[14]=tm[4];
        post_string[15]=tm[5];
	
	mat_no_y=16;
	for(copy_no=0; copy_no<15 ; ++copy_no)
	{post_string[mat_no_y+copy_no]=imei[copy_no];}//imei 
	
        mat_no_y+=copy_no;//31
	for(copy_no=0; copy_no<22 ; ++copy_no)
	{post_string[mat_no_y+copy_no]=iccid_no[copy_no];}//iccid no
	
        mat_no_y+=copy_no; //53

	post_string[mat_no_y]=cell_id[0];++mat_no_y;// cell id
	post_string[mat_no_y]=cell_id[1];++mat_no_y;
	post_string[mat_no_y]=cell_id[2];++mat_no_y;
	post_string[mat_no_y]=cell_id[3];++mat_no_y;
	
	post_string[mat_no_y]=lac[0];++mat_no_y;//
	post_string[mat_no_y]=lac[1];++mat_no_y;
	post_string[mat_no_y]=lac[2];++mat_no_y;
	post_string[mat_no_y]=lac[3];++mat_no_y;
	
	post_string[mat_no_y]=gsm_signal[0];++mat_no_y;// gsm_signal strength
	post_string[mat_no_y]=gsm_signal[1];++mat_no_y;

	post_string[mat_no_y]=mnc[0];++mat_no_y;//
	post_string[mat_no_y]=mnc[1];++mat_no_y;
//65
        post_string[mat_no_y]=cfd + 0x30;
			
        ++mat_no_y;			 //66
	post_string[mat_no_y]='\0';
}







void make_helth_data()
{
	
        read_clock();
	
	temp_long=current_date_sequence_helth;
	post_string[3]=ascii_number[temp_long%10];
	temp_long/=10;
	post_string[2]=ascii_number[temp_long%10];
	temp_long/=10;
	post_string[1]=ascii_number[temp_long%10];
	temp_long/=10;
	post_string[0]=ascii_number[temp_long%10];

	post_string[4]=date_ist[0];// discuss
	post_string[5]=date_ist[1];
	post_string[6]=date_ist[2];
	post_string[7]=date_ist[3];
	post_string[8]=date_ist[4];
	post_string[9]=date_ist[5];
	post_string[10]=time_ist[0];
	post_string[11]=time_ist[1];
	post_string[12]=time_ist[2];
	post_string[13]=time_ist[3];
	post_string[14]=time_ist[4];
	post_string[15]=time_ist[5];
	
		
	mat_no_y=16;
	for(copy_no=0; copy_no<15 ; ++copy_no)
	{post_string[mat_no_y+copy_no]=imei[copy_no];}//imei
	
	mat_no_y+=copy_no;
	for(copy_no=0; copy_no<22 ; ++copy_no)
	{post_string[mat_no_y+copy_no]=iccid_no[copy_no];}//iccid no
	
	mat_no_y+=copy_no;
	post_string[mat_no_y]=gsm_signal[0];++mat_no_y;// gsm_signal strength
	post_string[mat_no_y]=gsm_signal[1];++mat_no_y;
	
	post_string[mat_no_y]=cell_id[0];++mat_no_y;// cell id
	post_string[mat_no_y]=cell_id[1];++mat_no_y;
	post_string[mat_no_y]=cell_id[2];++mat_no_y;
	post_string[mat_no_y]=cell_id[3];++mat_no_y;
	
	post_string[mat_no_y]=lac[0];++mat_no_y;//
	post_string[mat_no_y]=lac[1];++mat_no_y;
	post_string[mat_no_y]=lac[2];++mat_no_y;
	post_string[mat_no_y]=lac[3];++mat_no_y;

	post_string[mat_no_y]=mnc[0];++mat_no_y;//
	post_string[mat_no_y]=mnc[1];++mat_no_y;
	
        temp_int = ram_store_loc;
        post_string[mat_no_y+2]=(temp_int % 10)+0x30;
        temp_int /= 10;
        post_string[mat_no_y+1]=(temp_int % 10)+0x30;;
        temp_int /= 10;
        post_string[mat_no_y]=(temp_int % 10)+0x30;
        mat_no_y += 3;
        temp_int = rolling_mem;
        post_string[mat_no_y+1]=(temp_int % 10)+0x30;;
        temp_int /= 10;
        post_string[mat_no_y]=(temp_int % 10)+0x30;
        mat_no_y += 2;
        temp_int = ram_sent_loc;
        post_string[mat_no_y+2]=(temp_int % 10)+0x30;
        temp_int /= 10;
        post_string[mat_no_y+1]=(temp_int % 10)+0x30;;
        temp_int /= 10;
        post_string[mat_no_y]=(temp_int % 10)+0x30;
        mat_no_y += 3;
        post_string[mat_no_y]='0' ;++mat_no_y;
        post_string[mat_no_y]='0' ;++mat_no_y;
        post_string[mat_no_y]='0' ;++mat_no_y;
        post_string[mat_no_y]='0' ;++mat_no_y;

//	post_string[mat_no_y]=loacal_ip_calculated[0];++mat_no_y;// tcp ip 12 digit
//	post_string[mat_no_y]=loacal_ip_calculated[1];++mat_no_y;
//	post_string[mat_no_y]=loacal_ip_calculated[2];++mat_no_y;
//	post_string[mat_no_y]=loacal_ip_calculated[3];++mat_no_y;
//	post_string[mat_no_y]=loacal_ip_calculated[4];++mat_no_y;
//	post_string[mat_no_y]=loacal_ip_calculated[5];++mat_no_y;
//	post_string[mat_no_y]=loacal_ip_calculated[6];++mat_no_y;
//	post_string[mat_no_y]=loacal_ip_calculated[7];++mat_no_y;
//	post_string[mat_no_y]=loacal_ip_calculated[8];++mat_no_y;
//	post_string[mat_no_y]=loacal_ip_calculated[9];++mat_no_y;
//	post_string[mat_no_y]=loacal_ip_calculated[10];++mat_no_y;
//	post_string[mat_no_y]=loacal_ip_calculated[11];++mat_no_y;
	
	post_string[mat_no_y]=int_bat_matrix[0];++mat_no_y;// internal battery matrix
	post_string[mat_no_y]=int_bat_matrix[1];++mat_no_y;
	post_string[mat_no_y]=int_bat_matrix[2];++mat_no_y;

        if(power_ip==1)
        {post_string[mat_no_y]='1';++mat_no_y;}
        else
        {post_string[mat_no_y]='0';++mat_no_y;}

	temp_long=wakeup_hours;
	post_string[mat_no_y+5]=ascii_number[temp_long%10];
	temp_long/=10;
	post_string[mat_no_y+4]=ascii_number[temp_long%10];
	temp_long/=10;
	post_string[mat_no_y+3]=ascii_number[temp_long%10];
	temp_long/=10;
	post_string[mat_no_y+2]=ascii_number[temp_long%10];
	temp_long/=10;
	post_string[mat_no_y+1]=ascii_number[temp_long%10];
	temp_long/=10;
	post_string[mat_no_y+0]=ascii_number[temp_long%10];


        copy_no=6;
        mat_no_y+=copy_no;
        post_string[mat_no_y]='\0';++mat_no_y;
		
}


//Make the monitoring data
void make_monitoring_data(unsigned char *dt, unsigned char *tm,unsigned char alert)
{
    temp_long=current_date_sequence_monitering;

    post_string[3]=ascii_number[temp_long%10];
    temp_long/=10;
    post_string[2]=ascii_number[temp_long%10];
    temp_long/=10;
    post_string[1]=ascii_number[temp_long%10];
    temp_long/=10;
    post_string[0]=ascii_number[temp_long%10];

    post_string[4]=dt[0];// discuss
    post_string[5]=dt[1];
    post_string[6]=dt[2];
    post_string[7]=dt[3];
    post_string[8]=dt[4];
    post_string[9]=dt[5];
    post_string[10]=tm[0];
    post_string[11]=tm[1];
    post_string[12]=tm[2];
    post_string[13]=tm[3];
    post_string[14]=tm[4];
    post_string[15]=tm[5];

    mat_no_y=16;
    for(copy_no=0; copy_no<15 ; ++copy_no)
    {post_string[mat_no_y+copy_no]=imei[copy_no];}//imei

    mat_no_y+=copy_no;
    for(copy_no=0; copy_no<22 ; ++copy_no)
    {post_string[mat_no_y+copy_no]=iccid_no[copy_no];}//iccid

    mat_no_y+=copy_no;

    post_string[mat_no_y]=cell_id[0];++mat_no_y;// cell id
    post_string[mat_no_y]=cell_id[1];++mat_no_y;
    post_string[mat_no_y]=cell_id[2];++mat_no_y;
    post_string[mat_no_y]=cell_id[3];++mat_no_y;

    post_string[mat_no_y]=lac[0];++mat_no_y;//
    post_string[mat_no_y]=lac[1];++mat_no_y;
    post_string[mat_no_y]=lac[2];++mat_no_y;
    post_string[mat_no_y]=lac[3];++mat_no_y;

    post_string[mat_no_y]=gsm_signal[0];++mat_no_y;// gsm_signal strength
    post_string[mat_no_y]=gsm_signal[1];++mat_no_y;

    post_string[mat_no_y]=mnc[0];++mat_no_y;//
    post_string[mat_no_y]=mnc[1];++mat_no_y;

    temp_long=alert;

    post_string[mat_no_y+1]=ascii_number[temp_long%10];
    temp_long/=10;
    post_string[mat_no_y]=ascii_number[temp_long%10];

    ++mat_no_y;++mat_no_y;

    post_string[mat_no_y]='\0';
}


void make_summary_data()
{
	
	read_clock();
	temp_long=current_date_sequence_summary;
	
	post_string[3]=ascii_number[temp_long%10];
	temp_long/=10;
	post_string[2]=ascii_number[temp_long%10];
	temp_long/=10;
	post_string[1]=ascii_number[temp_long%10];
	temp_long/=10;
	post_string[0]=ascii_number[temp_long%10];
	
	mat_no_y=4;
	for(copy_no=0; copy_no<6 ; ++copy_no)
	{post_string[mat_no_y+copy_no]=firm_ver[copy_no];}//firmware_version
        mat_no_y+=copy_no; //10

	for(copy_no=0; copy_no<15 ; ++copy_no)
	{post_string[mat_no_y+copy_no]=imei[copy_no];}//imei
        mat_no_y+=copy_no;  //25

	for(copy_no=0; copy_no<22 ; ++copy_no)
	{post_string[mat_no_y+copy_no]=iccid_no[copy_no];}//iccid
        mat_no_y+=copy_no; // 47
	
	post_string[mat_no_y]=cell_id[0];++mat_no_y;// cell id
	post_string[mat_no_y]=cell_id[1];++mat_no_y;
	post_string[mat_no_y]=cell_id[2];++mat_no_y;
	post_string[mat_no_y]=cell_id[3];++mat_no_y;
	
	post_string[mat_no_y]=lac[0];++mat_no_y;//
	post_string[mat_no_y]=lac[1];++mat_no_y;
	post_string[mat_no_y]=lac[2];++mat_no_y;
	post_string[mat_no_y]=lac[3];++mat_no_y;
	
	post_string[mat_no_y]=gsm_signal[0];++mat_no_y;// gsm_signal strength
	post_string[mat_no_y]=gsm_signal[1];++mat_no_y;

	post_string[mat_no_y]=mnc[0];++mat_no_y;//mnc
	post_string[mat_no_y]=mnc[1];++mat_no_y;
//59
	
        temp_long=heart_beat_o;
        if (which_summary ==0)temp_long=heart_beat;  // Current Summary
        post_string[mat_no_y+1]=ascii_number[temp_long%10];
        temp_long/=10;
        post_string[mat_no_y]=ascii_number[temp_long%10];++mat_no_y;
        ++mat_no_y;
//61
        if (which_summary ==1) {
            post_string[mat_no_y]=last_heart_beat_time_o[0];++mat_no_y;//total heart beat sent today
            post_string[mat_no_y]=last_heart_beat_time_o[1];++mat_no_y;
            post_string[mat_no_y]=last_heart_beat_time_o[2];++mat_no_y;//total heart beat sent today
            post_string[mat_no_y]=last_heart_beat_time_o[3];++mat_no_y;
        } else {
            post_string[mat_no_y]=last_heart_beat_time[0];++mat_no_y;//total heart beat sent today
            post_string[mat_no_y]=last_heart_beat_time[1];++mat_no_y;
            post_string[mat_no_y]=last_heart_beat_time[2];++mat_no_y;//total heart beat sent today
            post_string[mat_no_y]=last_heart_beat_time[3];++mat_no_y;
        }
//65
        temp_long=good_button_count_o;
        if (which_summary ==0)temp_long=good_button_count;  // Current Summary
	post_string[mat_no_y+3]=ascii_number[temp_long%10];//++mat_no_y;
	temp_long/=10;
	post_string[mat_no_y+2]=ascii_number[temp_long%10];//++mat_no_y;
	temp_long/=10;
	post_string[mat_no_y+1]=ascii_number[temp_long%10];//++mat_no_y;
	temp_long/=10;
	post_string[mat_no_y]=ascii_number[temp_long%10];//++mat_no_y;
        mat_no_y+=4; //69

        temp_long=ok_button_count_o;
        if (which_summary ==0)temp_long=ok_button_count;
	post_string[mat_no_y+3]=ascii_number[temp_long%10];//
	temp_long/=10;
	post_string[mat_no_y+2]=ascii_number[temp_long%10];//++mat_no_y;
	temp_long/=10;
	post_string[mat_no_y+1]=ascii_number[temp_long%10];//++mat_no_y;
	temp_long/=10;
	post_string[mat_no_y]=ascii_number[temp_long%10];//++mat_no_y;
        mat_no_y+=4; //73

        temp_long=bad_button_count_o;
        if (which_summary ==0)temp_long=bad_button_count;
	post_string[mat_no_y+3]=ascii_number[temp_long%10];//++mat_no_y;
	temp_long/=10;
	post_string[mat_no_y+2]=ascii_number[temp_long%10];//++mat_no_y;
	temp_long/=10;
	post_string[mat_no_y+1]=ascii_number[temp_long%10];//++mat_no_y;
	temp_long/=10;
	post_string[mat_no_y]=ascii_number[temp_long%10];//++mat_no_y;
        mat_no_y+=4; // 77

        if (which_summary ==0){
            for(copy_no=0; copy_no<6 ; ++copy_no)
            {post_string[mat_no_y+copy_no]=last_button_press_record_time[copy_no];}//last button press count time
        }else {
            for(copy_no=0; copy_no<6 ; ++copy_no)
            {post_string[mat_no_y+copy_no]=last_button_press_record_time_o[copy_no];}//last button press count time
        }

        mat_no_y+=copy_no;//83
        temp_long=no_of_power_outage_o;
        if (which_summary ==0)temp_long=no_of_power_outage;
        post_string[mat_no_y+1]=ascii_number[temp_long%10];
        temp_long/=10;
        post_string[mat_no_y]=ascii_number[temp_long%10];++mat_no_y;
        ++mat_no_y;
//85
        if (which_summary ==0) {
            for(copy_no=0; copy_no<4 ; ++copy_no)
            {post_string[mat_no_y+copy_no]=last_power_outage_time[copy_no];}//last button press count time
        } else {
            for(copy_no=0; copy_no<4 ; ++copy_no)
            {post_string[mat_no_y+copy_no]=last_power_outage_time_o[copy_no];}//last button press count time
        }
        mat_no_y+=copy_no; // 89
        temp_long=su_mode_act_cnt_o;
        if (which_summary ==0) temp_long=su_mode_act_cnt;
	post_string[mat_no_y]=ascii_number[temp_long%10];++mat_no_y;
//90
        if (which_summary ==1)  {
            for(copy_no=0; copy_no<4 ; ++copy_no)
            {post_string[mat_no_y+copy_no]=last_su_mode_time_o[copy_no];}//last button press count time
        } else {
            for(copy_no=0; copy_no<4 ; ++copy_no)
            {post_string[mat_no_y+copy_no]=last_su_mode_time[copy_no];}//last button press count time
        }
        mat_no_y+=copy_no; //94

        temp_long=ldr_time_o;
        if (which_summary ==0)temp_long=ldr_time;
	post_string[mat_no_y]=ascii_number[temp_long%10];++mat_no_y;
//95
        if (which_summary ==1) {
            for(copy_no=0; copy_no<4 ; ++copy_no)
            {post_string[mat_no_y+copy_no]=last_ldr_time_o[copy_no];}//last button press count time
        } else {
            for(copy_no=0; copy_no<4 ; ++copy_no)
            {post_string[mat_no_y+copy_no]=last_ldr_time[copy_no];}//last button press count time
        }
	
        mat_no_y+=copy_no; // 99
        post_string[mat_no_y]='\0';
	
}


void get_iccid_no()
{

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    send_gsm("AT\r");ok_check(30);
    get_iccid_bit=1;
    send_gsm("at+qccid\r");ok_check(50);
    //qccid_counter=0;
    if (get_iccid_bit != 0) {
        time_out=5;
        while((get_iccid_bit!=0)&&(time_out>0))
        {
            HAL_Delay(100);
            --time_out;
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
        }
    }
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
}



void find_cell_info()
{
    // Read only the serving Cell details  (2,0).
    // 2,3 would give serving cell and 1-6 other cells near by
    if(power_on_gsm==0) return;
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    send_gsm("AT\r");ok_check(30);
    eng_read=1;
    send_gsm("AT+QENG=2,0\r");ok_check(50);

    if (eng_read != 0){
        // Wait for the command to complete
        time_delay=15;
        while((eng_read !=0 )&&(time_delay > 0))
        {--time_delay;HAL_Delay(100);
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
        }
        eng_read = 0;
    }
    // Switch off Engineering mode
     send_gsm("AT+QENG=0\r\n"); ok_check(30);
     eng_read = 0;
     Refresh_IWDG();// internal wdt reset
     HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

 }



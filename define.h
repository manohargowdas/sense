#include "stm32f0xx_hal.h"


/* Defines BEGIN ---------------------------------------------------------------------- */
#define latch_0_counter_value 5

#define hw_gsm_power_on HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET)
#define hw_gsm_power_off HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET)
#define e_off_0 0
#define e_off_1 1

#define gsm_power_key_0 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET)
#define gsm_power_key_1 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET)
#define WDI_PORT GPIOB
#define WDI_PIN GPIO_PIN_8

#define en_wdt_port GPIOB
#define en_wdt_pin GPIO_PIN_9

#define op1_port GPIOC
#define op1_pin  GPIO_PIN_13
#define ip1_port GPIOF//ignition
#define ip1_pin GPIO_PIN_1 
#define ip2_port GPIOF//door
#define ip2_pin GPIO_PIN_0
#define ip3_port GPIOC// ac 
#define ip3_pin GPIO_PIN_15
#define ip4_port GPIOC
#define ip4_pin GPIO_PIN_14
#define ext_bat_port GPIOB
#define ext_bat_pin GPIO_PIN_0
//-----------------------------------------------------
#define op1_0 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET)
#define op1_1 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET)
#define op1_port GPIOC
#define op1_pin GPIO_PIN_13 


#define led_gps_1 1// HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET)
#define led_gps_0 0//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET)
#define led_gsm_1 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET)
#define led_gsm_0 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET)
#define led_power_1 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET)
#define led_power_0 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET)


#define normal 1
#define critical 2
#define acknoledge 3
#define gsm_reset_counter_value 7200

#define auto_test_deact_time 300

#define FLASH_PROG_UPDATE_BIT_START_ADDR  ((uint32_t)0x0801f800) //program update_bit address
#define FLASH_USER_NECESSARY_ADDR  ((uint32_t)0x0801f000) //program update_bit address
#define FLASH_USER_Monitoring_ADDR  ((uint32_t)0x0801e800) //program update_bit address

#define cfd_data 0
#define health_data 1
#define summary_data 2
#define monitoring_data 3

/* Defines END ---------------------------------------------------------------------- */

/* External variables BEGIN ---------------------------------------------------------------------- */
extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc;
extern IWDG_HandleTypeDef hiwdg;
extern unsigned char read_clock_bit,test_bit,power_on_gsm,su_mode_bit,latch_0_counter;
extern unsigned char ip1,ip2,ip3,qintz_bit;
extern unsigned int temp_int;
extern unsigned long  ten_minute_counter_value,batdata_minute_counter_value,health_Data_Timer,health_Timer_Value;
extern unsigned long device_reset_counter_value, device_restart_counter;
extern unsigned char send_fix_tiemr,sms_ready,call_ready;
extern unsigned int rolling_mem,util_dataCount, bat_dataCount;
extern unsigned char data_count;
extern unsigned char restart_gsm;
extern unsigned char idea_apn[20];
extern unsigned char voda_apn[20];
extern unsigned char sense_apn[20];
extern unsigned char bsnl_apn[20];
extern unsigned char mtnl_apn[20];
extern unsigned char airtel_apn[20];
extern unsigned char aircel_apn[20];
extern unsigned char operator_name[15];
extern unsigned char ok_gsm, qosim;
extern unsigned int temp_char;
extern unsigned int main_reset_counter;
extern unsigned char read_adc_bit;
extern unsigned char ascii_number[18];//={0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,'A','B','C','D','E','F'};//inc 16 to 18
extern unsigned char rcvd_data2[2];
extern unsigned char phone_no_admin[12];
extern unsigned char apn[20];
extern unsigned char apn_mem[20];
extern unsigned int version_loaded;
extern unsigned char mode_value,power_ip;
extern unsigned int autotest_deactivate_time, su_deactivate_time;
extern unsigned char url[100];
extern unsigned char dataurl[50];
extern unsigned char alerturl[50];
extern unsigned char url_cfd[20];
extern unsigned char url_summary[20];
extern unsigned char url_helth[20];
extern unsigned char url_monitoring[20];
extern unsigned int rcv_ctr2,monitoring_send_data_counter;
extern unsigned char UartReady;
extern unsigned int time_out_ctr;
extern unsigned long ten_minute_counter,batdata_minute_counter;  // When 0 , main loop will try to send the data
extern unsigned char time_ist[8];
extern unsigned char date_ist[10];//="556677,";
extern unsigned char imei[18];
extern unsigned char status_connect1,restart_counter;
extern unsigned int gsm_reset_counter;
extern unsigned char get_iccid_value,which_summary;
extern unsigned int mem_ctr;
extern unsigned int date_int_prev;
extern unsigned char latch_op1,happy,ok,sad;
extern unsigned long temp_long,current_date_sequence_button,current_date_sequence_helth,current_date_sequence_summary;
extern unsigned char fota_user[15];
extern unsigned char fota_pass[15];
extern unsigned char fota_file[14];
extern unsigned char fota_folder[30];
extern 	unsigned char no_of_power_outage,ldr_time,su_mode_act_count;
extern	unsigned int good_button_count,ok_button_count,bad_button_count;
extern	unsigned char no_of_power_outage_o,ldr_time_o,su_mode_act_cnt_o;
extern	unsigned int good_button_count_o,ok_button_count_o,bad_button_count_o,heart_beat_o;
extern	unsigned int heart_beat;//mod
extern	unsigned char last_power_outage_time[4];
extern	unsigned char last_su_mode_time[4];
extern	unsigned char last_ldr_time[4];
extern	unsigned char last_heart_beat_time[4];
extern	unsigned char last_button_press_record_time[6];
extern	unsigned char last_power_outage_time_o[4];
extern	unsigned char last_su_mode_time_o[4];
extern	unsigned char last_ldr_time_o[4];
extern	unsigned char last_heart_beat_time_o[4];
extern	unsigned char last_button_press_record_time_o[6];
//extern long floor (float);
extern unsigned long bank_start_datetime[18];
extern unsigned char error_http, mno;
extern unsigned int write_flash_timer;
extern unsigned char error_bit,local_ip_error;
extern 	unsigned long  FLASH_USER_History_ADDR;
extern unsigned char connect_http;
extern unsigned int time_out;
extern unsigned char gsm_rx_dt[2500];
extern unsigned char fota_ip[20];
extern unsigned char fota_port[6];
extern unsigned char firm_ver[8];
extern unsigned char gsm_signal[5];
extern unsigned char network;
extern unsigned char read_clock_bit,done,read_signal_bit, done_sum,done_rt;
extern unsigned int box_open_bit, box_removed_bit;
extern unsigned int ldr_event_gap;
extern unsigned char sms_recieved,message_update_counter,sdsc_ok;
extern unsigned char read_network_bit;
extern unsigned int loc_counter;
extern unsigned char get_iccid_bit,get_iccid_counter;
extern unsigned char iccid_no[25];
extern unsigned char	send_command,cell_info_count,eng_read;
extern unsigned char gsm_signal[5];
extern unsigned long wakeup_hours;
extern unsigned int copy_no;
extern unsigned char mat_no_y;
extern unsigned int time_delay;
extern unsigned char load_default_bit;
extern unsigned char post_string[750];
extern unsigned char url_length_matrix[5];
extern unsigned int data_store_counter;
extern unsigned char send_heart_bit,fota_update_bit;
extern unsigned char sms_delevered_bit;
extern uint32_t Address,PageError;
extern unsigned char int_bat_matrix[3];
extern unsigned char readed_prog_update_value;
extern unsigned char cpin_check;
extern unsigned char booting;
extern unsigned char get_imei_bit;
extern unsigned char get_ip_local;
extern unsigned char get_operator_bit;
extern unsigned char alert_loc1;
extern unsigned long ram_datetime[682] ;
extern unsigned char ram_cfd[682] ;
extern unsigned int  ram_store_loc;
extern unsigned int  ram_sent_loc ;
extern unsigned long lastSentData;
extern unsigned char flash_store_bit,su_mode_act_cnt;
extern unsigned int summary_wait_time;
extern unsigned char sessionset;
extern unsigned char custom_apn[20];
extern unsigned char useCustomAPN,bypass_wdt;
extern unsigned int lowBatLevel, criticalBatLevel;
extern unsigned int  resend_data_freq;

/* External variables END ---------------------------------------------------------------------- */


/* External functions BEGIN ---------------------------------------------------------------------- */
//extern void make_mem_value(void);
extern void make_helth_data(void);
extern void send_health(void); //ip1.c
extern void find_operator(void);
extern void select_apn(void);
extern void get_imei(void);  // TCP_IP.c
extern void send_gsm(unsigned char *q);  // TCP_IP.c
extern void init_gsm(void);  // TCP_IP.c
extern void get_admin_values(void);
extern void load_default(void);
extern void initialize_gsm(void); // TCP_IP.C
extern void send_http_string(unsigned char package_type);  //ip1.c
extern void send_cfd(void); // ip1.c
extern void read_clock(void);
extern void read_summary_data(void);
extern void write_summary_data(void);
extern void update_values(void);
extern void read_network(void);
extern void read_signal(void);
extern void make_monitoring_data(unsigned char *dt, unsigned char *tm,unsigned char alert); // TCP_IP.c
extern void send_monitering(void);// ip1.c
extern void make_summary_data(void);
extern void send_summary(void);
extern void read_necessary_value(void);
extern void write_necessary_value(void);
extern void read_local_ip(void);  // TCP_IP.C
extern void ok_check_http(void);
extern void ok_check(unsigned int);
extern void activate_gprs(void);  // ip1.C
extern void MX_USART1_UART_Init(void);
extern void get_iccid_no(void);
extern void find_cell_info(void);
extern void make_button_data(unsigned char *dt, unsigned char *tm, unsigned char cfd );//tcp_ip.c
extern void read_update_value(void);
extern void write_update_value(void);
extern void delay_ms(unsigned int dms);
extern void Refresh_IWDG(void);   // main.c
extern void RebootDevice(void);
extern void epoch_to_date_time(unsigned char *date ,unsigned char *time, unsigned long epoch);
extern unsigned long date_time_to_epoch(unsigned char *date, unsigned char *time);
extern void store_cfd_data(unsigned char cfd);
extern void write_ram_in_flash(unsigned char state);
extern void read_flash_in_ram(void);
extern unsigned int current_date_sequence_monitering;
extern void SetupSession(void);


/* External functions END ---------------------------------------------------------------------- */


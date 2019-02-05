/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "C:\Work\Sensorise\Product\SenseTise\FW\STM32\Code\Build\CFD\MDK-ARM\define.h"


/* USER CODE END Includes */

/* USER CODE BEGIN PV */

/* Private variables START ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;
IWDG_HandleTypeDef hiwdg;
UART_HandleTypeDef huart1;

uint32_t adc_buf[3];
unsigned char read_adc_bit = 0;
unsigned char bat_volt_counter = 0, bat_monit_sent;
unsigned int bat_volt;
unsigned long ten_minute_counter_value, batdata_minute_counter_value,health_Data_Timer, health_Timer_Value;
unsigned int date_int_prev,monitoring_send_data_counter;
unsigned long device_restart_counter,device_reset_counter_value;
unsigned char data_count,send_ready_data;
unsigned int box_removed_bit, box_open_bit;
unsigned int ldr_event_gap;
unsigned char sms_delevered_bit;
unsigned int creg_reset_counter, util_dataCount, bat_dataCount;
unsigned char int_bat_matrix[3];
unsigned char fota_ip[20];
unsigned char fota_port[6];
unsigned char fota_user[15];
unsigned char fota_pass[15];
unsigned char fota_file[14];
unsigned char fota_folder[30];
uint32_t val[3];
unsigned char readed_prog_update_value;
unsigned int version_loaded;
unsigned char happy,ok,sad, mno, qosim;
unsigned char test_bit,su_mode_bit;
unsigned int summary_wait_time = 0 ,write_flash_timer;
unsigned char sessionset = 0;
unsigned char done,read_signal_bit,done_sum,done_rt;
unsigned int lowBatLevel, criticalBatLevel;
unsigned int  resend_data_freq;

#define RCC_APB2Periph_SYSCFG            RCC_APB2ENR_SYSCFGEN
#define SYSCFG_MemoryRemap_SRAM                 ((uint8_t)0x03)
#define ADC_int_batt_CHANNEL ADC_CHANNEL_9
#define ADC_ext_ip_CHANNEL ADC_CHANNEL_4 

//static FLASH_EraseInitTypeDef EraseInitStruct;

//-------------------------------------------------------------------------



#define APPLICATION_ADDRESS     (uint32_t)0x08008000
static __IO uint32_t TimingDelay;

#if   (defined ( __CC_ARM ))
  __IO uint32_t VectorTable[48] __attribute__((at(0x20000000)));
#elif (defined (__ICCARM__))
#pragma location = 0x20000000
  __no_init __IO uint32_t VectorTable[48];
#elif defined   (  __GNUC__  )
  __IO uint32_t VectorTable[48] __attribute__((section(".RAMVectorTable")));
#elif defined ( __TASKING__ )
  __IO uint32_t VectorTable[48] __at(0x20000000);
#endif



/* Private variables END ---------------------------------------------------------*/

/* Static variables START ---------------------------------------------------------*/
//static FLASH_EraseInitTypeDef EraseInitStruct;//mod
static unsigned short days[4][12] =
{
    {   0,  31,  60,  91, 121, 152, 182, 213, 244, 274, 305, 335},
    { 366, 397, 425, 456, 486, 517, 547, 578, 609, 639, 670, 700},
    { 731, 762, 790, 821, 851, 882, 912, 943, 974,1004,1035,1065},
    {1096,1127,1155,1186,1216,1247,1277,1308,1339,1369,1400,1430},
};
/* Static variables END ---------------------------------------------------------*/

/* Private define START ---------------------------------------------*/
#define SYSCFG_MemoryRemap_SRAM     ((uint8_t)0x03)
//#define APPLICATION_ADDRESS         (uint32_t)0x08008000

#if   (defined ( __CC_ARM ))
  __IO uint32_t VectorTable[48] __attribute__((at(0x20000000)));
#elif (defined (__ICCARM__))
#pragma location = 0x20000000
  __no_init __IO uint32_t VectorTable[48];
#elif defined   (  __GNUC__  )
  __IO uint32_t VectorTable[48] __attribute__((section(".RAMVectorTable")));
#elif defined ( __TASKING__ )
  __IO uint32_t VectorTable[48] __at(0x20000000);
#endif

#define top_value 2800
#define	bottom_value 2800
/* Private define END ---------------------------------------------*/

/* USER CODE END PV */


/* USER CODE BEGIN PFP */
/* Private functions prototypes START --------------------------------------------*/
void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
void SYSCFG_MemoryRemapConfig(uint32_t SYSCFG_MemoryRemap);
void SystemClock_Config(void);
void Error_Handler(void);
void Refresh_IWDG(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
void RebootDevice(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// Refresh the external Watchdog timer
void Refresh_IWDG()
{
   HAL_IWDG_Refresh(&hiwdg);    // COMMENT for debug
}

/*
 * This function is used to test the button connectivity
 */
void test_sw()
{
    unsigned char cnt1 = 0, cnt2 = 0, cnt3 =0;
    op1_0;
    while(test_bit==2)
    {
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
        main_reset_counter=0;
        HAL_Delay(200);
        latch_0_counter=latch_0_counter_value;
        latch_op1=1;
        // OK buttone press
        if((HAL_GPIO_ReadPin(ip2_port,ip2_pin)==SET)&&(HAL_GPIO_ReadPin(ip1_port,ip1_pin)==RESET)&&(HAL_GPIO_ReadPin(ip3_port,ip3_pin)==RESET))
        {
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            latch_0_counter=latch_0_counter_value;
            latch_op1=1;
            op1_1; HAL_Delay(900);
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            op1_0;
            cnt1 = 1;
        }

        // Green buttone press
        if((HAL_GPIO_ReadPin(ip1_port,ip1_pin)==SET)&&(HAL_GPIO_ReadPin(ip2_port,ip2_pin)==RESET)&&(HAL_GPIO_ReadPin(ip3_port,ip3_pin)==RESET))
        {
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            latch_0_counter=latch_0_counter_value;
            latch_op1=1;
            op1_1; HAL_Delay(900);
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            op1_0; HAL_Delay(800);
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            latch_0_counter=latch_0_counter_value;
            latch_op1=1;
            op1_1; HAL_Delay(900);
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            op1_0;
            cnt2 = 1;
        }

        // Red buttone press
        if((HAL_GPIO_ReadPin(ip3_port,ip3_pin)==SET)&&(HAL_GPIO_ReadPin(ip1_port,ip1_pin)==RESET)&&(HAL_GPIO_ReadPin(ip2_port,ip2_pin)==RESET))
        {
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            latch_0_counter=latch_0_counter_value;
            latch_op1=1;
            op1_1; HAL_Delay(900);
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            op1_0; HAL_Delay(800);
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            latch_0_counter=latch_0_counter_value;
            latch_op1=1;
            op1_1; HAL_Delay(900);
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            op1_0; HAL_Delay(800);
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            latch_0_counter=latch_0_counter_value;
            latch_op1=1;
            op1_1; HAL_Delay(900);
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            op1_0;
            cnt3 = 1;
        }

        if (((cnt1 ==1) && (cnt2==1) && (cnt3==1)) || ((cnt1+cnt2+cnt3)>9))
        {
            test_bit=0;autotest_deactivate_time=0; mode_value=2; // Exit the test mode and Send Alert that Test Mode exited
        }
        if((HAL_GPIO_ReadPin(ip3_port,ip3_pin)==SET)&&(HAL_GPIO_ReadPin(ip1_port,ip1_pin)==SET)&&(HAL_GPIO_ReadPin(ip2_port,ip2_pin)==SET))
        {test_bit=0;autotest_deactivate_time=0; mode_value=2; }  // Exit the test mode and Send Alert that Test Mode exited

        while((HAL_GPIO_ReadPin(ip3_port,ip3_pin)==SET)&&(HAL_GPIO_ReadPin(ip1_port,ip1_pin)==SET)&&(HAL_GPIO_ReadPin(ip2_port,ip2_pin)==SET))
        {
            HAL_GPIO_TogglePin(op1_port,GPIO_PIN_13); HAL_Delay(500);
            test_bit = 0; autotest_deactivate_time=0; mode_value=2;   // Send Alert that Test Mode exited
        }
        latch_op1=0;
    } // End of while loop
}



/* NOTE : This function should not be modified. When the callback is needed,
          function HAL_ADC_ConvCpltCallback must be implemented in the user file.
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

    if(hadc-> Instance == ADC1)
    {
                    val[0]=adc_buf[0];
                    val[1]=adc_buf[1];
                    val[2]=adc_buf[2];
    }
}




// Formats ICCID no and inserts leading 0 if number is shorter than 22 bytes
void formate_iccid_no()
{
    unsigned char max_no,location_counter;
    unsigned char iccid_formated[25];
    max_no=21;
	
    for(location_counter=0;location_counter<22; ++location_counter)
    {iccid_formated[location_counter]=0;}

    for(location_counter=0;iccid_no[location_counter]!='\0'; ++location_counter)
    {iccid_formated[max_no-location_counter]=iccid_no[location_counter];}

    location_counter=0;
    if(iccid_formated[0]==0)
    {iccid_formated[0]=0x30;location_counter=1;}

    if(iccid_formated[1]==0)
    {iccid_formated[1]=0x30;location_counter=2;}

    if(iccid_formated[2]==0)
    {	iccid_formated[2]=0x30;location_counter=3;}

    if(iccid_formated[3]==0)
    {	iccid_formated[3]=0x30;location_counter=4;}

    if(location_counter>=1)
    {iccid_no[0]='0';}

    if(location_counter>=2)
    {iccid_no[1]='0';}

    if(location_counter>=3)
    {iccid_no[2]='0';}

    if(location_counter>=4)
    {iccid_no[3]='0';}


    for(temp_char=0;location_counter<22; ++location_counter,++temp_char)
    {iccid_no[location_counter]=iccid_formated[max_no-temp_char];}
	
    iccid_no[22]='\0';
}



void load_zero()
{
    // Save copy in old
    last_heart_beat_time_o[0]=last_heart_beat_time[0];
    last_heart_beat_time_o[1]=last_heart_beat_time[1];
    last_heart_beat_time_o[2]=last_heart_beat_time[2];
    last_heart_beat_time_o[3]=last_heart_beat_time[3];

    last_button_press_record_time_o[0]=last_button_press_record_time[0];
    last_button_press_record_time_o[1]=last_button_press_record_time[1];
    last_button_press_record_time_o[2]=last_button_press_record_time[2];
    last_button_press_record_time_o[3]=last_button_press_record_time[3];
    last_button_press_record_time_o[4]=last_button_press_record_time[4];
    last_button_press_record_time_o[5]=last_button_press_record_time[5];

    last_power_outage_time_o[0]=last_power_outage_time[0];
    last_power_outage_time_o[1]=last_power_outage_time[1];
    last_power_outage_time_o[2]=last_power_outage_time[2];
    last_power_outage_time_o[3]=last_power_outage_time[3];

    last_su_mode_time_o[0]=last_su_mode_time[0];
    last_su_mode_time_o[1]=last_su_mode_time[1];
    last_su_mode_time_o[2]=last_su_mode_time[2];
    last_su_mode_time_o[3]=last_su_mode_time[3];

    last_ldr_time_o[0]=last_ldr_time[0];
    last_ldr_time_o[1]=last_ldr_time[1];
    last_ldr_time_o[2]=last_ldr_time[2];
    last_ldr_time_o[3]=last_ldr_time[3];


    no_of_power_outage_o = no_of_power_outage;
    ldr_time_o =ldr_time;
    su_mode_act_cnt_o =su_mode_act_cnt;
    good_button_count_o= good_button_count;
    ok_button_count_o = ok_button_count;
    bad_button_count_o = bad_button_count;
    heart_beat_o = heart_beat;
// Reset to 0
    last_heart_beat_time[0]='0';
    last_heart_beat_time[1]='0';
    last_heart_beat_time[2]='0';
    last_heart_beat_time[3]='0';

    last_button_press_record_time[0]='0';
    last_button_press_record_time[1]='0';
    last_button_press_record_time[2]='0';
    last_button_press_record_time[3]='0';
    last_button_press_record_time[4]='0';
    last_button_press_record_time[5]='0';

    last_power_outage_time[0]='0';
    last_power_outage_time[1]='0';
    last_power_outage_time[2]='0';
    last_power_outage_time[3]='0';

    last_su_mode_time[0]='0';
    last_su_mode_time[1]='0';
    last_su_mode_time[2]='0';
    last_su_mode_time[3]='0';

    last_ldr_time[0]='0';
    last_ldr_time[1]='0';
    last_ldr_time[2]='0';
    last_ldr_time[3]='0';

    no_of_power_outage = 0;ldr_time=0;su_mode_act_cnt=0;
    good_button_count= 0;ok_button_count = 0;bad_button_count = 0;heart_beat = 0;
    which_summary = 1;
}

/* USER CODE END 0 */

int main(void)
{
    // COMMENT for HEX  as Bootloader from here
  /* USER CODE BEGIN 1 */
     uint32_t i = 0;
     for(i = 0; i < 48; i++)
     {
       VectorTable[i] = *(__IO uint32_t*)(APPLICATION_ADDRESS + (i<<2));
     }

//  Enable the SYSCFG peripheral clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
//  Remap SRAM at 0x00000000
    SYSCFG_MemoryRemapConfig(SYSCFG_MemoryRemap_SRAM);
    // COMMENT for HEX  as Bootloader till here

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

  /* Configure the system clock */
    SystemClock_Config();

  /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC_Init();
    MX_USART1_UART_Init();
    MX_IWDG_Init();  // COMMENT for debug

  /* USER CODE BEGIN 2 */
	
    HAL_IWDG_Start(&hiwdg);   // COMMENT for debug
    Refresh_IWDG();
    HAL_ADC_Start_DMA(&hadc,(uint32_t*)adc_buf,3);
    HAL_ADC_Start_IT(&hadc);
    HAL_GPIO_WritePin(en_wdt_port,en_wdt_pin,GPIO_PIN_RESET);

    // Load default variable values for last even value
    // ENTER VERSION NUMBER OF THE SOFTWARE aa.bb.cc should be entered as aabb ignore cc here
    version_loaded=02; // This is for FOTA

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    latch_0_counter=15;
    latch_op1=1;
    op1_1; HAL_Delay(900);
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

    // ENTER VERSION NUMBER OF THE SOFTWARE to be reported as aa.bb.cc
    firm_ver[0]='0';
    firm_ver[1]='2';
    firm_ver[2]='0';
    firm_ver[3]='0';
    firm_ver[4]='1';
    firm_ver[5]='3';
    firm_ver[6]='\0';
    useCustomAPN = 0;
    mno = 0;
    qosim = 0;
    bypass_wdt = 0;
    write_flash_timer=0;  
		done = 1;  // write the update values
		
    last_heart_beat_time[0]='0';
    last_heart_beat_time[1]='0';
    last_heart_beat_time[2]='0';
    last_heart_beat_time[3]='0';

    last_button_press_record_time[0]='0';
    last_button_press_record_time[1]='0';
    last_button_press_record_time[2]='0';
    last_button_press_record_time[3]='0';
    last_button_press_record_time[4]='0';
    last_button_press_record_time[5]='0';

    last_power_outage_time[0]='0';
    last_power_outage_time[1]='0';
    last_power_outage_time[2]='0';
    last_power_outage_time[3]='0';

    last_su_mode_time[0]='0';
    last_su_mode_time[1]='0';
    last_su_mode_time[2]='0';
    last_su_mode_time[3]='0';

    last_ldr_time[0]='0';
    last_ldr_time[1]='0';
    last_ldr_time[2]='0';
    last_ldr_time[3]='0';

    no_of_power_outage = 0;ldr_time=0;su_mode_act_cnt=0;
    good_button_count= 0;ok_button_count = 0;bad_button_count = 0;heart_beat = 0;
    which_summary = 1;

    load_zero();
    load_default_bit=0;
    hw_gsm_power_off;
    happy=0;
    sad=0;
    ok=0;
    bat_monit_sent = 0;
    bat_volt=0;
    bat_volt_counter = 0;
    get_imei_bit = 0;
    get_ip_local = 0;
    get_operator_bit = 0; //Not looking for operator as of yet
    sms_recieved=0;//rcv_ctr2=0;
    rcv_ctr2 = 0;   // GSM receive buffer
    su_mode_bit  = 0;
    main_reset_counter=0;
    health_Data_Timer = 24*60*60;
    get_iccid_bit = 0;
    eng_read = 0;
    date_int_prev = 0;
    ram_store_loc = 0;
    ram_sent_loc = 0;
    flash_store_bit = 0;
    lastSentData = 0;
    summary_wait_time = 1 ;
    for (temp_char = 0; temp_char < 18; temp_char++) bank_start_datetime[temp_char]= 0;
    for (temp_char = 0 ; temp_char < 20; temp_char++) apn_mem[temp_char]='\0';

    load_default(); // Load Default values for APN, URLs and FOTA
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    read_update_value();// Read fota confiiguration parameters
    version_loaded=02; // This is for FOTA
		
    //Check if FOTA has been requested then go for Reboot
    if(readed_prog_update_value!=0x49)
    {
        readed_prog_update_value=0x49;
        done = 1;
        //write_update_value();
        //RebootDevice();
    }
    read_necessary_value();  // Read RUN time configuration
    read_summary_data();

    booting=1;
    HAL_UART_Receive_IT(&huart1,rcvd_data2,1);

    hw_gsm_power_on;
    led_gsm_0;//initially check the led
    led_gsm_1;//initially off the led
    initialize_gsm();
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

    booting=0;
    send_gsm("at\r");ok_check(30);

//-------------------------------------------------
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    HAL_Delay(100);

    led_gsm_1;//off the led gsm for while loop

    while(call_ready==0){}
    while(sms_ready==0) {}

    SetupSession();

    send_gsm("at\r");ok_check(30);
    send_gsm("at+csmp=49,167,0,241\r");ok_check(30);
    send_gsm("AT+CMGD=1\r");ok_check(30);
    send_gsm("AT+CMGD=1,4\r");ok_check(30);
    send_gsm("AT+QMGDA=\"DEL ALL\"\r");ok_check(50);

    send_fix_tiemr=30;
    gsm_reset_counter=gsm_reset_counter_value;
    device_restart_counter=device_reset_counter_value;

    read_signal_bit=0;   // bit to read signal strength
    send_command=0;  // bit to get the cell info
    get_iccid_value=0; // bit to get ICCID of SIM Cards
    monitoring_send_data_counter=0;
    wakeup_hours=0;
    current_date_sequence_helth=0;
    current_date_sequence_button=0;
    current_date_sequence_monitering =0;
    current_date_sequence_summary = 0;
    creg_reset_counter = 0;
    rolling_mem = 0;
    read_flash_in_ram();
    data_count = ram_store_loc - ram_sent_loc;

    if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==SET) {
        power_ip=1;
        send_ready_data=1 ;  // If data was not sent earlier then mark it to send
    }
    else {
        power_ip = 0;
        send_ready_data=0 ;
    }
    latch_op1=0;
    op1_0;

		int_bat_matrix[0] = '4';
		int_bat_matrix[1] = '7';
		int_bat_matrix[2] = '0';
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
      Refresh_IWDG();// internal wdt reset
      HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

      // Check if utility power  is restored
        if ((HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==SET) && (power_ip==0))
        {
            power_ip=1;
            mode_value= 4;
            send_monitering();
        }
        else if ((HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==RESET) && (power_ip==1))
        { // Utility power outage event
            power_ip=0;
            mode_value= 3;
            send_monitering();
            if (no_of_power_outage <9) ++no_of_power_outage;
            last_power_outage_time[0]=time_ist[0];
            last_power_outage_time[1]=time_ist[1];
            last_power_outage_time[2]=time_ist[2];
            last_power_outage_time[3]=time_ist[3];
            done_sum = 1;
            write_ram_in_flash(0);
            write_necessary_value();
            write_update_value();
            write_summary_data();
            write_flash_timer = 3600;
        }

        // Timer enabled to write in the flash
        if (write_flash_timer == 0) {
            write_flash_timer = 3600;
            write_ram_in_flash(0);
            write_necessary_value();
            write_update_value();
            write_summary_data();
        }

        // Check if test bit is enabled
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
        if(test_bit == 2)
        {
            mode_value=1;   // Test Mode Activated
            if (power_ip == 1) send_monitering();  // Activated, Only when utility power is there
            mode_value=0;

            test_sw();  // Test Mode
            if (power_ip == 1) send_monitering(); // Deactivated, Only when utility power is there
            mode_value=0;
        }

        if ((su_mode_bit==4) && (mode_value==10)) {
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            send_monitering();  // SU mode activated
            mode_value=0;
            latch_0_counter=latch_0_counter_value;
            latch_op1=1;
            op1_1; HAL_Delay(800);
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            op1_0; HAL_Delay(500);
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            latch_0_counter=latch_0_counter_value;
            latch_op1=1;
            op1_1; HAL_Delay(800);
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            op1_0; HAL_Delay(500);
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            latch_0_counter=latch_0_counter_value;
            latch_op1=1;
            op1_1; HAL_Delay(800);
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            latch_op1=0;
            op1_0;
            if (su_mode_act_cnt <9) ++su_mode_act_cnt;
            last_su_mode_time[0] =time_ist[0];
            last_su_mode_time[1] =time_ist[1];
            last_su_mode_time[2] =time_ist[2];
            last_su_mode_time[3] =time_ist[3];
            done_sum = 1;
        }

        if ((su_mode_bit>=5) && (mode_value!=0)) {
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            send_monitering();  // SU mode activated
            mode_value=0;
            su_mode_bit = 0;
            latch_0_counter=latch_0_counter_value;
            latch_op1=1;
            op1_1; HAL_Delay(800);
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            op1_0; HAL_Delay(500);
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            latch_0_counter=latch_0_counter_value;
            latch_op1=1;
            op1_1; HAL_Delay(800);
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            op1_0; HAL_Delay(500);
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            latch_0_counter=latch_0_counter_value;
            latch_op1=1;
            op1_1; HAL_Delay(800);
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            latch_op1=0;
            op1_0;
        }

        // Check Battery Level , LDRs
        if(read_adc_bit==1)
        {
            read_adc_bit=0;  // reset it for next time
            HAL_ADC_Start_DMA(&hadc,(uint32_t*)adc_buf,3);
            HAL_ADC_Start_IT(&hadc);

            // Battery
            temp_int=val[2]/9.37;
            if(temp_int<470) {bat_volt=bat_volt+temp_int; ++bat_volt_counter;}
            if(bat_volt_counter>=10)
            {
                temp_int=bat_volt/bat_volt_counter;
                int_bat_matrix[2]=(temp_int%10)+0x30;
                temp_int/=10;
                int_bat_matrix[1]=(temp_int%10)+0x30;
                temp_int/=10;
                int_bat_matrix[0]=(temp_int%10)+0x30;
                if ((bat_volt <= (lowBatLevel * bat_volt_counter)) && (bat_monit_sent==0)) {
                    bat_monit_sent=1;
                    mode_value = 7;
                    send_monitering();
                    mode_value=0;
                    write_flash_timer = 10;
                }
                if ((bat_volt <= (criticalBatLevel * bat_volt_counter)) && (bat_monit_sent==1)) {
                    bat_monit_sent=2;
                    mode_value = 8;
                    send_monitering();
                    mode_value=0;
                    write_flash_timer = 10;
                }
                if (bat_volt > (criticalBatLevel * bat_volt_counter)) bat_monit_sent=1;
                if (bat_volt > (lowBatLevel * bat_volt_counter)) bat_monit_sent=0;
                bat_volt=0;
                bat_volt_counter=0;
            }

            // LDR2
            if((val[0]<=top_value)&&(box_open_bit==0)&&(su_mode_bit < 3))
            {
                Refresh_IWDG();// internal wdt reset
                HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
                mode_value = 6;
                send_monitering();
                mode_value=0;
                box_open_bit=1;
                if(ldr_time<9) ++ldr_time;
                last_ldr_time[0]=time_ist[0];
                last_ldr_time[1]=time_ist[1];
                last_ldr_time[2]=time_ist[2];
                last_ldr_time[3]=time_ist[3];
                done_sum = 1;
            }

            // LDR2 Closed
            //if((val[0]>top_value)&&(box_open_bit>0))
            //{	box_open_bit=0;}
				
            // LDR1 Removed
            if ((val[1]<=bottom_value) && (box_removed_bit==0) &&(su_mode_bit < 3))
            {
                Refresh_IWDG();// internal wdt reset
                HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
                mode_value = 5;
                send_monitering();
                box_removed_bit=1;
                mode_value=0;
                if(ldr_time<9) ++ldr_time;
                last_ldr_time[0]=time_ist[0];
                last_ldr_time[1]=time_ist[1];
                last_ldr_time[2]=time_ist[2];
                last_ldr_time[3]=time_ist[3];
                done_sum = 1;
            }
            // LDR1 Closed
            //if((val[1]>bottom_value)&&(box_removed_bit > 0))
            //{	box_removed_bit=0;}

            // ACDC Check


        } // read_adc_bit

        // Send alerts if they existing in Q
        if ((alert_loc1 !=0) && (monitoring_send_data_counter==0))  // alerts in the Q try sending it
        {
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            mode_value = 0;
            send_monitering();
            monitoring_send_data_counter=120;// if monitoring send data counter fails then repeat in 2 minute
        }
			
        //----Send Health Data----on Reboot-------------------------------------------
        //----Send Health Data------------Once 24 Hours-----------------------------------
        if((send_fix_tiemr==1) || (health_Data_Timer ==0))
        {
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            send_health();
            if (error_bit != 0) {
                send_fix_tiemr = resend_data_freq;
                if (health_Data_Timer ==0) health_Data_Timer = 24*60*60;
            }
            else if (health_Data_Timer ==0){
                send_fix_tiemr = health_Timer_Value;
                health_Data_Timer = 24*60*60;
                box_open_bit = 0;
                box_removed_bit = 0;
            } else {
                send_fix_tiemr = health_Timer_Value;
                box_open_bit = 0;
                box_removed_bit = 0;
            }
        }

        main_reset_counter=0;	// Indicates the loop is running

        // intialize GSM if GSM did not have power
        if((power_ip==1)&&(power_on_gsm==0))
        {
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            init_gsm();
            SetupSession();
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            power_on_gsm=1;
        }

        // If on battery then switch off GSM if no health packet to be sent
//        if((power_ip==0)&&(power_on_gsm==1)&&(health_Data_Timer>1800))
//        {
//            power_on_gsm=0;
//            hw_gsm_power_off;
//            batdata_minute_counter = batdata_minute_counter_value;
//        }

        if(gsm_reset_counter==0)
        {
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            init_gsm();
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            power_on_gsm=1 ;
            SetupSession();
            gsm_reset_counter=gsm_reset_counter_value;
        }

        // FOTA REQUEST RECEIVED
        if(fota_update_bit==1)
        {
            fota_update_bit=0;
            read_update_value();
            readed_prog_update_value=0xca;
            done = 1;
            RebootDevice();
        }
		
        // No activity for 12 hours then reboot
        if(device_restart_counter==0) {RebootDevice();}

        //------------------------------------------------------

        // If there was an eror in gprs connection and power exists then restart GSM module
	if((local_ip_error==1)&&(power_on_gsm==1)){restart_gsm=1;}
		
        if (restart_gsm==1)
        {
            restart_gsm=0;
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            init_gsm();
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            SetupSession();
            power_on_gsm=1 ;
        }
		

 	//-------------------------------------------------------
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

        // If no network
        if(network==0){creg_reset_counter+=3;delay_ms(100);}
        if((network!=1)&&(network!=5)){++creg_reset_counter;delay_ms(100);}
        if((network==1)||(network==5)){creg_reset_counter=0;}
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

        // If network not found reset GSM
        if(creg_reset_counter>=900)
        {
            network=0;
            creg_reset_counter=0;
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            init_gsm();
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            SetupSession();
            power_on_gsm=1 ;
            if((network==0) || (sessionset==0)) //restart the system
            {
                booting=1;
                RebootDevice();
            }
        }// Not network found
		
        if (summary_wait_time == 1)  {summary_wait_time=0;send_summary();}

        // Read clock every 10 Seconds and when we send data
        if(read_clock_bit==1)
        {
            read_clock();
            read_clock_bit=0;
            temp_int=date_ist[0]-0x30;
            temp_int*=10;
            temp_int+=(date_ist[1]-0x30);

            if(date_int_prev == 0) date_int_prev=temp_int;

            if(date_int_prev!=temp_int)
            {
                load_zero();
                done_sum = 1;
                date_int_prev=temp_int;
                done_rt = 1;
                summary_wait_time = imei[12] - 0x30;
                if (summary_wait_time >3) summary_wait_time =3;
                summary_wait_time *= 100;
                summary_wait_time += (imei[13] - 0x30)*10;
                summary_wait_time += (imei[14] - 0x30);
                if (summary_wait_time ==0) summary_wait_time = 1;
                summary_wait_time *= 60; // Convert to seconds
                if (device_restart_counter <= summary_wait_time) device_restart_counter=summary_wait_time + 1800;
                Refresh_IWDG();// internal wdt reset
                HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
                write_ram_in_flash(0);
                write_update_value();
                write_summary_data(); // must be written
                write_necessary_value();
                write_flash_timer = 3600;
            }  // Alert FLASH writer to store RUN time values
        }

        // Event to read network and refresh
        if(read_network_bit==1)
        {
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            read_network();
            read_network_bit=0;
        }

        // Read SIgnal strength
        if(read_signal_bit==1)
        {
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            read_signal();
            read_signal_bit=0;
        }

        // Refresh ICCID
        if(get_iccid_value==1)
        {
            sessionset = 0;
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            get_iccid_value=0;
            get_iccid_no();
            sessionset = 1;
            formate_iccid_no();
        }

        // Get CellInfo
        if(send_command==1)
        {
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            sessionset = 0;
            find_cell_info();
            send_command=0;
            sessionset = 1;
        }

//---------------------------------------------------------------		
        // Factory Reset
        if(load_default_bit==1)
        {	load_default();
                load_default_bit=0;
                done = 1;
        }

//-------------------------store feedback data
        if(happy > 0) {store_cfd_data(1); --happy;++good_button_count;done_sum=1;}
        if(ok > 0) {store_cfd_data(2);--ok;++ok_button_count;done_sum=1;}
        if(sad > 0)  {store_cfd_data(3); --sad;++bad_button_count;done_sum=1;}
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

        // RAM is full so dump top 341 feedback in the FLASH
        if (flash_store_bit==1) {flash_store_bit = 0 ; write_ram_in_flash(1);}

        // if on utility power
        if((power_ip==1) && ((data_count>=util_dataCount)||((ten_minute_counter==0)&&(data_count!=0)))) {send_ready_data=1;ten_minute_counter = ten_minute_counter_value;}
        // if on battery power
        if((power_ip==0) && (((bat_dataCount>0) && (data_count>=bat_dataCount))||((batdata_minute_counter==0)&&(data_count!=0)))) {send_ready_data=1;batdata_minute_counter = batdata_minute_counter_value;}
//        if((power_ip==0) && (batdata_minute_counter==0) && (data_count!=0)) {send_ready_data=1;batdata_minute_counter = batdata_minute_counter_value;}

        //if(data_count < 10) {send_ready_data=0;}
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
        if(send_ready_data==1){send_ready_data=0;send_cfd();} // Send data
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

        // REmote configuration commands via SMS
        if(sms_recieved>=2)
        {  // Read SMS and process it
            //rcv_ctr2=0;
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            send_gsm("AT+CMGR=1\r");ok_check(50);
            update_values();  // Take action on SMS
            send_gsm("AT+CMGD=1\r");ok_check(300);
            send_gsm("AT+CMGD=1,4\r");ok_check(300);
            send_gsm("AT+QMGDA=\"DEL ALL\"\r");ok_check(100);
            sms_recieved=0;
        }
        write_necessary_value();
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
        write_update_value();
    } // While Loop
		
  /* USER CODE END 3 */

} // Main function



/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(op_led_GPIO_Port, op_led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_PWR_Pin|LED_GSM_Pin|DTR_GSM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GSM_POWER_Pin|POWER_KEY_GSM_Pin|WDT_WDI_Pin|WDT_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : op_led_Pin */
  GPIO_InitStruct.Pin = op_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(op_led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IP4_Pin IP3_Pin */
  GPIO_InitStruct.Pin = IP4_Pin|IP3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IP2_Pin IP1_Pin */
  GPIO_InitStruct.Pin = IP2_Pin|IP1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_PWR_Pin LED_GSM_Pin DTR_GSM_Pin */
  GPIO_InitStruct.Pin = LED_PWR_Pin|LED_GSM_Pin|DTR_GSM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : EXT_BAT_DET_Pin GSM_VDD_EXT_Pin RI_GSM_Pin */
  GPIO_InitStruct.Pin = EXT_BAT_DET_Pin|GSM_VDD_EXT_Pin|RI_GSM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : GSM_POWER_Pin */
  GPIO_InitStruct.Pin = GSM_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GSM_POWER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : POWER_KEY_GSM_Pin WDT_WDI_Pin WDT_EN_Pin */
  GPIO_InitStruct.Pin = POWER_KEY_GSM_Pin|WDT_WDI_Pin|WDT_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */




void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	
	if(huart->Instance==USART1)
	{HAL_UART_DeInit(&huart1);
	MX_USART1_UART_Init();
	HAL_UART_Receive_IT(&huart1,rcvd_data2,1);
	}
 
}













void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_RCC_APB2_PERIPH(RCC_APB2Periph));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    RCC->APB2ENR |= RCC_APB2Periph;
  }
  else
  {
    RCC->APB2ENR &= ~RCC_APB2Periph;
  }
}

void SYSCFG_MemoryRemapConfig(uint32_t SYSCFG_MemoryRemap)
{
  uint32_t tmpctrl = 0;

  /* Check the parameter */
  assert_param(IS_SYSCFG_MEMORY_REMAP(SYSCFG_MemoryRemap));

  /* Get CFGR1 register value */
  tmpctrl = SYSCFG->CFGR1;

  /* Clear MEM_MODE bits */
  tmpctrl &= (uint32_t) (~SYSCFG_CFGR1_MEM_MODE);

  /* Set the new MEM_MODE bits value */
  tmpctrl |= (uint32_t) SYSCFG_MemoryRemap;

  /* Set CFGR1 register with the new memory remap configuration */
  SYSCFG->CFGR1 = tmpctrl;
}








/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

unsigned long date_time_to_epoch(unsigned char *date, unsigned char *time)
{
    unsigned int day = (date[0] - 0x30)*10 + (date[1] - 0x30) - 1;
    unsigned int month = (date[2] - 0x30)*10 + (date[3] - 0x30) - 1;
    unsigned int year = (date[4] - 0x30)*10 + (date[5] - 0x30);
    unsigned int hour   = (time[0] - 0x30)*10 + (time[1] - 0x30);
    unsigned int minute = (time[2] - 0x30)*10 + (time[3] - 0x30);
    unsigned int second = (time[4] - 0x30)*10 + (time[5] - 0x30);
    return (((year/4*(365*4+1)+days[year%4][month]+day)*24+hour)*60+minute)*60+second;
}


void epoch_to_date_time(unsigned char *date ,unsigned char *time, unsigned long epoch)
{
    unsigned int second = epoch%60; epoch /= 60;
    unsigned int minute = epoch%60; epoch /= 60;
    unsigned int hour   = epoch%24; epoch /= 24;
    unsigned int years = epoch/(365*4+1)*4; epoch %= 365*4+1;

    unsigned int year;
    for (year=3; year>0; year--)
    {
        if (epoch >= days[year][0])
            break;
    }

    unsigned int month;
    for (month=11; month>0; month--)
    {
        if (epoch >= days[year][month])
            break;
    }

    unsigned int day   = epoch+1-days[year][month];
    year  = years+year;
    month = month+1;

    date[0] = day/10 + 0x30;
    date[1] = day%10 + 0x30;
    date[2] = month/10 + 0x30;
    date[3] = month%10 + 0x30;
    date[4] = year/10 + 0x30;
    date[5] = year%10 + 0x30;
    date[6] = '\0';

    time[0] = hour/10 + 0x30;
    time[1] = hour%10 + 0x30;
    time[2] = (minute/10) + 0x30;
    time[3] = minute%10 + 0x30;
    time[4] = (second/10) + 0x30;
    time[5] = second%10 + 0x30;
    time[6] = '\0';
}

void RebootDevice()
{
    write_ram_in_flash(0);
    write_necessary_value();
    write_update_value();
    write_summary_data();
    bypass_wdt = 1;
    while (1) {} //AFTER DEBUGGING UNCOMMENT IT
}

void SetupSession(){
    if (power_on_gsm == 0) return;
    sessionset  = 0;
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    find_operator();
    if (error_bit==1) return;
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    select_apn();
    read_network();
    if (error_bit==1) return;
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    activate_gprs();
    if (error_bit == 1) return;
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    read_local_ip();
    if ((local_ip_error == 1)||(error_bit == 1)) return;
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    get_iccid_no();
    formate_iccid_no();
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    find_cell_info();
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    read_signal();
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    gsm_reset_counter=gsm_reset_counter_value;
    sessionset  = 1;
}

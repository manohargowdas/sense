#include "C:\Work\Sensorise\Product\SenseTise\FW\STM32\Code\Build\CFD\MDK-ARM\define.h"


// Externally used Variables ---------------------------------------------
unsigned char mat_no_y;
unsigned int copy_no,current_date_sequence_monitering;
unsigned long current_date_sequence_summary;
unsigned char sdsc_ok;
unsigned char post_string[750];
unsigned char url[100];
unsigned char dataurl[50];
unsigned char alerturl[50];
unsigned char url_cfd[20];
unsigned char url_summary[20];
unsigned char url_helth[20];
unsigned char url_monitoring[20];
unsigned char error_http;
unsigned char url_length_matrix[5];
unsigned char alert_mode_q[10];
unsigned long alert_dttm_q[10];
unsigned char alert_loc1 = 0;
// Locally used Variables ---------------------------------------------


// Functions ---------------------------------------------------


/* Activates the gprs connection ----------------------------------- */
void activate_gprs()
{
    if(power_on_gsm==0) return;
    error_http=0;
    //send_gsm("AT+CREG?\r");ok_check(50);          //Network Registration
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);

    if((network==1)||(network==5))  //HOME (1) or ROAMING (5) network
    {
        send_gsm("AT+QIFGCNT=0\r");ok_check(30);  //Set the context 0 as FGCNT
        send_gsm("AT+QICSGP=1,\"");                 // Select GPRS as  bearer, command not completed
        send_gsm(&apn[0]);                      // Add apn to the previous command
        send_gsm("\"\r");                       //Compelte the command
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);
        ok_check(30);

        send_gsm("AT+QIREGAPP\r");ok_check(30);  // Execute the TCP task - Is it really needed?
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);
        Refresh_IWDG();// internal wdt reset
        send_gsm("AT+QIACT\r");ok_check(1500);    // Activate GPRS context
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);

        if(error_bit==1)
        {ten_minute_counter=ten_minute_counter_value;batdata_minute_counter=batdata_minute_counter_value;}       // Reset the counter so the we do not try to send this data
    }
}

void send_http_string(unsigned char package_type)
{
    //activate_gprs();
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);
    connect_http=0;
    error_http  = 0;
    switch(package_type)
    {
        case 0: // 122
                for(temp_char=0;((dataurl[temp_char]!='\0')&&(temp_char<50));++temp_char)
                {url[temp_char]=dataurl[temp_char];url[temp_char+1]='\0';}

                for(temp_int=0;((url_cfd[temp_int]!='\0')&&(temp_char<100));++temp_int,++temp_char)
                {url[temp_char]=url_cfd[temp_int];url[temp_char+1]='\0';}

                break;
        case 1:	// health packet 145
                for(temp_char=0;((alerturl[temp_char]!='\0')&&(temp_char<50));++temp_char)
                {url[temp_char]=alerturl[temp_char];url[temp_char+1]='\0';}

                for(temp_int=0;((url_helth[temp_int]!='\0')&&(temp_char<100));++temp_int,++temp_char)
                {url[temp_char]=url_helth[temp_int];url[temp_char+1]='\0';}

                break;
        case 2:	// summary_packet 158
                for(temp_char=0;((dataurl[temp_char]!='\0')&&(temp_char<50));++temp_char)
                {url[temp_char]=dataurl[temp_char];url[temp_char+1]='\0';}

                for(temp_int = 0;((url_summary[temp_int]!='\0')&&(temp_char<100));++temp_int,++temp_char)
                {url[temp_char]=url_summary[temp_int];url[temp_char+1]='\0';}

                break;
        case 3:	// monitering_packet 124
                for(temp_char=0;((alerturl[temp_char]!='\0')&&(temp_char<50));++temp_char)
                {url[temp_char]=alerturl[temp_char];url[temp_char+1]='\0';}

                for(temp_int=0;((url_monitoring[temp_int]!='\0')&&(temp_char<100));++temp_int,++temp_char)
                {url[temp_char]=url_monitoring[temp_int];url[temp_char+1]='\0';}

                break;
    }
    temp_int = 0;
    for (temp_char = 0; post_string[temp_char]!='\0';++temp_char,++temp_int);
    for (temp_char = 0; url[temp_char]!='\0';++temp_char,++temp_int);
    url_length_matrix[3]='\0';
    url_length_matrix[2]='\0';
    if (temp_int > 99) {
        url_length_matrix[2]=ascii_number[temp_int%10];
        temp_int/=10;
    }
    url_length_matrix[1]=ascii_number[temp_int%10];
    temp_int/=10;
    url_length_matrix[0]=ascii_number[temp_int%10];

    send_gsm("AT+QHTTPURL=");
    send_gsm(&url_length_matrix[0]);
    send_gsm(",30\r");//ok_check(300);
    if (connect_http==0) {
        temp_char=60;
        while((temp_char)&&(connect_http==0))
        {--temp_char; HAL_Delay(500);
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);
        }
    }
    if(connect_http==1)
    {
        send_gsm(&url[0]);
        send_gsm(&post_string[0]);
        send_gsm("\r");
        ok_check_http();
        if(ok_gsm!=1) {error_http=1;}
        send_gsm("AT+QHTTPGET=60\r"); ok_check(900);
        if((ok_gsm ==0) || (error_bit ==1))
        {error_http=1;}

        if(error_http==0)
        {
            sdsc_ok=0;
            send_gsm("AT+QHTTPREAD=50\r");
            ok_check(700);
            if (package_type <2) {
                temp_char=100;
                while((temp_char)&&(sdsc_ok==0))
                {HAL_Delay(30);--temp_char;
                    Refresh_IWDG();// internal wdt reset
                    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);
                }
            }
            if((sdsc_ok==0)&& (package_type<2))error_http=1;
        }

     }
     else
       error_http=1;

    if (error_http==0){
        gsm_reset_counter=gsm_reset_counter_value;
        device_restart_counter=device_reset_counter_value;
    }

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);
}



//----------------------------------------------------------------------------------
void send_health()
{
    // First check if there is no connectivity problem
    error_bit=1;
    if (power_on_gsm == 0) {init_gsm();power_on_gsm=1;local_ip_error=1;}
    if (local_ip_error!=0)
    {
        power_on_gsm=1;
        SetupSession();
    }
    if (local_ip_error != 0) return;  // could not be sent

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);
    // Wait for session
    temp_char = 10;
    while ((sessionset == 0) && (temp_char >0)) {
        HAL_Delay(100); --temp_char;
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    }

    make_helth_data();
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);
    send_http_string(health_data);

    error_bit=1;
    if (error_http != 1) {
        error_bit=0;
        send_fix_tiemr = health_Timer_Value;
        device_restart_counter=device_reset_counter_value;
        ++current_date_sequence_helth;
        if (current_date_sequence_helth>9999) current_date_sequence_helth = 0;
        last_heart_beat_time[0]=time_ist[0];
        last_heart_beat_time[1]=time_ist[1];
        last_heart_beat_time[2]=time_ist[2];
        last_heart_beat_time[3]=time_ist[3];
        ++heart_beat;
        done_sum = 1;
    } else {
        send_fix_tiemr = resend_data_freq;
    }

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);
}

void send_summary()
{
    if (power_on_gsm == 0) {init_gsm();power_on_gsm=1;local_ip_error=1;}
    if (local_ip_error!=0)
    {
        power_on_gsm=1;
        SetupSession();
    }

    // Wait for ession
    temp_char = 10;
    while ((sessionset == 0) && (temp_char >0))
    {
        HAL_Delay(100); --temp_char;
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    }

    make_summary_data();
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);
    send_http_string(summary_data);
    time_delay=30;
    while((time_delay)&&(error_http==1))
    {
        HAL_Delay(100);--time_delay;
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);
    }
    which_summary = 1;
    if (error_http != 1) {
        device_restart_counter=device_reset_counter_value;
        ++current_date_sequence_summary;
        if (current_date_sequence_summary>9999) current_date_sequence_summary = 0;
        summary_wait_time = 0;
    } else {
        summary_wait_time = resend_data_freq;
    }

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);
}


void send_monitering()// current loc data
{
    unsigned char i,j;
    unsigned char dt[7], tm[7];

    // First check if there is no connectivity problem
    if (power_on_gsm == 0) {init_gsm();power_on_gsm=1;local_ip_error=1;}
    if (local_ip_error!=0)
    {
        power_on_gsm=1;
        SetupSession();
    }

    // Add the packet in the Q if possible
    if ((alert_loc1 < 10)&&(mode_value>0))
    {
        read_clock();  // If only new alert is available then get time
        alert_mode_q[alert_loc1] = mode_value;
        alert_dttm_q[alert_loc1] = date_time_to_epoch(date_ist, time_ist);
        alert_loc1++;
    }
    mode_value = 0; // reset the alert
    if (local_ip_error != 0) return ;

    // Wait for the session
    temp_char = 10;
    while ((sessionset == 0) && (temp_char >0)) {
        HAL_Delay(100); --temp_char;
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);
    }
    i = 0;
    while (i < alert_loc1)
    {
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
        epoch_to_date_time(dt,tm,alert_dttm_q[i]);
        make_monitoring_data(dt,tm,alert_mode_q[i]);
        send_http_string(monitoring_data);

        if (error_http == 1)
            break;  // Cound not send
        else {
            i++;
            device_restart_counter=device_reset_counter_value;
            ++current_date_sequence_monitering;
            if(current_date_sequence_monitering>9999) current_date_sequence_monitering=0;
        }
    }
    // Push the unsent alerts above if at least one of the existing alert was sent
    if (i>0)
    {
        for (j = i; j < alert_loc1; j++)
        {
            alert_dttm_q[j-i] = alert_dttm_q[j];
            alert_mode_q[j-i] = alert_mode_q[j];
        }
        alert_loc1 = alert_loc1 - i;
    }

    mode_value = 0; // reset the alert
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);
}




void send_cfd()// current loc data
{
    unsigned int i;
    unsigned char dt[7], tm[7];

    if (power_on_gsm == 0) {init_gsm();power_on_gsm=1;local_ip_error=1;}
    if (local_ip_error!=0)
    {
        power_on_gsm=1;
        SetupSession();
    }

    // Wait for the session to be available
    temp_char = 10;
    while ((sessionset == 0) && (temp_char >0)) {
        HAL_Delay(100); --temp_char;
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);
    }
    // Still an error in getting tcp ip connection
    if(local_ip_error!=0)
    {
        ten_minute_counter=ten_minute_counter_value;
        batdata_minute_counter = batdata_minute_counter_value;
        return;
    }


    for (i=ram_sent_loc; i < ram_store_loc; i++)
    {
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
        epoch_to_date_time(dt,tm,ram_datetime[i]);
        if ((ram_cfd[i] > 0) && (ram_cfd[i] < 4))
        {
            make_button_data(dt,tm,ram_cfd[i]);

            send_http_string(cfd_data);

            if(error_http==1)  break;
            lastSentData = ram_datetime[i];
            device_restart_counter=device_reset_counter_value;
            done_rt = 1;
            ++current_date_sequence_button;
            if (current_date_sequence_button>9999) current_date_sequence_button = 0;
            last_button_press_record_time[0]=tm[0];
            last_button_press_record_time[1]=tm[1];
            last_button_press_record_time[2]=tm[2];
            last_button_press_record_time[3]=tm[3];
            last_button_press_record_time[4]=tm[4];
            last_button_press_record_time[5]=tm[5];
            done_sum = 1;
        }
        if (data_count > 0) --data_count;
    }

    ram_sent_loc = i;
    // if we have sent all the data for first page then dump the page out
    if (ram_sent_loc > 340) write_ram_in_flash(1);
    data_count = 0;
    ten_minute_counter=ten_minute_counter_value;
    batdata_minute_counter = batdata_minute_counter_value;
}








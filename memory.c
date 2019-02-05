#include "C:\Work\Sensorise\Product\SenseTise\FW\STM32\Code\Build\CFD\MDK-ARM\define.h"

static FLASH_EraseInitTypeDef EraseInitStruct;
unsigned int data_store_counter;
unsigned int mem_ctr,temp_int;

unsigned int rolling_mem = 0;
static unsigned long bank[18]={0x08015800,0x08016000,0x08016800,0x08017000,0x08017800,0x08018000,0x08018800,0x08019000,0x08019800,0x0801a000,0x0801a800,0x0801b000,0x0801b800,0x0801c000,0x0801c800,0x0801d000,0x0801d800,0x0801e000};
//static unsigned long bank[18]={0x0800f000,0x0800f800,0x08010000,0x08010800,0x08011000,0x08011800,0x08012000,0x08012800,0x08013000,0x08013800,0x08014000,0x08014800,0x08015000,0x08015800,0x08016000,0x08016800,0x08017000,0x08017800};//,0x08018000,0x08012800,0x08012000,0x08012800,0x08012000,0x08012800,};
unsigned long bank_start_datetime[18]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
unsigned long ram_datetime[682] ;
unsigned char ram_cfd[682] ;
unsigned int  ram_store_loc = 0;
unsigned int  ram_sent_loc = 0;
unsigned char flash_store_bit = 0;

unsigned long lastSentData = 0;
unsigned long  FLASH_USER_History_ADDR;
unsigned char dirty1 = 0, dirty2 = 0;  // which ram section is dirty

void store_cfd_data(unsigned char cfd)
{
    read_clock();
    flash_store_bit = 0;
    if (ram_store_loc > 681) {flash_store_bit = 1; return;} // ignore should not happen
    ram_cfd[ram_store_loc] = cfd;
    ram_datetime[ram_store_loc] = date_time_to_epoch(date_ist, time_ist);
    ++data_count;  // Data ready to be send
    ++ram_store_loc;
    if (ram_store_loc > 681)
        flash_store_bit = 1;
    else {
        ram_datetime[ram_store_loc] = 0; // end of record
        ram_cfd[ram_store_loc] = 0;
    }
    if (ram_store_loc <= 341)
        dirty1= 1;
    else
        dirty2 = 1;
}


// state: 0 full ram save due to reboot
// state: 1 top page only and rellocate the data
void write_ram_in_flash(unsigned char state)
{
    unsigned int i, cnt,m ;

    if (dirty1 == 1)
    { // if only top page is dirty we need to save it
        FLASH_USER_History_ADDR=bank[rolling_mem];
        mem_ctr=0;//temp_ign=FLASH_USER_START_ADDR;
        HAL_FLASH_Unlock();
        delay_ms(100);
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
        EraseInitStruct.TypeErase = TYPEERASE_PAGES;
        EraseInitStruct.PageAddress = FLASH_USER_History_ADDR;
        EraseInitStruct.NbPages =1;// (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;

        HAL_FLASHEx_Erase(&EraseInitStruct,&PageError);
        delay_ms(100);
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

        // initialize bank start date
        bank_start_datetime[rolling_mem] = ram_datetime[0]; // Store start date of the bank
        done_rt = 1;
        cnt = 341;
        if (ram_store_loc < cnt) cnt = ram_store_loc;  // adjust to the available data

        mem_ctr = 0;
        temp_int = 0x43;  // Some Bit representation which is not the default state of FLASH
        HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_History_ADDR+mem_ctr),temp_int);mem_ctr+=2;
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
        for (i=0; i < cnt ; i++)
        {
            // first store date time
            temp_long= ram_datetime[i];
            temp_long&=0xffff0000;
            temp_long>>=16;
            HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_History_ADDR+mem_ctr),temp_long);mem_ctr+=2;
            temp_long= ram_datetime[i];
            temp_long&=0x0000ffff;
            HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_History_ADDR+mem_ctr),temp_long);mem_ctr+=2;
            // then store feedback
            temp_int = ram_cfd[i];
            HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_History_ADDR+mem_ctr),temp_int);mem_ctr+=2;
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
        }
        if (cnt < 341) { // RAM Page1 is not full then put 0
            temp_int = 0; // end of record
            HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_History_ADDR+mem_ctr),temp_int);mem_ctr+=2;
            HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_History_ADDR+mem_ctr),temp_int);mem_ctr+=2;
            // then store feedback
            HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_History_ADDR+mem_ctr),temp_int);mem_ctr+=2;
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
        }
        dirty1 = 0;
        HAL_FLASH_Lock();
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    }

    // Store the 2nd page only if required (state == 0 - all)
    if ((dirty2 == 1)&&(state==0))
    {
        m = rolling_mem + 1;
        if (m > 17) m=0;  // rollover the FLASH bank
        FLASH_USER_History_ADDR=bank[m];
        HAL_FLASH_Unlock();
        delay_ms(100);
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
        EraseInitStruct.TypeErase = TYPEERASE_PAGES;
        EraseInitStruct.PageAddress = FLASH_USER_History_ADDR;
        EraseInitStruct.NbPages =1;// (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;

        HAL_FLASHEx_Erase(&EraseInitStruct,&PageError);
        delay_ms(100);
        mem_ctr = 0;
        temp_int = 0x43;  // Some random memory structure to indicate that data is stored here
        HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_History_ADDR+mem_ctr),temp_int);mem_ctr+=2;
        bank_start_datetime[m] = ram_datetime[341];
        done_rt = 1;
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
        for (i=341; i < ram_store_loc ; i++)
        {
            // first store date time
            temp_long=ram_datetime[i];
            temp_long&=0xffff0000;
            temp_long>>=16;
            HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_History_ADDR+mem_ctr),temp_long);mem_ctr+=2;
            temp_long=ram_datetime[i];
            temp_long&=0x0000ffff;
            HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_History_ADDR+mem_ctr),temp_long);mem_ctr+=2;
            // then store feedback
            temp_int = ram_cfd[i];
            HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_History_ADDR+mem_ctr),temp_int);mem_ctr+=2;
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
        }
        if (ram_store_loc < 682) {
            temp_int = 0; // end of record
            HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_History_ADDR+mem_ctr),temp_int);mem_ctr+=2;
            HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_History_ADDR+mem_ctr),temp_int);mem_ctr+=2;
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            // then store feedback
            HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_History_ADDR+mem_ctr),temp_int);mem_ctr+=2;
        }
        dirty2 = 0;
        HAL_FLASH_Lock();
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    } // stored page 2 also

    write_flash_timer = 3600;  // Next write after 1 hour
    if (state == 0) return;  // full dump due to reboot

    // If state==1 then move the page2 up provided we have data in page2
    if (ram_store_loc >= 341)
    {
        for (i=341; i < ram_store_loc ; i++)
        {
            ram_cfd[i-341] = ram_cfd[i];
            ram_datetime[i-341] = ram_datetime[i];
            if (ram_datetime[i]==0) break;
        }
        ram_sent_loc -= 341;
        ++rolling_mem;
        if (rolling_mem > 17) rolling_mem = 0;
    }
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
}



void read_flash_in_ram()
{
    // read blocks with last sent data and greater
    // find the block with largest datetime
    // We can only read latest two block at max
    unsigned int i , m;
    m = 0;
    for (i=1; i <= 17; i++) if (bank_start_datetime[i] > bank_start_datetime[m]) m = i;
    if (bank_start_datetime[m] == 0) return; // nothing to read
    rolling_mem = m;// read this block
    if (bank_start_datetime[m] > lastSentData) {
        // Read previous and this block
        if ((m==0) && (bank_start_datetime[17]>0)) rolling_mem =17;
        if ((m>0) && (bank_start_datetime[m-1]>0)) rolling_mem =m-1;
    }

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    ram_sent_loc = 0;
    ram_store_loc = 0;
    dirty1 = 0;
    dirty2 = 0;

    FLASH_USER_History_ADDR=bank[rolling_mem];
    HAL_FLASH_Unlock();
    //i = 0;
    mem_ctr = 0;
    temp_int =*((uint16_t *)( FLASH_USER_History_ADDR+mem_ctr));mem_ctr+=2;
    if (temp_int != 0x43) {HAL_FLASH_Lock();return;}
    ram_sent_loc = 0;
    for(ram_store_loc = 0; ram_store_loc < 341; ram_store_loc++)
    {
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
        /// Read date time in epoch
        ram_datetime[ram_store_loc]=*((uint16_t *)( FLASH_USER_History_ADDR+mem_ctr));mem_ctr+=2;
        ram_datetime[ram_store_loc]<<=16;
        ram_datetime[ram_store_loc]|=*((uint16_t *)( FLASH_USER_History_ADDR+mem_ctr));mem_ctr+=2;
        //read cfd
        temp_int =*((uint16_t *)( FLASH_USER_History_ADDR+mem_ctr));mem_ctr+=2;
        ram_cfd[ram_store_loc] = temp_int;
        if (ram_datetime[ram_store_loc]==0) break;  // End of data
        if (ram_cfd[ram_store_loc]==0) break;  // End of data
        // set location from where data need to be sent
        if ((ram_datetime[ram_store_loc]<=lastSentData)) {ram_sent_loc = ram_store_loc+1; }
        //if ((ram_datetime[ram_store_loc]>lastSentData) && (i==0)) {ram_sent_loc = ram_store_loc; i = 1;}
    }
    HAL_FLASH_Lock();

    if ((rolling_mem != m)&& (ram_store_loc==341)) {
        // read another block
        //rolling_mem = m;  // Do not change the rolling pointer as first page points to the previous bank
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
        FLASH_USER_History_ADDR=bank[m];
        mem_ctr = 0;
        temp_int =*((uint16_t *)( FLASH_USER_History_ADDR+mem_ctr));mem_ctr+=2;
        if (temp_int != 0x43) {HAL_FLASH_Lock();return;}
        for(; ram_store_loc < 682; ram_store_loc++)
        {
            Refresh_IWDG();// internal wdt reset
            HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
            /// Read date time in epoch
            ram_datetime[ram_store_loc]=*((uint16_t *)( FLASH_USER_History_ADDR+mem_ctr));mem_ctr+=2;
            ram_datetime[ram_store_loc]<<=16;
            ram_datetime[ram_store_loc]|=*((uint16_t *)( FLASH_USER_History_ADDR+mem_ctr));mem_ctr+=2;
            //read cfd
            temp_int =*((uint16_t *)( FLASH_USER_History_ADDR+mem_ctr));mem_ctr+=2;
            ram_cfd[ram_store_loc] = temp_int;
            if (ram_datetime[ram_store_loc]==0) break;  // End of data
            if (ram_cfd[ram_store_loc]==0) break;  // End of data
            if ((ram_datetime[ram_store_loc]<=lastSentData)) {ram_sent_loc = ram_store_loc+1; }
            //if ((ram_datetime[ram_store_loc]>lastSentData) && (i==0)) {ram_sent_loc = ram_store_loc; i = 1;}
        }
        HAL_FLASH_Lock();
    }
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
}

void write_summary_data()
{
    if (done_sum == 0) return;

    HAL_FLASH_Unlock();
    HAL_Delay(100);
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

    EraseInitStruct.TypeErase = TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_USER_Monitoring_ADDR;
    EraseInitStruct.NbPages =1;// (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;
    HAL_FLASHEx_Erase(&EraseInitStruct,&PageError);
    HAL_Delay(100);

    mem_ctr=0;
    // previous day
    temp_int = 0x43;
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_Monitoring_ADDR+mem_ctr),temp_int);mem_ctr+=2;
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_Monitoring_ADDR+mem_ctr),heart_beat_o);mem_ctr+=2;
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_Monitoring_ADDR+mem_ctr),good_button_count_o);mem_ctr+=2;
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_Monitoring_ADDR+mem_ctr),ok_button_count_o);mem_ctr+=2;
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_Monitoring_ADDR+mem_ctr),bad_button_count_o);mem_ctr+=2;
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_Monitoring_ADDR+mem_ctr),no_of_power_outage_o);mem_ctr+=2;
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_Monitoring_ADDR+mem_ctr),su_mode_act_cnt_o);mem_ctr+=2;
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_Monitoring_ADDR+mem_ctr),ldr_time_o);mem_ctr+=2;
    mem_ctr+=2;  // FREE

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<4;temp_char++)
    {HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_Monitoring_ADDR+mem_ctr),last_heart_beat_time_o[temp_char]);mem_ctr+=2;}

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<6;temp_char++)
    {HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_Monitoring_ADDR+mem_ctr),last_button_press_record_time_o[temp_char]);mem_ctr+=2;}

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<4;temp_char++)
    {HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_Monitoring_ADDR+mem_ctr),last_power_outage_time_o[temp_char]);mem_ctr+=2;}

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<4;temp_char++)
    {HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_Monitoring_ADDR+mem_ctr),last_su_mode_time_o[temp_char]);mem_ctr+=2;}

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<4;temp_char++)
    {HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_Monitoring_ADDR+mem_ctr),last_ldr_time_o[temp_char]);mem_ctr+=2;}

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<4;temp_char++) mem_ctr+=2; // FREE

    // Current day
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_Monitoring_ADDR+mem_ctr),heart_beat);mem_ctr+=2;
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_Monitoring_ADDR+mem_ctr),good_button_count);mem_ctr+=2;
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_Monitoring_ADDR+mem_ctr),ok_button_count);mem_ctr+=2;
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_Monitoring_ADDR+mem_ctr),bad_button_count);mem_ctr+=2;
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_Monitoring_ADDR+mem_ctr),no_of_power_outage);mem_ctr+=2;
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_Monitoring_ADDR+mem_ctr),su_mode_act_cnt);mem_ctr+=2;
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_Monitoring_ADDR+mem_ctr),ldr_time);mem_ctr+=2;
    mem_ctr+=2;   // FREE

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<4;temp_char++)
    {HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_Monitoring_ADDR+mem_ctr),last_heart_beat_time[temp_char]);mem_ctr+=2;}

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<6;temp_char++)
    {HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_Monitoring_ADDR+mem_ctr),last_button_press_record_time[temp_char]);mem_ctr+=2;}

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<4;temp_char++)
    {HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_Monitoring_ADDR+mem_ctr),last_power_outage_time[temp_char]);mem_ctr+=2;}

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<4;temp_char++)
    {HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_Monitoring_ADDR+mem_ctr),last_su_mode_time[temp_char]);mem_ctr+=2;}

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<4;temp_char++)
    {HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_Monitoring_ADDR+mem_ctr),last_ldr_time[temp_char]);mem_ctr+=2;}

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<4;temp_char++) mem_ctr+=2;  // FREE

    HAL_FLASH_Lock();
    done_sum = 0;
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
}

void read_summary_data()
{
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

    HAL_FLASH_Unlock();mem_ctr=0;

    temp_int =	*((uint16_t *)( FLASH_USER_Monitoring_ADDR+mem_ctr));mem_ctr+=2;
    if (temp_int != 0x43) {HAL_FLASH_Lock(); return;}

    heart_beat_o =	*((uint16_t *)( FLASH_USER_Monitoring_ADDR+mem_ctr));mem_ctr+=2;
    good_button_count_o=	*((uint16_t *)( FLASH_USER_Monitoring_ADDR+mem_ctr));mem_ctr+=2;
    ok_button_count_o=	*((uint16_t *)( FLASH_USER_Monitoring_ADDR+mem_ctr));mem_ctr+=2;
    bad_button_count_o=	*((uint16_t *)( FLASH_USER_Monitoring_ADDR+mem_ctr));mem_ctr+=2;
    no_of_power_outage_o=	*((uint16_t *)( FLASH_USER_Monitoring_ADDR+mem_ctr));mem_ctr+=2;
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    su_mode_act_cnt_o =	*((uint16_t *)( FLASH_USER_Monitoring_ADDR+mem_ctr));mem_ctr+=2;
    ldr_time_o=	*((uint16_t *)( FLASH_USER_Monitoring_ADDR+mem_ctr));mem_ctr+=2;
    mem_ctr+=2;
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

    for(temp_char=0;temp_char<4;temp_char++)
    {last_heart_beat_time_o[temp_char]=	*((uint16_t *)( FLASH_USER_Monitoring_ADDR+mem_ctr));mem_ctr+=2;}

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<6;temp_char++)
    {last_button_press_record_time_o[temp_char]=	*((uint16_t *)( FLASH_USER_Monitoring_ADDR+mem_ctr));mem_ctr+=2;}

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<4;temp_char++)
    {last_power_outage_time_o[temp_char]=	*((uint16_t *)( FLASH_USER_Monitoring_ADDR+mem_ctr));mem_ctr+=2;}

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<4;temp_char++)
    {last_su_mode_time_o[temp_char]=	*((uint16_t *)( FLASH_USER_Monitoring_ADDR+mem_ctr));mem_ctr+=2;}

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<4;temp_char++)
    {last_ldr_time_o[temp_char]=	*((uint16_t *)( FLASH_USER_Monitoring_ADDR+mem_ctr));mem_ctr+=2;}

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<4;temp_char++) mem_ctr+=2;  // FREE

    //Current day
    heart_beat=	*((uint16_t *)( FLASH_USER_Monitoring_ADDR+mem_ctr));mem_ctr+=2;
    good_button_count=	*((uint16_t *)( FLASH_USER_Monitoring_ADDR+mem_ctr));mem_ctr+=2;
    ok_button_count=	*((uint16_t *)( FLASH_USER_Monitoring_ADDR+mem_ctr));mem_ctr+=2;
    bad_button_count=	*((uint16_t *)( FLASH_USER_Monitoring_ADDR+mem_ctr));mem_ctr+=2;
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    no_of_power_outage=	*((uint16_t *)( FLASH_USER_Monitoring_ADDR+mem_ctr));mem_ctr+=2;
    su_mode_act_cnt =	*((uint16_t *)( FLASH_USER_Monitoring_ADDR+mem_ctr));mem_ctr+=2;
    ldr_time=	*((uint16_t *)( FLASH_USER_Monitoring_ADDR+mem_ctr));mem_ctr+=2;
    mem_ctr+=2;  // FREE

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<4;temp_char++)
    {last_heart_beat_time[temp_char]=	*((uint16_t *)( FLASH_USER_Monitoring_ADDR+mem_ctr));mem_ctr+=2;}

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<6;temp_char++)
    {last_button_press_record_time[temp_char]=	*((uint16_t *)( FLASH_USER_Monitoring_ADDR+mem_ctr));mem_ctr+=2;}

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<4;temp_char++)
    {last_power_outage_time[temp_char]=	*((uint16_t *)( FLASH_USER_Monitoring_ADDR+mem_ctr));mem_ctr+=2;}

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<4;temp_char++)
    {last_su_mode_time[temp_char]=	*((uint16_t *)( FLASH_USER_Monitoring_ADDR+mem_ctr));mem_ctr+=2;}

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<4;temp_char++)
    {last_ldr_time[temp_char]=	*((uint16_t *)( FLASH_USER_Monitoring_ADDR+mem_ctr));mem_ctr+=2;}

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<4;temp_char++) mem_ctr+=2;  // FREE

    HAL_FLASH_Lock();
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

}


// Write RUN time values in the FLASH for future retrieval
void write_necessary_value()
{
    unsigned long temp_long;
    unsigned short i;
    unsigned temp_int;

    if (done_rt == 0) return;

    mem_ctr=0;//temp_ign=FLASH_USER_START_ADDR;

    HAL_FLASH_Unlock();
    delay_ms(100);
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

    EraseInitStruct.TypeErase = TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_USER_NECESSARY_ADDR;
    EraseInitStruct.NbPages =1;// (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;
	
    HAL_FLASHEx_Erase(&EraseInitStruct,&PageError);
    delay_ms(100);

    temp_int = 0x43;
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_NECESSARY_ADDR+mem_ctr),temp_int);mem_ctr+=2;
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset


    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_NECESSARY_ADDR+mem_ctr),date_int_prev);mem_ctr+=2;

    temp_long=lastSentData;
    temp_long&=0xffff0000;
    temp_long>>=16;
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_NECESSARY_ADDR+mem_ctr),temp_long);mem_ctr+=2;
    temp_long=lastSentData;
    temp_long&=0x0000ffff;
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_NECESSARY_ADDR+mem_ctr),temp_long);mem_ctr+=2;
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

    // Store bank_start_DateTime
    for(i = 0; i < 18; i++)
    {
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
        temp_long=bank_start_datetime[i];
        temp_long&=0xffff0000;
        temp_long>>=16;

        HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_NECESSARY_ADDR+mem_ctr),temp_long);mem_ctr+=2;
        temp_long=bank_start_datetime[i];
        temp_long&=0x0000ffff;
        HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_USER_NECESSARY_ADDR+mem_ctr),temp_long);mem_ctr+=2;
    }

    HAL_FLASH_Lock();
    done_rt = 0;
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
}


// Read run time information stored in RAM
void read_necessary_value()
{
    unsigned int i;//mod

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    mem_ctr=0;
    HAL_FLASH_Unlock();

    temp_int=	*((uint16_t *)( FLASH_USER_NECESSARY_ADDR));mem_ctr+=2;
    if (temp_int != 0x43) {HAL_FLASH_Lock();return;}


    date_int_prev = *((uint16_t *)( FLASH_USER_NECESSARY_ADDR+mem_ctr));mem_ctr+=2;

    lastSentData = *((uint16_t *)( FLASH_USER_NECESSARY_ADDR+mem_ctr));mem_ctr+=2;
    lastSentData<<=16;
    lastSentData |= *((uint16_t *)( FLASH_USER_NECESSARY_ADDR+mem_ctr));mem_ctr+=2;
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(i = 0; i < 18; i++)
    {
        Refresh_IWDG();// internal wdt reset
        HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
        bank_start_datetime[i] = *((uint16_t *)( FLASH_USER_NECESSARY_ADDR+mem_ctr));mem_ctr+=2;
        bank_start_datetime[i] <<= 16;
        bank_start_datetime[i] |= *((uint16_t *)( FLASH_USER_NECESSARY_ADDR+mem_ctr));mem_ctr+=2;
    }


    HAL_FLASH_Lock();
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

}


	
// Reads FOTA parameters in the FLASH
void read_update_value()
{
    HAL_FLASH_Unlock();
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);

    mem_ctr=2;
    temp_int=	*((uint16_t *)( FLASH_PROG_UPDATE_BIT_START_ADDR));mem_ctr+=2;
    if (temp_int != 0x45) {HAL_FLASH_Lock();return;}

    mem_ctr=0;
    readed_prog_update_value=	*((uint16_t *)( FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr));mem_ctr+=2;
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);
    mem_ctr=4;
    for(temp_char=0;temp_char<19;temp_char++)
    {fota_ip[temp_char]=	*((uint16_t *)( FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr));fota_ip[temp_char+1]='\0';mem_ctr+=2;}
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

    for(temp_char=0;temp_char<14;temp_char++)
    {fota_user[temp_char]=	*((uint16_t *)( FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr));fota_user[temp_char+1]='\0';mem_ctr+=2;}
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

    for(temp_char=0;temp_char<14;temp_char++)
    {fota_pass[temp_char]=	*((uint16_t *)( FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr));fota_pass[temp_char+1]='\0';mem_ctr+=2;}
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

    for(temp_char=0;temp_char<29;temp_char++)mem_ctr+=2;
    //{fota_folder[temp_char]=	*((uint16_t *)( FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr));fota_folder[temp_char+1]='\0';mem_ctr+=2;}
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

    for(temp_char=0;temp_char<14;temp_char++)
    {fota_file[temp_char]=	*((uint16_t *)( FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr));fota_file[temp_char+1]='\0';mem_ctr+=2;}
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

    for(temp_char=0;temp_char<10;temp_char++)
    {phone_no_admin[temp_char]=	*((uint16_t *)( FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr));phone_no_admin[temp_char+1]='\0';mem_ctr+=2;}
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

    useCustomAPN =	*((uint16_t *)( FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr));mem_ctr+=2;
    if (useCustomAPN != 1) useCustomAPN = 0;

    for(temp_char=0;temp_char<20;temp_char++)
    {custom_apn[temp_char]=	*((uint16_t *)( FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr));custom_apn[temp_char+1]='\0';mem_ctr+=2;}
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

    util_dataCount = *((uint16_t *)( FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr));mem_ctr+=2;
//124
    ten_minute_counter_value = *((uint16_t *)( FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr));mem_ctr+=2;
    ten_minute_counter_value<<=16;
    ten_minute_counter_value |= *((uint16_t *)( FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr));mem_ctr+=2;

//126
    bat_dataCount = *((uint16_t *)( FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr));mem_ctr+=2;

//127
    batdata_minute_counter_value = *((uint16_t *)( FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr));mem_ctr+=2;
    batdata_minute_counter_value<<=16;
    batdata_minute_counter_value |= *((uint16_t *)( FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr));mem_ctr+=2;
//129
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

    health_Timer_Value = *((uint16_t *)( FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr));mem_ctr+=2;
    health_Timer_Value<<=16;
    health_Timer_Value |= *((uint16_t *)( FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr));mem_ctr+=2;

//131
    device_reset_counter_value = *((uint16_t *)( FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr));mem_ctr+=2;
    device_reset_counter_value<<=16;
    device_reset_counter_value |= *((uint16_t *)( FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr));mem_ctr+=2;

//133
    lowBatLevel = *((uint16_t *)( FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr));mem_ctr+=2;
    criticalBatLevel = *((uint16_t *)( FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr));mem_ctr+=2;
    ldr_event_gap = *((uint16_t *)( FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr));mem_ctr+=2;

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

    for(temp_char=0;temp_char<11;temp_char++) {mem_ctr+=2;} // Keep 24 int space free

    version_loaded=	*((uint16_t *)( FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr));mem_ctr+=2;
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

    for(temp_char=0;temp_char<50;temp_char++)
    {dataurl[temp_char]=	*((uint16_t *)( FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr));dataurl[temp_char+1]='\0';mem_ctr+=2;}
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);

    for(temp_char=0;temp_char<50;temp_char++)
    {alerturl[temp_char]=	*((uint16_t *)( FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr));alerturl[temp_char+1]='\0';mem_ctr+=2;}
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);

    for(temp_char=0;temp_char<19;temp_char++)
    {apn_mem[temp_char]=	*((uint16_t *)( FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr));apn_mem[temp_char+1]='\0';mem_ctr+=2;}
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

    fota_port[0]='2';
    fota_port[1]='1';
    fota_port[2]='\0';


    HAL_FLASH_Lock();
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);
}




// Writes FOTA parameters in the FLASH
void write_update_value()
{
    if (done == 0) return;

    HAL_FLASH_Unlock();
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);
    EraseInitStruct.TypeErase = TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_PROG_UPDATE_BIT_START_ADDR;
    EraseInitStruct.NbPages =1;
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
      {
        return;  // FIX to send ALERT
      }
    HAL_Delay(100);

    mem_ctr=2;
    temp_int = 0x45;
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr),temp_int);
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

    mem_ctr=0;
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr),readed_prog_update_value);mem_ctr+=2;HAL_Delay(10);
//2
    mem_ctr=4;
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<19;temp_char++)
    {HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr),fota_ip[temp_char]);mem_ctr+=2;HAL_Delay(10);}
//21
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<14;temp_char++)
    {HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr),fota_user[temp_char]);mem_ctr+=2;HAL_Delay(10);}
//35
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<14;temp_char++)
    {HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr),fota_pass[temp_char]);mem_ctr+=2;HAL_Delay(10);}
//49

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<29;temp_char++)mem_ctr+=2;
    //{HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr),fota_folder[temp_char]);mem_ctr+=2;HAL_Delay(10);}
//78

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<14;temp_char++)
    {HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr),fota_file[temp_char]);mem_ctr+=2;HAL_Delay(10);}
//92
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<10;temp_char++)
    {HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr),phone_no_admin[temp_char]);mem_ctr+=2;HAL_Delay(10);}
//102
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr),useCustomAPN);mem_ctr+=2;HAL_Delay(10);
//103
    for(temp_char=0;temp_char<20;temp_char++)
    {HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr),custom_apn[temp_char]);mem_ctr+=2;HAL_Delay(10);}
//123
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr),util_dataCount);mem_ctr+=2;HAL_Delay(10);
//124
    temp_long=ten_minute_counter_value;
    temp_long&=0xffff0000;
    temp_long>>=16;
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr),temp_long);mem_ctr+=2;
    temp_long=ten_minute_counter_value;
    temp_long&=0x0000ffff;
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr),temp_long);mem_ctr+=2;
//126
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr),bat_dataCount);mem_ctr+=2;HAL_Delay(10);
//127
    temp_long=batdata_minute_counter_value;
    temp_long&=0xffff0000;
    temp_long>>=16;
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr),temp_long);mem_ctr+=2;
    temp_long=batdata_minute_counter_value;
    temp_long&=0x0000ffff;
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr),temp_long);mem_ctr+=2;
//129
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

    temp_long=health_Timer_Value;
    temp_long&=0xffff0000;
    temp_long>>=16;
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr),temp_long);mem_ctr+=2;
    temp_long=health_Timer_Value;
    temp_long&=0x0000ffff;
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr),temp_long);mem_ctr+=2;
//131
    temp_long=device_reset_counter_value;
    temp_long&=0xffff0000;
    temp_long>>=16;
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr),temp_long);mem_ctr+=2;
    temp_long=device_reset_counter_value;
    temp_long&=0x0000ffff;
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr),temp_long);mem_ctr+=2;
//133
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr),lowBatLevel);mem_ctr+=2;HAL_Delay(10);
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr),criticalBatLevel);mem_ctr+=2;HAL_Delay(10);
    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr),ldr_event_gap);mem_ctr+=2;HAL_Delay(10);

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

    for(temp_char=0;temp_char<11;temp_char++) {mem_ctr+=2;} // Keep 11 int space free
//147

    HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr),version_loaded);mem_ctr+=2;HAL_Delay(10);

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<50;temp_char++)
    {HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr),dataurl[temp_char]);mem_ctr+=2;HAL_Delay(10);}

    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<50;temp_char++)
    {HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr),alerturl[temp_char]);mem_ctr+=2;HAL_Delay(10);}

    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset
    for(temp_char=0;temp_char<19;temp_char++)
    {HAL_FLASH_Program(TYPEPROGRAM_HALFWORD,(FLASH_PROG_UPDATE_BIT_START_ADDR+mem_ctr),apn_mem[temp_char]);mem_ctr+=2;HAL_Delay(10);}
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);// external wdt reset

    HAL_FLASH_Lock();
    Refresh_IWDG();// internal wdt reset
    HAL_GPIO_TogglePin(WDI_PORT,WDI_PIN);
    done  = 0;
}

 Sense is the application name developed under Ministry of urban affairs for Swatch Bharath 
as developed an embedded device (referred CFD) using ST Microelectronics STM32F0 MCU and Quectel 
M66 GSM module. Device has three interrupts on STM32 MCU
1) System Clock (programmed for every mili-second)
2) External IC Watch Dog Reset Timer (HAL_GPIO_TogglePin) (1.26 Sec)
3) Internal Software Watch Dog Reset Timer (HAL_IWDG_Refresh) (26 Sec)
MCU reads following digital or analog inputs through GPIO or Analog input pins. 
1) Sensor connected to GPIO
2) Battery level (Analog input)
3) LDR1 (Device Removal) and LDR2 (Device Open) through GPIO
4) Utility Power Status
Controller Clock is 48MHz and UART Baud Rate is 9600
Two types of memory are in use – FLASH and RAM. FLASH is for storing the code and configuration 

Firmware is in the continuous loop mode and hence takes lot of battery. 600mAH battery lasts only 4 
hours. So improved the time for the battery by using Real-Time Clock (RTC) and deep sleep mode of the 
MCU. Device should wake up every 10-15 Seconds and check the GPIO readings, take action if any and go to sleep 
again. 
Suggested development environment is Keil 

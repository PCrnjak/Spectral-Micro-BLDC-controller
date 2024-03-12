#include "hw_init.h"
#include "pwm_init.h"
#include "utils.h"
#include "stm32f1xx_hal.h"
#include "adc_init.h"
#include "iodefs.h"
#include "qfplib-m3.h"
#include "SerialPacketParser.h"
#include "constants.h"
#include "motor_control.h"
#include <SPI.h>
#include "common.h"
#include "foc.h"
#include "temperature_table.h"
#include "CAN.h"
#include <Wire.h>
#include <I2C_eeprom.h>
#include "communication.h"
#include "EEPROM.h"
#include "communication_CAN.h"


// Define serial port
#define Serial Serialx
HardwareSerial Serialx(RX_COM, TX_COM); // PA3, PA2 RX,TX

// Define timer for PWM
HardwareTimer *MyTim2;
ADC_HandleTypeDef hadc1;

void setup()
{

  SPI.setMOSI(MOSI);
  SPI.setMISO(MISO);
  SPI.setSCLK(CLK);
  SPI.setClockDivider(SPI_CLOCK_DIV8); // High speed (128 / 8 = 16 MHz SPI_1 speed)

  Init_Digital_Inputs();
  Init_Digital_Outputs();

  // Sleep mode input.Logic high to enable device;
  // logic low to enter low-power sleep mode; internal pulldown
  digitalWriteFast(SLEEP, HIGH);
  // Reset input. Active-low reset input initializes internal logic, clears faults,
  // and disables the outputs, internal pulldown
  digitalWriteFast(RESET, HIGH);

  digitalWriteFast(EN1, HIGH);
  digitalWriteFast(EN2, HIGH);
  digitalWriteFast(EN3, HIGH);
  digitalWriteFast(LED, HIGH);

  // Init system
  HAL_Init();
  SystemClock_Config();
  TM_SystemClock_Config(14); //15, Clock speed = 8*15

  // Init ADC
  MX_ADC1_Init();
  HAL_ADCEx_Calibration_Start(&hadc1);

  // Begin serial
  Serial.begin(SERIAL_SPEED);

  delay(50);

  digitalWrite(EEPROM_WP, LOW);
  Init_EEPROM();
  //Write_config();
  read_config();
  delay(100);

  // Begin PWM 50000 = 25Khz -> center aligned
  // PWM duty for 25khz goes from 0 - 5120 if CPU clock is 128 Mhz
  MyTim2 = pwm_high(PWM_CH1, PWM_FREQ * 2);
  MyTim2 = pwm_high(PWM_CH2, PWM_FREQ * 2);
  MyTim2 = pwm_high(PWM_CH3, PWM_FREQ * 2);
  pwm_align(MyTim2);

  // Collect data before entering the ISR
  // This fixes velocity error
  for (int i = 0; i < 20; i++)
  {
    Collect_data();
    delayMicroseconds(20);
  }

  Ticker_init(TIM3, LOOP_FREQ, IT_callback);

  Setup_CAN_bus();
}

//////////////////////////////////////////////////////////////////////////////

void loop()
{

  uint32_t ms = HAL_GetTick();
  /*
    HANDLE LED
  */
  if (controller.LED_ON_OFF == 1)
  {

    LED_status(ms);
  }

  static uint32_t last_time = 0;

  if ((ms - last_time) > 500) // run every x ms
  {

    last_time = ms;
  }

  /*
   HANDLE CALIBRATION REPORT
  */
  Calib_report(Serial);
  /*
   UART CLI INTERFACE
  */
  UART_protocol(Serial);
  Cyclic_UART(Serial, ms);
  /*
  HANDLE CAN BUS
  */
  CAN_protocol(Serial);
  CAN_heartbeat(ms);
  CAN_watchdog(ms);



}

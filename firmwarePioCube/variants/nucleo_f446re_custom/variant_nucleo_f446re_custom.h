/*
 *******************************************************************************
 * Copyright (c) 2011-2021, STMicroelectronics
 * All rights reserved.
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */
#pragma once
#include <stdint.h>


/*----------------------------------------------------------------------------
 *        STM32 pins number
 *----------------------------------------------------------------------------*/
#define PA3                     PIN_A14
#define PA2                     PIN_A15
#define PA10                    2
#define PB3                     3
#define PB5                     4
#define PB4                     5
#define PB10                    6
#define PA8                     7
#define PA9                     8
#define PC7                     9
#define PB6                     10
#define PA7                     PIN_A6
#define PA6                     PIN_A7
#define PA5                     PIN_A8 // LD2
#define PB9                     14
#define PB8                     15
// ST Morpho
// CN7 Left Side
#define PC10                    16
#define PC12                    17
// 18 is NC - BOOT0
#define PA13                    19 // SWD
#define PA14                    20 // SWD
#define PA15                    21
#define PB7                     22
#define PC13                    23 // USER_BTN
#define PC14                    24 // NC by default SB49 opened
#define PC15                    25 // NC by default SB48 opened
#define PH0                     26 // NC by default SB55 opened
#define PH1                     27
#define PC2                     PIN_A9
#define PC3                     PIN_A10
// CN7 Right Side
#define PC11                    30
#define PD2                     31
// CN10 Left Side
#define PC9                     32
// CN10 Right side
#define PC8                     33
#define PC6                     34
#define PC5                     PIN_A11
#define PA12                    36
#define PA11                    37
#define PB12                    38
// 39 is NC
#define PB2                     40
#define PB1                     PIN_A12
#define PB15                    42
#define PB14                    43
#define PB13                    44
#define PC4                     PIN_A13
#define PA0                     PIN_A0
#define PA1                     PIN_A1
#define PA4                     PIN_A2
#define PB0                     PIN_A3
#define PC1                     PIN_A4
#define PC0                     PIN_A5

// Alternate pins number
#define PA0_ALT1                (PA0  | ALT1)
#define PA0_ALT2                (PA0  | ALT2)
#define PA1_ALT1                (PA1  | ALT1)
#define PA1_ALT2                (PA1  | ALT2)
#define PA2_ALT1                (PA2  | ALT1)
#define PA2_ALT2                (PA2  | ALT2)
#define PA3_ALT1                (PA3  | ALT1)
#define PA3_ALT2                (PA3  | ALT2)
#define PA4_ALT1                (PA4  | ALT1)
#define PA5_ALT1                (PA5  | ALT1)
#define PA6_ALT1                (PA6  | ALT1)
#define PA7_ALT1                (PA7  | ALT1)
#define PA7_ALT2                (PA7  | ALT2)
#define PA7_ALT3                (PA7  | ALT3)
#define PA15_ALT1               (PA15 | ALT1)
#define PB0_ALT1                (PB0  | ALT1)
#define PB0_ALT2                (PB0  | ALT2)
#define PB1_ALT1                (PB1  | ALT1)
#define PB1_ALT2                (PB1  | ALT2)
#define PB3_ALT1                (PB3  | ALT1)
#define PB4_ALT1                (PB4  | ALT1)
#define PB5_ALT1                (PB5  | ALT1)
#define PB8_ALT1                (PB8  | ALT1)
#define PB8_ALT2                (PB8  | ALT2)
#define PB9_ALT1                (PB9  | ALT1)
#define PB9_ALT2                (PB9  | ALT2)
#define PB14_ALT1               (PB14 | ALT1)
#define PB14_ALT2               (PB14 | ALT2)
#define PB15_ALT1               (PB15 | ALT1)
#define PB15_ALT2               (PB15 | ALT2)
#define PC0_ALT1                (PC0  | ALT1)
#define PC0_ALT2                (PC0  | ALT2)
#define PC1_ALT1                (PC1  | ALT1)
#define PC1_ALT2                (PC1  | ALT2)
#define PC2_ALT1                (PC2  | ALT1)
#define PC2_ALT2                (PC2  | ALT2)
#define PC3_ALT1                (PC3  | ALT1)
#define PC3_ALT2                (PC3  | ALT2)
#define PC4_ALT1                (PC4  | ALT1)
#define PC5_ALT1                (PC5  | ALT1)
#define PC6_ALT1                (PC6  | ALT1)
#define PC7_ALT1                (PC7  | ALT1)
#define PC8_ALT1                (PC8  | ALT1)
#define PC9_ALT1                (PC9  | ALT1)
#define PC10_ALT1               (PC10 | ALT1)
#define PC11_ALT1               (PC11 | ALT1)

#define NUM_DIGITAL_PINS        52
#define NUM_ANALOG_INPUTS       16

// On-board user button
#ifndef USER_BTN
  #define USER_BTN              PC13
#endif

static const uint32_t NOPIN = 0xFFFFFFFF;

#define SERIAL_TX_BUFFER_SIZE 256
#define SERIAL_RX_BUFFER_SIZE 256

// Timer Definitions
// Use TIM6/TIM7 when possible as servo and tone don't need GPIO output pin
#ifndef TIMER_TONE
  #define TIMER_TONE            TIM6
#endif
#ifndef TIMER_SERVO
  #define TIMER_SERVO           TIM7
#endif

// UART Definitions
#ifndef SERIAL_UART_INSTANCE
  #define SERIAL_UART_INSTANCE  2 //Connected to ST-Link
#endif

// Motor Pins
#ifndef L_WHEEL_FORW_PIN
  #define L_WHEEL_FORW_PIN  PB6
#endif
#ifndef L_WHEEL_BACK_PIN
  #define L_WHEEL_BACK_PIN  PB7
#endif
#ifndef R_WHEEL_FORW_PIN
  #define R_WHEEL_FORW_PIN  PA7
#endif
#ifndef R_WHEEL_BACK_PIN
  #define R_WHEEL_BACK_PIN  PA9 // PA6
#endif

// Encoder pins
// J4
#define L_ENCODER_PIN1  PA0 // TIM5_CH1
#define L_ENCODER_PIN2  PA1 // TIM5_CH2
#define R_ENCODER_PIN1  PA5 // TIM2_CH1
#define R_ENCODER_PIN2  PB3 // TIM2_CH2

// GPS Pins
#define GPS_RX_PIN PC10 // RX on the GPS
#define GPS_TX_PIN PC11 // TX on the GPS
#define GPS_RESET_N_PIN PA15
#define GPS_1PPS_PIN PC12 // 1PPS on the GPS
#define GPS_FORCE_ON_N_PIN PD2

// Default pin used for 'Serial' instance (ex: ST-Link)
// Mandatory for Firmata
#ifndef PIN_SERIAL_RX
  #define PIN_SERIAL_RX         PA3
#endif
#ifndef PIN_SERIAL_TX
  #define PIN_SERIAL_TX         PA2
#endif

// Level Shift
#define ENCODER_LVL_SHIFTER_EN  PB5

/* HAL modules */
#define HAL_SD_MODULE_DISABLED
#define HAL_RTC_MODULE_DISABLED
#define HAL_ETH_MODULE_DISABLED
#define HAL_EXTI_MODULE_DISABLED
#define HAL_QSPI_MODULE_DISABLED
#define HAL_SAI_MODULE_DISABLED
#define HAL_ICACHE_MODULE_DISABLED
#define HAL_CAN_LEGACY_MODULE_DISABLED
#define HAL_CAN_LEGACY_MODULE_DISABLED
#define HAL_CEC_MODULE_DISABLED
#define HAL_COMP_MODULE_DISABLED
#define HAL_CORDIC_MODULE_DISABLED
#define HAL_CRC_MODULE_DISABLED
#define HAL_CRYP_MODULE_DISABLED
#define HAL_DCMI_MODULE_DISABLED
#define HAL_DFSDM_MODULE_DISABLED
#define HAL_DMA2D_MODULE_DISABLED
#define HAL_DSI_MODULE_DISABLED
#define HAL_EXTI_MODULE_DISABLED // interrupt API does not use the module
#define HAL_FDCAN_MODULE_DISABLED
#define HAL_FIREWALL_MODULE_DISABLED
#define HAL_FMAC_MODULE_DISABLED
#define HAL_FMPI2C_MODULE_DISABLED
#define HAL_GFXMMU_MODULE_DISABLED
#define HAL_GTZC_MODULE_DISABLED
#define HAL_HASH_MODULE_DISABLED
#define HAL_HCD_MODULE_DISABLED
#define HAL_HRTIM_MODULE_DISABLED
#define HAL_IRDA_MODULE_DISABLED
#define HAL_IWDG_MODULE_DISABLED // IWD built-in library uses LL
#define HAL_JPEG_MODULE_DISABLED
#define HAL_LCD_MODULE_DISABLED
#define HAL_LPTIM_MODULE_DISABLED
#define HAL_LTDC_MODULE_DISABLED
#define HAL_MDIOS_MODULE_DISABLED
#define HAL_MDMA_MODULE_DISABLED
#define HAL_MMC_MODULE_DISABLED
#define HAL_NAND_MODULE_DISABLED
#define HAL_NOR_MODULE_DISABLED
#define HAL_OPAMP_MODULE_DISABLED
#define HAL_OTFDEC_MODULE_DISABLED
#define HAL_PCCARD_MODULE_DISABLED
#define HAL_PKA_MODULE_DISABLED
#define HAL_RAMECC_MODULE_DISABLED
#define HAL_RNG_MODULE_DISABLED
#define HAL_SDADC_MODULE_DISABLED
#define HAL_SDRAM_MODULE_DISABLED
#define HAL_SMARTCARD_MODULE_DISABLED
#define HAL_SMBUS_MODULE_DISABLED
#define HAL_SPDIFRX_MODULE_DISABLED
#define HAL_SRAM_MODULE_DISABLED
#define HAL_SUBGHZ_MODULE_DISABLED
#define HAL_SWPMI_MODULE_DISABLED
#define HAL_TSC_MODULE_DISABLED
#define HAL_USART_MODULE_DISABLED
#define HAL_WWDG_MODULE_DISABLED

#if !defined(HAL_DAC_MODULE_DISABLED)
  #define HAL_DAC_MODULE_ENABLED
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
  // These serial port names are intended to allow libraries and architecture-neutral
  // sketches to automatically default to the correct port name for a particular type
  // of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
  // the first hardware serial port whose RX/TX pins are not dedicated to another use.
  //
  // SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
  //
  // SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
  //
  // SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
  //
  // SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
  //
  // SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
  //                            pins are NOT connected to anything by default.
  #ifndef SERIAL_PORT_MONITOR
    #define SERIAL_PORT_MONITOR   Serial
  #endif
  #ifndef SERIAL_PORT_HARDWARE
    #define SERIAL_PORT_HARDWARE  Serial
  #endif
#endif

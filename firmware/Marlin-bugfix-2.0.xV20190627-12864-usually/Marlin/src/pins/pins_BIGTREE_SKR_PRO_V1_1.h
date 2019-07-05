/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * STM32F407VET6 with RAMPS-like shield
 * 'Black' STM32F407VET6 board - http://wiki.stm32duino.com/index.php?title=STM32F407
 * Shield - https://github.com/jmz52/Hardware
 */

#pragma once

#if !defined(STM32F4) && !defined(STM32F4xx)
  #error "Oops! Select an STM32F4 board in 'Tools > Board.'"
#endif

#define DEFAULT_MACHINE_NAME "STM32F407ZGT6"
//#define BOARD_NAME "Black STM32F4VET6"

//#define I2C_EEPROM
//#define E2END 0x1FFF // EEPROM end address (8kB)
#define EEPROM_EMULATED_WITH_SRAM

#if HOTENDS > 3 || E_STEPPERS > 3
  #error "Black STM32F4VET6 supports up to 3 hotends / E-steppers."
#endif

//
// Servos
//

//#define SERVO0_PIN        PC9  
#define SERVO0_PIN        PA1

//
// Limit Switches
//
#define X_MIN_PIN          PB10
#define X_MAX_PIN          PE15
#define Y_MIN_PIN          PE12
#define Y_MAX_PIN          PE10
#define Z_MIN_PIN          PG8
#define Z_MAX_PIN          PG5

//
// Steppers
//
#define X_STEP_PIN         PE9
#define X_DIR_PIN          PF1
#define X_ENABLE_PIN       PF2
#ifndef X_CS_PIN
  #define X_CS_PIN         PA15
#endif


#define Y_STEP_PIN         PE11
#define Y_DIR_PIN          PE8
#define Y_ENABLE_PIN       PD7
 #ifndef Y_CS_PIN
  #define Y_CS_PIN         PB8
#endif

#define Z_STEP_PIN         PE13
#define Z_DIR_PIN          PC2
#define Z_ENABLE_PIN       PC0
#ifndef Z_CS_PIN
  #define Z_CS_PIN         PB9
#endif


#define E0_STEP_PIN        PE14
#define E0_DIR_PIN         PA0
#define E0_ENABLE_PIN      PC3
#ifndef E0_CS_PIN
  #define E0_CS_PIN        PB3
#endif


#define E1_STEP_PIN        PD15
#define E1_DIR_PIN         PE7
#define E1_ENABLE_PIN      PA3
#ifndef E1_CS_PIN
  #define E1_CS_PIN        PG15
#endif


#define E2_STEP_PIN        PD13
#define E2_DIR_PIN         PG9
#define E2_ENABLE_PIN      PF0
#ifndef E2_CS_PIN
  #define E2_CS_PIN        PG12
#endif



//
// Software SPI pins for TMC2130 stepper drivers
//
#if ENABLED(TMC_USE_SW_SPI)
  #define TMC_SW_MOSI      PC12
  #define TMC_SW_MISO      PC11
  #define TMC_SW_SCK       PC10

#endif
#if HAS_DRIVER(TMC2208)
    /**
   * TMC2208 stepper drivers
   *
   * Hardware serial communication ports.
   * If undefined software serial is used according to the pins below
   */
  //#define X_HARDWARE_SERIAL  Serial
  //#define X2_HARDWARE_SERIAL Serial1
  //#define Y_HARDWARE_SERIAL  Serial1
  //#define Y2_HARDWARE_SERIAL Serial1
  //#define Z_HARDWARE_SERIAL  Serial1
  //#define Z2_HARDWARE_SERIAL Serial1
  //#define E0_HARDWARE_SERIAL Serial1
  //#define E1_HARDWARE_SERIAL Serial1
  //#define E2_HARDWARE_SERIAL Serial1
  //#define E3_HARDWARE_SERIAL Serial1
  //#define E4_HARDWARE_SERIAL Serial1

  //
  // Software serial
  //   
  #define X_SERIAL_TX_PIN  PC13   // PC13
  #define X_SERIAL_RX_PIN  PC13  //  PE4

  #define Y_SERIAL_TX_PIN  PE3
  #define Y_SERIAL_RX_PIN  PE3  //PE2

  #define Z_SERIAL_TX_PIN  PE1
  #define Z_SERIAL_RX_PIN  PE1 //E0

  #define E0_SERIAL_TX_PIN PD4  //PD2
  #define E0_SERIAL_RX_PIN PD4

  #define E1_SERIAL_TX_PIN PD1 //PD0
  #define E1_SERIAL_RX_PIN PD1

  #define Z2_SERIAL_TX_PIN PD6 //
  #define Z2_SERIAL_RX_PIN PD6 //PD5

#endif
//
// Temperature Sensors
//
#define TEMP_0_PIN         PF3 // T0
#define TEMP_1_PIN         PF4  // T1
#define TEMP_2_PIN         PF5  // T2
#define TEMP_BED_PIN       PF6  // TB

//
// Heaters / Fans
//
#define HEATER_0_PIN       PB1  // Heater0
#define HEATER_1_PIN       PD14 // Heater1
#define HEATER_2_PIN       PB0  // Heater1
#define HEATER_BED_PIN     PD12 // Hotbed

#define FAN_PIN            PC8  // Fan0
#define FAN1_PIN            PE5  // Fan1
#define FAN2_PIN            PE6  // Fan2
//
// POWER_SUPPLY
//
#define PS_ON_PIN         PF8
//
// Misc. Functions
//
#define SDSS               PB12

/*
|               _____                                             _____
|           NC | · · | GND                                    5V | · · | GND
|        RESET | · · | PF12(SD_DETECT)             (LCD_D7)  PG7 | · · | PG6  (LCD_D6)
|   (MOSI)PB15 | · · | PF11(BTN_EN2)               (LCD_D5)  PG3 | · · | PG2  (LCD_D4)
|  (SD_SS)PB12 | · · | PG10(BTN_EN1)               (LCD_RS) PD10 | · · | PD11 (LCD_EN)
|    (SCK)PB13 | · · | PB14(MISO)                 (BTN_ENC)  PA8 | · · | PG4  (BEEPER)
|               ￣￣                                               ￣￣
|               EXP2                                              EXP1
*/
#if ENABLED(ULTRA_LCD)
  #define BEEPER_PIN        PG4
  #define BTN_ENC           PA8

  #if ENABLED(CR10_STOCKDISPLAY)
    #define LCD_PINS_RS     PG6

    #define BTN_EN1         PD11
    #define BTN_EN2         PG2

    #define LCD_PINS_ENABLE PG7
    #define LCD_PINS_D4     PG3

  #else

    #define LCD_PINS_RS     PD10

    #define BTN_EN1         PG10
    #define BTN_EN2         PF11
    #define SD_DETECT_PIN   PF12

    #define LCD_SDSS        PB12

    #define LCD_PINS_ENABLE PD11
    #define LCD_PINS_D4     PG2

    #if ENABLED(ULTIPANEL)
      #define LCD_PINS_D5   PG3
      #define LCD_PINS_D6   PG6
      #define LCD_PINS_D7   PG7
    #endif

  #endif

#endif // ULTRA_LCD


#define SCK_PIN          PB13
#define MISO_PIN         PB14
#define MOSI_PIN         PB15
#define SS_PIN           PB12   // Chip select for SD card used by Marlin

/*
//
// LCD / Controller
//
#define SD_DETECT_PIN      PC5
//#define SD_DETECT_PIN      PA8  // SDIO SD_DETECT_PIN, external SDIO card reader only

#define BEEPER_PIN         PD10
#define LCD_PINS_RS        PE15
#define LCD_PINS_ENABLE    PD8
#define LCD_PINS_D4        PE10
#define LCD_PINS_D5        PE12
#define LCD_PINS_D6        PD1
#define LCD_PINS_D7        PE8
#define BTN_ENC            PD9
#define BTN_EN1            PD4
#define BTN_EN2            PD13

#define DOGLCD_CS          LCD_PINS_D5
#define DOGLCD_A0          LCD_PINS_D6
*/
// Alter timing for graphical display
#ifndef ST7920_DELAY_1
  #define ST7920_DELAY_1 DELAY_NS(96)
#endif
#ifndef ST7920_DELAY_2
  #define ST7920_DELAY_2 DELAY_NS(48)
#endif
#ifndef ST7920_DELAY_3
  #define ST7920_DELAY_3 DELAY_NS(715)
#endif
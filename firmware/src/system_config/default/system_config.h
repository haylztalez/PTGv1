/*******************************************************************************
  MPLAB Harmony System Configuration Header

  File Name:
    system_config.h

  Summary:
    Build-time configuration header for the system defined by this MPLAB Harmony
    project.

  Description:
    An MPLAB Project may have multiple configurations.  This file defines the
    build-time options for a single configuration.

  Remarks:
    This configuration header must not define any prototypes or data
    definitions (or include any files that do).  It only provides macro
    definitions for build-time configuration options that are not instantiated
    until used by another MPLAB Harmony module or application.

    Created with MPLAB Harmony Version 2.05
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
// DOM-IGNORE-END

#ifndef _SYSTEM_CONFIG_H
#define _SYSTEM_CONFIG_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
/*  This section Includes other configuration headers necessary to completely
    define this configuration.
*/


// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: System Service Configuration
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
/* Common System Service Configuration Options
*/
#define SYS_VERSION_STR           "2.05"
#define SYS_VERSION               20500

// *****************************************************************************
/* Clock System Service Configuration Options
*/
#define SYS_CLK_FREQ                        12288000ul
#define SYS_CLK_BUS_PERIPHERAL_1            12288000ul
#define SYS_CLK_BUS_PERIPHERAL_2            12288000ul
#define SYS_CLK_BUS_PERIPHERAL_3            12288000ul
#define SYS_CLK_BUS_PERIPHERAL_4            12288000ul
#define SYS_CLK_BUS_PERIPHERAL_5            12288000ul
#define SYS_CLK_BUS_PERIPHERAL_6            3072000ul
#define SYS_CLK_BUS_PERIPHERAL_7            12288000ul
#define SYS_CLK_BUS_REFERENCE_1             3073500ul
#define SYS_CLK_BUS_REFERENCE_4             48000ul
#define SYS_CLK_CONFIG_PRIMARY_XTAL         24576000ul
#define SYS_CLK_CONFIG_SECONDARY_XTAL       32768ul
   
/*** Ports System Service Configuration ***/
#define SYS_PORT_A_ANSEL        0xE26F
#define SYS_PORT_A_TRIS         0xFFEF
#define SYS_PORT_A_LAT          0x0000
#define SYS_PORT_A_ODC          0x0000
#define SYS_PORT_A_CNPU         0x0000
#define SYS_PORT_A_CNPD         0x0000
#define SYS_PORT_A_CNEN         0x0000

#define SYS_PORT_B_ANSEL        0x0187
#define SYS_PORT_B_TRIS         0xFFFF
#define SYS_PORT_B_LAT          0x0000
#define SYS_PORT_B_ODC          0x0000
#define SYS_PORT_B_CNPU         0x0000
#define SYS_PORT_B_CNPD         0x0000
#define SYS_PORT_B_CNEN         0x0000

#define SYS_PORT_C_ANSEL        0xFC1F
#define SYS_PORT_C_TRIS         0xFFFF
#define SYS_PORT_C_LAT          0x0000
#define SYS_PORT_C_ODC          0x0000
#define SYS_PORT_C_CNPU         0x0000
#define SYS_PORT_C_CNPD         0x0000
#define SYS_PORT_C_CNEN         0x0000

#define SYS_PORT_D_ANSEL        0xFE9F
#define SYS_PORT_D_TRIS         0xFFFF
#define SYS_PORT_D_LAT          0x0000
#define SYS_PORT_D_ODC          0x0000
#define SYS_PORT_D_CNPU         0x0000
#define SYS_PORT_D_CNPD         0x0000
#define SYS_PORT_D_CNEN         0x0000

#define SYS_PORT_E_ANSEL        0x3FFF
#define SYS_PORT_E_TRIS         0x3FFF
#define SYS_PORT_E_LAT          0x0000
#define SYS_PORT_E_ODC          0x0000
#define SYS_PORT_E_CNPU         0x0000
#define SYS_PORT_E_CNPD         0x0000
#define SYS_PORT_E_CNEN         0x0000

#define SYS_PORT_F_ANSEL        0xFFFC
#define SYS_PORT_F_TRIS         0xFFFF
#define SYS_PORT_F_LAT          0x0000
#define SYS_PORT_F_ODC          0x0000
#define SYS_PORT_F_CNPU         0x0000
#define SYS_PORT_F_CNPD         0x0000
#define SYS_PORT_F_CNEN         0x0000

#define SYS_PORT_G_ANSEL        0xFE3F
#define SYS_PORT_G_TRIS         0xFFFF
#define SYS_PORT_G_LAT          0x0000
#define SYS_PORT_G_ODC          0x0000
#define SYS_PORT_G_CNPU         0x0000
#define SYS_PORT_G_CNPD         0x0000
#define SYS_PORT_G_CNEN         0x0000


/*** Interrupt System Service Configuration ***/
#define SYS_INT                     true

// *****************************************************************************
// *****************************************************************************
// Section: Driver Configuration
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
/* USART Driver Configuration Options
*/
#define DRV_USART_INSTANCES_NUMBER                  1
#define DRV_USART_CLIENTS_NUMBER                    1
#define DRV_USART_INTERRUPT_MODE                    true
#define DRV_USART_BYTE_MODEL_SUPPORT                true
#define DRV_USART_READ_WRITE_MODEL_SUPPORT          false
#define DRV_USART_BUFFER_QUEUE_SUPPORT              false

// *****************************************************************************
// *****************************************************************************
// Section: Middleware & Other Library Configuration
// *****************************************************************************
// *****************************************************************************



// *****************************************************************************
// *****************************************************************************
// Section: Application Configuration
// *****************************************************************************
// *****************************************************************************
/*** Application Defined Pins ***/

/*** Functions for TRIGGER pin ***/
#define TRIGGER_PORT PORT_CHANNEL_A
#define TRIGGER_PIN PORTS_BIT_POS_7
#define TRIGGER_PIN_MASK (0x1 << 7)

/*** Functions for DAC_ZEROL pin ***/
#define DAC_ZEROL_PORT PORT_CHANNEL_G
#define DAC_ZEROL_PIN PORTS_BIT_POS_6
#define DAC_ZEROL_PIN_MASK (0x1 << 6)

/*** Functions for DAC_ZEROH pin ***/
#define DAC_ZEROH_PORT PORT_CHANNEL_G
#define DAC_ZEROH_PIN PORTS_BIT_POS_7
#define DAC_ZEROH_PIN_MASK (0x1 << 7)

/*** Functions for ENC_B pin ***/
#define ENC_B_PORT PORT_CHANNEL_A
#define ENC_B_PIN PORTS_BIT_POS_12
#define ENC_B_PIN_MASK (0x1 << 12)

/*** Functions for ENC_A pin ***/
#define ENC_A_PORT PORT_CHANNEL_A
#define ENC_A_PIN PORTS_BIT_POS_11
#define ENC_A_PIN_MASK (0x1 << 11)

/*** Functions for LED2 pin ***/
#define LED2_PORT PORT_CHANNEL_E
#define LED2_PIN PORTS_BIT_POS_14
#define LED2_PIN_MASK (0x1 << 14)

/*** Functions for LED1 pin ***/
#define LED1_PORT PORT_CHANNEL_E
#define LED1_PIN PORTS_BIT_POS_15
#define LED1_PIN_MASK (0x1 << 15)

/*** Functions for BAR_DAC_CS pin ***/
#define BAR_DAC_CS_PORT PORT_CHANNEL_A
#define BAR_DAC_CS_PIN PORTS_BIT_POS_4
#define BAR_DAC_CS_PIN_MASK (0x1 << 4)

/*** Functions for BUTTON2 pin ***/
#define BUTTON2_PORT PORT_CHANNEL_F
#define BUTTON2_PIN PORTS_BIT_POS_1
#define BUTTON2_PIN_MASK (0x1 << 1)

/*** Functions for BUTTON1 pin ***/
#define BUTTON1_PORT PORT_CHANNEL_B
#define BUTTON1_PIN PORTS_BIT_POS_10
#define BUTTON1_PIN_MASK (0x1 << 10)

/*** Functions for BUTTON4 pin ***/
#define BUTTON4_PORT PORT_CHANNEL_B
#define BUTTON4_PIN PORTS_BIT_POS_11
#define BUTTON4_PIN_MASK (0x1 << 11)

/*** Functions for BUTTON3 pin ***/
#define BUTTON3_PORT PORT_CHANNEL_B
#define BUTTON3_PIN PORTS_BIT_POS_12
#define BUTTON3_PIN_MASK (0x1 << 12)


/*** Application Instance 0 Configuration ***/
#define APP_DRV_USART                     0

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif // _SYSTEM_CONFIG_H
/*******************************************************************************
 End of File
*/

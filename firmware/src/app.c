/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include <stdio.h>

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

/* defining LEDS with peripheral library*/
#define LED1_ON() PLIB_PORTS_PinSet( PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_15);
#define LED1_OFF() PLIB_PORTS_PinClear( PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_15);
#define LED2_ON() PLIB_PORTS_PinSet( PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_14);
#define LED2_OFF() PLIB_PORTS_PinClear( PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_14);

/* defining LEDS using LAT register */
#define LED1 LATEbits.LATE15

/* defining BUTTONS using PORT registers */
#define BUTTON1 PORTBbits.RB10
#define BUTTON2 PORTFbits.RF1
#define BUTTON3 PORTBbits.RB12
#define BUTTON4 PORTBbits.RB11


#define SS3_TRIGGER LATFbits.LATF0


/* delay function, used as delay_ms(100) to delay 100 ms */
void delay_ms(int n){
    float k,l;
    k = 3.73254;
    l = 17.234;
	int i;
	int j;
	for (i=0;i<n;i++)
		for (j=0;j<48193;j++)
		{k = k*l;}
}


APP_DATA appData;
static uint8_t __attribute__ ((aligned (16))) app_spi_tx_buffer[] = {'H'};

static uint8_t __attribute__ ((aligned (16))) app_spi_rx_buffer[sizeof(app_spi_tx_buffer)];

//static uint8_t app_tx_buf[] = "Hello World\r\n";
static enum 
{
    USART_BM_INIT,
    USART_BM_WORKING,
    USART_BM_DONE,
} usartBMState;



/* allows us to use "printf()" to write to serial port */
void _mon_putc(const char print_byte)
{
    while(DRV_USART_TransmitBufferIsFull(appData.handleUSART0)) 
    {
        
    }

    DRV_USART_WriteByte(appData.handleUSART0, print_byte);
}

static unsigned char tx_packet[]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

void send_packet(unsigned char x)
{
   
    DRV_SPI_BufferAddWrite( appData.handleSPI0, tx_packet, x, NULL, NULL );
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* state machine for the SPI */
static void SPI_Task(void)
{
    /* run the state machine here for SPI */
    switch (appData.spiStateMachine)
    {
        default:
        case APP_SPI_STATE_START:
            /* Blocking: When the call exits, all data has been exchanged (if we get a valid handle) */
            SS3_TRIGGER = 0;
            appData.drvSPIBufferHandle = DRV_SPI_BufferAddWriteRead(appData.handleSPI0,
                app_spi_tx_buffer, sizeof(app_spi_tx_buffer),
                app_spi_rx_buffer, sizeof(app_spi_rx_buffer),
                0, 0);

            if (DRV_SPI_BUFFER_HANDLE_INVALID != appData.drvSPIBufferHandle)
            {
                /* Blocking Write/Read has finished */
                appData.spiStateMachine =  APP_SPI_STATE_DONE;
            }

            if (DRV_SPI_BUFFER_HANDLE_INVALID == appData.drvSPIBufferHandle)
            {
                /* try again if we get a bad handle */
                appData.spiStateMachine =  APP_SPI_STATE_START;
            }
        break;

        case APP_SPI_STATE_DONE:
            SS3_TRIGGER = 1;
        break;
    }
}

/******************************************************************************
  Function:
    static void USART_Task (void)
    
   Remarks:
    Feeds the USART transmitter by reading characters from a specified pipe.  The pipeRead function is a 
    standard interface that allows data to be exchanged between different automatically 
    generated application modules.  Typically, the pipe is connected to the application's
    USART receive function, but could be any other Harmony module which supports the pipe interface. 
*/
//static void USART_Task (void)
//{
//    switch (usartBMState)
//    {
//        default:
//        case USART_BM_INIT:
//        {
//            appData.tx_count = 0;
//            usartBMState = USART_BM_WORKING;
//            break;
//        }
//
//        case USART_BM_WORKING:
//        {
//            
////            if (appData.tx_count < sizeof(app_tx_buf)) 
////            {
////                if(!DRV_USART_TransmitBufferIsFull(appData.handleUSART0))
////                {
////                    DRV_USART_WriteByte(appData.handleUSART0, app_tx_buf[appData.tx_count]);
////                    appData.tx_count++;
////                }
////            }
////
////            /* Have we finished? */
////            if (appData.tx_count == sizeof(app_tx_buf))
////            {
////                usartBMState = USART_BM_DONE;
////            }
//            break;
//        }
//
//        case USART_BM_DONE:
//        {
//            break;
//        }
//    }
//}

/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    appData.handleSPI0 = DRV_HANDLE_INVALID;
    appData.handleUSART0 = DRV_HANDLE_INVALID;
    //appData.handleSPI3 = DRV_HANDLE_INVALID;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
       

            if (DRV_HANDLE_INVALID == appData.handleSPI0)
            {
                appData.handleSPI0 = DRV_SPI_Open(0, DRV_IO_INTENT_READWRITE);
                appInitialized &= (DRV_HANDLE_INVALID != appData.handleSPI0);
            }
            if (appData.handleUSART0 == DRV_HANDLE_INVALID)
            {
                appData.handleUSART0 = DRV_USART_Open(APP_DRV_USART, DRV_IO_INTENT_READWRITE|DRV_IO_INTENT_NONBLOCKING);
                appInitialized &= ( DRV_HANDLE_INVALID != appData.handleUSART0 );
            }
//            
//            if (appData.handleSPI3 == DRV_HANDLE_INVALID)
//            {
//                appData.handleSPI3 = DRV_SPI_Open(APP_DRV_SPI3, DRV_IO_INTENT_READWRITE|DRV_IO_INTENT_BLOCKING);
//                appInitialized &= ( DRV_HANDLE_INVALID != appData.handleSPI3 );
//            }
        
            if (appInitialized)
            {
                /* initialize the SPI state machine */
                appData.spiStateMachine = APP_SPI_STATE_START;
                /* initialize the USART state machine */
                usartBMState = USART_BM_INIT;
            
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            /* run the state machine for servicing the SPI */
            SPI_Task();
			
            
            tx_packet[0] = 'H';
			//USART_Task(); - original harmony code, unsure if still needed
            if(!BUTTON1)
            {
                send_packet(1);
            }
            
            
            if(appData.spiStateMachine == APP_SPI_STATE_START)
            {
                printf("start\n");
            }
            else if (appData.spiStateMachine == APP_SPI_STATE_WAIT)
            {
                printf("wait\n");
            }
            else 
            {
                 printf("done\n");
                 //appData.spiStateMachine = APP_SPI_STATE_START;
            }
               
            
            //printf("Hi\n");
            
            LED1 = !BUTTON2;
            
//            if(!BUTTON2)  // I think this means if button 1 is pressed
//            {
//                LED2_ON();
//                               
//            }
//            else
//            {
//                LED2_OFF();
//                //LED1_OFF();
//            }

            
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */

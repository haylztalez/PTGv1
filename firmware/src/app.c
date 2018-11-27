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
#include "system_config/default/system_definitions.h"
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
#define LED2 LATEbits.LATE14

/* defining BUTTONS using PORT registers */
#define BUTTON1 PORTBbits.RB10
#define BUTTON2 PORTFbits.RF1
#define BUTTON3 PORTBbits.RB12
#define BUTTON4 PORTBbits.RB11

/* defining Slave Select ports  */
#define SS3_TRIGGER LATFbits.LATF0
#define SS2_TRIGGER LATAbits.LATA4



/* delay function, used as delay_ms(100) to delay 100 ms */
//void delay_ms(int n){
//    float k,l;
//    k = 3.73254;
//    l = 17.234;
//	int i;
//	int j;
//	for (i=0;i<n;i++)
//		for (j=0;j<48193;j++)
//		{k = k*l;}
//}

void delay_ms(unsigned int count)
{
	T1CON = 0x8030;		// turn on timer, prescaler to 256 (type B timer)
	while(count--)
	{
		TMR1 = 0;
		while(TMR1 < 0x4e);
	}
	T1CONbits.ON = 0;
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
static unsigned char dac_packet[] ={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

void send_packet(unsigned char x)
{
    SS3_TRIGGER = 0;
    DRV_SPI_BufferAddWriteRead( appData.handleSPI0, tx_packet, x, app_spi_rx_buffer, sizeof(app_spi_rx_buffer), 0, 0);
    SS3_TRIGGER = 1;
}

void write_DAC(unsigned char d)
{
    SS2_TRIGGER = 0;
    DRV_SPI_BufferAddWriteRead( appData.handleSPI1, dac_packet, d, app_spi_rx_buffer, sizeof(app_spi_rx_buffer), 0, 0);
    SS2_TRIGGER = 1;
    
}

/* defining DDS stuff */
#define BUFFER_SIZE 10 //
DRV_I2S_BUFFER_HANDLE bufferHandle1;
DRV_I2S_BUFFER_HANDLE bufferHandle2;

static unsigned char buffer1[BUFFER_SIZE*8] = {0xFF};
static unsigned char buffer2[BUFFER_SIZE*8];

//DCH0SSA = KVA_TO_PA(&buffer1[0]);
//DCH1SSA = KVA_TO_PA(&buffer2[0]);

//short buffer_a[BUFFER_SIZE];
//short buffer_b[BUFFER_SIZE];
//short* buffer_pp;            // buffer_pp = buffer play pointer.
//
//extern unsigned char isFillFlag;
//extern volatile unsigned char bufferAFull;
//extern volatile unsigned char bufferBFull;
//
//// test variables - for debugging purposes only!
//unsigned long accum1t = 0;
//unsigned long accum2t = 0;
//unsigned long tuningWord1t = 90;
//unsigned long tuningWord2t = 90;

//void generate_sine() {
//    
//  //source: https://github.com/pyrohaz
//  unsigned int n = 0;
//  short int sample = 0;
//  for (n = 0; n < BUFFER_SIZE; n++) {
//    
//    if (n & 0x01) {
//      //sample = (short int)wavetable[accum1t >> 20];
//      sample = 0;
//      accum1t += tuningWord1t;
//    }
//    else {
//      sample = (short int)wavetable[accum2t >> 20];
//      accum2t += tuningWord2t;
//    }
//
//    buffer_pp[n] = sample;
//
//  }
//    
//}

void APP_MyBufferEventHandler( DRV_I2S_BUFFER_EVENT event, DRV_I2S_BUFFER_HANDLE bufferHandle, uintptr_t context )
{
    //printf("here");
    switch(event)
    {
        case DRV_I2S_BUFFER_EVENT_COMPLETE:
            if(bufferHandle == bufferHandle1)
            {
                //DRV_I2S_BufferAddWrite(appData.handleI2S0,&bufferHandle1,buffer1,BUFFER_SIZE);
            }
            else if(bufferHandle == bufferHandle2)
            {
                //DRV_I2S_BufferAddWrite(appData.handleI2S0,&bufferHandle2,buffer2,BUFFER_SIZE);
            }
            // Handle the completed buffer.
            break;

        case DRV_I2S_BUFFER_EVENT_ERROR:
        default:

            // Handle error.
            break;
    }
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
//static void SPI_Task(void)
//{
//    /* run the state machine here for SPI */
//    switch (appData.spiStateMachine)
//    {
//        default:
//        case APP_SPI_STATE_START:
//            /* Blocking: When the call exits, all data has been exchanged (if we get a valid handle) */
//            SS3_TRIGGER = 0;
//            appData.drvSPIBufferHandle = DRV_SPI_BufferAddWriteRead(appData.handleSPI0,
//                app_spi_tx_buffer, sizeof(app_spi_tx_buffer),
//                app_spi_rx_buffer, sizeof(app_spi_rx_buffer),
//                0, 0);
//
//            if (DRV_SPI_BUFFER_HANDLE_INVALID != appData.drvSPIBufferHandle)
//            {
//                /* Blocking Write/Read has finished */
//                appData.spiStateMachine =  APP_SPI_STATE_DONE;
//            }
//
//            if (DRV_SPI_BUFFER_HANDLE_INVALID == appData.drvSPIBufferHandle)
//            {
//                /* try again if we get a bad handle */
//                appData.spiStateMachine =  APP_SPI_STATE_START;
//            }
//        break;
//
//        case APP_SPI_STATE_DONE:
//            SS3_TRIGGER = 1;
//        break;
//    }
//}

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
    appData.handleSPI1 = DRV_HANDLE_INVALID;
    appData.handleI2S = DRV_HANDLE_INVALID;
    
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
    //YES printf("are you getting here?\n");
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            //NO printf("are you getting here?\n");
            bool appInitialized = true;
            printf("%d\n",appInitialized);
       

            if (DRV_HANDLE_INVALID == appData.handleSPI0)
            {
                appData.handleSPI0 = DRV_SPI_Open(0, DRV_IO_INTENT_READWRITE);
                appInitialized &= (DRV_HANDLE_INVALID != appData.handleSPI0);
            }
            
            if (appData.handleSPI1 == DRV_HANDLE_INVALID )
            {
                appData.handleSPI1 = DRV_SPI_Open(1, DRV_IO_INTENT_READWRITE);
                appInitialized &= (DRV_HANDLE_INVALID != appData.handleSPI1);
            }
            
            if (appData.handleI2S == DRV_HANDLE_INVALID )
            {
                appData.handleI2S = DRV_I2S_Open(0, DRV_IO_INTENT_READWRITE);
              
//                if(DRV_HANDLE_INVALID != appData.handleI2S0)
//                {
//               
//                    DRV_I2S_BufferEventHandlerSet(appData.handleI2S0, &APP_MyBufferEventHandler, 0);
//                    DRV_I2S_ReceiveErrorIgnore(appData.handleI2S0,true);
//                    DRV_I2S_TransmitErrorIgnore(appData.handleI2S0,true);
//                    memset(buffer1,0,BUFFER_SIZE);
//                    memset(buffer2,0xFF,BUFFER_SIZE);
//                    DRV_I2S_BufferAddWrite(appData.handleI2S0,&bufferHandle1,buffer1,BUFFER_SIZE);
//                    DRV_I2S_BufferAddWrite(appData.handleI2S0,&bufferHandle2,buffer1,BUFFER_SIZE);
//                    
//                }
                appInitialized &= (DRV_HANDLE_INVALID != appData.handleI2S);
                printf("I2SHandle: 0x%X\n", appData.handleI2S);
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
                //appData.spiStateMachine = APP_SPI_STATE_START;
                /* initialize the USART state machine */
                usartBMState = USART_BM_INIT;
            
                appData.state = APP_STATE_INIT_PERIPHERALS;
                printf("App is now initialized\n");
            }
            break;
        }
        
        case APP_STATE_INIT_PERIPHERALS:
            
            /*setting registers to line level(0dBv) amplitude*/
            printf("Initializing 0dBv amplitude in DAC\n");
            dac_packet[0] = 16;
            dac_packet[1] = 255 - (uint8_t)(9.5f/.5f);
            dac_packet[2] = 17;
            dac_packet[3] = 255 - (uint8_t)(9.5f/.5f);
            write_DAC(4);
            
            printf("Initializing LCD screen\n");
            memcpy(tx_packet, "INITIALIZED", strlen("INITIALIZED"));
            send_packet(strlen("INITIALIZED"));
            
            int32_t i2s_status = (int32_t)DRV_I2S_Status(sysObj.drvI2S0);
            printf("Initializing I2S... status = %d\n", i2s_status);
            DRV_I2S_BufferEventHandlerSet(appData.handleI2S, &APP_MyBufferEventHandler, 0);

            DRV_I2S_TransmitErrorIgnore(appData.handleI2S, true);
            DRV_I2S_BaudSet(appData.handleI2S, (48000*2), 48000);
            
            //buffer1 = 26;
            //strcpy(buffer1, 0XFF);
            //memset(buffer1,0xFF,BUFFER_SIZE*8);
            //memset(buffer2,0xFF,BUFFER_SIZE*8);
            printf("Before AddBuffer\n");
            DRV_I2S_BufferAddWrite(appData.handleI2S,&appData.drvI2SBufferHandle,buffer1,BUFFER_SIZE*8);
            printf("After AddBuffer\n");
            //DRV_I2S_BufferAddWrite(appData.handleI2S,&appData.drvI2SBufferHandle2,buffer2,BUFFER_SIZE*8);
            printf("End of INIT_PERIPH");
            
            appData.state = APP_STATE_SERVICE_TASKS;
            break;

        case APP_STATE_SERVICE_TASKS:
        {
           
            static uint32_t _sanity_print = 0;
            if ( _sanity_print++ % 1000000 == 0 ) {
                printf(".");
            }

            
            LED1 = !BUTTON2;
            
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

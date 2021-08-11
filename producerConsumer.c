/* Standard includes. */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Hardware includes. */
#include "inc/tm4c123gh6pm.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"

/* Optional includes for USB serial output */
#ifdef USB_SERIAL_OUTPUT
#include "driverlib/rom.h"
#include "utils/uartstdio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#endif

/*local includes*/
#include "assert.h"

// Included for GPIO_O_LOCK and others
#include "inc/hw_gpio.h" //for macro declaration of GPIO_O_LOCK and GPIO_O_CR
#include <inc/hw_types.h>

#include "timers.h"
#include "timing_verify.h"

#include "queue.h"

#define SPEED_TEST 0


#define LED_R (1<<1)
#define LED_G (1<<3)
#define LED_B (1<<2)
#define SW1   (1<<4)
#define SW2   (1<<0)

#define LED_ON(x) (GPIO_PORTF_DATA_R |= (x))
#define LED_OFF(x) (GPIO_PORTF_DATA_R &= ~(x))
#define LED(led,on) ((on)?LED_ON(led):LED_OFF(led))
#define pdTICKSTOMS( xTicks ) ((xTicks * 1000 ) / configTICK_RATE_HZ )

uint32_t SystemCoreClock;

#ifdef USB_SERIAL_OUTPUT

//*****************************************************************************
//
//: Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
static void
_configureUART(void)
{
    //
    // Enable UART0
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    //
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

#endif

struct producerData {
  uint32_t data;
  uint32_t producerId;
  xQueueHandle queue;
};

static void
_setupHardware(void)
{
    //
    // Enable the GPIO port that is used for the on-board LED.
    // This is a TiveDriver library function
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_AFSEL) |= 0x01;

    // Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    // These are TiveDriver library functions
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, (LED_G|LED_R|LED_B));

    //
    // Set the clocking to run at (SYSDIV_2_5) 80.0 MHz from the PLL.
    //                            (SYSDIV_3) 66.6 MHz
    //                            (SYSDIV_4) 50.0 MHz
    //                            (SYSDIV_5) 40.0 MHz
    //                            (SYSDIV_6) 33.3 MHz
    //                            (SYSDIV_8) 25.0 MHz
    //                            (SYSDIV_10) 20.0 MHz
    //
    SystemCoreClock = 80000000;  // Required for FreeRTOS.

    SysCtlClockSet( SYSCTL_SYSDIV_2_5 |
                    SYSCTL_USE_PLL |
                    SYSCTL_XTAL_16MHZ |
                    SYSCTL_OSC_MAIN);

}

static void
_producer( void * pvParameters)
{
  uint32_t message = 1;
  struct producerData *prodData;
  prodData = (struct producerData *) pvParameters;
  while(1) {
    prodData->data = message;
    uint32_t delay = (rand() % (150 - 50 + 1)) + 50;
    vTaskDelay(delay / portTICK_RATE_MS);
    #ifdef USB_SERIAL_OUTPUT
      UARTprintf("Producing\n");
    #endif
    assert( xQueueSend( prodData->queue, prodData, 0) == pdPASS );
    message++;
  }
}

static void
_consumer( void * xQueue1 )
{
  struct producerData xRxed;
  while(1) {
    if( xQueueReceive( xQueue1,
                      &xRxed,
                      1 ) == pdPASS )
    {
      #ifdef USB_SERIAL_OUTPUT
        UARTprintf("Consumed %i from producer %i\n", xRxed.data, xRxed.producerId);
      #endif
      if ( xRxed.producerId == 1 ) {
        LED_ON(LED_B);
        vTaskDelay(35 / portTICK_RATE_MS);
        LED_OFF(LED_B);
      } else {
        LED_ON(LED_R);
        vTaskDelay(35 / portTICK_RATE_MS);
        LED_OFF(LED_R);
      }
    }
  }
}

static void
_heartbeat( void *notUsed )
{
    uint32_t green500ms = 500; // 1 second
    uint32_t ledOn = 0;

    while(true)
    {
        ledOn = !ledOn;
        LED(LED_G, ledOn);

        vTaskDelay(green500ms / portTICK_RATE_MS);
    }
}

int main( void )
{
    _setupHardware();

    #ifdef USB_SERIAL_OUTPUT
    	void spinDelayMs(uint32_t ms);
    	_configureUART();
    	spinDelayMs(1000);  // Allow UART to setup
    	UARTprintf("Hello from producerConsumer main()\n");
      UARTprintf("Hello, modification for step 18\n");
    #endif
    xTaskCreate(_heartbeat,
                "green",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1,  // higher numbers are higher priority..
                NULL );

    xQueueHandle xQueue1;
    xQueue1 = xQueueCreate( 20, sizeof( struct producerData ) );
    if (xQueue1 == NULL) {
      #ifdef USB_SERIAL_OUTPUT
      	UARTprintf("Queue failed to generate\n");
      #endif
    }

    struct producerData producer1data;
    producer1data.producerId = 1;
    producer1data.queue = xQueue1;

    struct producerData producer2data;
    producer2data.producerId = 2;
    producer2data.queue = xQueue1;

    uint32_t producerPriority = 3;
    uint32_t consumerPriority = 2;

    xTaskCreate(_producer,
                "producer1",
                configMINIMAL_STACK_SIZE,
                &producer1data,
                tskIDLE_PRIORITY + producerPriority,  // higher numbers are higher priority..
                NULL );

    xTaskCreate(_producer,
                "producer2",
                configMINIMAL_STACK_SIZE,
                &producer2data,
                tskIDLE_PRIORITY + producerPriority,  // higher numbers are higher priority..
                NULL );

    xTaskCreate(_consumer,
                "consumer",
                configMINIMAL_STACK_SIZE,
                ( void * ) xQueue1,
                tskIDLE_PRIORITY + consumerPriority,  // higher numbers are higher priority..
                NULL );

    /* Start the tasks and timer running. */\
    vTaskStartScheduler();

    assert(0); // we should never get here..

    return 0;
}

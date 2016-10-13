/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

 ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
 ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

 ***************************************************************************
 *                                                                       *
 *    FreeRTOS provides completely free yet professionally developed,    *
 *    robust, strictly quality controlled, supported, and cross          *
 *    platform software that is more than just the market leader, it     *
 *    is the industry's de facto standard.                               *
 *                                                                       *
 *    Help yourself get started quickly while simultaneously helping     *
 *    to support the FreeRTOS project by purchasing a FreeRTOS           *
 *    tutorial book, reference manual, or both:                          *
 *    http://www.FreeRTOS.org/Documentation                              *
 *                                                                       *
 ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
 */

/******************************************************************************
 * See http://www.freertos.org/EFM32-Giant-Gecko-Pearl-Gecko-tickless-RTOS-demo.html
 *
 * This project provides two demo applications.  A simple blinky style project
 * that demonstrates low power tickless functionality, and a more comprehensive
 * test and demo application.  The configCREATE_LOW_POWER_DEMO setting, which is
 * defined in FreeRTOSConfig.h, is used to select between the two, and to select
 * the clock used when demonstrating tickless functionality.
 *
 * The simply blinky low power demo is implemented and described in
 * main_low_power.c.  The more comprehensive test and demo application is
 * implemented and described in main_full.c.
 *
 * This file implements the code that is not demo specific, including the
 * hardware setup and standard FreeRTOS hook functions.
 *
 * ENSURE TO READ THE DOCUMENTATION PAGE FOR THIS PORT AND DEMO APPLICATION ON
 * THE http://www.FreeRTOS.org WEB SITE FOR FULL INFORMATION ON USING THIS DEMO
 * APPLICATION, AND ITS ASSOCIATE FreeRTOS ARCHITECTURE PORT!
 *
 */

/* ---- FreeRTOS includes -------------------------------------------------- */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* ---- SiLabs includes ---------------------------------------------------- */
#include "em_emu.h"
#include "em_usart.h"
#include "bsp.h"
#include "bsp_trace.h"
#include "sleep.h"
#include "i2cspm.h"
#include "capsense.h"
#include "si7013.h"

/* ---- Constants ---------------------------------------------------------- */
/* Priorities at which the tasks are created. */
#define mainQUEUE_RECEIVE_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define	mainQUEUE_SEND_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )

/* The rate at which data is sent to the queue.  The 200ms value is converted
 * to ticks using the portTICK_PERIOD_MS constant. */
#define mainQUEUE_SEND_FREQUENCY_MS			pdMS_TO_TICKS( 1000 )

/* The number of items the queue can hold.  This is 1 as the receive task
 * will remove items as they are added, meaning the send task should always find
 * the queue empty. */
#define mainQUEUE_LENGTH					( 1 )

/* The LED toggled by the Rx task. */
#define mainTASK_LED						( 0 )

/* The LED controlled by USART RX */
#define usartRX_LED							( 1 )

/* The queue used by both tasks. */
static QueueHandle_t xQueue = NULL;

/* Receive buffer */
#define USART0_RX_IRQ_ENABLE
#ifdef USART0_RX_IRQ_ENABLE
#define RXBUFSIZE    	8                       /**< Buffer size for RX */
static volatile int     rxReadIndex  = 0;       /**< Index in buffer to be read */
static volatile int     rxWriteIndex = 0;       /**< Index in buffer to be written to */
static volatile int     rxCount      = 0;       /**< Keeps track of how much data which are stored in the buffer */
static volatile uint8_t rxBuffer[RXBUFSIZE];    /**< Buffer to store data */
#endif

I2CSPM_Init_TypeDef i2cInit = I2CSPM_INIT_DEFAULT;

/* ---- Function prototypes ------------------------------------------------ */

/* Prototypes for the standard FreeRTOS callback/hook functions implemented
 * within this file. */
void vApplicationMallocFailedHook( void );
void vApplicationIdleHook( void );
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName );
void vApplicationTickHook( void );

/* Configure the hardware as necessary to run this demo. */
static void prvSetupHardware( void );
static void SerialInit( void );
#define SerialTxByte		USART_Tx		/* Set TX to USART_Tx */
#define SerialRxByte		USART_Rx		/* Set RX to USART_Rx */

void SerialTxBytes( USART_TypeDef *usart, char* charPtr, int count );
void SerialRxBytes( USART_TypeDef *usart, char* charPtr, int count );

/* The tasks as described in the comments at the top of this file. */
static void prvQueueReceiveTask( void *pvParameters );
static void prvQueueSendTask( void *pvParameters );

/* ---- main --------------------------------------------------------------- */

int main( void )
{
	/* Configure the hardware ready to run the demo. */
	prvSetupHardware();

	/* Create the queue. */
	xQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof( uint32_t ) );

	if( xQueue != NULL )
	{
		/* Start the two tasks as described in the comments at the top of this
		 * file. */
		xTaskCreate(
				prvQueueReceiveTask,				/* The function that implements the task. */
				"Rx", 								/* The text name assigned to the task - for debug only as it is not used by the kernel. */
				configMINIMAL_STACK_SIZE, 			/* The size of the stack to allocate to the task. */
				NULL, 								/* The parameter passed to the task - not used in this case. */
				mainQUEUE_RECEIVE_TASK_PRIORITY, 	/* The priority assigned to the task. */
				NULL );								/* The task handle is not required, so NULL is passed. */

		xTaskCreate( prvQueueSendTask, "TX", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_SEND_TASK_PRIORITY, NULL );

		/* Start the tasks and timer running. */
		vTaskStartScheduler();
	}

	/* If all is well, the scheduler will now be running, and the following
	line will never be reached.  If the following line does execute, then
	there was insufficient FreeRTOS heap memory available for the Idle and/or
	timer tasks to be created.  See the memory management section on the
	FreeRTOS web site for more details on the FreeRTOS heap
	http://www.freertos.org/a00111.html. */
	for( ;; );

	return( 0 );
}

/* ---- Send Task ---------------------------------------------------------- */

static void prvQueueSendTask( void *pvParameters )
{
	TickType_t xNextWakeTime;
	const uint32_t ulValueToSend = 100UL;

	/* Remove compiler warning about unused parameter. */
	( void ) pvParameters;

	/* Initialize xNextWakeTime - this only needs to be done once. */
	xNextWakeTime = xTaskGetTickCount();

	for( ;; )
	{
		/* Place this task in the blocked state until it is time to run again. */
		vTaskDelayUntil( &xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS );

		/* Send to the queue - causing the queue receive task to unblock and
		toggle the LED.  0 is used as the block time so the sending operation
		will not block - it shouldn't need to block as the queue should always
		be empty at this point in the code. */
		xQueueSend( xQueue, &ulValueToSend, 0U );
	}
}

/* ---- Receive Task ------------------------------------------------------- */

static void prvQueueReceiveTask( void *pvParameters )
{
	static uint8_t bCount = 0;
	static int32_t tempData = 0;
	static uint32_t rhData = 0;
	char buff[50];

	uint32_t ulReceivedValue;
	const uint32_t ulExpectedValue = 100UL;
	const TickType_t xShortDelay = pdMS_TO_TICKS( 10 );

	/* Remove compiler warning about unused parameter. */
	( void ) pvParameters;

	for( ;; )
	{
		/* Wait until something arrives in the queue - this task will block
		indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
		FreeRTOSConfig.h. */
		xQueueReceive( xQueue, &ulReceivedValue, portMAX_DELAY );

		/*  To get here something must have been received from the queue, but
		is it the expected value?  If it is, toggle the LED. */
		if( ulReceivedValue == ulExpectedValue )
		{
			/* Turn the LED on for a brief time only so it doens't distort the
			energy reading. */
			BSP_LedSet( mainTASK_LED );
			vTaskDelay( xShortDelay );
			BSP_LedClear( mainTASK_LED );
			ulReceivedValue = 0U;

			/* Read CAP sensor */
			CAPSENSE_Sense();

			if (CAPSENSE_getPressed(BUTTON1_CHANNEL) &&
					!CAPSENSE_getPressed(BUTTON0_CHANNEL))
			{
				bCount++;
			}

			else if (CAPSENSE_getPressed(BUTTON0_CHANNEL) &&
					!CAPSENSE_getPressed(BUTTON1_CHANNEL))
			{
				if( bCount > 0 )
					bCount--;
			}

			/* Read Temp/Humidity sensor */
			Si7013_MeasureRHAndTemp( i2cInit.port, SI7021_ADDR, &rhData, &tempData );

			sprintf( buff, "Temp = %d | Humidity = %d | Button = %d\r", tempData, rhData, bCount );
			SerialTxBytes( USART0, buff, sizeof(buff) );
		}
	}
}

/* ---- Configure the board ------------------------------------------------ */

static void prvSetupHardware( void )
{
	EMU_DCDCInit_TypeDef xDCDInit = EMU_DCDCINIT_STK_DEFAULT;
	CMU_HFXOInit_TypeDef xHFXOInit = CMU_HFXOINIT_STK_DEFAULT;

	/* Chip errata */
	CHIP_Init();

	/* Init DCDC regulator and HFXO with kit specific parameters */
	EMU_DCDCInit( &xDCDInit );
	CMU_HFXOInit( &xHFXOInit );

	/* Switch HFCLK to HFXO and disable HFRCO */
	CMU_ClockSelectSet( cmuClock_HF, cmuSelect_HFXO );
	CMU_OscillatorEnable( cmuOsc_HFRCO, false, false );

	/* Initialize LED driver. */
	BSP_LedsInit();
	BSP_LedSet( 0 );
	BSP_LedClear( 1 );

	/* Enable Si7021 sensor */
	I2CSPM_Init( &i2cInit );
	GPIO_PinModeSet(gpioPortD, 9, gpioModePushPull, 1);	//Enable Si7021 sensor isolation switch
	Si7013_Detect(i2cInit.port, SI7021_ADDR, 0);

	/* Start capacitive sense buttons */
	CAPSENSE_Init();

	/* Initialize UART */
	SerialInit();

	/* Create new page on the serial terminal */
	SerialTxByte( USART0, 0x0C );

	SerialTxBytes( USART0, "EFM32 FreeRTOS demo!\n\r", sizeof("EFM32 FreeRTOS demo!\n\r") );
}

/**
 * @brief	UART initialization
 */
static void SerialInit( void )
{
	/* Enable peripheral clocks */
	CMU_ClockEnable(cmuClock_HFPER, true);
	/* Configure GPIO pins */
	CMU_ClockEnable(cmuClock_GPIO, true);
	/* To avoid false start, configure output as high */
	GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 1);
	GPIO_PinModeSet(gpioPortA, 1, gpioModeInput, 0);
	/* Re-target USART0 to DBG USB VCOM port */
	GPIO_PinModeSet(BSP_BCC_ENABLE_PORT, BSP_BCC_ENABLE_PIN, gpioModePushPull, 1);

	USART_TypeDef           *usart = USART0;
	USART_InitAsync_TypeDef init   = USART_INITASYNC_DEFAULT;

	CMU_ClockEnable(cmuClock_USART0, true);

	/* Configure USART for basic async operation */
	init.enable = usartDisable;
	USART_InitAsync(usart, &init);

	/* Enable pins at correct UART/USART location. */
#if defined( USART_ROUTEPEN_RXPEN )
	usart->ROUTEPEN = USART_ROUTEPEN_RXPEN | USART_ROUTEPEN_TXPEN;
	usart->ROUTELOC0 =
			( usart->ROUTELOC0 & ~( _USART_ROUTELOC0_TXLOC_MASK | _USART_ROUTELOC0_RXLOC_MASK ) ) |
			( _USART_ROUTELOC0_TXLOC_LOC0 << _USART_ROUTELOC0_TXLOC_SHIFT ) |
			( _USART_ROUTELOC0_RXLOC_LOC0 << _USART_ROUTELOC0_RXLOC_SHIFT );
#else
	usart->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN | RETARGET_LOCATION;
#endif

	/* Enable USART0 receive interrupt */
#ifdef USART0_RX_IRQ_ENABLE
	/* Clear previous RX interrupts */
	USART_IntClear(USART0, USART_IF_RXDATAV);
	NVIC_ClearPendingIRQ(USART0_RX_IRQn);

	/* Enable RX interrupts */
	USART_IntEnable(USART0, USART_IF_RXDATAV);
	NVIC_EnableIRQ(USART0_RX_IRQn);
#endif

	/* Finally enable it */
	USART_Enable(usart, usartEnable);
}

/**
 * @brief	Send 'count' number of bytes over UART.
 * 			Terminate if a NULL character is encountered.
 * @param	usart port number
 * @param	charPtr Pointer to array of bytes that need to be transmitted
 * @param	count number of bytes to transmit
 */
void SerialTxBytes( USART_TypeDef *usart, char* charPtr, int count )
{
	while( count-- )
	{
		if( *charPtr == '\0' )
			break;

		SerialTxByte( usart, *charPtr++ );
	}
}

/**
 * @brief	Read 'count' number of bytes from the UART
 * @param	usart port number
 * @param	charPtr pointer to array where the received bytes should be stored
 * @param	count number of bytes to read
 */
void SerialRxBytes( USART_TypeDef *usart, char* charPtr, int count )
{
	while( count-- )
		*charPtr++ = SerialRxByte( usart );

	/* NULL terminate the received array */
	*charPtr = '\0';
}

/**
 * @brief	UART0 receive interrupt handler
 * @warn	Don't spend a lot of time here, if you need to do a lot of work,
 * 			unblock a task using a queue
 */
#ifdef USART0_RX_IRQ_ENABLE
void USART0_RX_IRQHandler( void )
{
	if( USART0->STATUS & USART_STATUS_RXDATAV )
	{
		/* Store Data */
		rxBuffer[rxWriteIndex] = SerialRxByte( USART0 );

		if( '1' == rxBuffer[rxWriteIndex] )
			BSP_LedSet( usartRX_LED );
		else if( '0' == rxBuffer[rxWriteIndex] )
			BSP_LedClear( usartRX_LED );

		rxWriteIndex++;
		rxCount++;
		if( rxWriteIndex == RXBUFSIZE )
		{
			rxWriteIndex = 0;
		}
		/* Check for overflow - flush buffer */
		if( rxCount > RXBUFSIZE )
		{
			rxWriteIndex = 0;
			rxCount      = 0;
			rxReadIndex  = 0;
		}
	}
}
#endif

/* ---- FreeRTOS house keeping routines ------------------------------------ */

void vApplicationMallocFailedHook( void )
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

void vApplicationIdleHook( void )
{
	volatile size_t xFreeHeapSpace;

	/* This is just a trivial example of an idle hook.  It is called on each
	cycle of the idle task.  It must *NOT* attempt to block.  In this case the
	idle task just queries the amount of FreeRTOS heap that remains.  See the
	memory management section on the http://www.FreeRTOS.org web site for memory
	management options.  If there is a lot of heap memory free then the
	configTOTAL_HEAP_SIZE value in FreeRTOSConfig.h can be reduced to free up
	RAM. */
	xFreeHeapSpace = xPortGetFreeHeapSize();

	/* Remove compiler warning about xFreeHeapSpace being set but never used. */
	( void ) xFreeHeapSpace;
}

void vApplicationTickHook( void )
{
	/* The full demo includes tests that run from the tick hook. */
#if( configCREATE_LOW_POWER_DEMO == 0 )
	{
		extern void vFullDemoTickHook( void );

		/* Some of the tests and demo tasks executed by the full demo include
		interaction from an interrupt - for which the tick interrupt is used
		via the tick hook function. */
		vFullDemoTickHook();
	}
#endif
}

/* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
	/* If the buffers to be provided to the Idle task are declared inside this
	function then they must be declared static - otherwise they will be allocated on
	the stack and so not exists after this function exits. */
	static StaticTask_t xIdleTaskTCB;
	static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

	/* Pass out a pointer to the StaticTask_t structure in which the Idle task's
	state will be stored. */
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

	/* Pass out the array that will be used as the Idle task's stack. */
	*ppxIdleTaskStackBuffer = uxIdleTaskStack;

	/* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
	Note that, as the array is necessarily of type StackType_t,
	configMINIMAL_STACK_SIZE is specified in words, not bytes. */
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/* configUSE_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
application must provide an implementation of vApplicationGetTimerTaskMemory()
to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
	/* If the buffers to be provided to the Timer task are declared inside this
	function then they must be declared static - otherwise they will be allocated on
	the stack and so not exists after this function exits. */
	static StaticTask_t xTimerTaskTCB;
	static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

	/* Pass out a pointer to the StaticTask_t structure in which the Timer
	task's state will be stored. */
	*ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

	/* Pass out the array that will be used as the Timer task's stack. */
	*ppxTimerTaskStackBuffer = uxTimerTaskStack;

	/* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
	Note that, as the array is necessarily of type StackType_t,
	configMINIMAL_STACK_SIZE is specified in words, not bytes. */
	*pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

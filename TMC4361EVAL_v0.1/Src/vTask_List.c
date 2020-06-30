
/**
  ******************************************************************************
  * @file    vTask_list.c
  * @author  Wimansha Wijekoon  
  * @brief   Header file for the task list assist by freeRTOS.
  ******************************************************************************
  * @attention
  *   + Need to have "cmsis_os.h" to have osDelay to work or replace it with 
  *     suitable delay function 
  *   + uses the stm32f429xx eval board 
	*
  ******************************************************************************
  */ 
	
#include "vTask_List.h"

extern uint8_t my_buff;
extern QueueHandle_t xQueue1,xQueue2,xQueue3;
extern uint8_t welcomeArt;
extern uint8_t input_format;
extern uint8_t wTask_buff;
extern uint8_t wCommand_buff;
extern SPI_HandleTypeDef hspi5;
extern	uint8_t wTxData[5];
extern 	uint8_t wRxData[5];
extern 	SemaphoreHandle_t xSemaphore;




//#include "stm32f7xx_hal.h"


/**
  *@brief  Function implimenting test task for RTOS test
  *@param  argument: not in use
  *@retval None
	*@depend rtos depends and HAL gpio
*/
void wTask_LED(void *arg)
{
	for(;;)
	{
		HAL_GPIO_TogglePin(wLED_GPIO_Port,wLED_Pin);
		vTaskDelay(1000);
	}
}


/**
  *@brief  Function controling Vertual com port data in/out
  *@param  argument: not in use
  *@retval None
	*@depend rtos depends and USB_mid ware 
*/
void wTask_USB_FS(void *arg)
{

	for(;;)
	{

	if( xQueueReceive( xQueue1,&wTask_buff,0)  == pdTRUE )
		{
			if( xSemaphoreTake( xSemaphore, 0 ) == pdTRUE )
				{
					while(CDC_Transmit_FS(&wTask_buff,sizeof wTask_buff)!= USBD_OK){}
					if( xSemaphoreGive( xSemaphore ) != pdTRUE ){}
				}
				
		}
				
	}
}

/**
  *@brief  Function controling MotorControl
  *@param  argument: not in use
  *@retval None
	*@depend None
*/

void wTask_MotorCtr(void *arguments)
	
{
	for(;;)
	{
		GPIOF->ODR &= ~(1UL << 6);
			HAL_SPI_Transmit(&hspi5,(uint8_t *)wTxData,5,5000);
			HAL_SPI_Receive(&hspi5,(uint8_t *)wRxData,5,1000);
		GPIOF->ODR |= 1UL << 6; 
		vTaskDelay(2000);
	}
}




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

extern QueueHandle_t xQueue1,xQueue2,xQueue3;
extern 	SemaphoreHandle_t xSemaphore;

extern uint8_t wTask_buff;
extern uint8_t wCommand_Task_buff;

extern SPI_HandleTypeDef hspi5;
extern	uint8_t wTxData[5];
extern 	uint8_t wRxData[5];

extern char wCnvt[48];





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
  *@brief  Function to send data to vcp (shows what types using key board)
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
//		GPIOF->ODR &= ~(1UL << 6);
//			HAL_SPI_Transmit(&hspi5,(uint8_t *)wTxData,5,5000);
//			HAL_SPI_Receive(&hspi5,(uint8_t *)wRxData,5,1000);
//		GPIOF->ODR |= 1UL << 6; 
//		vTaskDelay(2000);
	}
}


/**
  *@brief  Function handles the commands reciew from the teminal via com port 
  *@param  argument: not in use
  *@retval None
	*@depend None
*/
void wTask_CmdHandle(void * arguments) 
{
	char *rw_flag ;
	char *adress ;
	char *data ;
	char printBuff[48] = {0};
	char *datastream = malloc(2*sizeof(char));

	for(;;)
	{

	if( xQueueReceive( xQueue2,&wCnvt,0)  == pdTRUE )
		{
			
			memset(printBuff,0,sizeof printBuff);
			// if buffer is just "\r" do not process it 
			if(strcmp(wCnvt,"\r") != 0 && strcmp(wCnvt,"d\r") != 0) 
			{
							rw_flag = strtok(wCnvt," ");
							if( rw_flag  == NULL)
							{
								if( xSemaphoreTake( xSemaphore, 0 ) == pdTRUE )
										{
											while(CDC_Transmit_FS((uint8_t *)"Error in user Input \r\n", 24)!= USBD_OK){}
											if( xSemaphoreGive( xSemaphore ) != pdTRUE ){}
										}
							}
							else
							{
									adress 	=  strtok(NULL," ");
								if(adress == NULL)
								{
									while(CDC_Transmit_FS((uint8_t *)"Error in user Input \r\n", 24)!= USBD_OK){}
									if( xSemaphoreGive( xSemaphore ) != pdTRUE ){}
																		
								}
								else
								{
									data=  strtok(NULL," ");
									if(data == NULL)
									{
										while(CDC_Transmit_FS((uint8_t *)"Error in user Input \r\n", 24)!= USBD_OK){}
										if( xSemaphoreGive( xSemaphore ) != pdTRUE ){}
									}
									else
									{
										
										sprintf(printBuff , "rw_flag = %s \r\nadress = %s \r\n data= %s \r\n",rw_flag,adress,data);
										if( xSemaphoreTake( xSemaphore, 0 ) == pdTRUE )
											{
												while(CDC_Transmit_FS((uint8_t *)printBuff,sizeof printBuff)!= USBD_OK){}
												if( xSemaphoreGive( xSemaphore ) != pdTRUE ){}
											}
											
											wTxData[0] = strtol(adress , NULL , 16);
											
											for(int i = 0 ; i<4; i++)
											{
												memcpy(datastream,data+(i*2),2*sizeof (char));
												wTxData[i+1] = strtol(datastream, NULL ,16);
												/* send TxData via a queu to mtrTask*/
											}											
											
									}
								}
							}
				
			}
				
				
		}
				
	}
	
}

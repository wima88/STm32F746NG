
/**
  ******************************************************************************
  * @file    vTask_list.h
  * @author  Wimansha Wijekoon  
  * @brief   Header file for the task list assist by freeRTOS.
  ******************************************************************************
  * @attention
  *   +None
  ******************************************************************************
  */ 


#ifndef __VTASK_LIST_H
#define __VTASK_LIST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "cmsis_os.h"
#include "freertos.h"
#include "usbd_cdc_if.h"

void wTask_LED(void * arguments); // Test task for rtos 
void wTask_USB_FS(void * arguments); // usb vertual com 
void wTask_MotorCtr(void *arguments); // motor control task

#endif

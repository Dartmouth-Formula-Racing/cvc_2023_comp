/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "adc.h"
#include "cmsis_os.h"

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 32;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
HAL_StatusTypeDef canTestLoop(void) {
	CAN_TxHeaderTypeDef header;
	CAN_RxHeaderTypeDef rxHeader;
	CAN_FilterTypeDef sFilterConfig;

	uint8_t data[8];
	uint8_t rxData[8];
	uint32_t mbox = 1;

	// Mostly taken from the sample code in the FW repo.
	//
	// Enable filter 0 in MASK mode with no bits set.
	// (This matches all messages received)
	//
	// Send the received messages to RxFifo 0.
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	// Enable the filter (before starting CAN)
	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		/* Filter configuration Error */
		Error_Handler();
	}

	// Push the start button...
	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		Error_Handler();
	}

	// Send out a data message with two bytes of data using ID 0x222
	//  (Note - no idea if this is a real message ID or not)
	header.IDE = CAN_ID_STD;
	header.StdId = 0x222;
	header.RTR = CAN_RTR_DATA;
	header.DLC = 2;

	// Data to be sent.
//	data[0] = 0x55;
//	data[1] = 0xaa;

	uint32_t reading;
	HAL_ADC_Start(&hadc1);

	// Forever
	while (1) {

		HAL_ADC_PollForConversion(&hadc1, 5);
		reading = HAL_ADC_GetValue(&hadc1);

		data[0] = (uint8_t) (reading & 0x000F);
		data[1] = (uint8_t) (reading & 0x00F0 >> 8);
		data[2] = (uint8_t) (reading & 0x0F00 >> 16);
		data[3] = (uint8_t) (reading & 0xF000 >> 24);

		while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3) {}

		// Drop the message in the TxFifo
		if (HAL_CAN_AddTxMessage(&hcan1, &header, data, &mbox) != HAL_OK) {
			Error_Handler();
		}

		osDelay(10);

		// Check the button (not debounced, will very likely send
		//  multiple copies of the message.
//		if ( HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET ){
//
//			// Turn the LED off.
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
//
//			// Busy wait until we are un-wedged.
//			while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3) {
//			}
//
//			// Drop the message in the TxFifo
//			if (HAL_CAN_AddTxMessage(&hcan1, &header, data, &mbox) != HAL_OK) {
//				Error_Handler();
//			}
//		}

		// Check for a received message
//		if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0) {
//
//			// Grab it
//			if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
//				/* Reception Error */
//				Error_Handler();
//			}
//
//			// Is this the same one we (actually - the other guy running this same code)
//			// we sent?  If not die horribly
//			if ((rxHeader.StdId != 0x222) || (rxHeader.RTR != CAN_RTR_DATA)
//					|| (rxHeader.IDE != CAN_ID_STD) || (rxHeader.DLC != 2)
//					|| ((rxData[0] << 8 | rxData[1]) != 0x55AA)) {
//				/* Rx message Error */
//				Error_Handler();
//			}
//
//			// Turn on the LED.
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//		}
	}

	return HAL_OK; /* Test Passed */
}
/* USER CODE END 1 */

/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    Console.c
 * @brief   This file provides code for the configuration
 *          of the USART instances.
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
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "stdarg.h"

#define DEBUG_PORT (&huart2)

/* USER CODE BEGIN 0 */
#define DBG_BUF_LEN 512
static char DBG_BUFFER[DBG_BUF_LEN];

void Console_INFO(const char *fmt, ...)
{
	va_list args;
	int len = 0;

	///// Prepare the output buffer
	len += sprintf(&DBG_BUFFER[len], "\033[32m[%lu] ", HAL_GetTick());
	va_start(args, fmt);
	len += vsprintf(&DBG_BUFFER[len], fmt, args);
	va_end(args);
	len += sprintf(&DBG_BUFFER[len], "\033[0m\r\n");
	HAL_UART_Transmit(DEBUG_PORT, (uint8_t *)(DBG_BUFFER), len, len);
}

void Console_DEBUG(const char *fmt, ...)
{
	va_list args;
	int len = 0;

	///// Prepare the output buffer
	len += sprintf(&DBG_BUFFER[len], "\033[36m[%lu] ",
				   HAL_GetTick());
	va_start(args, fmt);
	len += vsprintf(&DBG_BUFFER[len], fmt, args);
	va_end(args);
	len += sprintf(&DBG_BUFFER[len], "\033[0m\r\n");
	HAL_UART_Transmit(DEBUG_PORT, (uint8_t *)(DBG_BUFFER), len, len);
}

void Console_WARNING(const char *fmt, ...)
{
	va_list args;
	int len = 0;

	///// Prepare the output buffer
	len += sprintf(&DBG_BUFFER[len], "\033[33m[%lu] ", HAL_GetTick());
	va_start(args, fmt);
	len += vsprintf(&DBG_BUFFER[len], fmt, args);
	va_end(args);
	len += sprintf(&DBG_BUFFER[len], "\033[0m\r\n");
	HAL_UART_Transmit(DEBUG_PORT, (uint8_t *)(DBG_BUFFER), len, len);
}

void Console_ERROR(const char *fmt, ...)
{
	va_list args;
	int len = 0;

	///// Prepare the output buffer
	len += sprintf(&DBG_BUFFER[len], "\033[31m[%lu] ", HAL_GetTick());
	va_start(args, fmt);
	len += vsprintf(&DBG_BUFFER[len], fmt, args);
	va_end(args);
	len += sprintf(&DBG_BUFFER[len], "\033[0m\r\n");
	HAL_UART_Transmit(DEBUG_PORT, (uint8_t *)(DBG_BUFFER), len, len);
}

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

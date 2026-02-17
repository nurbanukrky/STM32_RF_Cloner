/**
 * @file main.h
 * @brief Main header file with pin definitions
 * @author Bitirme Projesi
 */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/

/* User Button (NUCLEO onboard) */
#define USER_BTN_Pin          GPIO_PIN_13
#define USER_BTN_GPIO_Port    GPIOC

/* SPI1 Pins */
#define SPI1_SCK_Pin          GPIO_PIN_5
#define SPI1_SCK_GPIO_Port    GPIOA
#define SPI1_MISO_Pin         GPIO_PIN_6
#define SPI1_MISO_GPIO_Port   GPIOA
#define SPI1_MOSI_Pin         GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port   GPIOA

/* CC1101 RX Module */
#define CS_RX_Pin             GPIO_PIN_6
#define CS_RX_GPIO_Port       GPIOB
#define RX_GDO0_Pin           GPIO_PIN_0    /* TIM2_CH1 - Input Capture */
#define RX_GDO0_GPIO_Port     GPIOA
#define RX_GDO2_Pin           GPIO_PIN_0    /* Optional - Carrier Sense */
#define RX_GDO2_GPIO_Port     GPIOB

/* CC1101 TX Module */
#define CS_TX_Pin             GPIO_PIN_4
#define CS_TX_GPIO_Port       GPIOA
#define TX_GDO0_Pin           GPIO_PIN_1    /* GPIO Output - TX Data */
#define TX_GDO0_GPIO_Port     GPIOA

/* USART2 Pins (ST-Link Virtual COM Port) */
#define USART_TX_Pin          GPIO_PIN_2
#define USART_TX_GPIO_Port    GPIOA
#define USART_RX_Pin          GPIO_PIN_3
#define USART_RX_GPIO_Port    GPIOA

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/** @file   encoder.h
 *  @brief  Header file of modular absolute encoder - CUI AMT22.
 *  @author: Ang Chin Xian
 *  @by: Rehabilitation Research Institute of Singapore
 *
 */


/* SPI commands */
#define AMT22_NOP       0x00  //Read Position
#define AMT22_RESET     0x60
#define AMT22_ZERO      0x70

/* Define special ascii characters */
#define NEWLINE         0x0A
#define TAB             0x09

/* Define macros so we can write code once compatible with 12 or 14 bit encoders */
#define RES12           12
#define RES14           14

/* Maximum encoder value 2*resolution*/
/* 4096 or 16384*/
#define ENCODER_MAX 	16384

/*EMA Filter*/
#define EXPONENTIAL_ALPHA 0.85

/* SPI pins */
#define SPI_CS1_PIN       			GPIO_PIN_10
#define SPI_CS1_GPIO_PORT            		GPIOD
#define SPI_CS2_PIN            			GPIO_PIN_11
#define SPI_CS2_GPIO_PORT           		GPIOD
#define SPI_MOSI_PIN        			GPIO_PIN_15
#define SPI_MOSI_GPIO_PORT       		GPIOB
#define SPI_MISO_PIN        			GPIO_PIN_14
#define SPI_MISO_GPIO_PORT        		GPIOB
#define SPI_SCLK_PIN        			GPIO_PIN_13
#define SPI_SCLK_GPIO_PORT        		GPIOB

// Controlling definition of AMT22
#define CS1_HIGH					   	HAL_GPIO_WritePin(SPI_CS1_GPIO_PORT, SPI_CS1_PIN, GPIO_PIN_SET)
#define CS1_LOW							HAL_GPIO_WritePin(SPI_CS1_GPIO_PORT, SPI_CS1_PIN, GPIO_PIN_RESET)
#define CS2_HIGH					  	HAL_GPIO_WritePin(SPI_CS2_GPIO_PORT, SPI_CS2_PIN, GPIO_PIN_SET)
#define CS2_LOW							HAL_GPIO_WritePin(SPI_CS2_GPIO_PORT, SPI_CS2_PIN, GPIO_PIN_RESET)


#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f429xx.h"
#include "dwt_delay.h"
#include "main.h"
#include <math.h>


/**
  * @brief  state LOW and HIGH enumeration
  */
typedef enum
{
  LOW = 0,
  HIGH
}state;

/**
  * @brief  state LOW and HIGH enumeration
  */
typedef enum
{
  ENC1 = 0,
  ENC2
}Encoder;

/** @brief  Initializes AMT22.
  * @param  None.
  * @retval None.
  */
void encoder_Init(void);


/** @brief  set SPI state AMT22.
  * @param  encoder: 1/2.csLine:0/1
  * @retval None.
  */
void setCSLine (Encoder encoder, state csLine);

/** @brief  SPI transfer AMT22.
  * @param  sendByte is the byte to transmit.
  * Use releaseLine to let the spiWriteRead function know if it should release
  * @retval 1byte data.
  */
uint8_t spiWriteRead(uint8_t sendByte, Encoder encoder, state releaseLine);

/** @brief  This function gets the absolute position from the AMT22 encoder using the SPI bus. The AMT22 position includes 2 checkbits to use
 * for position verification. Both 12-bit and 14-bit encoders transfer position via two bytes, giving 16-bits regardless of resolution.
 * For 12-bit encoders the position is left-shifted two bits, leaving the right two bits as zeros. This gives the impression that the encoder
 * is actually sending 14-bits, when it is actually sending 12-bit values, where every number is multiplied by 4.
  * @param
  * @retval.
  */
uint16_t getPositionSPI(Encoder encoder, uint8_t resolution);

/** @brief  Set current encoder position to 0
  * @param
  * @retval.
  */
void setZeroSPI(Encoder encoder);

/** @brief  reset encoder
  * @param
  * @retval.
  */
void resetPositionSPI(Encoder encoder);

void encoderRead(uint16_t *encoder_vals);
void calcVelFromEncoder(uint16_t *encoder_vals, double *velocities);

#endif

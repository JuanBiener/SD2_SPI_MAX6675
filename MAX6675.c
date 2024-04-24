/*****************************************************************************
 *   MAX6675.c:  Source code file for MAX6675 Termocuple Controler
 *
 *   Copyright 2009, Embedded Artists AB
 *   Copyright 2023, DSI FCEIA UNR - Sistemas Digitales 2
 *    DSI: http://www.dsi.fceia.unr.edu.ar/
 *   Copyright 2023, Juan Ignacio Biener
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*==================[inclusions]=============================================*/

#include <stdio.h>
#include "fsl_debug_console.h"
#include "SD2_board.h"
#include "MAX6675.h"
#include "fsl_port.h"
#include "fsl_gpio.h"

/*==================[macros and definitions]=================================*/

#define SPI_MASTER_SSEL_GPIO GPIOE
#define SPI_MASTER_SSEL_PIN 16U
#define SENSOR_OPEN 1
#define SENSOR_OK 0
#define MASK_SENS_OPEN 0b00000100

/*==================[internal data declaration]==============================*/

static uint8_t MAX_6675_Data [] = {0,0};

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

void MAX6675_Init(void){

}

void MAX6675_SPI_Transfer(void){

	/* Assert the chip select for the MAX6675 */
	GPIO_PinWrite(SPI_MASTER_SSEL_GPIO, SPI_MASTER_SSEL_PIN, 0U);

	/* Perform the transfer */
	board_SPIReceive(MAX_6675_Data, 2);

	/* De-assert the chip select for the MAX6675 */
	GPIO_PinWrite(SPI_MASTER_SSEL_GPIO, SPI_MASTER_SSEL_PIN, 1U);

}

uint8_t MAX6675_Get_8bit_Raw_Data_1(void) {

	return MAX_6675_Data[1];

}

uint8_t MAX6675_Get_8bit_Raw_Data_0(void) {

	return MAX_6675_Data[0];

}

uint16_t MAX6675_Read_Raw_Data(void) {

	MAX6675_SPI_Transfer();

    return ((MAX_6675_Data[0] << 8) | MAX_6675_Data[1]);

}

float MAX6675_Read_Float_Temp(void){

	return ((MAX6675_Read_Raw_Data()>> 3) * 0.25f);

}

bool MAX6675_Get_Sensor(void){

	MAX6675_SPI_Transfer();

	return (MAX_6675_Data[0] & MASK_SENS_OPEN);

}



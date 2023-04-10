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

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

void MAX6675_ReadTemp(void) {

	MAX6675_Temp_t temp_MAX6675;
	uint8_t data [] = {0,0};

	/* Assert the chip select for the MAX6675 */
    GPIO_PinWrite(SPI_MASTER_SSEL_GPIO, SPI_MASTER_SSEL_PIN, 0U);

    /* Perform the transfer */
    board_SPIReceive(data, 2);

    /* De-assert the chip select for the MAX6675 */
    GPIO_PinWrite(SPI_MASTER_SSEL_GPIO, SPI_MASTER_SSEL_PIN, 1U);

    /* Convert the received data to temperature */
    uint16_t tempData = (data[0] << 8) | data[1];
    temp_MAX6675.sensor = data[0] & MASK_SENS_OPEN;
    temp_MAX6675.temp = (tempData >> 3) * 0.25f;
    temp_MAX6675.valor = temp_MAX6675.temp*100;
    temp_MAX6675.cifra1 = (uint16_t)temp_MAX6675.valor/1000;
    temp_MAX6675.cifra2 = (uint16_t)temp_MAX6675.valor%1000/100;
    temp_MAX6675.cifra3 = (uint16_t)temp_MAX6675.valor%1000%100/10;
    temp_MAX6675.cifra4 = (uint16_t)temp_MAX6675.valor%1000%100%10;

	/* Print the temperature to the serial terminal */

	char buffer[100];
	if (temp_MAX6675.sensor) {
		snprintf(buffer, sizeof(buffer),"Sensor Desconectado \r\n");
		PRINTF("%s", buffer);
	}
	else {
		snprintf(buffer, sizeof(buffer),
			"Temperature: %d%d.%d%d C\r\n",
			temp_MAX6675.cifra1, temp_MAX6675.cifra2,
			temp_MAX6675.cifra3, temp_MAX6675.cifra4
			);
		PRINTF("%s", buffer);
	}
}

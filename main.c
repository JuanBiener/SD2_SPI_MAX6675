/* Copyright 2023, DSI FCEIA UNR - Sistemas Digitales 2
 *    DSI: http://www.dsi.fceia.unr.edu.ar/
 * Copyright 2023, Guido Cicconi
 * All rights reserved.
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

/* TODO: insert other include files here. */

#include <stdio.h>
#include "fsl_debug_console.h"
#include "SD2_board.h"
#include "oled.h"
#include "fsl_port.h"
#include "fsl_gpio.h"

#define SPI_MASTER_SSEL_GPIO GPIOE
#define SPI_MASTER_SSEL_PIN 16U

typedef struct
{
	uint8_t cifra1;
    uint8_t cifra2;
    uint8_t cifra3;
    uint8_t cifra4;
    float valor;
    float temp;

} temperature_t;

int32_t Temp_Max; // Temporizador para la medicion temperatura

/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */
void MAX6675_ReadTemp(void) {

	temperature_t temp_MAX6675;
	uint8_t data [] = {1,2};
    /* Assert the chip select for the MAX6675 */

    GPIO_PinWrite(SPI_MASTER_SSEL_GPIO, SPI_MASTER_SSEL_PIN, 0U);

    /* Prepare the transfer data */

    /*xfer.txData = &MAX6675_READ;
    xfer.rxData = data;
    xfer.dataSize = sizeof(data);
    xfer.configFlags = kSPI_FrameAssert;*/


    /* Perform the transfer */
    //SPI_MasterTransferBlocking(SPI_MASTER_BASEADDR, &xfer);

    board_SPIReceive(data, 2);
    //data[1] = board_SPIReceive(data, 2);

    /* De-assert the chip select for the MAX6675 */

    GPIO_PinWrite(SPI_MASTER_SSEL_GPIO, SPI_MASTER_SSEL_PIN, 1U);

    /* Convert the received data to temperature */

    uint16_t tempData = (data[0] << 8) | data[1];
    //float temperature = (tempData >> 3) * 0.25f;
    temp_MAX6675.temp = (tempData >> 3) * 0.25f;
    temp_MAX6675.valor = temp_MAX6675.temp*100;
    temp_MAX6675.cifra1 = (uint16_t)temp_MAX6675.valor/1000;
    temp_MAX6675.cifra2 = (uint16_t)temp_MAX6675.valor%1000/100;
    temp_MAX6675.cifra3 = (uint16_t)temp_MAX6675.valor%1000%100/10;
    temp_MAX6675.cifra4 = (uint16_t)temp_MAX6675.valor%1000%100%10;

	/* Print the temperature to the serial terminal */

	char buffer[100];
	snprintf(buffer, sizeof(buffer),
			"Temperature: %d%d.%d%d C\r\n",
			temp_MAX6675.cifra1, temp_MAX6675.cifra2,
			temp_MAX6675.cifra3, temp_MAX6675.cifra4
			);

	PRINTF("%s", buffer);

	/*snprintf(buffer, sizeof(buffer), "Temperature: %d C\r\n", (uint16_t)temperature*100);
	PRINTF("%s", buffer);*/

    /*char buffer[20];
    snprintf(buffer, sizeof(buffer), "Temperature: %.2f C\r\n", data);
    PRINTF("%s", buffer);*/
}

int main(void) {
	/* Inits */
    board_init();
    board_configSPI0();

    /* inicializa interrupciÃ³n de systick cada 1 ms */
	SysTick_Config(SystemCoreClock / 1000U);
	Temp_Max = 500; // Para iniciar la secuencia

	/*oled_init();
	oled_setContrast(16);

	oled_clearScreen(OLED_COLOR_BLACK);*/

	/* Drawing */

    /*oled_fillRect(32, 16, 32+64, 16+32, OLED_COLOR_WHITE);
	oled_fillRect(32+8, 16+8, 32+64-8, 16+32-8, OLED_COLOR_BLACK);
	oled_putString(56, 29, (uint8_t *)"SD2", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	oled_circle(64, 32, 31, OLED_COLOR_WHITE);*/

    while(1) {

    	if (Temp_Max == 0) {
    		Temp_Max = 500;
    		MAX6675_ReadTemp();
    	}

    }

    return 0 ;
}

void SysTick_Handler(void) // OJO cambiar
{
	//key_periodicTask1ms();

	if (Temp_Max > 0) {
		Temp_Max--;
	}
}

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

/*==================[inclusions]=============================================*/

#include <stdio.h>
#include "fsl_debug_console.h"
#include "SD2_board.h"
#include "MAX6675.h"
#include "fsl_port.h"
#include "fsl_gpio.h"

/*==================[macros and definitions]=================================*/

#define TIEMPO_MUESTRAS 2000

/*==================[internal data declaration]==============================*/

static int32_t Time_Temp_Max; // Temporizador para la medicion temperatura
static float Float_Temp_MAX6675=0.0;
static uint16_t Raw_Data_MAX6675=0;
static uint8_t Raw_Data_0=0;
static uint8_t Raw_Data_1=0;
static char buffer[100];

/*==================[internal functions declaration]=========================*/

void Print_MAX6675_Temp(float Temp){

		/*Print the temperature to the serial terminal*/

		uint8_t Temp_cifra1;
		uint8_t Temp_cifra2;
		uint8_t Temp_cifra3;
		uint8_t Temp_cifra4;

		//Migrar campos de cifra a parte entera y parte decimal.

	 	Temp_cifra1 = (uint16_t)(Temp*100)/1000;
	 	Temp_cifra2 = (uint16_t)(Temp*100)%1000/100;
	 	Temp_cifra3 = (uint16_t)(Temp*100)%1000%100/10;
	 	Temp_cifra4 = (uint16_t)(Temp*100)%1000%100%10;

	 	snprintf(buffer, sizeof(buffer),
			"Temperature: %d%d.%d%d C\r\n",
			Temp_cifra1, Temp_cifra2,
			Temp_cifra3, Temp_cifra4
			);
	 	PRINTF("%s", buffer);
}

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

int main(void) {

    board_init();
    board_configSPI0();
	SysTick_Config(SystemCoreClock / 1000U);
	Time_Temp_Max = TIEMPO_MUESTRAS;

    while(1) {

    	if (Time_Temp_Max == 0) {
    		Time_Temp_Max = TIEMPO_MUESTRAS;
    		//Float_Temp_MAX6675 = MAX6675_Read_Float_Temp();
    		Raw_Data_MAX6675 = MAX6675_Read_Raw_Data();
    		Raw_Data_0 = MAX6675_Get_8bit_Raw_Data_0();
    		Raw_Data_1 = MAX6675_Get_8bit_Raw_Data_1();
    		/*if (MAX6675_Get_Sensor())
    		{
				snprintf(buffer, sizeof(buffer),"Sensor Desconectado \r\n");
				PRINTF("%s", buffer);
			}
    		else {
    			snprintf(buffer, sizeof(buffer),"Raw Data: %d \r\n",Raw_Data_MAX6675);
    			PRINTF("%s", buffer);
    			Print_MAX6675_Temp(Float_Temp_MAX6675);
    		}*/
    		snprintf(buffer, sizeof(buffer),"Raw Data 0: %d \r\n",Raw_Data_0);
			PRINTF("%s", buffer);
			snprintf(buffer, sizeof(buffer),"Raw Data 1: %d \r\n",Raw_Data_1);
			PRINTF("%s", buffer);
			snprintf(buffer, sizeof(buffer),"Raw Data: %d \r\n",Raw_Data_MAX6675);
			PRINTF("%s", buffer);

    	}
    }
    return 0 ;
}



void SysTick_Handler(void)
{

	if (Time_Temp_Max > 0) {
		Time_Temp_Max--;
	}
}

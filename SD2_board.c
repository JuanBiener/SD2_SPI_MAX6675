/* Copyright 2023, DSI FCEIA UNR - Sistemas Digitales 2
 *    DSI: http://www.dsi.fceia.unr.edu.ar/
 * Copyright 2017, Diego Alegrechi
 * Copyright 2017, Gustavo Muro
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
#include <SD2_board.h>

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"

#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"
#include "pin_mux.h"
#include "fsl_spi.h"

/*==================[macros and definitions]=================================*/

#define SPI_MASTER              SPI0
#define SPI_MASTER_SOURCE_CLOCK kCLOCK_BusClk
#define SPI_MASTER_CLK_FREQ     CLOCK_GetFreq(kCLOCK_BusClk)
#define SPI_MODE_1

/*==================[internal data declaration]==============================*/

static const board_gpioInfo_type board_gpioLeds[] =
{
    {PORTE, GPIOE, 29},     /* LED ROJO */
    {PORTD, GPIOD, 5},      /* LED VERDE */
};

static const board_gpioInfo_type board_gpioSw[] =
{
    {PORTC, GPIOC, 3},      /* SW1 */
    {PORTC, GPIOC, 12},     /* SW3 */
};

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

static spi_master_handle_t handle;
static volatile bool masterFinished = false;

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

void board_init(void)
{
	int32_t i;
	gpio_pin_config_t gpio_led_config =
	{
		.outputLogic = 1,
		.pinDirection = kGPIO_DigitalOutput,
	};
	gpio_pin_config_t gpio_sw_config = {
		.pinDirection = kGPIO_DigitalInput,
		.outputLogic = 0U
	};

	const port_pin_config_t port_led_config = {
		/* Internal pull-up/down resistor is disabled */
		.pullSelect = kPORT_PullDisable,
		/* Slow slew rate is configured */
		.slewRate = kPORT_SlowSlewRate,
		/* Passive filter is disabled */
		.passiveFilterEnable = kPORT_PassiveFilterDisable,
		/* Low drive strength is configured */
		.driveStrength = kPORT_LowDriveStrength,
		/* Pin is configured as PTC3 */
		.mux = kPORT_MuxAsGpio,
	};

	const port_pin_config_t port_sw_config = {
		/* Internal pull-up resistor is enabled */
		.pullSelect = kPORT_PullUp,
		/* Fast slew rate is configured */
		.slewRate = kPORT_FastSlewRate,
		/* Passive filter is disabled */
		.passiveFilterEnable = kPORT_PassiveFilterDisable,
		/* Low drive strength is configured */
		.driveStrength = kPORT_LowDriveStrength,
		/* Pin is configured as PTC3 */
		.mux = kPORT_MuxAsGpio,
	};

	BOARD_InitBootClocks();
    BOARD_InitBootPins();
    BOARD_InitDebugConsole();

	CLOCK_EnableClock(kCLOCK_PortA);
	CLOCK_EnableClock(kCLOCK_PortC);
	CLOCK_EnableClock(kCLOCK_PortD);
	CLOCK_EnableClock(kCLOCK_PortE);

	/* Inicialización de leds */
	for (i = 0 ; i < BOARD_LED_ID_TOTAL ; i++)
	{
		PORT_SetPinConfig(board_gpioLeds[i].port, board_gpioLeds[i].pin, &port_led_config);
		GPIO_PinInit(board_gpioLeds[i].gpio, board_gpioLeds[i].pin, &gpio_led_config);
	}

	/* Inicialización de SWs */
	for (i = 0 ; i < BOARD_SW_ID_TOTAL ; i++)
	{
		PORT_SetPinConfig(board_gpioSw[i].port, board_gpioSw[i].pin, &port_sw_config);
		GPIO_PinInit(board_gpioSw[i].gpio, board_gpioSw[i].pin, &gpio_sw_config);
	}

	/*Inicialización de los pines GPIO necesarios para manejar la termocupla MAX6675*/

	const port_pin_config_t port_max6675_config = {
		/* Internal pull-up/down resistor is disabled */
		.pullSelect = kPORT_PullDisable,
		/* Fast slew rate is configured */
		.slewRate = kPORT_FastSlewRate,
		/* Passive filter is disabled */
		.passiveFilterEnable = kPORT_PassiveFilterDisable,
		/* Low drive strength is configured */
		.driveStrength = kPORT_LowDriveStrength,
		/* Pin is configured as GPIO */
		.mux = kPORT_MuxAsGpio,
	};

	gpio_pin_config_t gpio_max6675_config =
	{
		.outputLogic = 0,
		.pinDirection = kGPIO_DigitalOutput,
	};

	PORT_SetPinConfig(PORTE, 16, &port_max6675_config);
	GPIO_PinInit(GPIOE, 16, &gpio_max6675_config);
}

void board_setLed(board_ledId_enum id, board_ledMsg_enum msg)
{
    switch (msg)
    {
        case BOARD_LED_MSG_OFF:
        	GPIO_PortSet(board_gpioLeds[id].gpio, 1<<board_gpioLeds[id].pin);
            break;

        case BOARD_LED_MSG_ON:
        	GPIO_PortClear(board_gpioLeds[id].gpio, 1<<board_gpioLeds[id].pin);
            break;

        case BOARD_LED_MSG_TOGGLE:
        	GPIO_PortToggle(board_gpioLeds[id].gpio, 1<<board_gpioLeds[id].pin);
            break;

        default:
            break;
    }
}

bool board_getSw(board_swId_enum id)
{
    return !GPIO_ReadPinInput(board_gpioSw[id].gpio, board_gpioSw[id].pin);
}

void board_configSPI0(){

	const port_pin_config_t port_spi_config = {
		/* Internal pull-up resistor is disabled */
		.pullSelect = kPORT_PullDisable,
		/* Fast slew rate is configured */
		.slewRate = kPORT_FastSlewRate,
		/* Passive filter is disabled */
		.passiveFilterEnable = kPORT_PassiveFilterDisable,
		/* Low drive strength is configured */
		.driveStrength = kPORT_LowDriveStrength,
		/* Pin is configured as SPI0_x */
		.mux = kPORT_MuxAlt2,
	};

	//PORT_SetPinConfig(PORTE, 16, &port_spi_config); // SPI0_PCS0 o SPI0_SS
	//GPIO_PinInit(GPIOE, 16, &gpio_oled_config);
	PORT_SetPinConfig(PORTE, 17, &port_spi_config); // SPI0_SCK
	PORT_SetPinConfig(PORTE, 18, &port_spi_config); // SPI0_MOSI
	PORT_SetPinConfig(PORTE, 19, &port_spi_config); // SPI0_MOSI

	CLOCK_EnableClock(kCLOCK_Spi0);

	spi_master_config_t userConfig;

	SPI_MasterGetDefaultConfig(&userConfig);
	board_SPI_Master_Config_Mode(&userConfig);
	SPI_MasterInit(SPI_MASTER, &userConfig, SPI_MASTER_CLK_FREQ);
    SPI_MasterTransferCreateHandle(SPI_MASTER, &handle, NULL, NULL);
}

void board_SPI_Master_Config_Mode(spi_master_config_t *config){

	/* MODO SPI 3 -  CPOL = 1 - CPHA = 1*/ // Func. OK

	#ifdef SPI_MODE_3
	config->polarity = kSPI_ClockPolarityActiveLow;
	config->phase = kSPI_ClockPhaseSecondEdge;
	#endif

	/*!< Active-low SPI clock (idles high). */
	/*!< First edge on SPSCK occurs at the start of the first cycle of a data transfer. */

	/* MODO SPI 2 -  CPOL = 1 - CPHA = 0*/ // Func. MAL

	#ifdef SPI_MODE_2
	config->polarity = kSPI_ClockPolarityActiveLow;
	config->phase = kSPI_ClockPhaseFirstEdge;
	#endif

	/*!< First edge on SPSCK occurs at the middle of the first cycle of a data transfer. */

	/* MODO SPI 1 -  CPOL = 0 - CPHA = 1*/ // Func. OK

	#ifdef SPI_MODE_1
	config->polarity = kSPI_ClockPolarityActiveHigh;
	config->phase = kSPI_ClockPhaseSecondEdge;
	#endif

	/*!< Active-high SPI clock (idles low). */

	/* MODO SPI 0 -  CPOL = 0 - CPHA = 0*/ // Func. OK

	#ifdef SPI_MODE_0
	config->polarity = kSPI_ClockPolarityActiveHigh;
	config->phase = kSPI_ClockPhaseFirstEdge;
	#endif

	config->baudRate_Bps = 500000U;
	config->direction = kSPI_MsbFirst;
	//config->direction = kSPI_LsbFirst;

}

void board_SPISend(uint8_t* buf, size_t len){
	spi_transfer_t xfer;

	xfer.txData = buf;
	xfer.rxData = NULL;
	xfer.dataSize  = len;

	SPI_MasterTransferNonBlocking(SPI_MASTER, &handle, &xfer);
}

void board_SPIReceive(uint8_t* buf, size_t len){
	spi_transfer_t xfer;

	xfer.txData = NULL;
	xfer.rxData = buf;
	xfer.dataSize  = len;

	SPI_MasterTransferBlocking(SPI_MASTER, &xfer);

}
/*==================[end of file]============================================*/

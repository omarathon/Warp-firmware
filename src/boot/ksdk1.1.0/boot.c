/*
	Authored 2016-2018. Phillip Stanley-Marbell.

	Additional contributions, 2018 onwards: See git blame.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"
#include "fsl_lpuart_driver.h"
#include "glaux.h"
#include "warp.h"
#include "errstrs.h"
#include "gpio_pins.h"
#include "SEGGER_RTT.h"


#define							kWarpConstantStringI2cFailure		"\rI2C failed, reg 0x%02x, code %d\n"
#define							kWarpConstantStringErrorInvalidVoltage	"\rInvalid supply voltage [%d] mV!"
#define							kWarpConstantStringErrorSanity		"\rSanity check failed!"

#if (WARP_BUILD_ENABLE_DEVAT45DB || WARP_BUILD_ENABLE_DEVIS25xP)
	#define WARP_BUILD_ENABLE_FLASH 1
#else
	#define WARP_BUILD_ENABLE_FLASH 0
#endif

/*
* Include all sensors because they will be needed to decode flash.
*/

#include "devMMA8451Q.h"
#include "devSSD1331.h" // O. Tanner

// O.Tanner

#define PORTA_BASE (0x40049000u)
#define PORTB_BASE (0x4004A000u)
#define HW_GPIOA (0U)
#define HW_GPIOB (1U)

#if (!WARP_BUILD_ENABLE_FRDMKL03)
	#include "devADXL362.h"
	#include "devAMG8834.h"
	#include "devMAG3110.h"
	#include "devL3GD20H.h"
	#include "devBME680.h"
	#include "devBMX055.h"
	#include "devCCS811.h"
	#include "devHDC1000.h"
	#include "devRV8803C7.h"
#endif


#if (WARP_BUILD_ENABLE_DEVADXL362)
	volatile WarpSPIDeviceState			deviceADXL362State;
#endif

#if (WARP_BUILD_ENABLE_DEVIS25xP)
	#include "devIS25xP.h"
	volatile WarpSPIDeviceState			deviceIS25xPState;
#endif

#if (WARP_BUILD_ENABLE_DEVISL23415)
	#include "devISL23415.h"
	volatile WarpSPIDeviceState			deviceISL23415State;
#endif

#if (WARP_BUILD_ENABLE_DEVAT45DB)
	#include "devAT45DB.h"
	volatile WarpSPIDeviceState			deviceAT45DBState;
#endif

#if (WARP_BUILD_ENABLE_DEVICE40)
	#include "devICE40.h"
	volatile WarpSPIDeviceState			deviceICE40State;
#endif

#if (WARP_BUILD_ENABLE_DEVBMX055)
	volatile WarpI2CDeviceState			deviceBMX055accelState;
	volatile WarpI2CDeviceState			deviceBMX055gyroState;
	volatile WarpI2CDeviceState			deviceBMX055magState;
#endif

#if (WARP_BUILD_ENABLE_DEVMMA8451Q)
	volatile WarpI2CDeviceState			deviceMMA8451QState;
#endif

#if (WARP_BUILD_ENABLE_DEVLPS25H)
	#include "devLPS25H.h"
	volatile WarpI2CDeviceState			deviceLPS25HState;
#endif

#if (WARP_BUILD_ENABLE_DEVHDC1000)
	volatile WarpI2CDeviceState			deviceHDC1000State;
#endif

#if (WARP_BUILD_ENABLE_DEVMAG3110)
	volatile WarpI2CDeviceState			deviceMAG3110State;
#endif

#if (WARP_BUILD_ENABLE_DEVSI7021)
	#include "devSI7021.h"
	volatile WarpI2CDeviceState			deviceSI7021State;
#endif

#if (WARP_BUILD_ENABLE_DEVL3GD20H)
	volatile WarpI2CDeviceState			deviceL3GD20HState;
#endif

#if (WARP_BUILD_ENABLE_DEVBME680)
	volatile WarpI2CDeviceState			deviceBME680State;
	volatile uint8_t				deviceBME680CalibrationValues[kWarpSizesBME680CalibrationValuesCount];
#endif

#if (WARP_BUILD_ENABLE_DEVTCS34725)
	#include "devTCS34725.h"
	volatile WarpI2CDeviceState			deviceTCS34725State;
#endif

#if (WARP_BUILD_ENABLE_DEVSI4705)
	#include "devSI4705.h"
	volatile WarpI2CDeviceState			deviceSI4705State;
#endif

#if (WARP_BUILD_ENABLE_DEVCCS811)
	volatile WarpI2CDeviceState			deviceCCS811State;
#endif

#if (WARP_BUILD_ENABLE_DEVAMG8834)
	volatile WarpI2CDeviceState			deviceAMG8834State;
#endif

#if (WARP_BUILD_ENABLE_DEVAS7262)
	#include "devAS7262.h"
	volatile WarpI2CDeviceState			deviceAS7262State;
#endif

#if (WARP_BUILD_ENABLE_DEVAS7263)
	#include "devAS7263.h"
	volatile WarpI2CDeviceState			deviceAS7263State;
#endif

#if (WARP_BUILD_ENABLE_DEVRV8803C7)
	volatile WarpI2CDeviceState			deviceRV8803C7State;
#endif

#if (WARP_BUILD_ENABLE_DEVBGX)
	#include "devBGX.h"
	volatile WarpUARTDeviceState			deviceBGXState;
#endif

typedef enum
{
	kWarpFlashReadingCountBitField 	= 0b1,
	kWarpFlashRTCTSRBitField 		= 0b10,
	kWarpFlashRTCTPRBitField 		= 0b100,
	kWarpFlashADXL362BitField 		= 0b1000,
	kWarpFlashAMG8834BitField 		= 0b10000,
	kWarpFlashMMA8541QBitField		= 0b100000,
	kWarpFlashMAG3110BitField		= 0b1000000,
	kWarpFlashL3GD20HBitField		= 0b10000000,
	kWarpFlashBME680BitField		= 0b100000000,
	kWarpFlashBMX055BitField		= 0b1000000000,
	kWarpFlashCCS811BitField		= 0b10000000000,
	kWarpFlashHDC1000BitField		= 0b100000000000,
	kWarpFlashRV8803C7BitField		= 0b100000000000000,
	kWarpFlashNumConfigErrors		= 0b1000000000000000,
} WarpFlashSensorBitFieldEncoding;

volatile i2c_master_state_t		  i2cMasterState;
volatile spi_master_state_t		  spiMasterState;
volatile spi_master_user_config_t spiUserConfig;
volatile lpuart_user_config_t	  lpuartUserConfig;
volatile lpuart_state_t			  lpuartState;

volatile bool		  gWarpBooted						   = false;
volatile uint32_t	  gWarpI2cBaudRateKbps				   = kWarpDefaultI2cBaudRateKbps;
volatile uint32_t	  gWarpUartBaudRateBps				   = kWarpDefaultUartBaudRateBps;
volatile uint32_t	  gWarpSpiBaudRateKbps				   = kWarpDefaultSpiBaudRateKbps;
volatile uint32_t	  gWarpSleeptimeSeconds				   = kWarpDefaultSleeptimeSeconds;
volatile WarpModeMask gWarpMode							   = kWarpModeDisableAdcOnSleep;
volatile uint32_t	  gWarpI2cTimeoutMilliseconds		   = kWarpDefaultI2cTimeoutMilliseconds;
volatile uint32_t	  gWarpSpiTimeoutMicroseconds		   = kWarpDefaultSpiTimeoutMicroseconds;
volatile uint32_t	  gWarpUartTimeoutMilliseconds		   = kWarpDefaultUartTimeoutMilliseconds;
volatile uint32_t	  gWarpMenuPrintDelayMilliseconds	   = kWarpDefaultMenuPrintDelayMilliseconds;
volatile uint32_t	  gWarpSupplySettlingDelayMilliseconds = kWarpDefaultSupplySettlingDelayMilliseconds;
volatile uint16_t	  gWarpCurrentSupplyVoltage			   = kWarpDefaultSupplyVoltageMillivolts;

char		  gWarpPrintBuffer[kWarpDefaultPrintBufferSizeBytes];

#if WARP_BUILD_EXTRA_QUIET_MODE
	volatile bool gWarpExtraQuietMode = true;
#else
	volatile bool gWarpExtraQuietMode = false;
#endif

/*
 *	Since only one SPI transaction is ongoing at a time in our implementaion
 */
uint8_t							gWarpSpiCommonSourceBuffer[kWarpMemoryCommonSpiBufferBytes];
uint8_t							gWarpSpiCommonSinkBuffer[kWarpMemoryCommonSpiBufferBytes];

static void						sleepUntilReset(void);
static void						lowPowerPinStates(void);

#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT && !WARP_BUILD_ENABLE_FRDMKL03)
	static void					disableTPS62740(void);
	static void					enableTPS62740(uint16_t voltageMillivolts);
	static void					setTPS62740CommonControlLines(uint16_t voltageMillivolts);
#endif

static void						dumpProcessorState(void);
// static void						repeatRegisterReadForDeviceAndAddress(WarpSensorDevice warpSensorDevice, uint8_t baseAddress,
// 								bool autoIncrement, int chunkReadsPerAddress, bool chatty,
// 								int spinDelay, int repetitionsPerAddress, uint16_t sssupplyMillivolts,
// 								uint16_t adaptiveSssupplyMaxMillivolts, uint8_t referenceByte);
static int						char2int(int character);
// static void						activateAllLowPowerSensorModes(bool verbose);
static void						powerupAllSensors(void);
static uint8_t						readHexByte(void);
static int						read4digits(void);
static void 					writeAllSensorsToFlash(int menuDelayBetweenEachRun, int loopForever);
static void						printAllSensors(bool printHeadersAndCalibration, bool hexModeFlag, int menuDelayBetweenEachRun, bool loopForever);

/*
 *	TODO: change the following to take byte arrays
 */
WarpStatus						writeByteToI2cDeviceRegister(uint8_t i2cAddress, bool sendCommandByte, uint8_t commandByte, bool sendPayloadByte, uint8_t payloadByte);
WarpStatus						writeBytesToSpi(uint8_t *  payloadBytes, int payloadLength);


void							warpLowPowerSecondsSleep(uint32_t sleepSeconds, bool forceAllPinsIntoLowPowerState);

/*
* Flash related functions
*/
	WarpStatus					flashReadAllMemory();
#if (WARP_BUILD_ENABLE_FLASH)
	WarpStatus 					flashHandleEndOfWriteAllSensors();
	WarpStatus					flashWriteFromEnd(size_t nbyte, uint8_t* buf);
	WarpStatus					flashReadMemory(uint16_t startPageNumber, uint8_t startPageOffset, size_t nbyte, void *buf);
	void 						flashHandleReadByte(uint8_t readByte, uint8_t *  bytesIndex, uint8_t *  readingIndex, uint8_t *  sensorIndex, uint8_t *  measurementIndex, uint8_t *  currentSensorNumberOfReadings, uint8_t *  currentSensorSizePerReading, uint16_t *  sensorBitField, uint8_t *  currentNumberOfSensors, int32_t *  currentReading);
	uint8_t						flashGetNSensorsFromSensorBitField(uint16_t sensorBitField);
	void						flashDecodeSensorBitField(uint16_t sensorBitField, uint8_t sensorIndex, uint8_t* sizePerReading, uint8_t* numberOfReadings);
#endif

/*
 *	Derived from KSDK power_manager_demo.c BEGIN>>>
 */
clock_manager_error_code_t clockManagerCallbackRoutine(clock_notify_struct_t *  notify, void *  callbackData);

/*
 *	static clock callback table.
 */
clock_manager_callback_user_config_t		clockManagerCallbackUserlevelStructure =
									{
										.callback	= clockManagerCallbackRoutine,
										.callbackType	= kClockManagerCallbackBeforeAfter,
										.callbackData	= NULL
									};

static clock_manager_callback_user_config_t *	clockCallbackTable[] =
									{
										&clockManagerCallbackUserlevelStructure
									};

clock_manager_error_code_t
clockManagerCallbackRoutine(clock_notify_struct_t *  notify, void *  callbackData)
{
	clock_manager_error_code_t result = kClockManagerSuccess;

	switch (notify->notifyType)
	{
		case kClockManagerNotifyBefore:
			break;
		case kClockManagerNotifyRecover:
		case kClockManagerNotifyAfter:
			break;
		default:
			result = kClockManagerError;
			break;
	}

	return result;
}


/*
 *	Override the RTC IRQ handler
 */
void
RTC_IRQHandler(void)
{
	if (RTC_DRV_IsAlarmPending(0))
	{
		RTC_DRV_SetAlarmIntCmd(0, false);
	}
}

/*
 *	Override the RTC Second IRQ handler
 */
void
RTC_Seconds_IRQHandler(void)
{
	gWarpSleeptimeSeconds++;
}

/*
 *	LLW_IRQHandler override. Since FRDM_KL03Z48M is not defined,
 *	according to power_manager_demo.c, what we need is LLW_IRQHandler.
 *	However, elsewhere in the power_manager_demo.c, the code assumes
 *	FRDM_KL03Z48M _is_ defined (e.g., we need to use LLWU_IRQn, not
 *	LLW_IRQn). Looking through the code base, we see in
 *
 *		ksdk1.1.0/platform/startup/MKL03Z4/gcc/startup_MKL03Z4.S
 *
 *	that the startup initialization assembly requires LLWU_IRQHandler,
 *	not LLW_IRQHandler. See power_manager_demo.c, circa line 216, if
 *	you want to find out more about this dicsussion.
 */
void
LLWU_IRQHandler(void)
{
	/*
	 *	BOARD_* defines are defined in warp.h
	 */
	LLWU_HAL_ClearExternalPinWakeupFlag(LLWU_BASE, (llwu_wakeup_pin_t)BOARD_SW_LLWU_EXT_PIN);
}

/*
 *	IRQ handler for the interrupt from RTC, which we wire up
 *	to PTA0/IRQ0/LLWU_P7 in Glaux. BOARD_SW_LLWU_IRQ_HANDLER
 *	is a synonym for PORTA_IRQHandler.
 */
void
BOARD_SW_LLWU_IRQ_HANDLER(void)
{
	/*
	 *	BOARD_* defines are defined in warp.h
	 */
	PORT_HAL_ClearPortIntFlag(BOARD_SW_LLWU_BASE);
}

/*
 *	Power manager user callback
 */
power_manager_error_code_t
callback0(power_manager_notify_struct_t *  notify, power_manager_callback_data_t *  dataPtr)
{
	WarpPowerManagerCallbackStructure *		callbackUserData = (WarpPowerManagerCallbackStructure *) dataPtr;
	power_manager_error_code_t			status = kPowerManagerError;

	switch (notify->notifyType)
	{
		case kPowerManagerNotifyBefore:
			status = kPowerManagerSuccess;
			break;
		case kPowerManagerNotifyAfter:
			status = kPowerManagerSuccess;
			break;
		default:
			callbackUserData->errorCount++;
			break;
	}

	return status;
}
/*
 *	Derived from KSDK power_manager_demo.c <<END
 */



void
sleepUntilReset(void)
{
	while (1)
	{
#if (WARP_BUILD_ENABLE_DEVSI4705)
		GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
#endif

		warpLowPowerSecondsSleep(1, false /* forceAllPinsIntoLowPowerState */);

#if (WARP_BUILD_ENABLE_DEVSI4705)
		GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
#endif

		warpLowPowerSecondsSleep(60, true /* forceAllPinsIntoLowPowerState */);
	}
}


void
enableLPUARTpins(void)
{
	/*
	 *	Enable UART CLOCK
	 */
	CLOCK_SYS_EnableLpuartClock(0);

	/*
	 *	Set UART pin association. See, e.g., page 99 in
	 *
	 *		https://www.nxp.com/docs/en/reference-manual/KL03P24M48SF0RM.pdf
	 *
	 *	Setup:
	 *		PTB3/kWarpPinI2C0_SCL_UART_TX for UART TX
	 *		PTB4/kWarpPinI2C0_SCL_UART_RX for UART RX
	 *
	 *	TODO: we don't use hw flow control so don't need RTS/CTS
	 *		PTA6/kWarpPinSPI_MISO_UART_RTS for UART RTS
	 *		PTA7/kWarpPinSPI_MOSI_UART_CTS for UART CTS
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAlt3);

	//TODO: we don't use hw flow control so don't need RTS/CTS
	//	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);
	//	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);
	//	GPIO_DRV_SetPinOutput(kWarpPinSPI_MISO_UART_RTS);
	//	GPIO_DRV_SetPinOutput(kWarpPinSPI_MOSI_UART_CTS);

	/*
	 *	Initialize LPUART0. See KSDK13APIRM.pdf section 40.4.3, page 1353
	 */
	lpuartUserConfig.baudRate = gWarpUartBaudRateBps;
	lpuartUserConfig.parityMode = kLpuartParityDisabled;
	lpuartUserConfig.stopBitCount = kLpuartOneStopBit;
	lpuartUserConfig.bitCountPerChar = kLpuart8BitsPerChar;
	lpuartUserConfig.clockSource = kClockLpuartSrcMcgIrClk;

	LPUART_DRV_Init(0,(lpuart_state_t *)&lpuartState,(lpuart_user_config_t *)&lpuartUserConfig);
}


void
disableLPUARTpins(void)
{
	/*
	 *	LPUART deinit
	 */
	LPUART_DRV_Deinit(0);

	/*
	 *	Set UART pin association. See, e.g., page 99 in
	 *
	 *		https://www.nxp.com/docs/en/reference-manual/KL03P24M48SF0RM.pdf
	 *
	 *	Setup:
	 *		PTB3/kWarpPinI2C0_SCL_UART_TX for UART TX
	 *		PTB4/kWarpPinI2C0_SCL_UART_RX for UART RX
	 *		PTA6/kWarpPinSPI_MISO_UART_RTS for UART RTS
	 *		PTA7/kWarpPinSPI_MOSI_UART_CTS for UART CTS
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);

	/*
	 * We don't use the HW flow control and that messes with the SPI any way
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);

	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MISO_UART_RTS);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MOSI_UART_CTS);

	/*
	 *	Disable LPUART CLOCK
	 */
	CLOCK_SYS_DisableLpuartClock(0);
}



WarpStatus
sendBytesToUART(uint8_t *  bytes, size_t nbytes)
{
	lpuart_status_t	status;

	status = LPUART_DRV_SendDataBlocking(0, bytes, nbytes, gWarpUartTimeoutMilliseconds);
	if (status != 0)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}



void
warpEnableSPIpins(void)
{
	CLOCK_SYS_EnableSpiClock(0);

	/*	kWarpPinSPI_MISO_UART_RTS_UART_RTS --> PTA6 (ALT3)	*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAlt3);

	/*	kWarpPinSPI_MOSI_UART_CTS --> PTA7 (ALT3)	*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAlt3);

#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
	/*	kWarpPinSPI_SCK	--> PTA9	(ALT3)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortMuxAlt3);
#else
	/*	kWarpPinSPI_SCK	--> PTB0	(ALT3)		*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAlt3);
#endif

	/*
	 *	Initialize SPI master. See KSDK13APIRM.pdf Section 70.4
	 */
	uint32_t			calculatedBaudRate;
	spiUserConfig.polarity		= kSpiClockPolarity_ActiveHigh;
	spiUserConfig.phase		= kSpiClockPhase_FirstEdge;
	spiUserConfig.direction		= kSpiMsbFirst;
	spiUserConfig.bitsPerSec	= gWarpSpiBaudRateKbps * 1000;
	SPI_DRV_MasterInit(0 /* SPI master instance */, (spi_master_state_t *)&spiMasterState);
	SPI_DRV_MasterConfigureBus(0 /* SPI master instance */, (spi_master_user_config_t *)&spiUserConfig, &calculatedBaudRate);
}



void
warpDisableSPIpins(void)
{
	SPI_DRV_MasterDeinit(0);

	/*	kWarpPinSPI_MISO_UART_RTS	--> PTA6	(GPI)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);

	/*	kWarpPinSPI_MOSI_UART_CTS	--> PTA7	(GPIO)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);

#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
	/*	kWarpPinSPI_SCK	--> PTA9	(GPIO)			*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortMuxAsGpio);
#else
	/*	kWarpPinSPI_SCK	--> PTB0	(GPIO)			*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAsGpio);
#endif

	//TODO: we don't use HW flow control so can remove these since we don't use the RTS/CTS
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MOSI_UART_CTS);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MISO_UART_RTS);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_SCK);

	CLOCK_SYS_DisableSpiClock(0);
}



void
warpDeasserAllSPIchipSelects(void)
{
	/*
	 *	By default, assusme pins are currently disabled (e.g., by a recent lowPowerPinStates())
	 *
	 *	Drive all chip selects high to disable them. Individual drivers call this routine before
	 *	appropriately asserting their respective chip selects.
	 *
	 *	Setup:
	 *		PTA12/kWarpPinISL23415_SPI_nCS	for GPIO
	 *		PTA9/kWarpPinAT45DB_SPI_nCS	for GPIO
	 *		PTA8/kWarpPinADXL362_SPI_nCS	for GPIO
	 *		PTB1/kWarpPinFPGA_nCS		for GPIO
	 *
	 *		On Glaux
									PTB2/kGlauxPinFlash_SPI_nCS for GPIO
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 8, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortMuxAsGpio);
#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
	PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortMuxAsGpio);
#endif

#if (WARP_BUILD_ENABLE_DEVISL23415)
	GPIO_DRV_SetPinOutput(kWarpPinISL23415_SPI_nCS);
#endif

#if (WARP_BUILD_ENABLE_DEVAT45DB)
	GPIO_DRV_SetPinOutput(kWarpPinAT45DB_SPI_nCS);
#endif

#if (WARP_BUILD_ENABLE_DEVADXL362)
	GPIO_DRV_SetPinOutput(kWarpPinADXL362_SPI_nCS);
#endif

#if (WARP_BUILD_ENABLE_DEVICE40)
	GPIO_DRV_SetPinOutput(kWarpPinFPGA_nCS);
#endif

#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
	GPIO_DRV_SetPinOutput(kGlauxPinFlash_SPI_nCS);
#endif
}



void
debugPrintSPIsinkBuffer(void)
{
	for (int i = 0; i < kWarpMemoryCommonSpiBufferBytes; i++)
	{
		warpPrint("\tgWarpSpiCommonSinkBuffer[%d] = [0x%02X]\n", i, gWarpSpiCommonSinkBuffer[i]);
	}
	warpPrint("\n");
}



void
warpEnableI2Cpins(void)
{
	/*
	* Returning here if Glaux variant doesn't work. The program hangs. It seems to be okay if it is done only in the disable function.
	*/
// #if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
		// return;
// #else
	CLOCK_SYS_EnableI2cClock(0);

	/*
	 *	Setup:
	 *
	 *		PTB3/kWarpPinI2C0_SCL_UART_TX	-->	(ALT2 == I2C)
	 *		PTB4/kWarpPinI2C0_SDA_UART_RX	-->	(ALT2 == I2C)
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAlt2);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAlt2);

	I2C_DRV_MasterInit(0 /* I2C instance */, (i2c_master_state_t *)&i2cMasterState);
// #endif
}



void
warpDisableI2Cpins(void)
{
#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
		return;
#else
	I2C_DRV_MasterDeinit(0 /* I2C instance */);

	/*
	 *	Setup:
	 *
	 *		PTB3/kWarpPinI2C0_SCL_UART_TX	-->	disabled
	 *		PTB4/kWarpPinI2C0_SDA_UART_RX	-->	disabled
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);

	CLOCK_SYS_DisableI2cClock(0);
#endif
}


#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
void
lowPowerPinStates(void)
{
	/*
		 *	Following Section 5 of "Power Management for Kinetis L Family" (AN5088.pdf),
		 *	we configure all pins as output and set them to a known state, except for the
		 *	sacrificial pins (WLCSP package, Glaux) where we set them to disabled. We choose
		 *	to set non-disabled pins to '0'.
	 *
	 *	NOTE: Pin state "disabled" means default functionality is active.
	 */

	/*
	 *			PORT A
	 */
	/*
	 *	Leave PTA0/1/2 SWD pins in their default state (i.e., as SWD / Alt3).
	 *
	 *	See GitHub issue https://github.com/physical-computation/Warp-firmware/issues/54
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAlt3);

	/*
		 *	PTA3 and PTA4 are the EXTAL0/XTAL0. They are also connected to the clock output
		 *	of the RV8803 (and PTA4 is a sacrificial pin for PTA3), so do not want to drive them.
		 *	We however have to configure PTA3 to Alt0 (kPortPinDisabled) to get the EXTAL0
		 *	functionality.
	 *
	 *	NOTE:	kPortPinDisabled is the equivalent of `Alt0`
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTA_BASE, 4, kPortPinDisabled);

	/*
	 *	Disable PTA5
	 *
	 *	NOTE: Enabling this significantly increases current draw
	 *	(from ~180uA to ~4mA) and we don't need the RTC on Glaux.
	 *
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 5, kPortPinDisabled);

	/*
	 *	PTA6, PTA7, PTA8, and PTA9 on Glaux are SPI and sacrificial SPI.
	 *
	 *	Section 2.6 of Kinetis Energy Savings – Tips and Tricks says
	 *
	 *		"Unused pins should be configured in the disabled state, mux(0),
		 *		to prevent unwanted leakage (potentially caused by floating inputs)."
	 *
		 *	However, other documents advice to place pin as GPIO and drive low or high.
		 *	For now, leave disabled. Filed issue #54 low-power pin states to investigate.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTA_BASE, 8, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortPinDisabled);

	/*
	 *	NOTE: The KL03 has no PTA10 or PTA11
	 */

	/*
		 *	In Glaux, PTA12 is a sacrificial pin for SWD_RESET, so careful not to drive it.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortPinDisabled);



	/*
	 *			PORT B
	 *
	 *	PTB0 is LED on Glaux. PTB1 is unused, and PTB2 is FLASH_!CS
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortMuxAsGpio);

	/*
	 *	PTB3 and PTB4 (I2C pins) are true open-drain and we
	 *	purposefully leave them disabled since they have pull-ups.
	 *	PTB5 is sacrificial for I2C_SDA, so disable.
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 5, kPortPinDisabled);

	/*
	 *	NOTE:
	 *
	 *	The KL03 has no PTB8, PTB9, or PTB12.  Additionally, the WLCSP package
	 *	we in Glaux has no PTB6, PTB7, PTB10, or PTB11.
	 */

	/*
		 *	In Glaux, PTB13 is a sacrificial pin for SWD_RESET, so careful not to drive it.
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 13, kPortPinDisabled);

	GPIO_DRV_SetPinOutput(kGlauxPinFlash_SPI_nCS);
	GPIO_DRV_ClearPinOutput(kGlauxPinLED);

	return;
}
#else
void
lowPowerPinStates(void)
{
	/*
		 *	Following Section 5 of "Power Management for Kinetis L Family" (AN5088.pdf),
		 *	we configure all pins as output and set them to a known state. We choose
		 *	to set them all to '0' since it happens that the devices we want to keep
		 *	deactivated (SI4705) also need '0'.
	 */

	/*
	 *			PORT A
	 */
	/*
	 *	For now, don't touch the PTA0/1/2 SWD pins. Revisit in the future.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAlt3);

	/*
		 *	PTA3 and PTA4 are the EXTAL0/XTAL0. They are also connected to the clock output
		 *	of the RV8803 (and PTA4 is a sacrificial pin for PTA3), so do not want to drive them.
		 *	We however have to configure PTA3 to Alt0 (kPortPinDisabled) to get the EXTAL0
		 *	functionality.
	 *
	 *	NOTE:	kPortPinDisabled is the equivalent of `Alt0`
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTA_BASE, 4, kPortPinDisabled);

	/*
	 *	Disable PTA5
	 *
	 *	NOTE: Enabling this significantly increases current draw
	 *	(from ~180uA to ~4mA) and we don't need the RTC on revC.
	 *
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 5, kPortPinDisabled);

	/*
	 *	Section 2.6 of Kinetis Energy Savings – Tips and Tricks says
	 *
	 *		"Unused pins should be configured in the disabled state, mux(0),
		 *		to prevent unwanted leakage (potentially caused by floating inputs)."
	 *
		 *	However, other documents advice to place pin as GPIO and drive low or high.
		 *	For now, leave disabled. Filed issue #54 low-power pin states to investigate.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTA_BASE, 8, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortPinDisabled);

	/*
	 *	NOTE: The KL03 has no PTA10 or PTA11
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortPinDisabled);


	/*
	 *			PORT B
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 5, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 6, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 7, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 10, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 11, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 13, kPortPinDisabled);
}
#endif


#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT && !WARP_BUILD_ENABLE_FRDMKL03)
void
disableTPS62740(void)
{
	GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_REGCTRL);
}
#endif


#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT && !WARP_BUILD_ENABLE_FRDMKL03)
void
enableTPS62740(uint16_t voltageMillivolts)
{
	/*
	 *	By default, assusme pins are currently disabled (e.g., by a recent lowPowerPinStates())
	 *
	 *	Setup:
	 *		PTB5/kWarpPinTPS62740_REGCTRL for GPIO
	 *		PTB6/kWarpPinTPS62740_VSEL4 for GPIO
	 *		PTB7/kWarpPinTPS62740_VSEL3 for GPIO
	 *		PTB10/kWarpPinTPS62740_VSEL2 for GPIO
	 *		PTB11/kWarpPinTPS62740_VSEL1 for GPIO
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 5, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 6, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 7, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 10, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 11, kPortMuxAsGpio);

	setTPS62740CommonControlLines(voltageMillivolts);
	GPIO_DRV_SetPinOutput(kWarpPinTPS62740_REGCTRL);
}
#endif


#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT && !WARP_BUILD_ENABLE_FRDMKL03)
void
setTPS62740CommonControlLines(uint16_t voltageMillivolts)
{
		switch(voltageMillivolts)
	{
		case 1800:
		{
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

			break;
		}

		case 1900:
		{
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

			break;
		}

		case 2000:
		{
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

			break;
		}

		case 2100:
		{
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

			break;
		}

		case 2200:
		{
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

			break;
		}

		case 2300:
		{
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

			break;
		}

		case 2400:
		{
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

			break;
		}

		case 2500:
		{
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

			break;
		}

		case 2600:
		{
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

			break;
		}

		case 2700:
		{
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

			break;
		}

		case 2800:
		{
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

			break;
		}

		case 2900:
		{
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

			break;
		}

		case 3000:
		{
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

			break;
		}

		case 3100:
		{
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

			break;
		}

		case 3200:
		{
			GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

			break;
		}

		case 3300:
		{
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
			GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

			break;
		}

		/*
		 *	Should never happen, due to previous check in warpScaleSupplyVoltage()
		 */
		default:
		{
				warpPrint(RTT_CTRL_RESET RTT_CTRL_BG_BRIGHT_YELLOW RTT_CTRL_TEXT_BRIGHT_WHITE kWarpConstantStringErrorSanity RTT_CTRL_RESET "\n");
		}
	}

	/*
		 *	Vload ramp time of the TPS62740 is 800us max (datasheet, Table 8.5 / page 6)
	 */
	OSA_TimeDelay(gWarpSupplySettlingDelayMilliseconds);
}
#endif



void
warpScaleSupplyVoltage(uint16_t voltageMillivolts)
{
	if (voltageMillivolts == gWarpCurrentSupplyVoltage)
	{
		return;
	}

#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT && !WARP_BUILD_ENABLE_FRDMKL03)
	if (voltageMillivolts >= 1800 && voltageMillivolts <= 3300)
	{
		enableTPS62740(voltageMillivolts);
		gWarpCurrentSupplyVoltage = voltageMillivolts;
	}
	else
	{
			warpPrint(RTT_CTRL_RESET RTT_CTRL_BG_BRIGHT_RED RTT_CTRL_TEXT_BRIGHT_WHITE kWarpConstantStringErrorInvalidVoltage RTT_CTRL_RESET "\n", voltageMillivolts);
	}
#endif
}



void
warpDisableSupplyVoltage(void)
{
#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT && !WARP_BUILD_ENABLE_FRDMKL03)
	disableTPS62740();

	/*
		 *	Vload ramp time of the TPS62740 is 800us max (datasheet, Table 8.5 / page 6)
	 */
	OSA_TimeDelay(gWarpSupplySettlingDelayMilliseconds);
#endif
}


void
warpLowPowerSecondsSleep(uint32_t sleepSeconds, bool forceAllPinsIntoLowPowerState)
{
	WarpStatus	status = kWarpStatusOK;

	/*
	 *	Set all pins into low-power states. We don't just disable all pins,
	 *	as the various devices hanging off will be left in higher power draw
	 *	state. And manuals say set pins to output to reduce power.
	 */
	if (forceAllPinsIntoLowPowerState)
	{
		lowPowerPinStates();
	}

	warpSetLowPowerMode(kWarpPowerModeVLPR, 0 /* Sleep Seconds */);
	if ((status != kWarpStatusOK) && (status != kWarpStatusPowerTransitionErrorVlpr2Vlpr))
	{
		warpPrint("warpSetLowPowerMode(kWarpPowerModeVLPR, 0 /* sleep seconds : irrelevant here */)() failed...\n");
	}

	status = warpSetLowPowerMode(kWarpPowerModeVLPS, sleepSeconds);
	if (status != kWarpStatusOK)
	{
		warpPrint("warpSetLowPowerMode(kWarpPowerModeVLPS, 0 /* sleep seconds : irrelevant here */)() failed...\n");
	}
}

/*
void
printPinDirections(void)
{
	warpPrint("I2C0_SDA:%d\n", GPIO_DRV_GetPinDir(kWarpPinI2C0_SDA_UART_RX));
	OSA_TimeDelay(100);
	warpPrint("I2C0_SCL:%d\n", GPIO_DRV_GetPinDir(kWarpPinI2C0_SCL_UART_TX));
	OSA_TimeDelay(100);
	warpPrint("SPI_MOSI:%d\n", GPIO_DRV_GetPinDir(kWarpPinSPI_MOSI_UART_CTS));
	OSA_TimeDelay(100);
	warpPrint("SPI_MISO:%d\n", GPIO_DRV_GetPinDir(kWarpPinSPI_MISO_UART_RTS));
	OSA_TimeDelay(100);
	warpPrint("SPI_SCK_I2C_PULLUP_EN:%d\n", GPIO_DRV_GetPinDir(kWarpPinSPI_SCK_I2C_PULLUP_EN));
	OSA_TimeDelay(100);
				warpPrint("ADXL362_CS:%d\n", GPIO_DRV_GetPinDir(kWarpPinADXL362_CS));
				OSA_TimeDelay(100);
}
*/



void
dumpProcessorState(void)
{
	uint32_t	cpuClockFrequency;

	CLOCK_SYS_GetFreq(kCoreClock, &cpuClockFrequency);
	warpPrint("\r\n\n\tCPU @ %u KHz\n", (cpuClockFrequency / 1000));
	warpPrint("\r\tCPU power mode: %u\n", POWER_SYS_GetCurrentMode());
	warpPrint("\r\tCPU clock manager configuration: %u\n", CLOCK_SYS_GetCurrentConfiguration());
	warpPrint("\r\tRTC clock: %d\n", CLOCK_SYS_GetRtcGateCmd(0));
	warpPrint("\r\tSPI clock: %d\n", CLOCK_SYS_GetSpiGateCmd(0));
	warpPrint("\r\tI2C clock: %d\n", CLOCK_SYS_GetI2cGateCmd(0));
	warpPrint("\r\tLPUART clock: %d\n", CLOCK_SYS_GetLpuartGateCmd(0));
	warpPrint("\r\tPORT A clock: %d\n", CLOCK_SYS_GetPortGateCmd(0));
	warpPrint("\r\tPORT B clock: %d\n", CLOCK_SYS_GetPortGateCmd(1));
	warpPrint("\r\tFTF clock: %d\n", CLOCK_SYS_GetFtfGateCmd(0));
	warpPrint("\r\tADC clock: %d\n", CLOCK_SYS_GetAdcGateCmd(0));
	warpPrint("\r\tCMP clock: %d\n", CLOCK_SYS_GetCmpGateCmd(0));
	warpPrint("\r\tVREF clock: %d\n", CLOCK_SYS_GetVrefGateCmd(0));
	warpPrint("\r\tTPM clock: %d\n", CLOCK_SYS_GetTpmGateCmd(0));
}


void
printBootSplash(uint16_t gWarpCurrentSupplyVoltage, uint8_t menuRegisterAddress, WarpPowerManagerCallbackStructure *  powerManagerCallbackStructure)
{
	/*
	 *	We break up the prints with small delays to allow us to use small RTT print
	 *	buffers without overrunning them when at max CPU speed.
	 */
	warpPrint("\r\n\n\n\n[ *\t\t\t\tWarp (HW revision C) / Glaux (HW revision B)\t\t\t* ]\n");
	warpPrint("\r[  \t\t\t\t      Cambridge / Physcomplab / OMAR CW2   \t\t\t\t  ]\n\n");
	warpPrint("\r\tSupply=%dmV,\tDefault Target Read Register=0x%02x\n",
			  gWarpCurrentSupplyVoltage, menuRegisterAddress);
	warpPrint("\r\tI2C=%dkb/s,\tSPI=%dkb/s,\tUART=%db/s,\tI2C Pull-Up=%d\n\n",
			  gWarpI2cBaudRateKbps, gWarpSpiBaudRateKbps, gWarpUartBaudRateBps);
	warpPrint("\r\tSIM->SCGC6=0x%02x\t\tRTC->SR=0x%02x\t\tRTC->TSR=0x%02x\n", SIM->SCGC6, RTC->SR, RTC->TSR);
	warpPrint("\r\tMCG_C1=0x%02x\t\t\tMCG_C2=0x%02x\t\tMCG_S=0x%02x\n", MCG_C1, MCG_C2, MCG_S);
	warpPrint("\r\tMCG_SC=0x%02x\t\t\tMCG_MC=0x%02x\t\tOSC_CR=0x%02x\n", MCG_SC, MCG_MC, OSC_CR);
	warpPrint("\r\tSMC_PMPROT=0x%02x\t\t\tSMC_PMCTRL=0x%02x\t\tSCB->SCR=0x%02x\n", SMC_PMPROT, SMC_PMCTRL, SCB->SCR);
	warpPrint("\r\tPMC_REGSC=0x%02x\t\t\tSIM_SCGC4=0x%02x\tRTC->TPR=0x%02x\n\n", PMC_REGSC, SIM_SCGC4, RTC->TPR);
	warpPrint("\r\t%ds in RTC Handler to-date,\t%d Pmgr Errors\n", gWarpSleeptimeSeconds, powerManagerCallbackStructure->errorCount);
}

void
blinkLED(int pin)
{
	GPIO_DRV_SetPinOutput(pin);
	OSA_TimeDelay(200);
	GPIO_DRV_ClearPinOutput(pin);
	OSA_TimeDelay(200);

	return;
}

void
warpPrint(const char *fmt, ...)
{
	if (gWarpExtraQuietMode)
	{
		return;
	}

	int	fmtlen;
	va_list	arg;

/*
 *	We use an ifdef rather than a C if to allow us to compile-out
 *	all references to SEGGER_RTT_*printf if we don't want them.
 *
 *	NOTE: SEGGER_RTT_vprintf takes a va_list* rather than a va_list
 *	like usual vprintf. We modify the SEGGER_RTT_vprintf so that it
 *	also takes our print buffer which we will eventually send over
 *	BLE. Using SEGGER_RTT_vprintf() versus the libc vsnprintf saves
 *	2kB flash and removes the use of malloc so we can keep heap
 *	allocation to zero.
 */
#if (WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF)
	/*
	 *	We can't use SEGGER_RTT_vprintf to format into a buffer
	 *	since SEGGER_RTT_vprintf formats directly into the special
	 *	RTT memory region to be picked up by the RTT / SWD mechanism...
	 */
	va_start(arg, fmt);
		fmtlen = SEGGER_RTT_vprintf(0, fmt, &arg, gWarpPrintBuffer, kWarpDefaultPrintBufferSizeBytes);
	va_end(arg);

	if (fmtlen < 0)
	{
		SEGGER_RTT_WriteString(0, gWarpEfmt);

	#if (WARP_BUILD_ENABLE_DEVBGX)
		if (gWarpBooted)
		{
					WarpStatus	status;

			enableLPUARTpins();
			initBGX(kWarpDefaultSupplyVoltageMillivoltsBGX);
					status = sendBytesToUART((uint8_t *)gWarpEfmt, strlen(gWarpEfmt)+1);
			if (status != kWarpStatusOK)
			{
				SEGGER_RTT_WriteString(0, gWarpEuartSendChars);
			}
			disableLPUARTpins();

			/*
			 *	We don't want to deInit() the BGX since that would drop
			 *	any remote terminal connected to it.
			 */
					//deinitBGX();
		}
	#endif

		return;
	}

	/*
	 *	If WARP_BUILD_ENABLE_DEVBGX, also send the fmt to the UART / BLE.
	 */
	#if (WARP_BUILD_ENABLE_DEVBGX)
	if (gWarpBooted)
	{
				WarpStatus	status;

		enableLPUARTpins();
		initBGX(kWarpDefaultSupplyVoltageMillivoltsBGX);

				status = sendBytesToUART((uint8_t *)gWarpPrintBuffer, max(fmtlen, kWarpDefaultPrintBufferSizeBytes));
		if (status != kWarpStatusOK)
		{
			SEGGER_RTT_WriteString(0, gWarpEuartSendChars);
		}
		disableLPUARTpins();

		/*
		 *	We don't want to deInit() the BGX since that would drop
		 *	any remote terminal connected to it.
		 */
				//deinitBGX();
	}
	#endif

#else
	/*
	 *	If we are not compiling in the SEGGER_RTT_printf,
	 *	we just send the format string of warpPrint()
	 */
	SEGGER_RTT_WriteString(0, fmt);

	/*
	 *	If WARP_BUILD_ENABLE_DEVBGX, also send the fmt to the UART / BLE.
	 */
	#if (WARP_BUILD_ENABLE_DEVBGX)
	if (gWarpBooted)
	{
				WarpStatus	status;

		enableLPUARTpins();
		initBGX(kWarpDefaultSupplyVoltageMillivoltsBGX);
		status = sendBytesToUART(fmt, strlen(fmt));
		if (status != kWarpStatusOK)
		{
			SEGGER_RTT_WriteString(0, gWarpEuartSendChars);
		}
		disableLPUARTpins();

		/*
		 *	We don't want to deInit() the BGX since that would drop
		 *	any remote terminal connected to it.
		 */
				//deinitBGX();
	}
	#endif
#endif


	/*
	 *	Throttle to enable SEGGER to grab output, otherwise "run" mode may miss lines.
	 */
	OSA_TimeDelay(5);

	return;
}

int
warpWaitKey(void)
{
	/*
	 *	SEGGER'S implementation assumes the result of result of
	 *	SEGGER_RTT_GetKey() is an int, so we play along.
	 */
	int		rttKey, bleChar = kWarpMiscMarkerForAbsentByte;

/*
 *	Set the UART buffer to 0xFF and then wait until either the
 *	UART RX buffer changes or the RTT icoming key changes.
 *
 *	The check below on rttKey is exactly what SEGGER_RTT_WaitKey()
 *	does in SEGGER_RTT.c.
 */
#if (WARP_BUILD_ENABLE_DEVBGX)
	deviceBGXState.uartRXBuffer[0] = kWarpMiscMarkerForAbsentByte;
	enableLPUARTpins();
	initBGX(kWarpDefaultSupplyVoltageMillivoltsBGX);
#endif

	do
	{
		rttKey	= SEGGER_RTT_GetKey();

#if (WARP_BUILD_ENABLE_DEVBGX)
			bleChar	= deviceBGXState.uartRXBuffer[0];
#endif

		/*
		 *	NOTE: We ignore all chars on BLE except '0'-'9', 'a'-'z'/'A'-Z'
		 */
		if (!(bleChar > 'a' && bleChar < 'z') && !(bleChar > 'A' && bleChar < 'Z') && !(bleChar > '0' && bleChar < '9'))
		{
			bleChar = kWarpMiscMarkerForAbsentByte;
		}
	} while ((rttKey < 0) && (bleChar == kWarpMiscMarkerForAbsentByte));

#if (WARP_BUILD_ENABLE_DEVBGX)
	if (bleChar != kWarpMiscMarkerForAbsentByte)
	{
		/*
		 *	Send a copy of incoming BLE chars to RTT
		 */
		SEGGER_RTT_PutChar(0, bleChar);
		disableLPUARTpins();

		/*
		 *	We don't want to deInit() the BGX since that would drop
		 *	any remote terminal connected to it.
		 */
			//deinitBGX();

		return (int)bleChar;
	}

	/*
	 *	Send a copy of incoming RTT chars to BLE
	 */
		WarpStatus status = sendBytesToUART((uint8_t *)&rttKey, 1);
	if (status != kWarpStatusOK)
	{
		SEGGER_RTT_WriteString(0, gWarpEuartSendChars);
	}

	disableLPUARTpins();

	/*
	 *	We don't want to deInit() the BGX since that would drop
	 *	any remote terminal connected to it.
	 */
		//deinitBGX();
#endif

	return rttKey;
}

int
main(void)
{
	WarpStatus				status;
	uint8_t					key;
	WarpSensorDevice			menuTargetSensor		= kWarpSensorBMX055accel;
	volatile WarpI2CDeviceState *		menuI2cDevice			= NULL;
	uint8_t					menuRegisterAddress		= 0x00;
	rtc_datetime_t				warpBootDate;
	power_manager_user_config_t		warpPowerModeWaitConfig;
	power_manager_user_config_t		warpPowerModeStopConfig;
	power_manager_user_config_t		warpPowerModeVlpwConfig;
	power_manager_user_config_t		warpPowerModeVlpsConfig;
	power_manager_user_config_t		warpPowerModeVlls0Config;
	power_manager_user_config_t		warpPowerModeVlls1Config;
	power_manager_user_config_t		warpPowerModeVlls3Config;
	power_manager_user_config_t		warpPowerModeRunConfig;

	/*
	 *	We use this as a template later below and change the .mode fields for the different other modes.
	 */
	const power_manager_user_config_t	warpPowerModeVlprConfig = {
							.mode			= kPowerManagerVlpr,
							.sleepOnExitValue	= false,
							.sleepOnExitOption	= false
						};

	power_manager_user_config_t const *	powerConfigs[] = {
		/*
		 *	NOTE: POWER_SYS_SetMode() depends on this order
		 *
		 *	See KSDK13APIRM.pdf Section 55.5.3
		 */
		&warpPowerModeWaitConfig,
		&warpPowerModeStopConfig,
		&warpPowerModeVlprConfig,
		&warpPowerModeVlpwConfig,
		&warpPowerModeVlpsConfig,
		&warpPowerModeVlls0Config,
		&warpPowerModeVlls1Config,
		&warpPowerModeVlls3Config,
		&warpPowerModeRunConfig,
	};

	WarpPowerManagerCallbackStructure		powerManagerCallbackStructure;

	/*
	 *	Callback configuration structure for power manager
	 */
	const power_manager_callback_user_config_t callbackCfg0 = {
							callback0,
							kPowerManagerCallbackBeforeAfter,
							(power_manager_callback_data_t *) &powerManagerCallbackStructure};

	/*
	 *	Pointers to power manager callbacks.
	 */
	power_manager_callback_user_config_t const *	callbacks[] = {
								&callbackCfg0
						};

	/*
	 *	Enable clock for I/O PORT A and PORT B
	 */
	CLOCK_SYS_EnablePortClock(0);
	CLOCK_SYS_EnablePortClock(1);

	/*
	 *	Set board crystal value (Warp revB and earlier).
	 */
	g_xtal0ClkFreq = 32768U;

	/*
	 *	Initialize KSDK Operating System Abstraction layer (OSA) layer.
	 */
	OSA_Init();

	/*
	 *	Setup SEGGER RTT to output as much as fits in buffers.
	 *
	 *	Using SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL can lead to deadlock, since
	 *	we might have SWD disabled at time of blockage.
	 */
	SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);

	/*
	 *	When booting to CSV stream, we wait to be up and running as soon as possible after
	 *	a reset (e.g., a reset due to waking from VLLS0)
	 */
	if (!WARP_BUILD_BOOT_TO_CSVSTREAM)
	{
		warpPrint("\n\n\n\rBooting Warp, in 3... ");
		OSA_TimeDelay(1000);
		warpPrint("2... ");
		OSA_TimeDelay(1000);
		warpPrint("1...\n\n\n\r");
		OSA_TimeDelay(1000);
	}

	/*
	 *	Configure Clock Manager to default, and set callback for Clock Manager mode transition.
	 *
	 *	See "Clocks and Low Power modes with KSDK and Processor Expert" document (Low_Power_KSDK_PEx.pdf)
	 */
	CLOCK_SYS_Init(	g_defaultClockConfigurations,
			CLOCK_CONFIG_NUM, /* The default value of this is defined in fsl_clock_MKL03Z4.h as 2 */
			&clockCallbackTable,
			ARRAY_SIZE(clockCallbackTable)
			);
	CLOCK_SYS_UpdateConfiguration(CLOCK_CONFIG_INDEX_FOR_RUN, kClockManagerPolicyForcible);

	/*
	 *	Initialize RTC Driver (not needed on Glaux, but we enable it anyway for now
	 *	as that lets us use the current sleep routines). NOTE: We also don't seem to
	 *	be able to go to VLPR mode unless we enable the RTC.
	 */
	RTC_DRV_Init(0);

	/*
	 *	Set initial date to 1st January 2016 00:00, and set date via RTC driver
	 */
	warpBootDate.year	= 2016U;
	warpBootDate.month	= 1U;
	warpBootDate.day	= 1U;
	warpBootDate.hour	= 0U;
	warpBootDate.minute	= 0U;
	warpBootDate.second	= 0U;
	RTC_DRV_SetDatetime(0, &warpBootDate);

	/*
	 *	Setup Power Manager Driver
	 */
	memset(&powerManagerCallbackStructure, 0, sizeof(WarpPowerManagerCallbackStructure));

	warpPowerModeVlpwConfig = warpPowerModeVlprConfig;
	warpPowerModeVlpwConfig.mode = kPowerManagerVlpw;

	warpPowerModeVlpsConfig = warpPowerModeVlprConfig;
	warpPowerModeVlpsConfig.mode = kPowerManagerVlps;

	warpPowerModeWaitConfig = warpPowerModeVlprConfig;
	warpPowerModeWaitConfig.mode = kPowerManagerWait;

	warpPowerModeStopConfig = warpPowerModeVlprConfig;
	warpPowerModeStopConfig.mode = kPowerManagerStop;

	warpPowerModeVlls0Config = warpPowerModeVlprConfig;
	warpPowerModeVlls0Config.mode = kPowerManagerVlls0;

	warpPowerModeVlls1Config = warpPowerModeVlprConfig;
	warpPowerModeVlls1Config.mode = kPowerManagerVlls1;

	warpPowerModeVlls3Config = warpPowerModeVlprConfig;
	warpPowerModeVlls3Config.mode = kPowerManagerVlls3;

	warpPowerModeRunConfig.mode = kPowerManagerRun;

	POWER_SYS_Init(	&powerConfigs,
			sizeof(powerConfigs)/sizeof(power_manager_user_config_t *),
			&callbacks,
			sizeof(callbacks)/sizeof(power_manager_callback_user_config_t *)
			);

	/*
	 *	Switch CPU to Very Low Power Run (VLPR) mode
	 */
	if (WARP_BUILD_BOOT_TO_VLPR)
	{
		warpPrint("About to switch CPU to VLPR mode... ");
		status = warpSetLowPowerMode(kWarpPowerModeVLPR, 0 /* Sleep Seconds */);
		if ((status != kWarpStatusOK) && (status != kWarpStatusPowerTransitionErrorVlpr2Vlpr))
		{
			warpPrint("warpSetLowPowerMode(kWarpPowerModeVLPR() failed...\n");
		}
		warpPrint("done.\n\r");
	}

	/*
	 *	Initialize the GPIO pins with the appropriate pull-up, etc.,
	 *	defined in the inputPins and outputPins arrays (gpio_pins.c).
	 *
	 *	See also Section 30.3.3 GPIO Initialization of KSDK13APIRM.pdf
	 */
	warpPrint("About to GPIO_DRV_Init()... ");
	GPIO_DRV_Init(inputPins  /* input pins */, outputPins  /* output pins */);
	warpPrint("done.\n");

	/*
	 *	Make sure the SWD pins, PTA0/1/2 SWD pins in their ALT3 state (i.e., as SWD).
	 *
	 *	See GitHub issue https://github.com/physical-computation/Warp-firmware/issues/54
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAlt3);

	/*
	 *	Note that it is lowPowerPinStates() that sets the pin mux mode,
	 *	so until we call it pins are in their default state.
	 */
	warpPrint("About to lowPowerPinStates()... ");
	lowPowerPinStates();
	warpPrint("done.\n");

/*
 *	Toggle LED3 (kWarpPinSI4705_nRST on Warp revB, kGlauxPinLED on Glaux)
 */
#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
	blinkLED(kGlauxPinLED);
	// blinkLED(kGlauxPinLED);
	// blinkLED(kGlauxPinLED);
#endif

/*
 *	Initialize all the sensors
 */
#if (WARP_BUILD_ENABLE_DEVBMX055)
		initBMX055accel(0x18	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsBMX055accel	);
		initBMX055gyro(	0x68	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsBMX055gyro	);
		initBMX055mag(	0x10	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsBMX055mag	);
#endif

#if (WARP_BUILD_ENABLE_DEVMMA8451Q)
		initMMA8451Q(	0x1D	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsMMA8451Q	);
#endif

#if (WARP_BUILD_ENABLE_DEVLPS25H)
		initLPS25H(	0x5C	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsLPS25H	);
#endif

#if (WARP_BUILD_ENABLE_DEVHDC1000)
		initHDC1000(	0x43	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsHDC1000	);
#endif

#if (WARP_BUILD_ENABLE_DEVMAG3110)
		initMAG3110(	0x0E	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsMAG3110	);
#endif

#if (WARP_BUILD_ENABLE_DEVSI7021)
		initSI7021(	0x40	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsSI7021	);
#endif

#if (WARP_BUILD_ENABLE_DEVL3GD20H)
		initL3GD20H(	0x6A	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsL3GD20H	);
#endif

#if (WARP_BUILD_ENABLE_DEVBME680)
		initBME680(	0x77	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsBME680	);
#endif

#if (WARP_BUILD_ENABLE_DEVTCS34725)
		initTCS34725(	0x29	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsTCS34725	);
#endif

#if (WARP_BUILD_ENABLE_DEVSI4705)
		initSI4705(	0x11	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsSI4705	);
#endif

#if (WARP_BUILD_ENABLE_DEVCCS811)
		initCCS811(	0x5A	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsCCS811	);
#endif

#if (WARP_BUILD_ENABLE_DEVAMG8834)
		initAMG8834(	0x68	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsAMG8834	);
#endif

#if (WARP_BUILD_ENABLE_DEVAS7262)
		initAS7262(	0x49	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsAS7262	);
#endif

#if (WARP_BUILD_ENABLE_DEVAS7263)
		initAS7263(	0x49	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsAS7263	);
#endif

#if (WARP_BUILD_ENABLE_DEVRV8803C7)
		initRV8803C7(	0x32	/* i2cAddress */,					kWarpDefaultSupplyVoltageMillivoltsRV8803C7	);
		status = setRTCCountdownRV8803C7(0 /* countdown */, kWarpRV8803ExtTD_1HZ /* frequency */, false /* interupt_enable */);
	if (status != kWarpStatusOK)
	{
		warpPrint("setRTCCountdownRV8803C7() failed...\n");
	}
	else
	{
		warpPrint("setRTCCountdownRV8803C7() succeeded.\n");
	}

	/*
	 *	Set the CLKOUT frequency to 1Hz, to reduce CV^2 power on the CLKOUT pin.
	 *	See RV-8803-C7_App-Manual.pdf section 3.6 (register is 0Dh)
	 */
		uint8_t	extReg;
	status = readRTCRegisterRV8803C7(kWarpRV8803RegExt, &extReg);
	if (status != kWarpStatusOK)
	{
		warpPrint("readRTCRegisterRV8803C7() failed...\n");
	}
	else
	{
		warpPrint("readRTCRegisterRV8803C7() succeeded.\n");
	}

	/*
	 *	Set bits 3:2 (FD) to 10 (1Hz CLKOUT)
	 */
	extReg &= 0b11110011;
	extReg |= 0b00001000;
	status = writeRTCRegisterRV8803C7(kWarpRV8803RegExt, extReg);
	if (status != kWarpStatusOK)
	{
		warpPrint("writeRTCRegisterRV8803C7() failed...\n");
	}
	else
	{
		warpPrint("writeRTCRegisterRV8803C7() succeeded.\n");
	}
#endif

	/*
	 *	Initialization: Devices hanging off SPI
	 */

#if (WARP_BUILD_ENABLE_DEVADXL362)
	/*
	 *	Only supported in main Warp variant.
	 */
		initADXL362(kWarpPinADXL362_SPI_nCS,						kWarpDefaultSupplyVoltageMillivoltsADXL362	);

		status = readSensorRegisterADXL362(kWarpSensorConfigurationRegisterADXL362DEVID_AD, 1);
	if (status != kWarpStatusOK)
	{
		warpPrint("ADXL362: SPI transaction to read DEVID_AD failed...\n");
	}
	else
	{
			warpPrint("ADXL362: DEVID_AD = [0x%02X].\n", deviceADXL362State.spiSinkBuffer[2]);
	}

		status = readSensorRegisterADXL362(kWarpSensorConfigurationRegisterADXL362DEVID_MST, 1);
	if (status != kWarpStatusOK)
	{
		warpPrint("ADXL362: SPI transaction to read DEVID_MST failed...\n");
	}
	else
	{
			warpPrint("ADXL362: DEVID_MST = [0x%02X].\n", deviceADXL362State.spiSinkBuffer[2]);
	}
#endif

#if (WARP_BUILD_ENABLE_DEVIS25xP && WARP_BUILD_ENABLE_GLAUX_VARIANT)
	/*
	 *	Only supported in Glaux.
	 */
	initIS25xP(kGlauxPinFlash_SPI_nCS, kWarpDefaultSupplyVoltageMillivoltsIS25xP);

#elif (WARP_BUILD_ENABLE_DEVIS25xP)
	initIS25xP(kWarpPinIS25xP_SPI_nCS, kWarpDefaultSupplyVoltageMillivoltsIS25xP);
#endif

#if (WARP_BUILD_ENABLE_DEVISL23415)
	/*
	 *	Only supported in main Warp variant.
	 */
		initISL23415(kWarpPinISL23415_SPI_nCS, kWarpDefaultSupplyVoltageMillivoltsISL23415);

	/*
		 *	Take the DCPs out of shutdown by setting the SHDN bit in the ACR register
	 */
		status = writeDeviceRegisterISL23415(kWarpSensorConfigurationRegisterISL23415ACRwriteInstruction, 0x40);
	if (status != kWarpStatusOK)
	{
		warpPrint("ISL23415: SPI transaction to write ACR failed...\n");
	}

		status = readDeviceRegisterISL23415(kWarpSensorConfigurationRegisterISL23415ACRreadInstruction);
	if (status != kWarpStatusOK)
	{
		warpPrint("ISL23415: SPI transaction to read ACR failed...\n");
	}
	else
	{
		warpPrint("ISL23415 ACR=[0x%02X], ", deviceISL23415State.spiSinkBuffer[3]);
	}

		status = readDeviceRegisterISL23415(kWarpSensorConfigurationRegisterISL23415WRreadInstruction);
	if (status != kWarpStatusOK)
	{
		warpPrint("ISL23415: SPI transaction to read WR failed...\n");
	}
	else
	{
		warpPrint("WR=[0x%02X]\n", deviceISL23415State.spiSinkBuffer[3]);
	}
#endif

#if (WARP_BUILD_ENABLE_DEVAT45DB)
	/*
	 *	Only supported in main Warp variant.
	 */
	status =  initAT45DB(kWarpPinAT45DB_SPI_nCS,						kWarpDefaultSupplyVoltageMillivoltsAT45DB	);
	if (status != kWarpStatusOK)
	{
		warpPrint("AT45DB: initAT45DB() failed...\n");
	}

	status = spiTransactionAT45DB(&deviceAT45DBState, (uint8_t *)"\x9F\x00\x00\x00\x00\x00", 6 /* opCount */);
	if (status != kWarpStatusOK)
	{
		warpPrint("AT45DB: SPI transaction to read Manufacturer ID failed...\n");
	}
	else
	{
		warpPrint("AT45DB Manufacturer ID=[0x%02X], Device ID=[0x%02X 0x%02X], Extended Device Information=[0x%02X 0x%02X]\n",
			deviceAT45DBState.spiSinkBuffer[1],
			deviceAT45DBState.spiSinkBuffer[2], deviceAT45DBState.spiSinkBuffer[3],
			deviceAT45DBState.spiSinkBuffer[4], deviceAT45DBState.spiSinkBuffer[5]);
	}
#endif

#if (WARP_BUILD_ENABLE_DEVICE40)
	/*
	 *	Only supported in main Warp variant.
	 */
		initICE40(kWarpPinFPGA_nCS,							kWarpDefaultSupplyVoltageMillivoltsICE40	);
#endif

#if (WARP_BUILD_ENABLE_DEVBGX)
	warpPrint("Configuring BGX Bluetooth.\n");
	warpPrint("Enabling UART... ");
	enableLPUARTpins();
	warpPrint("done.\n");
	warpPrint("initBGX()... ");
	initBGX(kWarpDefaultSupplyVoltageMillivoltsBGX);
	warpPrint("done.\n");
#endif

	/*
	 *	If WARP_BUILD_DISABLE_SUPPLIES_BY_DEFAULT, will turn of the supplies
	 *	below which also means that the console via BLE will be disabled as
	 *	the BLE module will be turned off by default.
	 */

#if (WARP_BUILD_DISABLE_SUPPLIES_BY_DEFAULT)
	/*
	 *	Make sure sensor supplies are off.
	 *
	 *	(There's no point in calling activateAllLowPowerSensorModes())
	 */
	warpPrint("Disabling sensor supply... \n");
	warpDisableSupplyVoltage();
	warpPrint("done.\n");
#endif

	/*
	 *	At this point, we consider the system "booted" and, e.g., warpPrint()s
	 *	will also be sent to the BLE if that is compiled in.
	 */
	gWarpBooted = true;
	warpPrint("Boot done.\n");

#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT && WARP_BUILD_BOOT_TO_CSVSTREAM)
	int timer  = 0;
	int rttKey = -1;

    bool _originalWarpExtraQuietMode = gWarpExtraQuietMode;
    gWarpExtraQuietMode = false;

    // Run Display initialisation code - O. Tanner

    devSSD1331init();

    warpPrint("Press any key to show menu...\n");
    gWarpExtraQuietMode = _originalWarpExtraQuietMode;

	while (rttKey < 0 && timer < kWarpCsvstreamMenuWaitTimeMilliSeconds)
	{
		rttKey = SEGGER_RTT_GetKey();
		OSA_TimeDelay(1);
		timer++;
	}

	if (rttKey < 0)
	{
		printBootSplash(gWarpCurrentSupplyVoltage, menuRegisterAddress,
						&powerManagerCallbackStructure);

		/*
		 *	Force to printAllSensors
		 */
		gWarpI2cBaudRateKbps = 300;

		if (!WARP_BUILD_BOOT_TO_VLPR)
		{
			status = warpSetLowPowerMode(kWarpPowerModeRUN, 0 /* sleep seconds : irrelevant here */);
			if (status != kWarpStatusOK)
			{
				warpPrint("warpSetLowPowerMode(kWarpPowerModeRUN, 0 /* sleep seconds : irrelevant here */)() failed...\n");
			}
		}


#if (WARP_CSVSTREAM_TO_FLASH)
		warpPrint("\r\n\tWriting directly to flash. Press 'q' to exit.\n");
		writeAllSensorsToFlash(0, true);

#else
		printAllSensors(true /* printHeadersAndCalibration */, true /* hexModeFlag */,
						0 /* menuDelayBetweenEachRun */, true /* loopForever */);
#endif

		/*
		 *	Notreached
		 */
	}
#endif

#if (WARP_BUILD_ENABLE_GLAUX_VARIANT && WARP_BUILD_BOOT_TO_CSVSTREAM)
	warpScaleSupplyVoltage(3300);
	int timer  = 0;
	int rttKey = -1;

	bool _originalWarpExtraQuietMode = gWarpExtraQuietMode;
	gWarpExtraQuietMode = false;
	warpPrint("Press any key to show menu...\n");
	gWarpExtraQuietMode = _originalWarpExtraQuietMode;

	while (rttKey < 0 && timer < kWarpCsvstreamMenuWaitTimeMilliSeconds)
	{
		rttKey = SEGGER_RTT_GetKey();
		OSA_TimeDelay(1);
		timer++;
	}

	if (rttKey < 0)
	{
		printBootSplash(gWarpCurrentSupplyVoltage, menuRegisterAddress, &powerManagerCallbackStructure);

		warpPrint("About to loop with printSensorDataBME680()...\n");
		while (1)
		{
			blinkLED(kGlauxPinLED);
			for (int i = 0; i < kGlauxSensorRepetitionsPerSleepIteration; i++)
			{
#if (WARP_CSVSTREAM_TO_FLASH)
				writeAllSensorsToFlash(1, false);
#else
				printAllSensors(true /* printHeadersAndCalibration */, true /* hexModeFlag */, 0 /* menuDelayBetweenEachRun */, false /* loopForever */);
#endif
			}

			warpPrint("About to configureSensorBME680() for sleep...\n");
				status = configureSensorBME680(	0b00000000,	/*	payloadCtrl_Hum: Sleep							*/
								0b00000000,	/*	payloadCtrl_Meas: No temperature samples, no pressure samples, sleep	*/
								0b00001000	/*	payloadGas_0: Turn off heater						*/
			);
			if (status != kWarpStatusOK)
			{
				warpPrint("configureSensorBME680() failed...\n");
			}

			warpDisableI2Cpins();
			blinkLED(kGlauxPinLED);

				warpPrint("About to go into VLLS0...\n");
				status = warpSetLowPowerMode(kWarpPowerModeVLLS0, kGlauxSleepSecondsBetweenSensorRepetitions /* sleep seconds */);

			if (status != kWarpStatusOK)
			{
				warpPrint("warpSetLowPowerMode(kWarpPowerModeVLLS0, 10)() failed...\n");
			}
			warpPrint("Should not get here...");
		}
	}
#endif

	while (1)
	{
		/*
		 *	Do not, e.g., lowPowerPinStates() on each iteration, because we actually
		 *	want to use menu to progressiveley change the machine state with various
		 *	commands.
		 */
		gWarpExtraQuietMode = false;
		printBootSplash(gWarpCurrentSupplyVoltage, menuRegisterAddress, &powerManagerCallbackStructure);

		warpPrint("\rSelect:\n");
		warpPrint("\r- 'a': set default sensor.\n");
		warpPrint("\r- 'b': set I2C baud rate.\n");
		warpPrint("\r- 'c': set SPI baud rate.\n");
		warpPrint("\r- 'd': set UART baud rate.\n");
		warpPrint("\r- 'e': set default register address.\n");
		warpPrint("\r- 'f': write byte to sensor.\n");
		warpPrint("\r- 'g': set default sensor supply voltage.\n");
		warpPrint("\r- 'h': powerdown command to all sensors.\n");
		warpPrint("\r- 'i': set pull-up enable value.\n");
		warpPrint("\r- 'j': repeat read reg 0x%02x on sensor #%d.\n", menuRegisterAddress, menuTargetSensor);
		warpPrint("\r- 'k': sleep until reset.\n");
		warpPrint("\r- 'l': send repeated byte on I2C.\n");
		warpPrint("\r- 'm': send repeated byte on SPI.\n");
		warpPrint("\r- 'n': enable sensor supply voltage.\n");
		warpPrint("\r- 'o': disable sensor supply voltage.\n");
		warpPrint("\r- 'p': switch to VLPR mode.\n");
		warpPrint("\r- 'r': switch to RUN mode.\n");
		warpPrint("\r- 's': power up all sensors.\n");
		warpPrint("\r- 't': dump processor state.\n");
		warpPrint("\r- 'u': set I2C address.\n");

#if (WARP_BUILD_ENABLE_DEVAT45DB)
		warpPrint("\r- 'R': read bytes from Flash.\n");
		warpPrint("\r- 'Z': reset Flash.\n");
#elif (WARP_BUILD_ENABLE_DEVIS25xP)
		warpPrint("\r- 'R': read bytes from Flash.\n");
		warpPrint("\r- 'F': Open Flash menu.\n");
		warpPrint("\r- 'Z': reset Flash.\n");
#endif

#if (WARP_BUILD_ENABLE_DEVICE40)
		warpPrint("\r- 'P': write bytes to FPGA configuration.\n");
#endif

#if (WARP_BUILD_ENABLE_DEVRV8803C7)
		warpPrint("\r- 'v': Enter VLLS0 low-power mode for 3s, then reset\n");
#endif

		warpPrint("\r- 'x': disable SWD and spin for 10 secs.\n");
		warpPrint("\r- 'z': perpetually dump all sensor data.\n");

		warpPrint("\rEnter selection> ");
		key = warpWaitKey();

		switch (key)
		{
			/*
			 *  Monitor activity from accelerometer.
			 */
			case 'w':
			{
				warpPrint("Forever measuring activity from the MMA8451Q.\n");
				measureActivityForeverMMA8451Q();
				break;
			}
			default:
				break;
		}
	}
	return 0;
}

void
writeAllSensorsToFlash(int menuDelayBetweenEachRun, int loopForever)
{
#if (WARP_BUILD_ENABLE_FLASH)
	uint32_t timeAtStart = OSA_TimeGetMsec();
	/*
	 *	A 32-bit counter gives us > 2 years of before it wraps, even if sampling
	 *at 60fps
	 */
	uint32_t readingCount		  = 0;
	uint32_t numberOfConfigErrors = 0;

	/*
	 *	The first 3 bit fields are reserved for the measurement number, and the 2 time stamps.
	 */
	uint16_t sensorBitField = 0;

#if (WARP_CSVSTREAM_FLASH_PRINT_METADATA)
	sensorBitField = sensorBitField | kWarpFlashReadingCountBitField;
	sensorBitField = sensorBitField | kWarpFlashRTCTSRBitField;
	sensorBitField = sensorBitField | kWarpFlashRTCTPRBitField;
#endif

	uint8_t	 flashWriteBuf[128] = {0};

	int rttKey = -1;
	WarpStatus status;

#if (WARP_BUILD_DEVADXL362)

	sensorBitField = sensorBitField | kWarpFlashADXL362BitField;
#endif

#if (WARP_BUILD_ENABLE_DEVAMG8834)
	numberOfConfigErrors += configureSensorAMG8834(0x3F, /* Initial reset */
												   0x01	 /* Frame rate 1 FPS */
	);

	sensorBitField = sensorBitField | kWarpFlashAMG8834BitField;
#endif

#if (WARP_BUILD_ENABLE_DEVMMA8451Q)
	numberOfConfigErrors += configureSensorMMA8451Q(
		0x00, /* Payload: Disable FIFO */
		0x01  /* Normal read 8bit, 800Hz, normal, active mode */
	);
	sensorBitField = sensorBitField | kWarpFlashMMA8451QBitField;
#endif

#if (WARP_BUILD_ENABLE_DEVMAG3110)
	numberOfConfigErrors += configureSensorMAG3110(
		0x00, /*	Payload: DR 000, OS 00, 80Hz, ADC 1280, Full 16bit, standby mode
							 to set up register*/
		0xA0, /*	Payload: AUTO_MRST_EN enable, RAW value without offset */
		0x10);

	sensorBitField = sensorBitField | kWarpFlashMAG3110BitField;
#endif

#if (WARP_BUILD_ENABLE_DEVL3GD20H)
	numberOfConfigErrors += configureSensorL3GD20H(
		0b11111111, /* ODR 800Hz, Cut-off 100Hz, see table 21, normal mode, x,y,z
										 enable */
		0b00100000,
		0b00000000 /* normal mode, disable FIFO, disable high pass filter */
	);

	sensorBitField = sensorBitField | kWarpFlashL3GD20HBitField;
#endif

#if (WARP_BUILD_ENABLE_DEVBME680)
	numberOfConfigErrors += configureSensorBME680(
		0b00000001, /*	payloadCtrl_Hum: Humidity oversampling (OSRS) to 1x
					 */
		0b00100100, /*	payloadCtrl_Meas: Temperature oversample 1x, pressure
										 overdsample 1x, mode 00	*/
		0b00001000	/*	payloadGas_0: Turn off heater
					 */
	);

	sensorBitField = sensorBitField | kWarpFlashBME680BitField;
#endif

#if (WARP_BUILD_ENABLE_DEVBMX055)
	numberOfConfigErrors += configureSensorBMX055accel(
		0b00000011, /* Payload:+-2g range */
		0b10000000	/* Payload:unfiltered data, shadowing enabled */
	);
	numberOfConfigErrors += configureSensorBMX055mag(
		0b00000001, /* Payload:from suspend mode to sleep mode*/
		0b00000001	/* Default 10Hz data rate, forced mode*/
	);
	numberOfConfigErrors += configureSensorBMX055gyro(
		0b00000100, /* +- 125degrees/s */
		0b00000000, /* ODR 2000 Hz, unfiltered */
		0b00000000, /* normal mode */
		0b10000000	/* unfiltered data, shadowing enabled */
	);

	sensorBitField = sensorBitField | kWarpFlashBMX055BitField;
#endif

#if (WARP_BUILD_ENABLE_DEVCCS811)
	uint8_t payloadCCS811[1];
	payloadCCS811[0] = 0b01000000; /* Constant power, measurement every 250ms */
	numberOfConfigErrors += configureSensorCCS811(payloadCCS811);

	sensorBitField = sensorBitField | kWarpFlashCCS811BitField;
#endif

#if (WARP_BUILD_ENABLE_DEVHDC1000)
	numberOfConfigErrors += writeSensorRegisterHDC1000(
		kWarpSensorConfigurationRegisterHDC1000Configuration, /* Configuration register	*/
		(0b1010000 << 8));

	sensorBitField = sensorBitField | kWarpFlashHDC1000BitField;
#endif

	/*
	 * Add RV8803C7 to sensorBitField
	*/
#if (WARP_BUILD_ENABLE_DEVRV8803C7)
	sensorBitField = sensorBitField | kWarpFlashRV8803C7BitField;
#endif

	// Add readingCount, 1 x timing, numberofConfigErrors
	uint8_t sensorBitFieldSize = 2;
	uint8_t bytesWrittenIndex  = 0;

	/*
	 * Write sensorBitField to flash first, outside of the loop.
	*/
	flashWriteBuf[bytesWrittenIndex] = (uint8_t)(sensorBitField >> 8);
	bytesWrittenIndex++;
	flashWriteBuf[bytesWrittenIndex] = (uint8_t)(sensorBitField);
	bytesWrittenIndex++;

	do
	{
		bytesWrittenIndex = sensorBitFieldSize;

#if (WARP_CSVSTREAM_FLASH_PRINT_METADATA)
		flashWriteBuf[bytesWrittenIndex] = (uint8_t)(readingCount >> 24);
		bytesWrittenIndex++;
		flashWriteBuf[bytesWrittenIndex] = (uint8_t)(readingCount >> 16);
		bytesWrittenIndex++;
		flashWriteBuf[bytesWrittenIndex] = (uint8_t)(readingCount >> 8);
		bytesWrittenIndex++;
		flashWriteBuf[bytesWrittenIndex] = (uint8_t)(readingCount);
		bytesWrittenIndex++;

		uint32_t currentRTC_TSR = RTC->TSR;
		uint32_t currentRTC_TPR = RTC->TPR;

		flashWriteBuf[bytesWrittenIndex] = (uint8_t)(currentRTC_TSR >> 24);
		bytesWrittenIndex++;
		flashWriteBuf[bytesWrittenIndex] = (uint8_t)(currentRTC_TSR >> 16);
		bytesWrittenIndex++;
		flashWriteBuf[bytesWrittenIndex] = (uint8_t)(currentRTC_TSR >> 8);
		bytesWrittenIndex++;
		flashWriteBuf[bytesWrittenIndex] = (uint8_t)(currentRTC_TSR);
		bytesWrittenIndex++;

		flashWriteBuf[bytesWrittenIndex] = (uint8_t)(currentRTC_TPR >> 24);
		bytesWrittenIndex++;
		flashWriteBuf[bytesWrittenIndex] = (uint8_t)(currentRTC_TPR >> 16);
		bytesWrittenIndex++;
		flashWriteBuf[bytesWrittenIndex] = (uint8_t)(currentRTC_TPR >> 8);
		bytesWrittenIndex++;
		flashWriteBuf[bytesWrittenIndex] = (uint8_t)(currentRTC_TPR);
		bytesWrittenIndex++;
#endif

#if (WARP_BUILD_ENABLE_DEVADXL362)
		bytesWrittenIndex += appendSensorDataADXL362(flashWriteBuf + bytesWrittenIndex);
#endif

#if (WARP_BUILD_ENABLE_DEVAMG8834)
		bytesWrittenIndex += appendSensorDataAMG8834(flashWriteBuf + bytesWrittenIndex);
#endif

#if (WARP_BUILD_ENABLE_DEVMMA8451Q)
		//bytesWrittenIndex += appendSensorDataMMA8451Q(flashWriteBuf + bytesWrittenIndex);
#endif

#if (WARP_BUILD_ENABLE_DEVMAG3110)
		bytesWrittenIndex += appendSensorDataMAG3110(flashWriteBuf + bytesWrittenIndex);
#endif

#if (WARP_BUILD_ENABLE_DEVL3GD20H)
		bytesWrittenIndex += appendSensorDataL3GD20H(flashWriteBuf + bytesWrittenIndex);
#endif

#if (WARP_BUILD_ENABLE_DEVBME680)
		bytesWrittenIndex += appendSensorDataBME680(flashWriteBuf + bytesWrittenIndex);
#endif

#if (WARP_BUILD_ENABLE_DEVBMX055)
		bytesWrittenIndex += appendSensorDataBMX055accel(flashWriteBuf + bytesWrittenIndex);
		bytesWrittenIndex += appendSensorDataBMX055mag(flashWriteBuf + bytesWrittenIndex);
		// bytesWrittenIndex += appendSensorDataBMX055gyro(flashWriteBuf + bytesWrittenIndex);

#endif

#if (WARP_BUILD_ENABLE_DEVCCS811)
		bytesWrittenIndex += appendSensorDataCCS811(flashWriteBuf + bytesWrittenIndex);
#endif

#if (WARP_BUILD_ENABLE_DEVHDC1000)
		bytesWrittenIndex += appendSensorDataHDC1000(flashWriteBuf + bytesWrittenIndex);
#endif

#if (WARP_BUILD_ENABLE_DEVRV8803C7)
		bytesWrittenIndex += appendSensorDataRV8803C7(flashWriteBuf + bytesWrittenIndex);
#endif

		/*
		*	Number of config errors.
		*	Uncomment to write to flash. Don't forget to update the initial bitfield at the start of this function.
		*/
		// flashWriteBuf[bytesWrittenIndex] = (uint8_t)(numberOfConfigErrors >> 24);
		// bytesWrittenIndex++;
		// flashWriteBuf[bytesWrittenIndex] = (uint8_t)(numberOfConfigErrors >> 16);
		// bytesWrittenIndex++;
		// flashWriteBuf[bytesWrittenIndex] = (uint8_t)(numberOfConfigErrors >> 8);
		// bytesWrittenIndex++;
		// flashWriteBuf[bytesWrittenIndex] = (uint8_t)(numberOfConfigErrors);
		// bytesWrittenIndex++;

		/*
		*	Dump to flash
		*/
		status = flashWriteFromEnd(bytesWrittenIndex, flashWriteBuf);
		if (status != kWarpStatusOK)
		{
			warpPrint("\r\n\tflashWriteFromEnd failed: %d", status);
			return;
		}

		if (menuDelayBetweenEachRun > 0)
		{
			// while (OSA_TimeGetMsec() - timeAtStart < menuDelayBetweenEachRun)
			// {
			// }

			// timeAtStart = OSA_TimeGetMsec();
			status = warpSetLowPowerMode(kWarpPowerModeVLPS, menuDelayBetweenEachRun);
			if (status != kWarpStatusOK)
			{
				warpPrint("Failed to put into sleep: %d", status);
			}
		}

		readingCount++;

		rttKey = SEGGER_RTT_GetKey();

		if (rttKey == 'q')
		{
			status = flashHandleEndOfWriteAllSensors();
			if (status != kWarpStatusOK)
			{
				warpPrint("\r\n\tflashHandleEndOfWriteAllSensors failed: %d", status);
			}
			break;
		}
	}
	while (loopForever);
#endif
}




int
char2int(int character)
{
	if (character >= '0' && character <= '9')
	{
		return character - '0';
	}

	if (character >= 'a' && character <= 'f')
	{
		return character - 'a' + 10;
	}

	if (character >= 'A' && character <= 'F')
	{
		return character - 'A' + 10;
	}

	return 0;
}



uint8_t
readHexByte(void)
{
	uint8_t		topNybble, bottomNybble;

	topNybble = warpWaitKey();
	bottomNybble = warpWaitKey();

	return (char2int(topNybble) << 4) + char2int(bottomNybble);
}



int
read4digits(void)
{
	uint8_t		digit1, digit2, digit3, digit4;

	digit1 = warpWaitKey();
	digit2 = warpWaitKey();
	digit3 = warpWaitKey();
	digit4 = warpWaitKey();

	return (digit1 - '0')*1000 + (digit2 - '0')*100 + (digit3 - '0')*10 + (digit4 - '0');
}



WarpStatus
writeByteToI2cDeviceRegister(uint8_t i2cAddress, bool sendCommandByte, uint8_t commandByte, bool sendPayloadByte, uint8_t payloadByte)
{
	i2c_status_t	status;
	uint8_t		commandBuffer[1];
	uint8_t		payloadBuffer[1];
	i2c_device_t	i2cSlaveConfig =
			{
				.address = i2cAddress,
				.baudRate_kbps = gWarpI2cBaudRateKbps
			};

	commandBuffer[0] = commandByte;
	payloadBuffer[0] = payloadByte;

	status = I2C_DRV_MasterSendDataBlocking(
						0	/* instance */,
						&i2cSlaveConfig,
						commandBuffer,
						(sendCommandByte ? 1 : 0),
						payloadBuffer,
						(sendPayloadByte ? 1 : 0),
		gWarpI2cTimeoutMilliseconds);

	return (status == kStatus_I2C_Success ? kWarpStatusOK : kWarpStatusDeviceCommunicationFailed);
}



WarpStatus
writeBytesToSpi(uint8_t *  payloadBytes, int payloadLength)
{
	uint8_t		inBuffer[payloadLength];
	spi_status_t	status;

	warpEnableSPIpins();
	status = SPI_DRV_MasterTransferBlocking(0					/* master instance */,
						NULL					/* spi_master_user_config_t */,
		payloadBytes,
						inBuffer,
						payloadLength				/* transfer size */,
						gWarpSpiTimeoutMicroseconds		/* timeout in microseconds (unlike I2C which is ms) */);
	warpDisableSPIpins();

	return (status == kStatus_SPI_Success ? kWarpStatusOK : kWarpStatusCommsError);
}

void
powerupAllSensors(void)
{
/*
 *	BMX055mag
 *
 *	Write '1' to power control bit of register 0x4B. See page 134.
 */
#if (WARP_BUILD_ENABLE_DEVBMX055)
		WarpStatus	status = writeByteToI2cDeviceRegister(	deviceBMX055magState.i2cAddress		/*	i2cAddress		*/,
							true					/*	sendCommandByte		*/,
							0x4B					/*	commandByte		*/,
							true					/*	sendPayloadByte		*/,
							(1 << 0)				/*	payloadByte		*/);
	if (status != kWarpStatusOK)
	{
			warpPrint("\r\tPowerup command failed, code=%d, for BMX055mag @ 0x%02x.\n", status, deviceBMX055magState.i2cAddress);
	}
#else
	warpPrint("\r\tPowerup command failed. BMX055 disabled \n");
#endif
}

// void
// activateAllLowPowerSensorModes(bool verbose)
// {
// 	WarpStatus	status;
// /*
//  *	ADXL362:	See Power Control Register (Address: 0x2D, Reset: 0x00).
//  *
//  *	POR values are OK.
//  */

// /*
//  *	IS25XP:	Put in powerdown momde
//  */
// #if (WARP_BUILD_ENABLE_DEVIS25xP)
// 	/*
// 	 *	Put the Flash in deep power-down
// 	 */
// 		//TODO: move 0xB9 into a named constant
// 		//spiTransactionIS25xP({0xB9 /* op0 */,  0x00 /* op1 */,  0x00 /* op2 */, 0x00 /* op3 */, 0x00 /* op4 */, 0x00 /* op5 */, 0x00 /* op6 */}, 1 /* opCount */);
// #endif

// /*
// 	 *	BMX055accel: At POR, device is in Normal mode. Move it to Deep Suspend mode.
//  *
// 	 *	Write '1' to deep suspend bit of register 0x11, and write '0' to suspend bit of register 0x11. See page 23.
//  */
// #if WARP_BUILD_ENABLE_DEVBMX055
// 		status = writeByteToI2cDeviceRegister(	deviceBMX055accelState.i2cAddress	/*	i2cAddress		*/,
// 							true					/*	sendCommandByte		*/,
// 							0x11					/*	commandByte		*/,
// 							true					/*	sendPayloadByte		*/,
// 							(1 << 5)				/*	payloadByte		*/);
// 	if ((status != kWarpStatusOK) && verbose)
// 	{
// 			warpPrint("\r\tPowerdown command failed, code=%d, for BMX055accel @ 0x%02x.\n", status, deviceBMX055accelState.i2cAddress);
// 	}
// #else
// 	warpPrint("\r\tPowerdown command abandoned. BMX055 disabled\n");
// #endif

// /*
// 	 *	BMX055gyro: At POR, device is in Normal mode. Move it to Deep Suspend mode.
//  *
//  *	Write '1' to deep suspend bit of register 0x11. See page 81.
//  */
// #if (WARP_BUILD_ENABLE_DEVBMX055)
// 		status = writeByteToI2cDeviceRegister(	deviceBMX055gyroState.i2cAddress	/*	i2cAddress		*/,
// 							true					/*	sendCommandByte		*/,
// 							0x11					/*	commandByte		*/,
// 							true					/*	sendPayloadByte		*/,
// 							(1 << 5)				/*	payloadByte		*/);
// 	if ((status != kWarpStatusOK) && verbose)
// 	{
// 			warpPrint("\r\tPowerdown command failed, code=%d, for BMX055gyro @ 0x%02x.\n", status, deviceBMX055gyroState.i2cAddress);
// 	}
// #else
// 	warpPrint("\r\tPowerdown command abandoned. BMX055 disabled\n");
// #endif



// /*
//  *	BMX055mag: At POR, device is in Suspend mode. See page 121.
//  *
//  *	POR state seems to be powered down.
//  */



// /*
//  *	MMA8451Q: See 0x2B: CTRL_REG2 System Control 2 Register (page 43).
//  *
//  *	POR state seems to be not too bad.
//  */



// /*
//  *	LPS25H: See Register CTRL_REG1, at address 0x20 (page 26).
//  *
//  *	POR state seems to be powered down.
//  */



// /*
//  *	MAG3110: See Register CTRL_REG1 at 0x10. (page 19).
//  *
//  *	POR state seems to be powered down.
//  */



// /*
//  *	HDC1000: currently can't turn it on (3V)
//  */



// /*
//  *	SI7021: Can't talk to it correctly yet.
//  */



// /*
//  *	L3GD20H: See CTRL1 at 0x20 (page 36).
//  *
//  *	POR state seems to be powered down.
//  */
// #if (WARP_BUILD_ENABLE_DEVL3GD20H)
// 		status = writeByteToI2cDeviceRegister(	deviceL3GD20HState.i2cAddress	/*	i2cAddress		*/,
// 							true				/*	sendCommandByte		*/,
// 							0x20				/*	commandByte		*/,
// 							true				/*	sendPayloadByte		*/,
// 							0x00				/*	payloadByte		*/);
// 		if ((status != kWarpStatusOK) && verbose)
// 	{
// 			warpPrint("\r\tPowerdown command failed, code=%d, for L3GD20H @ 0x%02x.\n", status, deviceL3GD20HState.i2cAddress);
// 	}
// #else
// 	warpPrint("\r\tPowerdown command abandoned. L3GD20H disabled\n");
// #endif



// /*
//  *	BME680: TODO
//  */



// /*
//  *	TCS34725: By default, is in the "start" state (see page 9).
//  *
//  *	Make it go to sleep state. See page 17, 18, and 19.
//  */
// #if (WARP_BUILD_ENABLE_DEVTCS34725)
// 		status = writeByteToI2cDeviceRegister(	deviceTCS34725State.i2cAddress	/*	i2cAddress		*/,
// 							true				/*	sendCommandByte		*/,
// 							0x00				/*	commandByte		*/,
// 							true				/*	sendPayloadByte		*/,
// 							0x00				/*	payloadByte		*/);
// 	if ((status != kWarpStatusOK) && verbose)
// 	{
// 			warpPrint("\r\tPowerdown command failed, code=%d, for TCS34725 @ 0x%02x.\n", status, deviceTCS34725State.i2cAddress);
// 	}
// #else
// 	warpPrint("\r\tPowerdown command abandoned. TCS34725 disabled\n");
// #endif

// /*
// 	 *	SI4705: Send a POWER_DOWN command (byte 0x17). See AN332 page 124 and page 132.
//  *
//  *	For now, simply hold its reset line low.
//  */
// #if (WARP_BUILD_ENABLE_DEVSI4705)
// 	GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
// #endif
// }

#if (WARP_BUILD_ENABLE_FLASH)
WarpStatus
flashWriteFromEnd(size_t nbyte, uint8_t* buf)
{
	#if (WARP_BUILD_ENABLE_DEVAT45DB)
		return writeToAT45DBFromEndBuffered(nbyte, buf);
	#elif (WARP_BUILD_ENABLE_DEVIS25xP)
		return writeToIS25xPFromEnd(nbyte, buf);
	#endif
}
#endif

#if (WARP_BUILD_ENABLE_FLASH)
WarpStatus
flashHandleEndOfWriteAllSensors()
{
#if (WARP_BUILD_ENABLE_DEVAT45DB)
	/*
	 *	Write the remainder of buffer to main memory
	 */
	writeBufferAndSavePagePositionAT45DB();
#elif (WARP_BUILD_ENABLE_DEVIS25xP)
	/*
	 *	Do nothing
	 */
	return kWarpStatusOK;
#endif
}
#endif

#if (WARP_BUILD_ENABLE_FLASH)
WarpStatus
flashReadMemory(uint16_t startPageNumber, uint8_t startPageOffset, size_t nbyte, void *buf)
{
	#if (WARP_BUILD_ENABLE_DEVAT45DB)
		return readMemoryAT45DB(startPageNumber, nbyte, buf);
	#elif (WARP_BUILD_ENABLE_DEVIS25xP)
		return readMemoryIS25xP(startPageNumber, startPageOffset, nbyte, buf);
	#endif
}
#endif

#if (WARP_BUILD_ENABLE_FLASH)
void
flashHandleReadByte(uint8_t readByte, uint8_t *  bytesIndex, uint8_t *  readingIndex, uint8_t *  sensorIndex, uint8_t *  measurementIndex, uint8_t *  currentSensorNumberOfReadings, uint8_t *  currentSensorSizePerReading, uint16_t *  sensorBitField, uint8_t *  currentNumberOfSensors, int32_t *  currentReading)
{
	if (*measurementIndex == 0)
	{
		// reading sensorBitField
		// warpPrint("\n%d ", readByte);
		*sensorBitField = readByte << 8;
		*measurementIndex = *measurementIndex + 1;

		return;
	}
	else if (*measurementIndex == 1)
	{
		// warpPrint("%d\n", readByte);
		*sensorBitField |= readByte;
		*measurementIndex = *measurementIndex + 1;

		*currentNumberOfSensors = flashGetNSensorsFromSensorBitField(*sensorBitField);

		*sensorIndex	= 0;
		*readingIndex	= 0;
		*bytesIndex		= 0;

		return;
	}

	if (*readingIndex == 0 && *bytesIndex == 0)
	{
		flashDecodeSensorBitField(*sensorBitField, *sensorIndex, currentSensorSizePerReading, currentSensorNumberOfReadings);
		// warpPrint("\r\n\tsensorBit: %d, number of Sensors: %d, sensor index: %d, size: %d, readings: %d", sensorBitField, currentNumberOfSensors, sensorIndex, currentSensorSizePerReading, currentSensorNumberOfReadings);
	}

	if (*readingIndex < *currentSensorNumberOfReadings)
	{
		if (*bytesIndex < *currentSensorSizePerReading)
		{
			*currentReading |= readByte << (8 * (*currentSensorSizePerReading - *bytesIndex - 1));
			*bytesIndex = *bytesIndex + 1;
			*measurementIndex = *measurementIndex + 1;

			if (*bytesIndex == *currentSensorSizePerReading)
			{
				if (*currentSensorSizePerReading == 4)
				{
					warpPrint("%d, ", (int32_t)(*currentReading));
				}
				else if (*currentSensorSizePerReading == 2)
				{
					warpPrint("%d, ", (int16_t)(*currentReading));
				}
				else if (*currentSensorSizePerReading == 1)
				{
					warpPrint("%d, ", (int8_t)(*currentReading));
				}

				*currentReading	= 0;
				*bytesIndex		= 0;

				*readingIndex = *readingIndex + 1;
				*measurementIndex = *measurementIndex + 1;

				if (*readingIndex == *currentSensorNumberOfReadings)
				{
					*readingIndex = 0;
					*sensorIndex = *sensorIndex + 1;

					if (*sensorIndex == *currentNumberOfSensors)
					{
						*measurementIndex = 0;
						warpPrint("\b\b \n");
					}
				}
			}
		}
	}
}
#endif

WarpStatus
flashReadAllMemory()
{
	WarpStatus status;

#if (WARP_BUILD_ENABLE_FLASH)
	int pageSizeBytes;
	uint16_t pageOffsetStoragePage;
	size_t pageOffsetStorageSize;
	int initialPageNumber;
	int initialPageOffset;

#if (WARP_BUILD_ENABLE_DEVAT45DB)
	pageSizeBytes				= kWarpSizeAT45DBPageSizeBytes;
	pageOffsetStoragePage		= kWarpAT45DBPageOffsetStoragePage;
	pageOffsetStorageSize		= kWarpAT45DBPageOffsetStorageSize;
	initialPageNumber			= kWarpInitialPageNumberAT45DB;
	initialPageOffset			= kWarpInitialPageOffsetAT45DB;
#elif (WARP_BUILD_ENABLE_DEVIS25xP)
	pageSizeBytes				= kWarpSizeIS25xPPageSizeBytes;
	pageOffsetStoragePage		= kWarpIS25xPPageOffsetStoragePage;
	pageOffsetStorageSize		= kWarpIS25xPPageOffsetStorageSize;
	initialPageNumber			= kWarpInitialPageNumberIS25xP;
	initialPageOffset			= kWarpInitialPageOffsetIS25xP;
#endif

	uint8_t dataBuffer[pageSizeBytes];

	uint8_t pagePositionBuf[3];

	status = flashReadMemory(pageOffsetStoragePage, 0, pageOffsetStorageSize, pagePositionBuf);
	if (status != kWarpStatusOK)
	{
		return status;
	}

	uint8_t pageOffset			= pagePositionBuf[2];
	uint16_t pageNumberTotal 	= pagePositionBuf[1] | pagePositionBuf[0] << 8;

	warpPrint("\r\n\tPage number: %d", pageNumberTotal);
	warpPrint("\r\n\tPage offset: %d\n", pageOffset);
	warpPrint("\r\n\tReading memory. Press 'q' to stop.\n\n");

	uint8_t bytesIndex			= 0;
	uint8_t readingIndex		= 0;
	uint8_t sensorIndex			= 0;
	uint8_t measurementIndex	= 0;

	uint8_t currentSensorNumberOfReadings	= 0;
	uint8_t currentSensorSizePerReading		= 0;

	uint16_t sensorBitField			= 0;
	uint8_t currentNumberOfSensors	= 0;

	int32_t currentReading = 0;

	int rttKey = -1;

	for (uint32_t pageNumber = initialPageNumber; pageNumber < pageNumberTotal;
			 pageNumber++)
	{
		rttKey = SEGGER_RTT_GetKey();
		if (rttKey == 'q')
		{
			return kWarpStatusOK;
		}

		status = flashReadMemory(pageNumber, 0, pageSizeBytes, dataBuffer);
		if (status != kWarpStatusOK)
		{
			return status;
		}

		for (size_t i = 0; i < kWarpSizeAT45DBPageSizeBytes; i++)
		{
			flashHandleReadByte(dataBuffer[i], &bytesIndex, &readingIndex, &sensorIndex, &measurementIndex, &currentSensorNumberOfReadings, &currentSensorSizePerReading, &sensorBitField, &currentNumberOfSensors, &currentReading);
		}
	}

	if (pageOffset <= 0)
	{
		return status;
	}

	status = flashReadMemory(pageNumberTotal, 0, pageOffset, dataBuffer);

	if (status != kWarpStatusOK)
	{
		return status;
	}

	for (size_t i = 0; i < pageOffset; i++)
	{
		flashHandleReadByte(dataBuffer[i], &bytesIndex, &readingIndex, &sensorIndex, &measurementIndex, &currentSensorNumberOfReadings, &currentSensorSizePerReading, &sensorBitField, &currentNumberOfSensors, &currentReading);
	}
#endif

	return status;
}

#if (WARP_BUILD_ENABLE_FLASH)
uint8_t
flashGetNSensorsFromSensorBitField(uint16_t sensorBitField)
{
	uint8_t numberOfSensors = 0;

	while (sensorBitField != 0)
	{
		sensorBitField = sensorBitField & (sensorBitField - 1);
		numberOfSensors++;
	}

	return numberOfSensors;
}
#endif

#if (WARP_BUILD_ENABLE_FLASH)
void
flashDecodeSensorBitField(uint16_t sensorBitField, uint8_t sensorIndex, uint8_t* sizePerReading, uint8_t* numberOfReadings)
{
	uint8_t numberOfSensorsFound = 0;

	/*
	 * readingCount
	*/
	if (sensorBitField & kWarpFlashReadingCountBitField)
	{
		numberOfSensorsFound++;
		if (numberOfSensorsFound - 1 == sensorIndex)
		{
			*sizePerReading		= 4;
			*numberOfReadings = 1;
			return;
		}
	}

	/*
	 * RTC->TSR
	*/
	if (sensorBitField & kWarpFlashRTCTSRBitField)
	{
		numberOfSensorsFound++;
		if (numberOfSensorsFound - 1 == sensorIndex)
		{
			*sizePerReading		= 4;
			*numberOfReadings = 1;
			return;
		}
	}

	/*
	 * RTC->TPR
	*/
	if (sensorBitField & kWarpFlashRTCTPRBitField)
	{
		numberOfSensorsFound++;
		if (numberOfSensorsFound - 1 == sensorIndex)
		{
			*sizePerReading		= 4;
			*numberOfReadings = 1;
			return;
		}
	}

	/*
	 * ADXL362
	*/
	if (sensorBitField & kWarpFlashADXL362BitField)
	{
		numberOfSensorsFound++;
		if (numberOfSensorsFound - 1 == sensorIndex)
		{
			*sizePerReading		= bytesPerReadingADXL362;
			*numberOfReadings = numberOfReadingsPerMeasurementADXL362;
			return;
		}
	}

	/*
	 * AMG8834
	*/
	if (sensorBitField & kWarpFlashAMG8834BitField)
	{
		numberOfSensorsFound++;
		if (numberOfSensorsFound - 1 == sensorIndex)
		{
			*sizePerReading		= bytesPerReadingAMG8834;
			*numberOfReadings = numberOfReadingsPerMeasurementAMG8834;
			return;
		}
	}

	/*
	 * MMA8451Q
	*/
	if (sensorBitField & kWarpFlashMMA8541QBitField)
	{
		numberOfSensorsFound++;
		if (numberOfSensorsFound - 1 == sensorIndex)
		{
			*sizePerReading		= bytesPerReadingMMA8451Q;
			*numberOfReadings = numberOfReadingsPerMeasurementMMA8451Q;
			return;
		}
	}

	/*
	 * MAG3110
	*/
	if (sensorBitField & kWarpFlashMAG3110BitField)
	{
		numberOfSensorsFound++;
		if (numberOfSensorsFound - 1 == sensorIndex)
		{
			*sizePerReading		= bytesPerReadingMAG3110;
			*numberOfReadings = numberOfReadingsPerMeasurementMAG3110;
			return;
		}
	}

	/*
	 * L3GD0H
	*/
	if (sensorBitField & kWarpFlashL3GD20HBitField)
	{
		numberOfSensorsFound++;
		if (numberOfSensorsFound - 1 == sensorIndex)
		{
			*sizePerReading		= bytesPerReadingL3GD20H;
			*numberOfReadings = numberOfReadingsPerMeasurementL3GD20H;
			return;
		}
	}

	/*
	 * BME680
	*/
	if (sensorBitField & kWarpFlashBME680BitField)
	{
		numberOfSensorsFound++;
		if (numberOfSensorsFound - 1 == sensorIndex)
		{
			*sizePerReading		= bytesPerReadingBME680;
			*numberOfReadings = numberOfReadingsPerMeasurementBME680;
			return;
		}
	}

	/*
	 * BMX055
	*/
	if (sensorBitField & kWarpFlashBMX055BitField)
	{
		numberOfSensorsFound++;
		if (numberOfSensorsFound - 1 == sensorIndex)
		{
			*sizePerReading		= bytesPerReadingBMX055;
			*numberOfReadings = numberOfReadingsPerMeasurementBMX055;
			return;
		}
	}

	/*
	 * CCS811
	*/
	if (sensorBitField & kWarpFlashCCS811BitField)
	{
		numberOfSensorsFound++;
		if (numberOfSensorsFound - 1 == sensorIndex)
		{
			*sizePerReading		= bytesPerReadingCCS811;
			*numberOfReadings = numberOfReadingsPerMeasurementCCS811;
			return;
		}
	}

	/*
	 * HDC1000
	*/
	if (sensorBitField & kWarpFlashHDC1000BitField)
	{
		numberOfSensorsFound++;
		if (numberOfSensorsFound - 1 == sensorIndex)
		{
			*sizePerReading		= bytesPerReadingHDC1000;
			*numberOfReadings = numberOfReadingsPerMeasurementHDC1000;
			return;
		}
	}

	/*
	 * RV8803C7
	*/
	if (sensorBitField & kWarpFlashRV8803C7BitField)
	{
		numberOfSensorsFound++;
		if (numberOfSensorsFound - 1 == sensorIndex)
		{
			*sizePerReading		= bytesPerReadingRV8803C7;
			*numberOfReadings 	= numberOfReadingsPerMeasurementRV8803C7;
			return;
		}
	}

	/*
	 * Number of config errors
	*/
	if (sensorBitField & kWarpFlashNumConfigErrors)
	{
		numberOfSensorsFound++;
		if (numberOfSensorsFound - 1 == sensorIndex)
		{
			*sizePerReading		= 4;
			*numberOfReadings = 1;
			return;
		}
	}
}
#endif
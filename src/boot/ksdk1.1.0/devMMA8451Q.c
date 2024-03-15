/*
	Extended 2024 with `measureActivityForeverMMA8451Q`. Omar Tanner.

	The function for computing CDFs, `phi`, is modified from https://www.johndcook.com/blog/cpp_phi/. John D. Cook.
	The implementation of the Welford algorithm is based on the description given in https://changyaochen.github.io/welford. Changyao Chen.

	Originally authored 2016-2018. Phillip Stanley-Marbell. Additional contributors,
	2018-onwards, see git log.

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
#include <stdlib.h>

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

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

#include "math.h"


extern volatile WarpI2CDeviceState	deviceMMA8451QState;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;



void
initMMA8451Q(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
	deviceMMA8451QState.i2cAddress			= i2cAddress;
	deviceMMA8451QState.operatingVoltageMillivolts	= operatingVoltageMillivolts;

	return;
}

WarpStatus
writeSensorRegisterMMA8451Q(uint8_t deviceRegister, uint8_t payload)
{
	uint8_t		payloadByte[1], commandByte[1];
	i2c_status_t	status;

	switch (deviceRegister)
	{
		case 0x09: case 0x0a: case 0x0e: case 0x0f:
		case 0x11: case 0x12: case 0x13: case 0x14:
		case 0x15: case 0x17: case 0x18: case 0x1d:
		case 0x1f: case 0x20: case 0x21: case 0x23:
		case 0x24: case 0x25: case 0x26: case 0x27:
		case 0x28: case 0x29: case 0x2a: case 0x2b:
		case 0x2c: case 0x2d: case 0x2e: case 0x2f:
		case 0x30: case 0x31:
		{
			/* OK */
			break;
		}

		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
		{
		.address = deviceMMA8451QState.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);
	commandByte[0] = deviceRegister;
	payloadByte[0] = payload;
	warpEnableI2Cpins();

	status = I2C_DRV_MasterSendDataBlocking(
		0 /* I2C instance */,
		&slave,
		commandByte,
		1,
		payloadByte,
		1,
		gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
configureSensorMMA8451Q(uint8_t payloadF_SETUP, uint8_t payloadCTRL_REG1)
{
	WarpStatus	i2cWriteStatus1, i2cWriteStatus2;

	warpPrint("Configuring MMA8451Q: payloadF_SETUP=%d,payloadCTRL_REG1=%d", payloadF_SETUP, payloadCTRL_REG1);


	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);

	i2cWriteStatus1 = writeSensorRegisterMMA8451Q(kWarpSensorConfigurationRegisterMMA8451QF_SETUP /* register address F_SETUP */,
												  payloadF_SETUP /* payload: Disable FIFO */
	);

	i2cWriteStatus2 = writeSensorRegisterMMA8451Q(kWarpSensorConfigurationRegisterMMA8451QCTRL_REG1 /* register address CTRL_REG1 */,
												  payloadCTRL_REG1 /* payload */
	);

	return (i2cWriteStatus1 | i2cWriteStatus2);
}

WarpStatus
readSensorRegisterMMA8451Q(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;


	USED(numberOfBytes);
	switch (deviceRegister)
	{
		case 0x00: case 0x01: case 0x02: case 0x03:
		case 0x04: case 0x05: case 0x06: case 0x09:
		case 0x0a: case 0x0b: case 0x0c: case 0x0d:
		case 0x0e: case 0x0f: case 0x10: case 0x11:
		case 0x12: case 0x13: case 0x14: case 0x15:
		case 0x16: case 0x17: case 0x18: case 0x1d:
		case 0x1e: case 0x1f: case 0x20: case 0x21:
		case 0x22: case 0x23: case 0x24: case 0x25:
		case 0x26: case 0x27: case 0x28: case 0x29:
		case 0x2a: case 0x2b: case 0x2c: case 0x2d:
		case 0x2e: case 0x2f: case 0x30: case 0x31:
		{
			/* OK */
			break;
		}

		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
		{
		.address = deviceMMA8451QState.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);
	cmdBuf[0] = deviceRegister;
	warpEnableI2Cpins();

	status = I2C_DRV_MasterReceiveDataBlocking(
		0 /* I2C peripheral instance */,
		&slave,
		cmdBuf,
		1,
		(uint8_t *)deviceMMA8451QState.i2cBuffer,
		numberOfBytes,
		gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

/* Compute the CDF of a standard normal (Gaussian) random variable.
   Modified by Omar Tanner, originally version by John D. Cook from https://www.johndcook.com/blog/cpp_phi/.
 */

float phi(float x)
{
    // Constants.
    float a1 =  0.254829592;
    float a2 = -0.284496736;
    float a3 =  1.421413741;
    float a4 = -1.453152027;
    float a5 =  1.061405429;
    float p  =  0.3275911;

    // Save the sign of x.
    int sign = 1;
    if (x < 0)
        sign = -1;
    x = fabs(x)/sqrtf(2.0);

    // A&S formula 7.1.26.
    float t = 1.0/(1.0 + p*x);
    float y = 1.0 - (((((a5*t + a4)*t) + a3)*t + a2)*t + a1)*t*expf(-x*x);

    return 0.5*(1.0 + sign*y);
}

// Probability that a < v < b given mu(v) and sigma^2(v).
float PDF(float mean, float var, float a, float b) {
	return phi((b - mean) / sqrtf(var)) - phi((a - mean) / sqrtf(var));
}

void floatPrint(float to_print) {
#define PRECISION 10000
#define sign(x) (((x)<0)?-1:1)
    int intPart = (int)to_print;
    int decimalPart = (to_print - intPart) * PRECISION;

    warpPrint("%d.%04d", intPart, decimalPart*sign(decimalPart));
}

/* Continuously measure the stride time and activity being performed.
   Author: Omar Tanner & Phillip Stanley-Marbell.
 */
void
measureActivityForeverMMA8451Q()
{
	/* Algorithm variables. */
	uint32_t prevResultTimestamp = OSA_TimeGetMsec();

	uint16_t numMeasurements = 0;

	int16_t xAccMax = -32768;
	int16_t xAccMin = 32767;
	int16_t yAccMax = -32768;
	int16_t yAccMin = 32767;
	int16_t zAccMax = -32768;
	int16_t zAccMin = 32767;

	// For the current axis.
	float baseline = 0;
	uint8_t baselineAxis = 0; // 0 == x, 1 == y, 2 == z
	int16_t prevAccs[3] = {0, 0, 0}; // Unfiltered.
	float prevFilteredAcc = 0;
	uint32_t prevAccTimestamp = prevResultTimestamp;
	float prevStepTimestampMean = prevResultTimestamp;
	float prevStepTimestampDev = 0;

	// Time between steps running average.
	float timeBetweenStepsMean = 0;
	float timeBetweenStepsVar = 0; // Variance of the time between steps, sigma^2(\Delta t). These are random errors, e.g. from the person moving. These give a measure of consistency in the person's pace.
	float timeBetweenStepsMeanVar = 0; // Variance of the mean time between steps over the 10s window, sigma^2(\bar{\Delta t}). Measurement errors propogated via quadrature. These give a maeasure of the person's average pace.
	uint16_t timeBetweenStepsCount = 0;

	/* End of algorithm variables. */

	/* Measure the activity forever. */
	while(true) {

		/* Read the accelerations. */

		uint16_t	readSensorRegisterValueLSB;
		uint16_t	readSensorRegisterValueMSB;
		int16_t xAcc;
		int16_t yAcc;
		int16_t zAcc;
		WarpStatus	i2cReadStatus;

		warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);

		/*
		*	From the MMA8451Q datasheet:
		*
		*		"A random read access to the LSB registers is not possible.
		*		Reading the MSB register and then the LSB register in sequence
		*		ensures that both bytes (LSB and MSB) belong to the same data
		*		sample, even if a new data sample arrives between reading the
		*		MSB and the LSB byte."
		*
		*	We therefore do 2-byte read transactions, for each of the registers.
		*	We could also improve things by doing a 6-byte read transaction.
		*/
		i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 2 /* numberOfBytes */);
		readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
		readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
		xAcc = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

		/*
		*	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
		*/
		xAcc = (xAcc ^ (1 << 13)) - (1 << 13);

		i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Y_MSB, 2 /* numberOfBytes */);
		readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
		readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
		yAcc = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

		/*
		*	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
		*/
		yAcc = (yAcc ^ (1 << 13)) - (1 << 13);

		i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Z_MSB, 2 /* numberOfBytes */);
		readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
		readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
		zAcc = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

		/*
		*	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
		*/
		zAcc = (zAcc ^ (1 << 13)) - (1 << 13);

		/* ======== */
		/* Accelerations have been read into xAcc, yAcc and zAcc. Update algorithm. */

		numMeasurements++;

		uint32_t timestamp = OSA_TimeGetMsec();
		// check if the clock looped. if so, ignore this measurement as we don't know the time it happened
		if (timestamp <= prevAccTimestamp) {
			prevStepTimestampMean = timestamp;
			prevAccTimestamp = timestamp;
			prevResultTimestamp = timestamp;
			continue;
		}

		xAccMax = max(xAcc, xAccMax);
		yAccMax = max(yAcc, yAccMax);
		zAccMax = max(zAcc, zAccMax);
		xAccMin = min(xAcc, xAccMin);
		yAccMin = min(yAcc, yAccMin);
		zAccMin = min(zAcc, zAccMin);

		// warpPrint("xAcc=%d", xAcc);
		// warpPrint(",yAcc=%d", yAcc);
		// warpPrint(",zAcc=%d\n", zAcc);

		int16_t curAcc = baselineAxis == 0 ? xAcc : (baselineAxis == 1 ? yAcc : zAcc);
		float curFilteredAcc = (prevAccs[0] + prevAccs[1] + prevAccs[2] + curAcc) / 4.0f;

		prevAccs[0] = prevAccs[1];
		prevAccs[1] = prevAccs[2];
		prevAccs[2] = curAcc;

		// warpPrint("prevFilteredAcc=");
		// floatPrint(prevFilteredAcc);
		// warpPrint(",curFilteredAcc=");
		// floatPrint(curFilteredAcc);
		// warpPrint(",t=%u,prevTimestampMean=", timestamp);
		// floatPrint(prevStepTimestampMean);
		// warpPrint("\n");

		if (prevFilteredAcc < baseline && curFilteredAcc >= baseline && (timestamp - prevStepTimestampMean > 100)) {
			// We have a step.
			float errAcc = (2.68f/100.0f) * curFilteredAcc;
			float timestampStepLower = prevAccTimestamp + ((baseline - (prevFilteredAcc - errAcc)) * (timestamp - prevAccTimestamp) / ((curFilteredAcc + errAcc) - (prevFilteredAcc - errAcc)));
			float timestampStepUpper;
			if ((prevFilteredAcc + errAcc >= baseline) || (curFilteredAcc - errAcc <= baseline)) {
				timestampStepUpper = timestamp;
			}
			else {
				timestampStepUpper = prevAccTimestamp + ((baseline - (prevFilteredAcc + errAcc)) * (timestamp - prevAccTimestamp) / ((curFilteredAcc - errAcc) - (prevFilteredAcc + errAcc)));
			}
			float timestampStepMean = (timestampStepUpper + timestampStepLower) / 2.0f;
      		float timestampStepDev = (timestampStepUpper - timestampStepLower) / 2.0f;

			float timeDiffStepMean = timestampStepMean - prevStepTimestampMean;
			float timeDiffStepVar = timestampStepDev * timestampStepDev + prevStepTimestampDev * prevStepTimestampDev;

			// Update running average of stride time using the Welford method: https://changyaochen.github.io/welford and Quadrature.
			if (timeBetweenStepsCount == 0) {
				timeBetweenStepsMean = timeDiffStepMean;
				timeBetweenStepsVar = 0;
				timeBetweenStepsMeanVar = timeDiffStepVar;
			}
			else {
				float newMean = timeBetweenStepsMean + (timeDiffStepMean - timeBetweenStepsMean) / (timeBetweenStepsCount + 1);
				timeBetweenStepsVar = timeBetweenStepsVar + ((timeBetweenStepsCount * (timeBetweenStepsMean - timeDiffStepMean) * (timeBetweenStepsMean - timeDiffStepMean) - (timeBetweenStepsCount + 1) * timeBetweenStepsVar) / ((timeBetweenStepsCount + 1) * (timeBetweenStepsCount + 1)));
				timeBetweenStepsMean = newMean;

				// Quadrature.
				timeBetweenStepsMeanVar = ((timeBetweenStepsCount * timeBetweenStepsCount) * timeBetweenStepsMeanVar + timeDiffStepVar) / ((timeBetweenStepsCount + 1) * (timeBetweenStepsCount + 1));
			}
			timeBetweenStepsCount++;

			prevStepTimestampMean = timestampStepMean;
			prevStepTimestampDev = timestampStepDev;
		}

		// Give results every 10ms.
		if (timestamp - prevResultTimestamp >= 10000) {
			prevResultTimestamp = timestamp;

			warpPrint("t=%u: ", timestamp);
			warpPrint("time_between_steps=");
			floatPrint(timeBetweenStepsMean);
			warpPrint(",time_between_steps_var=");
			floatPrint(timeBetweenStepsVar);
			warpPrint(",time_between_steps_mean_var=");
			floatPrint(timeBetweenStepsMeanVar);
			warpPrint("\n");
			
			// Use CDF to compute event probabilities.
			// Try 3 different methods with variance error 1x and 2x respectively.
			// 1. with no mean variance error, 2. with 2x intervals, and 3. with 4x intervals.
			for (uint8_t var_mult = 1; var_mult <= 2; var_mult++) {
				float pWalkNoMeanVar = PDF(timeBetweenStepsMean, timeBetweenStepsVar, 545 - var_mult * (545 - 428), 545 + var_mult * (545 - 428));
				float pJogNoMeanVar = PDF(timeBetweenStepsMean, timeBetweenStepsVar, 400 - var_mult * (400 - 372), 400 + var_mult * (400 - 372));
				float pRunNoMeanVar = PDF(timeBetweenStepsMean, timeBetweenStepsVar, 343 - 2 * (343 - 314), 343 + 2 * (343 - 314));
				for (uint8_t method = 0; method < 3; method++) {
					float pWalk;
					float pJog;
					float pRun;
					switch (method) {
						case 0:
							pWalk = pWalkNoMeanVar;
							pJog = pJogNoMeanVar;
							pRun = pRunNoMeanVar;
							break;
						case 1:
							pWalk = pWalkNoMeanVar * PDF(timeBetweenStepsMean, timeBetweenStepsVar, 545 - 2 * (545 - 428), 545 + 2 * (545 - 428));
							pJog = pJogNoMeanVar * PDF(timeBetweenStepsMean, timeBetweenStepsVar, 400 - 2 * (400 - 372), 400 + 2 * (400 - 372));
							pRun = pRunNoMeanVar * PDF(timeBetweenStepsMean, timeBetweenStepsVar, 343 - 2 * (343 - 314), 343 + 2 * (343 - 314));
							break;
						default: // 2
							pWalk = pWalkNoMeanVar * PDF(timeBetweenStepsMean, timeBetweenStepsMeanVar, 545 - 4 * (545 - 428), 545 + 4 * (545 - 428));
							pJog = pJogNoMeanVar * PDF(timeBetweenStepsMean, timeBetweenStepsMeanVar, 400 - 4 * (400 - 372), 400 + 4 * (400 - 372));
							pRun = pRunNoMeanVar * PDF(timeBetweenStepsMean, timeBetweenStepsMeanVar, 343 - 4 * (343 - 314), 343 + 4 * (343 - 314));
					}
					float pNone = 1 - pWalk - pJog - pRun;
					warpPrint("method=%u: pWalk=", (var_mult * method));
					floatPrint(pWalk);
					warpPrint(",pJog=");
					floatPrint(pJog);
					warpPrint(",pRun=");
					floatPrint(pRun);
					warpPrint(",pNone=");
					floatPrint(pNone);
					warpPrint("\n");
				}
			}

			// Reset variables for the next window.
			timeBetweenStepsCount = 0;
			timeBetweenStepsMean = 0;
			timeBetweenStepsVar = 0;
			timeBetweenStepsMeanVar = 0;

			// Update max activity axis and baseline.
			baselineAxis = (xAccMax - xAccMin > yAccMax - yAccMin ? (xAccMax - xAccMin > zAccMax - zAccMin ? 0 : 2) : (yAccMax - yAccMin > zAccMax - zAccMin ? 1 : 2));
			if (baselineAxis == 0) {
				baseline = xAccMin + (xAccMax - xAccMin) / 2.0f;
			}
			else if (baselineAxis == 1) {
				baseline = yAccMin + (yAccMax - yAccMin) / 2.0f;
			}
			else {
				baseline = zAccMin + (zAccMax - zAccMin) / 2.0f;
			}

			warpPrint("baselineAxis=%u, baseline=", baselineAxis);
			floatPrint(baseline);
			warpPrint("\n");
			warpPrint("numMeasurements=%u,numSteps=%u\n", numMeasurements, timeBetweenStepsCount);

			// Reset baseline estimators.
			xAccMax = -32768;
			xAccMin = 32767;
			yAccMax = -32768;
			yAccMin = 32767;
			zAccMax = -32768;
			zAccMin = 32767;

			numMeasurements = 0;
		}

		prevAccTimestamp = timestamp;
		prevFilteredAcc = curFilteredAcc;
	}
}

// uint8_t
// appendSensorDataMMA8451Q(uint8_t* buf)
// {
// 	uint8_t index = 0;
// 	uint16_t readSensorRegisterValueLSB;
// 	uint16_t readSensorRegisterValueMSB;
// 	int16_t readSensorRegisterValueCombined;
// 	WarpStatus i2cReadStatus;

// 	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);

// 	/*
// 	 *	From the MMA8451Q datasheet:
// 	 *
// 	 *		"A random read access to the LSB registers is not possible.
// 	 *		Reading the MSB register and then the LSB register in sequence
// 	 *		ensures that both bytes (LSB and MSB) belong to the same data
// 	 *		sample, even if a new data sample arrives between reading the
// 	 *		MSB and the LSB byte."
// 	 *
// 	 *	We therefore do 2-byte read transactions, for each of the registers.
// 	 *	We could also improve things by doing a 6-byte read transaction.
// 	 */
// 	i2cReadStatus                   = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 2 /* numberOfBytes */);
// 	readSensorRegisterValueMSB      = deviceMMA8451QState.i2cBuffer[0];
// 	readSensorRegisterValueLSB      = deviceMMA8451QState.i2cBuffer[1];
// 	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

// 	/*
// 	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
// 	 */
// 	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

// 	if (i2cReadStatus != kWarpStatusOK)
// 	{
// 		buf[index] = 0;
// 		index += 1;

// 		buf[index] = 0;
// 		index += 1;
// 	}
// 	else
// 	{
// 		/*
// 		 * MSB first
// 		 */
// 		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
// 		index += 1;

// 		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
// 		index += 1;
// 	}

// 	i2cReadStatus                   = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Y_MSB, 2 /* numberOfBytes */);
// 	readSensorRegisterValueMSB      = deviceMMA8451QState.i2cBuffer[0];
// 	readSensorRegisterValueLSB      = deviceMMA8451QState.i2cBuffer[1];
// 	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

// 	/*
// 	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
// 	 */
// 	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

// 	if (i2cReadStatus != kWarpStatusOK)
// 	{
// 		buf[index] = 0;
// 		index += 1;

// 		buf[index] = 0;
// 		index += 1;
// 	}
// 	else
// 	{
// 		/*
// 		 * MSB first
// 		 */
// 		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
// 		index += 1;

// 		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
// 		index += 1;
// 	}

// 	i2cReadStatus                   = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Z_MSB, 2 /* numberOfBytes */);
// 	readSensorRegisterValueMSB      = deviceMMA8451QState.i2cBuffer[0];
// 	readSensorRegisterValueLSB      = deviceMMA8451QState.i2cBuffer[1];
// 	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

// 	/*
// 	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
// 	 */
// 	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

// 	if (i2cReadStatus != kWarpStatusOK)
// 	{
// 		buf[index] = 0;
// 		index += 1;

// 		buf[index] = 0;
// 		index += 1;
// 	}
// 	else
// 	{
// 		/*
// 		 * MSB first
// 		 */
// 		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
// 		index += 1;

// 		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
// 		index += 1;
// 	}
// 	return index;
// }
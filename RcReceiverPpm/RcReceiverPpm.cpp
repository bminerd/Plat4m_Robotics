//------------------------------------------------------------------------------
//       _______    __                           ___
//      ||  ___ \  || |             __          //  |
//      || |  || | || |   _______  || |__      //   |    _____  ___
//      || |__|| | || |  // ___  | ||  __|    // _  |   ||  _ \/ _ \
//      ||  ____/  || | || |  || | || |      // /|| |   || |\\  /\\ \
//      || |       || | || |__|| | || |     // /_|| |_  || | || | || |
//      || |       || |  \\____  | || |__  //_____   _| || | || | || |
//      ||_|       ||_|       ||_|  \\___|       ||_|   ||_| ||_| ||_|
//
//
// The MIT License (MIT)
//
// Copyright (c) 2016 Benjamin Minerd
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//------------------------------------------------------------------------------

///
/// @file FlightController.cpp
/// @author Ben Minerd
/// @date 12/21/2016
/// @brief FlightController class source file.
///

//------------------------------------------------------------------------------
// Include files
//------------------------------------------------------------------------------

#include <FlightController.h>

#include <math.h>

using Plat4m::FlightController;

//------------------------------------------------------------------------------
// Protected constructors
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
FlightController::FlightController(
							 InertialNavigationSystem& inertialNavigationSystem,
							 Controller& yawController,
							 Controller& pitchController,
							 Controller& rollController,
							 MotorSpeedController& motor1MotorSpeedController,
							 MotorSpeedController& motor2MotorSpeedController,
							 MotorSpeedController& motor3MotorSpeedController,
							 MotorSpeedController& motor4MotorSpeedController,
							 const uint32_t controlUpdateRateHz) :
	Module(),
	myInertialNavigationSystem(inertialNavigationSystem),
	myYawController(yawController),
	myPitchController(pitchController),
	myRollController(rollController),
	myMotor1MotorSpeedController(motor1MotorSpeedController),
	myMotor2MotorSpeedController(motor2MotorSpeedController),
	myMotor3MotorSpeedController(motor3MotorSpeedController),
	myMotor4MotorSpeedController(motor4MotorSpeedController),
	myControlUpdateThread(
		   createCallback(this, &FlightController::controlUpdateThreadCallback),
		   (uint32_t) round(1000.0 / controlUpdateRateHz))
{
}

//------------------------------------------------------------------------------
// Protected virtual destructors
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
FlightController::~FlightController()
{
}

//------------------------------------------------------------------------------
// Public methods
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
FlightController::Error FlightController::setFlightMode(
													const FlightMode flightMode)
{
	// Error checking

	switch (flightMode)
	{
		case FLIGHT_MODE_HORIZON:
		{
			// Disable rate controllers
			myPitchRateController.setEnabled(false);
			myRollRateController.setEnabled(false);

			// Enable angle controllers
			myPitchController.setEnabled(true);
			myRollController.setEnabled(true);

			break;
		}
		case FLIGHT_MODE_RATES:
		{
			// Enable angle controllers
			myPitchController.setEnabled(false);
			myRollController.setEnabled(false);

			// Disable rate controllers
			myPitchRateController.setEnabled(true);
			myRollRateController.setEnabled(true);

			break;
		}
		default:
		{
			break;
		}
	}

	myFlightMode = flightMode;

	return Error(ERROR_CODE_NONE);
}

//------------------------------------------------------------------------------
// Private inline methods
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
void FlightController::horizonModeInputUpdate()
{
	// Apply inputs to controllers
	myYawRateController.setSetPoint(myYawInput);
	myPitchController.setSetPoint(myPitchInput);
	myRollController.setSetPoint(myRollInput);
}

//------------------------------------------------------------------------------
void FlightController::ratesModeInputUpdate()
{
	// Apply inputs to controllers
	myYawRateController.setSetPoint(myYawInput);
	myPitchRateController.setSetPoint(myPitchInput);
	myRollRateController.setSetPoint(myRollInput);
}

//------------------------------------------------------------------------------
void FlightController::horizonModeControlUpdate()
{
	float pitchDegrees = StateEstimator::getPitchDegrees(myStateEstimate);
	float rollDegrees  = StateEstimator::getRollDegrees(myStateEstimate);

	myPitchController.update(pitchDegrees);
	myRollController.update(rollDegrees);
}

//------------------------------------------------------------------------------
void FlightController::ratesModeControlUpdate()
{
	yawRateControlUpdate();
	pitchRateControlUpdate();
	rollRateControlUpdate();
}

//------------------------------------------------------------------------------
void FlightController::yawRateControlUpdate()
{
	float yawRateOutput = myYawRateController.update(yawRateDegreesPs);

	myMotor1MotorSpeedController.shiftSpeed(pitchRateOutput);
	myMotor3MotorSpeedController.shiftSpeed(pitchRateOutput);
	myMotor2MotorSpeedController.shiftSpeed(-pitchRateOutput);
	myMotor4MotorSpeedController.shiftSpeed(-pitchRateOutput);
}

//------------------------------------------------------------------------------
void FlightController::pitchRateControlUpdate()
{
	float pitchRateOutput = myPitchRateController.update(pitchRateDegreesPs);

	myMotor1MotorSpeedController.shiftSpeed(pitchRateOutput);
	myMotor2MotorSpeedController.shiftSpeed(pitchRateOutput);
	myMotor3MotorSpeedController.shiftSpeed(-pitchRateOutput);
	myMotor4MotorSpeedController.shiftSpeed(-pitchRateOutput);
}

//------------------------------------------------------------------------------
void FlightController::rollRateControlUpdate()
{
	float rollRateOutput = myRollRateController.update(rollRateDegreesPs);

	myMotor1MotorSpeedController.shiftSpeed(rollRateOutput);
	myMotor4MotorSpeedController.shiftSpeed(rollRateOutput);
	myMotor2MotorSpeedController.shiftSpeed(-pitchRateOutput);
	myMotor3MotorSpeedController.shiftSpeed(-pitchRateOutput);
}

//------------------------------------------------------------------------------
// Private methods
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
void FlightController::inputUpdateThreadCallback()
{
	switch (myFlightMode)
	{
		case FLIGHT_MODE_HORIZON:
		{
			horizonModeInputUpdate();

			break;
		}
		case FLIGHT_MODE_RATES:
		{
			ratesModeInputUpdate();

			break;
		}
		default:
		{
			break;
		}
	}
}

//------------------------------------------------------------------------------
void FlightController::controlUpdateThreadCallback()
{
	myStateEstimate = myStateEstimator.getEstimate();

	float yawDegrees = StateEstimator::getYawDegrees(myStateEstimate);

	// Update controllers with state estimate
	myYawRateController.update(yawDegrees);

	// Update motor commands
	switch (myFlightMode)
	{
		case FLIGHT_MODE_HORIZON:
		{
			horizonModeControlUpdate();

			break;
		}
		case FLIGHT_MODE_RATES:
		{
			ratesModeControlUpdate();

			break;
		}
		default:
		{
			break;
		}
	}
}

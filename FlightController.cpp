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

using Plat4m::Robotics::FlightController;

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
		case FLIGHT_MODE_ANGLE:
		{
			// Disable rate controllers
			myPitchRateController.setEnabled(false);
			myRollRateController.setEnabled(false);

			// Enable angle controllers
			myPitchAngleController.setEnabled(true);
			myRollAngleController.setEnabled(true);

			break;
		}
		case FLIGHT_MODE_RATE:
		{
			// Disable angle controllers
			myPitchAngleController.setEnabled(false);
			myRollAngleController.setEnabled(false);

			// Enable rate controllers
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
FlightController::Error FlightController::setYawInput(const RealNumber yawInput)
{
	myYawInput = yawInput;
}

//------------------------------------------------------------------------------
FlightController::Error FlightController::setPitchInput(
													const RealNumber pitchInput)
{
	myPitchInput = pitchInput;
}

//------------------------------------------------------------------------------
FlightController::Error FlightController::setRollInput(
													 const RealNumber rollInput)
{
	myRollInput = rollInput;
}

//------------------------------------------------------------------------------
// Protected constructors
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
FlightController::FlightController(
							 InertialNavigationSystem& inertialNavigationSystem,
							 RemoteControlReceiver& remoteControlReceiver,
							 Controller& yawRateController,
							 Controller& pitchAngleController,
							 Controller& pitchRateController,
							 Controller& rollAngleController,
							 Controller& rollRateController,
							 MotorSpeedController& motor1MotorSpeedController,
							 MotorSpeedController& motor2MotorSpeedController,
							 MotorSpeedController& motor3MotorSpeedController,
							 MotorSpeedController& motor4MotorSpeedController,
							 HardwareTimer& controlUpdateHardwareTimer) :
	Module(),
	myInertialNavigationSystem(inertialNavigationSystem),
	myRemoteControlReceiver(remoteControlReceiver),
	myYawController(yawRateController),
	myPitchAngleController(pitchAngleController),
	myPitchRateController(pitchRateController),
	myRollAngleController(rollAngleController),
	myRollRateController(rollRateController),
	myMotor1MotorSpeedController(motor1MotorSpeedController),
	myMotor2MotorSpeedController(motor2MotorSpeedController),
	myMotor3MotorSpeedController(motor3MotorSpeedController),
	myMotor4MotorSpeedController(motor4MotorSpeedController),
	myControlUpdateHardwareTimer(controlUpdateHardwareTimer)
{
	myRemoteControlReceiver.setUpdateCallback(
				  createCallback(this, &FlightController::inputUpdateCallback));

	myControlUpdateHardwareTimer.setUpdateInterruptCallback(
				createCallback(this, &FlightController::controlUpdateCallback));
}

//------------------------------------------------------------------------------
// Protected virtual destructors
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
FlightController::~FlightController()
{
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
	yawRateControlUpdate();
	pitchAngleControlUpdate();
	rollAngleControlUpdate();
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
	RealNumber yawRateError = myYawRateController.update(yawRateDegreesPs);

	// This assumes props/motors spin "in"
	myMotor1MotorSpeedController.shiftSpeed(yawRateError);
	myMotor3MotorSpeedController.shiftSpeed(yawRateError);
	myMotor2MotorSpeedController.shiftSpeed(-yawRateError);
	myMotor4MotorSpeedController.shiftSpeed(-yawRateError);
}

//------------------------------------------------------------------------------
void FlightController::pitchAngleControlUpdate()
{
	RealNumber pitchDegrees = myInsMeasurement.pitchDegrees;

	myPitchController.update(pitchDegrees);
}

//------------------------------------------------------------------------------
void FlightController::pitchRateControlUpdate()
{
	RealNumber pitchRateError = myPitchRateController.update(pitchRateDegreesPs);

	myMotor1MotorSpeedController.shiftSpeed(pitchRateError);
	myMotor2MotorSpeedController.shiftSpeed(pitchRateError);
	myMotor3MotorSpeedController.shiftSpeed(-pitchRateError);
	myMotor4MotorSpeedController.shiftSpeed(-pitchRateError);
}

//------------------------------------------------------------------------------
void FlightController::rollAngleControlUpdate()
{
	AngleDegrees rollAngleDegrees = myInsMeasurement.rollAngleDegrees;

	myRollAngleController.update(rollAngleDegrees);
}

//------------------------------------------------------------------------------
void FlightController::rollRateControlUpdate()
{
	RealNumber rollRateError = myRollRateController.update(rollRateDegreesPs);

	myMotor1MotorSpeedController.shiftSpeed(rollRateError);
	myMotor4MotorSpeedController.shiftSpeed(rollRateError);
	myMotor2MotorSpeedController.shiftSpeed(-rollRateError);
	myMotor3MotorSpeedController.shiftSpeed(-rollRateError);
}

//------------------------------------------------------------------------------
// Private methods
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
void FlightController::inputUpdateCallback()
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
void FlightController::controlUpdateCallback()
{
	myStateEstimate = myStateEstimator.getEstimate();

	RealNumber yawDegrees = StateEstimator::getYawDegrees(myStateEstimate);

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

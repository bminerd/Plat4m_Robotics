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
/// @file FlightController.h
/// @author Ben Minerd
/// @date 12/21/2016
/// @brief FlightController class header file.
///

#ifndef PLAT4M_ROBOTICS_FLIGHT_CONTROLLER_H
#define PLAT4M_ROBOTICS_FLIGHT_CONTROLLER_H

//------------------------------------------------------------------------------
// Include files
//------------------------------------------------------------------------------

#include <ErrorTemplate.h>
#include <Module.h>
#include <Thread.h>
#include <Controller.h>
#include <HardwareTimer.h>

//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------

namespace Plat4m
{

namespace Robotics
{

//------------------------------------------------------------------------------
// Forward class declarations
//------------------------------------------------------------------------------

class InertialNavigationSystem;
class RemoteControlReceiver;
class Controller;
class MotorSpeedController;

//------------------------------------------------------------------------------
// Classes
//------------------------------------------------------------------------------

class FlightController : public Module
{
public:
    
    //--------------------------------------------------------------------------
    // Public types
    //--------------------------------------------------------------------------

	enum ErrorCode
	{
		ERROR_CODE_NONE = 0,
		ERROR_CODE_ENABLE
	};

	enum FlightMode
	{
		FLIGHT_MODE_ANGLE = 0,
		FLIGHT_MODE_RATE
	};

	struct Aircraft
	{

	};

	typedef ErrorTemplate<ErrorCode> Error;
    
    //--------------------------------------------------------------------------
    // Public methods
    //--------------------------------------------------------------------------

	Error setFlightMode(const FlightMode flightMode);

	Error setYawInput(const RealNumber yawInput);

	Error setPitchInput(const RealNumber pitchInput);

	Error setRollInput(const RealNumber rollInput);

protected:
    
    //--------------------------------------------------------------------------
    // Protected constructors
    //--------------------------------------------------------------------------

	FlightController(InertialNavigationSystem& inertialNavigationSystem,
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
					 HardwareTimer& controlUpdateHardwareTimer);

    //--------------------------------------------------------------------------
    // Protected virtual destructors
    //--------------------------------------------------------------------------

    virtual ~FlightController();
    
private:
    
    //--------------------------------------------------------------------------
    // Private data members
    //--------------------------------------------------------------------------

    InertialNavigationSystem& myInertialNavigationSystem;

    RemoteControlReceiver& myRemoteControlReceiver;

    Controller& myYawRateController;
    Controller& myPitchAngleController;
    Controller& myPitchRateController;
    Controller& myRollAngleController;
    Controller& myRollRateController;

    MotorSpeedController& myMotor1MotorSpeedController;
    MotorSpeedController& myMotor2MotorSpeedController;
    MotorSpeedController& myMotor3MotorSpeedController;
    MotorSpeedController& myMotor4MotorSpeedController;

    Mode myFlightMode;

    InertialNavigationSystem::Measurement myInsMeasurement;

    RealNumber myThrottleInput;
    RealNumber myYawInput;
    RealNumber myPitchInput;
    RealNumber myRollInput;

    HardwareTimer& myControlUpdateHardwareTimer;

    //--------------------------------------------------------------------------
    // Private inline methods
    //--------------------------------------------------------------------------

    inline void angleModeInputUpdate();

    inline void ratesModeInputUpdate();

    inline void angleModeControlUpdate();

    inline void ratesModeControlUpdate();

    inline void yawRateControlUpdate();

    inline void pitchAngleControlUpdate();

    inline void pitchRateControlUpdate();

    inline void rollAngleControlUpdate();

    inline void rollRateControlUpdate();

    //--------------------------------------------------------------------------
    // Private methods
    //--------------------------------------------------------------------------

    void inputUpdateCallback();

    void controlUpdateCallback();
};

}; // namspace Robotics

}; // namespace Plat4m

#endif // PLAT4M_ROBOTICS_FLIGHT_CONTROLLER_H

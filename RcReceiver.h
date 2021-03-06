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
// Copyright (c) 2017 Benjamin Minerd
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, inluding without limitation the rights
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
/// @file RcReceiver.h
/// @author Ben Minerd
/// @date 6/30/2017
/// @brief RcReceiver class header file.
///

#ifndef PLAT4M_ROBOTICS_RC_RECEIVER_H
#define PLAT4M_ROBOTICS_RC_RECEIVER_H

//------------------------------------------------------------------------------
// Include files
//------------------------------------------------------------------------------

#include <ErrorTemplate.h>
#include <Module.h>
#include <Thread.h>
#include <Controller.h>

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


//------------------------------------------------------------------------------
// Classes
//------------------------------------------------------------------------------

class RcReceiver : public Module
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

	typedef ErrorTemplate<ErrorCode> Error;
    
    //--------------------------------------------------------------------------
    // Public methods
    //--------------------------------------------------------------------------

protected:
    
    //--------------------------------------------------------------------------
    // Protected constructors
    //--------------------------------------------------------------------------

	RcReceiver();

    //--------------------------------------------------------------------------
    // Protected virtual destructors
    //--------------------------------------------------------------------------

    virtual ~RcReceiver();
    
private:
    
    //--------------------------------------------------------------------------
    // Private data members
    //--------------------------------------------------------------------------



    //--------------------------------------------------------------------------
    // Private inline methods
    //--------------------------------------------------------------------------


    //--------------------------------------------------------------------------
    // Private methods
    //--------------------------------------------------------------------------

};

}; // namspace Robotics

}; // namespace Plat4m

#endif // PLAT4M_ROBOTICS_RC_RECEIVER_H

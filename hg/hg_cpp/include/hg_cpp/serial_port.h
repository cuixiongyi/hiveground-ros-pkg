/*********************************************************************
* Software License Agreement (BSD License)
*
*  Serial port class, based upon code written by J.D.Medhurst (a.k.a. Tixy)
*  Copyright (c) 2010, Bob Mottram
*  fuzzgun@gmail.com
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef _serial_port_h_
#define _serial_port_h_

#include <ros/ros.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
 
namespace hg
{

class SerialPort
{
public:
		enum
		{
			PARITY_NONE = 0,
			PARITY_EVEN,
			PARITY_ODD
		};

    enum Errors {
        ErrorUnspecified    = -100, 
        ErrorInvalidPort    = -101, 
        ErrorPortInUse      = -102, 
        ErrorInvalidSettings= -103, 
        ErrorTransmitError  = -104, 
        ErrorReceiveError   = -105  
    };

    int SerialHandle; 

    SerialPort() { SerialHandle = 0; }
    virtual ~SerialPort();

    int Open(std::string device_name);
    int Initialise(unsigned baud, unsigned inDataBits, unsigned inStopBits, unsigned inParity);
    int Out(const uint8_t* data, size_t size, unsigned timeout);
    int In(uint8_t* data, size_t maxSize, unsigned timeout);
    void Close();
    int Error(int defaultError=ErrorUnspecified);

};

}
#endif

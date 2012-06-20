/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Mahisorn Wongphati
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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
 */

#ifndef BCAP_BCAP_H
#define BCAP_BCAP_H





namespace hg
{

namespace bcap
{

/* b-CAP argument constant */
static const int SIZE_ARGLEN = 4; //!< size of length
static const int SIZE_ARGTYPE = 2; //!< size of type
static const int SIZE_ARGARRAYS = 4; //!< size of arrays
static const int SIZE_ARGBASE = (SIZE_ARGLEN + SIZE_ARGTYPE + SIZE_ARGARRAYS);

static const int SIZE_ARGSTRLEN = 4; //!< size of string length

static const int MAX_PACKET_SIZE = 0x1000000; //!< max packet size (bytes)
static const int MAX_ARG_SIZE = 0x1000000; //!< max argument size (bytes)

/**
 * The b-CAP function IDs
 */
enum FunctionID
{
  kServiceStart = 1,
  kServiceStop = 2,
  kControllerConnect = 3,
  kControllerDisconnect = 4,
  kControllerGetRobot = 7,
  kControllerGetTask = 8,
  kControllerGetVariable = 9,
  kControllerExecute = 17,

  kRobotGetVariable = 62,
  kRobotExecute = 64,
  kRobotChange = 66,
  kRobotMove = 72,
  kRobotRelease = 84,

  kTaskGetVariable = 85,
  kTaskStart = 88,
  kTaskStop = 89,
  kTaskRelease = 99,

  kVariablePutValue = 102,
  kVariableGetValue = 101,
  kVariableRelease = 111
};

/**
 * The b-CAP Type id
 */
enum TypeID
{
  VT_EMPTY = 0,         //!< 0 byte
  VT_NULL = 1,          //!< 0 byte
  VT_ERROR = 10,        //!< 2 byte
  VT_UI1 = 17,          //!< 1 byte
  VT_I2 = 2,            //!< 2 byte
  VT_UI2 = 18,          //!< 2 byte
  VT_I4 = 3,            //!< 4 byte
  VT_UI4 = 19,          //!< 4 byte
  VT_R4 = 4,            //!< 4 byte
  VT_R8 = 5,            //!< 8 byte
  VT_CY = 6,            //!< 8 byte
  VT_DATE = 7,          //!< 8  byte
  VT_BOOL = 11,         //!< 2 byte
  VT_BSTR = 8,          //!< ASCII string length *2 + 4 byte

  // Double bytes per character
  VT_VARIANT = 12,      //!< Variant
  VT_ARRAY = 0x2000,    //!< Array
};

/**
 * The error code of b-Cap protocol
 */
enum Error
{
  kOK = 0, kNotImplement, //!< Not implemented function is called
  kAborted, //!< Function aborted
  kFail, //!< Function failed
  kFatal, //!< Fatal Error occurred
  kInvalidPacket, //!< Invalid packet is received.
                  //!< When this error is occurred, robot controller disconnect from client immediately.
                  //!< Please make sure the packet that you sent.

  kInvalidSentPacket, //!< Invalid packet is sent
  kInvalidArgumentType, //!< Invalid argument type
  kRobotIsBusy, //!< Robot is busy (Wait for a while)
  kInvalidCommand, //!< Invalid command string is received

  kReceivedPacketSizeOver, //!< Received packet size over ( > 16Mbytes)

  kArgumentSizeOver, //!< An argument siez over of the received packet. ( > 16Mbytes)
  kAccessDenined, //!< Access denied
  kInvalidHandle, //!< Invalid handle
  kOutOfMemory, //!< Out of memory
  kInvalidArgument //!< Invalid argument
};

}

}

#endif




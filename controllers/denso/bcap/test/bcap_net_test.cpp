/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Mahisorn Wongphati
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Southwest Research Institute, nor the names
 *      of its contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>

#include <bcap/bcap_net.h>


BCapNet bcap_("10.0.0.101", "5007", BCapNet::BCAP_UDP);
uint32_t hController_;
uint32_t hTask_;
uint32_t hRobot_;
float position[8];
float joint_angle_[8];
float joint_return_[8];

TEST(BCapNetTest, ServiceStart)
{
  BCAP_HRESULT hr;

  hr = bcap_.ServiceStart();
  EXPECT_EQ(BCAP_S_OK, hr);
}



TEST(BCapNetTest, ControllerConnect)
{
  BCAP_HRESULT hr;

  hr = bcap_.ControllerConnect("", "", "", "", &hController_);
  EXPECT_EQ(BCAP_S_OK, hr);
}

TEST(BCapNetTest, GetCurrentAutoMode)
{
  BCAP_HRESULT hr;
  int mode = 0;
  long result = 0;
  hr = bcap_.ControllerExecute2(
      hController_,
      "GetAutoMode",
      VT_EMPTY,
      1,
      &mode,
      &result);
  EXPECT_EQ(BCAP_S_OK, hr);
}

TEST(BCapNetTest, PutAutoModeExternal)
{
  BCAP_HRESULT hr;
  uint16_t mode = 2;
  long result = 0;
  hr = bcap_.ControllerExecute2(
      hController_,
      "PutAutoMode",
      VT_I2,
      1,
      &mode,
      &result);
  EXPECT_EQ(BCAP_S_OK, hr);
}

TEST(BCapNetTest, GetAutoModeExternal)
{
  BCAP_HRESULT hr;
  int mode = 0;
  long result = 0;
  hr = bcap_.ControllerExecute2(
      hController_,
      "GetAutoMode",
      VT_EMPTY,
      1,
      &mode,
      &result);
  EXPECT_EQ(BCAP_S_OK, hr);
  EXPECT_EQ(result, 2);
}

TEST(BCapNetTest,ControllerGetTask)
{
  BCAP_HRESULT hr;
  hr = bcap_.ControllerGetTask(
      hController_,
      "RobSlave",
      "", //not used
      &hTask_);
  EXPECT_EQ(BCAP_S_OK, hr);
}

TEST(BCapNetTest, TaskStart)
{
  BCAP_HRESULT hr;
  hr = bcap_.TaskStart(
      hTask_,
      1,
      ""); //not used
  EXPECT_EQ(BCAP_S_OK, hr);
  sleep(1);
}

TEST(BCapNetTest, ControllerGetRobot)
{
  BCAP_HRESULT hr;

  hr = bcap_.ControllerGetRobot(hController_, "ARM", "$IsIDHandle$", &hRobot_);
  EXPECT_EQ(BCAP_S_OK, hr);
}

TEST(BCapNetTest, TurnOffMotor)
{
  BCAP_HRESULT hr;
  int mode = 0;
  long result;
  hr = bcap_.RobotExecute2(hRobot_, "Motor", VT_I2, 1, &mode, &result);
  EXPECT_EQ(BCAP_S_OK, hr);
  sleep(5);
}

TEST(BCapNetTest, SlaveGetCurrentMode)
{
  BCAP_HRESULT hr;
  int mode = 0;
  long result = 0;
  hr = bcap_.RobotExecute2(
      hRobot_,
      "slvGetMode",
      VT_EMPTY,
      1,
      &mode,
      &result);
  EXPECT_EQ(BCAP_S_OK, hr);
}

TEST(BCapNetTest, ChangeModeToSlave)
{
  BCAP_HRESULT hr;
  int mode = 258;
  long result;
  hr = bcap_.RobotExecute2(
     hRobot_,
     "slvChangeMode",
     VT_I4,
     1,
     &mode,
     &result);
  EXPECT_EQ(BCAP_S_OK, hr);
}

TEST(BCapNetTest, SlaveGetModeSlave)
{
  BCAP_HRESULT hr;
  int mode = 0;
  long result = 0;
  hr = bcap_.RobotExecute2(
      hRobot_,
      "slvGetMode",
      VT_EMPTY,
      1,
      &mode,
      &result);
  EXPECT_EQ(BCAP_S_OK, hr);
  EXPECT_EQ(result, 258);
}


TEST(BCapNetTest, Get_Joint_information)
{
  BCAP_HRESULT hr;
  uint32_t hVariable;
  hr = bcap_.RobotGetVariable(
      hRobot_,
      "@CURRENT_POSITION",
      "",
      &hVariable);
  EXPECT_EQ(BCAP_S_OK, hr);

  hr = bcap_.VariableGetValue(hVariable, position);
  EXPECT_EQ(BCAP_S_OK, hr);

  std::cout << "position: ";
  for(int i = 0; i < 7; i++)
  {
    std::cout << position[i] << ",";
  }
  std::cout << std::endl;

  hr = bcap_.VariableRelease(hVariable);
  EXPECT_EQ(BCAP_S_OK, hr);

  hr = bcap_.RobotGetVariable(
    hRobot_,
    "@CURRENT_ANGLE",
    "",
    &hVariable);
  EXPECT_EQ(BCAP_S_OK, hr);

  hr = bcap_.VariableGetValue(hVariable, joint_angle_);
  EXPECT_EQ(BCAP_S_OK, hr);

  std::cout << "angle: ";
  for(int i = 0; i < 8; i++)
  {
    std::cout << position[i] << ",";
  }
  std::cout << std::endl;
}

TEST(BCapNetTest, TurnOnMotor)
{
  BCAP_HRESULT hr;
  int mode = 1;
  long result;
  hr = bcap_.RobotExecute2(hRobot_, "Motor", VT_I2, 1, &mode, &result);
  EXPECT_EQ(BCAP_S_OK, hr);
  sleep(5);
}

TEST(BCapNetTest, MoveArm)
{
  BCAP_HRESULT hr;
  hr = bcap_.RobotExecute2(hRobot_, "slvMove", VT_R4 | VT_ARRAY, 7, joint_angle_, joint_return_);
  EXPECT_EQ(BCAP_S_OK, hr);
  sleep(5);

  //for (int i = 0; i < 8; i++)
  //{
    //std::cout << "[" << i << "]" << joint_angle_[i] << ":" << joint_return_[i] << std::endl;
  //}
}

TEST(BCapNetTest, ChangeModeNormal)
{
  BCAP_HRESULT hr;
  int mode = 0;
  long result;
  hr = bcap_.RobotExecute2(
     hRobot_,
     "slvChangeMode",
     VT_I4,
     1,
     &mode,
     &result);
  EXPECT_EQ(BCAP_S_OK, hr);
  //std::cout << "press any key to continue\n";
  //getchar();
}

TEST(BCapNetTest, SlaveGetModeNormal)
{
  BCAP_HRESULT hr;
  int mode = 0;
  long result = 0;
  hr = bcap_.RobotExecute2(
      hRobot_,
      "slvGetMode",
      VT_EMPTY,
      1,
      &mode,
      &result);
  EXPECT_EQ(BCAP_S_OK, hr);
  EXPECT_EQ(result, 0);

  //std::cout << "slave mode: " << result << "\n";
  //getchar();
}

TEST(BCapNetTest, TurnOffMotor2)
{
  BCAP_HRESULT hr;
  int mode = 0;
  long result;
  hr = bcap_.RobotExecute2(hRobot_, "Motor", VT_I2, 1, &mode, &result);
  EXPECT_EQ(BCAP_S_OK, hr);
  sleep(5);
}

TEST(BCapNetTest, RobotRelease)
{
  BCAP_HRESULT hr;
  hr = bcap_.RobotRelease(hRobot_);
  EXPECT_EQ(BCAP_S_OK, hr);
}

TEST(BCapNetTest, TaskStop)
{
  BCAP_HRESULT hr;
  hr = bcap_.TaskStop(
      hTask_,
      1,
      ""); //not used
  EXPECT_EQ(BCAP_S_OK, hr);

  //std::cout << "press any key to continue\n";
  //getchar();
  sleep(1);
}

TEST(BCapNetTest, TaskRelease)
{
  BCAP_HRESULT hr;
  hr = bcap_.TaskRelease(hTask_);
  EXPECT_EQ(BCAP_S_OK, hr);
}

TEST(BCapNetTest, PutAutoMode)
{
  BCAP_HRESULT hr;
  uint16_t mode = 1;
  long result = 0;
  hr = bcap_.ControllerExecute2(
      hController_,
      "PutAutoMode",
      VT_I2,
      1,
      &mode,
      &result);
  EXPECT_EQ(BCAP_S_OK, hr);
  //std::cout << "auto mode: " << result << "\n";
}

TEST(BCapNetTest, GetAutoMode)
{
  BCAP_HRESULT hr;
  int mode = 0;
  long result = 0;
  hr = bcap_.ControllerExecute2(
      hController_,
      "GetAutoMode",
      VT_EMPTY,
      1,
      &mode,
      &result);
  EXPECT_EQ(BCAP_S_OK, hr);
  EXPECT_EQ(result, 1);
  //std::cout << "auto mode: " << result << "\n";
}


TEST(BCapNetTest, ControllerDisconnect)
{
  BCAP_HRESULT hr;
  hr = bcap_.ControllerDisconnect(hController_);
  EXPECT_EQ(BCAP_S_OK, hr);
}

TEST(BCapNetTest, ServiceStop)
{
  BCAP_HRESULT hr;

  hr = bcap_.ServiceStop();
  EXPECT_EQ(BCAP_S_OK, hr);
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}



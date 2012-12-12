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

#include <bcap/bcap_serial.h>


namespace
{

// The fixture for testing class Foo.
class BCapSerialTest : public ::testing::Test
{
protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  BCapSerialTest()
    : bcap_("/dev/ttyUSB0", 115200)
  {
    // You can do set-up work for each test here.
  }

  virtual ~BCapSerialTest()
  {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp()
  {
    // Code here will be called immediately after the constructor (right
    // before each test).
  }

  virtual void TearDown()
  {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case for Foo.
  BCapSerial bcap_;
  uint32_t hController_;
  uint32_t hTask_;
  uint32_t hRobot_;
  float joint_angle_[8];
  float joint_return_[8];
};

TEST_F(BCapSerialTest, ControllerConnect)
{
  BCAP_HRESULT hr;

  hr = bcap_.ControllerConnect("", "", "", "", &hController_);
  EXPECT_EQ(BCAP_S_OK, hr);
}

TEST_F(BCapSerialTest, ControllerGetRobot)
{
  BCAP_HRESULT hr;

  hr = bcap_.ControllerGetRobot(hController_, "VE026A", "$IsIDHandle$", &hRobot_);
  EXPECT_EQ(BCAP_S_OK, hr);
}

TEST_F(BCapSerialTest, TurnOffMotor)
{
  BCAP_HRESULT hr;
  int mode = 0;
  long result;
  hr = bcap_.RobotExecute2(hRobot_, "Motor", VT_I2, 1, &mode, &result);
  EXPECT_EQ(BCAP_S_OK, hr);
}

TEST_F(BCapSerialTest, SetSlaveMode)
{
  BCAP_HRESULT hr;
  //set slave mode
  int mode = 258;
  long result;
  hr = bcap_.RobotExecute2(hRobot_, "slvChangeMode", VT_I4, 1, &mode, &result);
  EXPECT_EQ(BCAP_S_OK, hr);
}

TEST_F(BCapSerialTest, SlaveGetMode)
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

TEST_F(BCapSerialTest, Get_Joint_information)
{
  BCAP_HRESULT hr;
  uint32_t hVariable;
  hr = bcap_.RobotGetVariable(hRobot_, "@CURRENT_ANGLE", "", &hVariable);
  EXPECT_EQ(BCAP_S_OK, hr);

  hr = bcap_.VariableGetValue(hVariable, joint_angle_);
  EXPECT_EQ(BCAP_S_OK, hr);
}

TEST_F(BCapSerialTest, TurnOnMotor)
{
  BCAP_HRESULT hr;
  int mode = 1;
  long result;
  hr = bcap_.RobotExecute2(hRobot_, "Motor", VT_I2, 1, &mode, &result);
  EXPECT_EQ(BCAP_S_OK, hr);
}

TEST_F(BCapSerialTest, MoveArm)
{
  BCAP_HRESULT hr;
  hr = bcap_.RobotExecute2(hRobot_, "slvMove", VT_R4 | VT_ARRAY, 7, joint_angle_, joint_return_);
  EXPECT_EQ(BCAP_S_OK, hr);

  //for (int i = 0; i < 8; i++)
  //{
    //std::cout << "[" << i << "]" << joint_angle_[i] << ":" << joint_return_[i] << std::endl;
  //}
}

TEST_F(BCapSerialTest, TurnOffMotor2)
{
  BCAP_HRESULT hr;
  int mode = 0;
  long result;
  hr = bcap_.RobotExecute2(hRobot_, "Motor", VT_I2, 1, &mode, &result);
  EXPECT_EQ(BCAP_S_OK, hr);
}

TEST_F(BCapSerialTest, RobotRelease)
{
  BCAP_HRESULT hr;
  hr = bcap_.RobotRelease(hRobot_);
  EXPECT_EQ(BCAP_S_OK, hr);
}

TEST_F(BCapSerialTest, ControllerDisconnect)
{
  BCAP_HRESULT hr;
  hr = bcap_.ControllerDisconnect(hController_);
  EXPECT_EQ(BCAP_S_OK, hr);
}

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}



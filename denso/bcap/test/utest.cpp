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

#include <bcap/bcap.h>

#define CONTROLLER_IP "10.0.0.101"
#define CONTROLLER_PORT 5007

namespace
{

// The fixture for testing class Foo.
class BCapTest : public ::testing::Test
{
protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  BCapTest()
    : bcap_(false)
  {

    // You can do set-up work for each test here.
  }

  virtual ~BCapTest()
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
  BCap bcap_;
  uint32_t hController_;
  uint32_t hTask_;
  uint32_t hRobot_;
};


TEST_F(BCapTest, bCap_Open)
{
  ASSERT_EQ(BCAP_S_OK, bcap_.bCap_Open(CONTROLLER_IP, CONTROLLER_PORT));
}

TEST_F(BCapTest, bCap_ControllerConnect)
{
  BCAP_HRESULT hr;

  hr = bcap_.bCap_ControllerConnect(
  "", //not used in RC7
  "", //not used in RC7
  "", //not used in RC7
  "", //not used in RC7
  &hController_);
  EXPECT_EQ(BCAP_S_OK, hr);
}

//TEST_(BCapTest, bCap_ControllerExecute)

TEST_F(BCapTest, bCap_ControllerGetTask)
{
  BCAP_HRESULT hr;
  hr = bcap_.bCap_ControllerGetTask(
      hController_,
      "RobSlave",
      "", //not used
      &hTask_);
  EXPECT_EQ(BCAP_S_OK, hr);
}

TEST_F(BCapTest, bCap_TaskStart)
{
  BCAP_HRESULT hr;
  hr = bcap_.bCap_TaskStart(
      hTask_,
      1,
      ""); //not used
  EXPECT_EQ(BCAP_S_OK, hr);

  //std::cout << "press any key to continue\n";
  //getchar();
  sleep(1);

}

TEST_F(BCapTest, bCap_ControllerGetRobot)
{
  BCAP_HRESULT hr;
  hr = bcap_.bCap_ControllerGetRobot(
      hController_,
      "ARM",
      "$IsIDHandle$",
      &hRobot_);
  EXPECT_EQ(BCAP_S_OK, hr);
  //std::cout << "press any key to continue\n";
  //getchar();
}


TEST_F(BCapTest, Execute_SlaveGetMode)
{
  BCAP_HRESULT hr;
  int mode = 0;
  long result = 0;
  hr = bcap_.bCap_RobotExecute2(
      hRobot_,
      "slvGetMode",
      VT_EMPTY,
      1,
      &mode,
      &result);
  EXPECT_EQ(BCAP_S_OK, hr);

  std::cout << "slave mode: " << result << "\n";
  //getchar();
}


TEST_F(BCapTest, Execute_SlaveChangeMode_Slave)
{
  BCAP_HRESULT hr;
  int mode = 258;
  long result;
  hr = bcap_.bCap_RobotExecute2(
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

TEST_F(BCapTest, Execute_SlaveGetMode2)
{
  BCAP_HRESULT hr;
  int mode = 0;
  long result = 0;
  hr = bcap_.bCap_RobotExecute2(
      hRobot_,
      "slvGetMode",
      VT_EMPTY,
      1,
      &mode,
      &result);
  EXPECT_EQ(BCAP_S_OK, hr);

  std::cout << "slave mode: " << result << "\n";
  //getchar();
}


TEST_F(BCapTest, Get_Joint_information)
{
  BCAP_HRESULT hr;
  uint32_t hVariable;
  hr = bcap_.bCap_RobotGetVariable(
      hRobot_,
      "@CURRENT_POSITION",
      "",
      &hVariable);
  EXPECT_EQ(BCAP_S_OK, hr);

  float position[8];

  hr = bcap_.bCap_VariableGetValue(hVariable, position);
  EXPECT_EQ(BCAP_S_OK, hr);

  std::cout << "position: ";
  for(int i = 0; i < 7; i++)
  {
    std::cout << position[i] << ",";
  }
  std::cout << std::endl;

  hr = bcap_.bCap_VariableRelease(hVariable);
  EXPECT_EQ(BCAP_S_OK, hr);

  hr = bcap_.bCap_RobotGetVariable(
    hRobot_,
    "@CURRENT_ANGLE",
    "",
    &hVariable);
  EXPECT_EQ(BCAP_S_OK, hr);

  hr = bcap_.bCap_VariableGetValue(hVariable, position);
  EXPECT_EQ(BCAP_S_OK, hr);

  std::cout << "angle";
  for(int i = 0; i < 8; i++)
  {
    std::cout << position[i] << ",";
  }
  std::cout << std::endl;

}

TEST_F(BCapTest, Execute_SlaveChangeMode_Normal)
{
  BCAP_HRESULT hr;
  int mode = 0;
  long result;
  hr = bcap_.bCap_RobotExecute2(
     hRobot_,
     "slvChangeMode",
     VT_I4,
     1,
     &mode,
     &result);
  EXPECT_EQ(BCAP_S_OK, hr);
  std::cout << "press any key to continue\n";
  //getchar();
}

TEST_F(BCapTest, Execute_SlaveGetMode3)
{
  BCAP_HRESULT hr;
  int mode = 0;
  long result = 0;
  hr = bcap_.bCap_RobotExecute2(
      hRobot_,
      "slvGetMode",
      VT_EMPTY,
      1,
      &mode,
      &result);
  EXPECT_EQ(BCAP_S_OK, hr);

  std::cout << "slave mode: " << result << "\n";
  //getchar();
}


TEST_F(BCapTest, bCap_RobotRelease)
{
  BCAP_HRESULT hr;
  hr = bcap_.bCap_RobotRelease(hRobot_);
  EXPECT_EQ(BCAP_S_OK, hr);
}

TEST_F(BCapTest, bCap_TaskStop)
{
  BCAP_HRESULT hr;
  hr = bcap_.bCap_TaskStop(
      hTask_,
      1,
      ""); //not used
  EXPECT_EQ(BCAP_S_OK, hr);
}

TEST_F(BCapTest, bCap_TaskRelease)
{
  BCAP_HRESULT hr;
  hr = bcap_.bCap_TaskRelease(hTask_);
  EXPECT_EQ(BCAP_S_OK, hr);
}

TEST_F(BCapTest, bCap_ControllerDisconnect)
{
  BCAP_HRESULT hr;
  hr = bcap_.bCap_ControllerDisconnect(hController_);
  EXPECT_EQ(BCAP_S_OK, hr);
}

TEST_F(BCapTest, bCap_Close)
{
  ASSERT_EQ(BCAP_S_OK, bcap_.bCap_Close());
}

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

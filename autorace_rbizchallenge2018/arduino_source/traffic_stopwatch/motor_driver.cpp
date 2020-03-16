/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Gilbert */

#include "motor_driver.h"

MotorDriver::MotorDriver()
: baudrate_(BAUDRATE),
  protocol_version_(PROTOCOL_VERSION),
  motor_id_1(MOTOR_ID_1),
  motor_id_2(MOTOR_ID_2)
{
}

MotorDriver::~MotorDriver()
{
  closeDynamixel();
}

bool MotorDriver::init(void)
{
  portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler_->openPort())
  {
    #ifdef DEBUG
    sprintf(log_msg, "Port is Opened");
    nh.loginfo(log_msg);
    #endif
  }
  else
  {
    return false;
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baudrate_))
  {
    #ifdef DEBUG
    sprintf(log_msg, "Baudrate is set");
    nh.loginfo(log_msg);
    #endif
  }
  else
  {
    return false;
  }

  // Enable Dynamixel Torque
  setTorque(motor_id_1, true);

  setProfileVelocity(motor_id_1, PROFILE_VELOCITY_VALUE);
  setProfileAcceleration(motor_id_1, PROFILE_ACCELERATION_VALUE);

  setTorque(motor_id_2, true);

  setProfileVelocity(motor_id_2, PROFILE_VELOCITY_VALUE);
  setProfileAcceleration(motor_id_2, PROFILE_ACCELERATION_VALUE);

  return true;
}

void MotorDriver::setAccelerationLimit(uint8_t id, int64_t acceleration_limit_value)
{
  uint8_t dxl_error = 0;
  int8_t dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, ADDR_X_PROFILE_ACCELERATION, acceleration_limit_value, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler_->getTxRxResult(dxl_comm_result);
  }
  
  else if(dxl_error != 0)
  {
    packetHandler_->getRxPacketError(dxl_error);
  }
}

void MotorDriver::setVelocityLimit(uint8_t id, int64_t velocity_limit_value)
{
  uint8_t dxl_error = 0;
  int8_t dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, ADDR_X_PROFILE_VELOCITY, velocity_limit_value, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler_->getTxRxResult(dxl_comm_result);
  }
  else if(dxl_error != 0)
  {
    packetHandler_->getRxPacketError(dxl_error);
  }
}


void MotorDriver::setTorque(uint8_t id, bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler_->getTxRxResult(dxl_comm_result);
  }
  else if(dxl_error != 0)
  {
    packetHandler_->getRxPacketError(dxl_error);
  }
}

void MotorDriver::setProfileAcceleration(uint8_t id, int64_t profile_acceleration_value)
{
  uint8_t dxl_error = 0;
  int8_t dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, ADDR_X_PROFILE_ACCELERATION, profile_acceleration_value, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler_->getTxRxResult(dxl_comm_result);
  }
  else if(dxl_error != 0)
  {
    packetHandler_->getRxPacketError(dxl_error);
  }
}

void MotorDriver::setProfileVelocity(uint8_t id, int64_t profile_velocity_value)
{
  uint8_t dxl_error = 0;
  int8_t dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, ADDR_X_PROFILE_VELOCITY, profile_velocity_value, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler_->getTxRxResult(dxl_comm_result);
  }
  else if(dxl_error != 0)
  {
    packetHandler_->getRxPacketError(dxl_error);
  }
}

void MotorDriver::closeDynamixel(void)
{
  // Disable Dynamixel Torque
  setTorque(motor_id_1, false);
  setTorque(motor_id_2, false);
  
  // Close port
  portHandler_->closePort();
}

void MotorDriver::controlPosition(uint8_t id, int64_t position_value)
{
  uint8_t dxl_error = 0;
  int8_t dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, ADDR_X_GOAL_POSITION, position_value, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler_->getTxRxResult(dxl_comm_result);
  }
  else if(dxl_error != 0)
  {
    packetHandler_->getRxPacketError(dxl_error);
  }
}

/* RBiz Challenge TurtleBot3 Auto Race - Level Crossing


|-----|
||---||----|-------|-------|-------|-------|
||DXL||    |   W   |   R   |   W   |   R   |
||---||----|-------|-------|-------|-------|
|     |
|     |
|     |
|     |  |---------|            |---------|            |---------|
|-----|  | O-----0 |            | O-----0 |            | O-----0 |
  | |    |   DMS   |            |   DMS   |            |   DMS   |
  | |    | Sensor1 |            | Sensor2 |            | Sensor3 |
  | |    |         |            |         |            |         |
=======------------===========------------===========------------===========

created 1 October 2017
by ROBOTIS CO,.LTD.

author : Leon Ryuwoon Jung
*/

#include "level_crossing.h"

/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<rbiz_autorace_msgs::SensorStateStopwatch> msgSensorStateStopwatch("sensor_state/stopwatch", callBack);

/*******************************************************************************
* Publisher
*******************************************************************************/
// DMS, battery, etc.

/*******************************************************************************
* Setup function
*******************************************************************************/
void setup()
{
  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  nh.getHardware()->setBaud(115200);
  nh.subscribe(msgSensorStateStopwatch);
  nh.subscribe(reset_sub);

  nh.loginfo("Connected to OpenCR board!");

  // Settings for OLLO DMS
  ollo.begin(1);  //DMS Module must be connected at port 1.
  ollo.begin(2);  //DMS Module must be connected at port 2.
  ollo.begin(3);  //DMS Module must be connected at port 3.

  // Settings for OLLO TS-10
  ollo.begin(4, TOUCH_SENSOR);

  // Setting for Dynamixel motors
  motorDriver.init();

  // Init Paramters
  fnInitStateLevelCrossing();
}

/*******************************************************************************
* Loop function
*******************************************************************************/
void loop()
{
  fnGetButtonPressed();

  fnReceiveSensorDistance();

  fnCheckVehicleStatus();

  fnLevelControl();

  fnControlLevelPose();

  nh.spinOnce();
}

/*******************************************************************************
* Publish msgs (Sensor data: Power, etc.)
*******************************************************************************/


/*******************************************************************************
* Callback function
*******************************************************************************/
void cbInitStateLevelCrossing(const rbiz_autorace_msgs::DoIt& msgInitStateLevelCrossing)
{
  if (msgInitStateLevelCrossing.doIt == true)
  {
    fnInitStateLevelCrossing();
  }
}

void cbTestStateLevelCrossing(const rbiz_autorace_msgs::DoIt& msgTestStateLevelCrossing)
{
  if (msgTestStateLevelCrossing.doIt == true)
  {
    fnTestStateLevelCrossing();
  }
}

/*******************************************************************************
* Normal function
*******************************************************************************/
void fnGetButtonPressed()
{
  if (ollo.read(4, TOUCH_SENSOR))
  {
    fnInitStateLevelCrossing();
  }
}

void fnInitStateLevelCrossing()
{
  fnSetStopWatch();
  
  level_turn_ = 0;
  sensor_distance[0] = 0;
  sensor_distance[1] = 0;
  sensor_distance[2] = 0;

  is_started[0] = false;
  is_started[1] = false;
  is_started[2] = false;

  is_able_to_pass_ = true;
  vehicle_state_ = WAITING_FOR_ENTER;
  level_status_ = LEVEL_OPENED;
  mode_ = ACTIVE_MODE;
    // mode_ = TEST_MODE;
}

void fnTestStateLevelCrossing()
{
  fnInitStateLevelCrossing();
  mode_ = TEST_MODE;
}

void fnReceiveSensorDistance()
{
  sensor_distance[0] = ollo.read(1);
  sensor_distance[1] = ollo.read(2);
  sensor_distance[2] = ollo.read(3);

  // Serial.print("DMS Sensor 1 ADC Value = ");
  // Serial.println(sensor_distance[0]); //read ADC value from OLLO port 1
  //
  // Serial.print("DMS Sensor 2 ADC Value = ");
  // Serial.println(sensor_distance[1]); //read ADC value from OLLO port 2
  //
  // Serial.print("DMS Sensor 3 ADC Value = ");
  // Serial.println(sensor_distance[2]); //read ADC value from OLLO port 3

  delay(100);
}

void fnCheckVehicleStatus()
{
  if (sensor_distance[0] > DISTANCE_THRESHOLD_PASS && vehicle_state_ == WAITING_FOR_ENTER)
  {
    vehicle_state_ = ENTERED;
  }
  else if (sensor_distance[1] > DISTANCE_THRESHOLD_PASS && vehicle_state_ == ENTERED)
  {
    vehicle_state_ = MUST_STOP;
  }
  else if (sensor_distance[2] > DISTANCE_THRESHOLD_PASS && vehicle_state_ == MUST_STOP)
  {
    vehicle_state_ = PASSED;
  }

  // Serial.print("State : ");
  // Serial.println(vehicle_state_);
}

void fnLevelControl()
{
  if (mode_ == ACTIVE_MODE)
  {
    if (vehicle_state_ == ENTERED)
    {
      if (is_started[0] == false)
      {
        //start stopwatch
        fnSetStopWatch();
        is_started[0] = true;
      }
      else
      {
        is_able_to_pass_ = false;

        level_status_ = LEVEL_CLOSED;

      }
    }
    else if (vehicle_state_ == MUST_STOP)
    {
        if (is_started[1] == false)
        {
          fnSetStopWatch();
          is_started[1] = true;
        }
        else
        {
          if (fnGetTimeSinceStart() > 5000.0)
          {
            level_status_ = LEVEL_OPENED;
            is_able_to_pass_ = true;
          }
          else
          {
            level_status_ = LEVEL_CLOSED;
            is_able_to_pass_ = false;
          }
        }
    }
    else if (vehicle_state_ == PASSED)
    {
      if (is_started[2] == false)
      {
        fnSetStopWatch();
        is_started[2] = true;
      }
      else
      {
        if (is_able_to_pass_ == false)
        {
          level_status_ = LEVEL_CLOSED;
        }
        else
        {
          if (level_turn_ == 1)          level_status_ = LEVEL_SUCCESS;
          else                           level_status_ = LEVEL_OPENED;
          
          if (millis()-pre_time >= 1000) 
          {
            level_turn_ = 1 - level_turn_;
            pre_time = millis();
          }
        }
      }
    }
  }
  // Serial.print(fnGetTimeSinceStart());
  // Serial.println(" ");
}

void fnControlLevelPose()
{
  if (level_status_ == LEVEL_OPENED)
  {
    motorDriver.controlPosition(DXL_ID, DXL_POSITION_VALUE_OPENED);
  }
  else if (level_status_ == LEVEL_CLOSED)
  {
    motorDriver.controlPosition(DXL_ID, DXL_POSITION_VALUE_CLOSED);
  }
  else if (level_status_ == LEVEL_MIDDLE)
  {
    motorDriver.controlPosition(DXL_ID, DXL_POSITION_VALUE_MIDDLE);
  }
  else if (level_status_ == LEVEL_SUCCESS)
  {
    motorDriver.controlPosition(DXL_ID, DXL_POSITION_VALUE_SUCCESS);
  }
  
}

double fnGetCurrentTime()
{
	return (double)millis();
}

double fnGetTimeSinceStart()
{
  double elapsed_time;

  elapsed_time = fnGetCurrentTime() - stopwatch_start_time_;
  if (elapsed_time < 0.0)
    stopwatch_start_time_ = fnGetCurrentTime();

  return elapsed_time;
}
void fnSetStopWatch()
{
  stopwatch_start_time_ = fnGetCurrentTime();
}
void callBack(const rbiz_autorace_msgs::SensorStateStopwatch& state_msg)
{
  int data;
  
  data = state_msg.vehicle_state;
  if (data == 2) fnInitStateLevelCrossing();
}

void resetCallback(const std_msgs::Bool& reset_msg)
{
  fnInitStateLevelCrossing();
}

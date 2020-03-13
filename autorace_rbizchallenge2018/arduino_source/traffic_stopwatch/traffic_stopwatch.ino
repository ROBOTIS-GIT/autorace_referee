/* RBiz Challenge TurtleBot3 Auto Race - Traffic Light

                      |-----|
                      ||---||
                      || R ||
                      ||---||
                      |     |
                      ||---||
                      || Y ||
                      ||---||
                      |     |
                      ||---||
                      || G ||
|---------|           ||---||            
| O-----0 |           |-----|              
|   DMS   |             | |               
| Sensor1 |             | |                 
|         |             | |                
===========-----------=======
 created 1 October 2018
 by ROBOTIS CO,.LTD.

 author : Gilbert
 */

#include "traffic_stopwatch.h"
/*******************************************************************************
* Publisher
*******************************************************************************/
// DMS, battery, etc.
rbiz_autorace_msgs::SensorStateStopwatch msgSensorStateStopwatch;
ros::Publisher pubSensorStateStopwatch("sensor_state/stopwatch", &msgSensorStateStopwatch);

/*******************************************************************************
* Set param
*******************************************************************************/
const int BLUE_push = BDPIN_GPIO_6;  //training mode
const int RED_push = BDPIN_GPIO_4;   //match mode
const int RED_YELLOW_delay = 3000;   // RED -> YELLOW delay
const int GREEN_RED_delay = 5000;    // GREEN -> RED delay 

/*******************************************************************************
* Setup function
*******************************************************************************/
void setup()
{
  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  nh.getHardware()->setBaud(115200);
  nh.subscribe(reset_sub);
  nh.subscribe(state_sub);
  nh.advertise(pubSensorStateStopwatch);

  nh.loginfo("Connected to OpenCR board!");

  // Settings for OLLO DMS
  ollo.begin(1);  //DMS Module must be connected at port 1.
  ollo.begin(2);  //DMS Module must be connected at port 2.
  // Settings for OLLO TS-10
  ollo.begin(4, TOUCH_SENSOR);

  // Pins for LED
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  pinMode(BLUE_push, INPUT_PULLUP);
  pinMode(RED_push, INPUT_PULLUP);

  // Setting for Dynamixel motors
  motorDriver.init();
  
  // Init Paramters
  fnInitStateTrafficLight();
  
}

/*******************************************************************************
* Loop function
*******************************************************************************/
void loop()
{  
  modeCheck();
    
  fnGetButtonPressed();

  fnReceiveSensorDistance();

  fnCheckVehicleStatus();

  fnLEDControl();

  fnControlLED();
  
  nh.spinOnce();
}

/*******************************************************************************
* Publish msgs (Sensor data: Power, etc.)
*******************************************************************************/
void pbSensorState()
{
  msgSensorStateStopwatch.stamp = nh.now();
  msgSensorStateStopwatch.sensor_distance[0] = sensor_distance[0];
  msgSensorStateStopwatch.sensor_distance[1] = sensor_distance[1];
  msgSensorStateStopwatch.sensor_distance[2] = sensor_distance[2];
  msgSensorStateStopwatch.vehicle_state = mission_state_;

  pubSensorStateStopwatch.publish(&msgSensorStateStopwatch);
}

/*******************************************************************************
* Normal function
*******************************************************************************/


void modeCheck()
{
  if ((mode_ == READY_MODE) && (digitalRead(BLUE_push) == LOW)) 
  {
    mode_ = TRAINING_MODE;
    trainig_start_time_ = fnGetCurrentTime();
    training_time = trainig_start_time_;
    mission_state_ = TRAINING_START;
    fnInitlevel();
    pbSensorState();
  }
  else if (((mode_ == TRAINING_MODE) && (digitalRead(RED_push) == LOW)) || stopWatchState_ == TRANINIG_TIMEOUT)  
  {
    mode_ = MATCH_MODE;
    stopWatchState_ = SETUP;
    match_start_time_ = fnGetCurrentTime();
    match_time = match_start_time_;
    fnGetRandomDelay();    
    mission_state_ = TRAINING_FINISH;
    fnInitlevel();
    pbSensorState();
  }

  else if(stopWatchState_ == MATCH_TIMEOUT)
  {
    mode_ = FINISH_MODE;
  }
}


void fnGetButtonPressed()
{
  if (ollo.read(4, TOUCH_SENSOR))
  {
    fnInitlevel();
    
    if (mode_ == READY_MODE)
    {
      if (led_color_ == LED_RED)          led_color_ = LED_YELLOW;
      else if (led_color_ == LED_YELLOW)  led_color_ = LED_GREEN;
      else if (led_color_ == LED_GREEN)   led_color_ = LED_RED;
      delay(100);
    }
    else if(mode_ == TRAINING_MODE)
    {
      training_time = fnGetCurrentTime();
      fnGetRandomDelay();
    }
    else fnInitStateTrafficLight();
  }
}

void fnInitStateTrafficLight()
{
  led_color_ = LED_RED;
  fnGetRandomDelay();
 
  fnSetStopWatch();
  fnInitlevel();
  sensor_distance[0] = 0;
  sensor_distance[1] = 0;
  sensor_distance[2] = 0;

  mode_ = READY_MODE;
  state_ = MISSION;
  mission_state_ = READY;
  stopWatchState_ = SETUP;
  mission_start = false;
  mission_trigger = false;
  stopwatch_trigger = false;
  trainig_start_time_ = 0.0;
  match_start_time_ = 0.0;
  training_time = 0.0;
  match_time = 0.0;
  light_loop_time_ = 0.0;
  fail_delay = 0.0;

  pbSensorState();
}

void fnReceiveSensorDistance()
{
  sensor_distance[0] = ollo.read(1);     // start line
  sensor_distance[1] = ollo.read(2);     // Sign 

//   Serial.print("DMS Sensor 1 ADC Value = ");
//   Serial.println(sensor_distance[0]); //read ADC value from OLLO port 1

  delay(50);
}

void fnCheckVehicleStatus()
{
  if (mode_ == MATCH_MODE)
  {
    if (sensor_distance[0] > DISTANCE_THRESHOLD_PASS && mission_trigger == false)
    {
      if (led_color_ == LED_GREEN)    state_ = PASS;
      
      else         
      {
        state_ = FAIL;
        fail_delay = (fnGetCurrentTime() - match_start_time_) - random_delay;
        
      }
      mission_trigger = true;

      if (mission_start == false)
      {
        mission_state_ = MATCH_START;          
        pbSensorState();
        mission_start = true;
      }
    }
  
    if (sensor_distance[0] > DISTANCE_THRESHOLD_PASS && stopwatch_trigger == true)
    {
      mission_state_ = MATCH_FINISH;
      pbSensorState();
      stopwatch_trigger = false;
    }

    if (sensor_distance[1] > DISTANCE_THRESHOLD_PASS)
    {
      stopwatch_trigger = true;
    }
  }
  
  if (sensor_distance[1] > DISTANCE_THRESHOLD_PASS && level_trigger == false)
  {
    if (random(2) == 0)      motorDriver.controlPosition(DXL_ID_1, DXL_1_UP);

    else                     motorDriver.controlPosition(DXL_ID_2, DXL_2_UP);

    level_trigger = true;
  }
}

void fnLEDControl()
{
  if (mode_ == TRAINING_MODE)
  {
    light_loop_time_ = fnGetCurrentTime() - training_time;

    if (light_loop_time_ <= RED_YELLOW_delay)                                          led_color_ = LED_RED;
    
    else if (light_loop_time_ > RED_YELLOW_delay && light_loop_time_ <= random_delay)  led_color_  = LED_YELLOW;

    else                                                                               led_color_  = LED_GREEN;
  }

  else if (mode_ == MATCH_MODE)
  {
    if (state_ == MISSION)
    {
      light_loop_time_ = fnGetCurrentTime() - match_time;
      
      if (light_loop_time_ <= RED_YELLOW_delay)                                                        led_color_ = LED_RED;
    
      else if (light_loop_time_ > RED_YELLOW_delay && light_loop_time_ <= random_delay)                led_color_  = LED_YELLOW;

      else if (light_loop_time_ > random_delay && light_loop_time_ <= random_delay + GREEN_RED_delay)
      {
        led_color_  = LED_GREEN;
        
        if (mission_start == false) 
        {
          mission_state_ = MATCH_START;          
          pbSensorState();
          mission_start = true;
        }
      }

      else 
      {
        match_time = fnGetCurrentTime();
        fnGetRandomDelay();
      }
    }

    else 
    {
      if (led_turn_ == 1)
      {
        if (state_ == FAIL)       led_color_ = LED_RED;
        
        else if (state_ == PASS)  led_color_ = LED_GREEN;        
      }
      
      else                        led_color_ = LED_ALL_LOW;

      if (millis()-pre_time >= 200) 
      {
        led_turn_ = 1 - led_turn_;
        pre_time = millis();
      }
    }
  }
}

void fnControlLED()
{
  if (led_color_ == LED_RED)
  {
    digitalWrite(9, HIGH);
    digitalWrite(10, LOW);
    digitalWrite(11, LOW);
  }
  else if (led_color_ == LED_YELLOW)
  {
    digitalWrite(9, LOW);
    digitalWrite(10, HIGH);
    digitalWrite(11, LOW);
  }
  else if (led_color_ == LED_GREEN)
  {
    digitalWrite(9, LOW);
    digitalWrite(10, LOW);
    digitalWrite(11, HIGH);
  }
  else
  {
    digitalWrite(9, LOW);
    digitalWrite(10, LOW);
    digitalWrite(11, LOW);
  }
}

double fnGetCurrentTime()
{
	return (double)millis();
}

void fnSetStopWatch()
{
  stopwatch_start_time_ = fnGetCurrentTime();
}

void fnGetRandomDelay()
{
  random_delay = random(3,11) * 1000 + RED_YELLOW_delay; // 3 ~ 10 sec + 3 sec
}

/*******************************************************************************
* Function level
*******************************************************************************/
void fnInitlevel()
{  
  level_trigger = false;
  motorDriver.controlPosition(DXL_ID_1, DXL_1_DOWN);
  motorDriver.controlPosition(DXL_ID_2, DXL_2_DOWN);
}

void resetCallback(const std_msgs::Bool& reset_msg)
{
  fnInitStateTrafficLight();
}

void stateCallback(const std_msgs::Int8& state_msg)
{
  switch(state_msg.data)
  {
    case 1:
    {
     stopWatchState_ = TRANINIG_TIMEOUT;
     break;
    }
    case 2:
    {
     stopWatchState_ = MATCH_TIMEOUT;
     break;      
    }
    default:
      break;
  }   
}

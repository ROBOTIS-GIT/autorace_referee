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
||---||            |---------|            |---------|            |---------|
|-----|            | O-----0 |            | O-----0 |            | O-----0 |
  | |              |   DMS   |            |   DMS   |            |   DMS   |
  | |              | Sensor1 |            | Sensor2 |            | Sensor3 |
  | |              |         |            |         |            |         |
=======------------===========------------===========------------===========

 created 1 October 2018
 by ROBOTIS CO,.LTD.

 author : Gilbert
 */

#include "traffic_stopwatch.h"

/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<rbiz_autorace_msgs::DoIt> subInitStateTrafficLight("init_state/traffic_light",cbInitStateTrafficLight);

/*******************************************************************************
* Publisher
*******************************************************************************/
// DMS, battery, etc.
rbiz_autorace_msgs::SensorStateTrafficLight msgSensorStateTrafficLight;
ros::Publisher pubSensorStateTrafficLight("sensor_state/traffic_light", &msgSensorStateTrafficLight);


/*******************************************************************************
* Set push button
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
  nh.subscribe(subInitStateTrafficLight);
  nh.advertise(pubSensorStateTrafficLight);

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

  pbSensorState();

  nh.spinOnce();
}

/*******************************************************************************
* Publish msgs (Sensor data: Power, etc.)
*******************************************************************************/
void pbSensorState()
{
  msgSensorStateTrafficLight.stamp = nh.now();
  msgSensorStateTrafficLight.elapsed_time = fnGetTimeSinceStart();
  msgSensorStateTrafficLight.sensor_distance[0] = sensor_distance[0];
  msgSensorStateTrafficLight.sensor_distance[1] = sensor_distance[1];
  msgSensorStateTrafficLight.sensor_distance[2] = sensor_distance[2];
//  msgSensorStateTrafficLight.is_started[0] = is_started[0];
//  msgSensorStateTrafficLight.is_started[1] = is_started[1];
//  msgSensorStateTrafficLight.is_started[2] = is_started[2];
//  msgSensorStateTrafficLight.is_able_to_pass = is_able_to_pass_;
//  msgSensorStateTrafficLight.vehicle_state = vehicle_state_;
  msgSensorStateTrafficLight.led_color = led_color_;
  msgSensorStateTrafficLight.battery = fncheckVoltage();

  pubSensorStateTrafficLight.publish(&msgSensorStateTrafficLight);
}

/*******************************************************************************
* Callback function
*******************************************************************************/

void cbInitStateTrafficLight(const rbiz_autorace_msgs::DoIt& msgInitStateTrafficLight)
{
  if (msgInitStateTrafficLight.doIt == true)
  {
    fnInitStateTrafficLight();
  }
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
  }
  else if ((mode_ == TRAINING_MODE) && (digitalRead(RED_push) == LOW))  
  {
    mode_ = MATCH_MODE;
    match_start_time_ = fnGetCurrentTime();
    match_time = match_start_time_;
    fnGetRandomDelay();    
  }

//  Serial.println(mode_);
}


void fnGetButtonPressed()
{
  if (ollo.read(4, TOUCH_SENSOR))
  {
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
  mission_trigger = false;
  fnSetStopWatch();
 
  sensor_distance[0] = 0;
  sensor_distance[1] = 0;
  sensor_distance[2] = 0;

  mode_ = READY_MODE;
  state_ = MISSION;
}

void fnTestStateTrafficLight()
{
  fnInitStateTrafficLight();
}

void fnReceiveSensorDistance()
{
  sensor_distance[0] = ollo.read(1);     // start line
  sensor_distance[1] = ollo.read(2);

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
      else                            state_ = FAIL;

      mission_trigger = true;
    }
  }
  // Serial.print("State : ");
  // Serial.println(vehicle_state_);
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

      else if (light_loop_time_ > random_delay && light_loop_time_ <= random_delay + GREEN_RED_delay)  led_color_  = LED_GREEN;

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
      
      led_turn_ = 1 - led_turn_;

      delay(200);
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

void fnGetRandomDelay()
{
  random_delay = random(3,11) * 1000 + RED_YELLOW_delay; // 3 ~ 10 sec + 3 sec
}
/*******************************************************************************
* Check voltage
*******************************************************************************/
float fncheckVoltage()
{
  float vol_value;

  vol_value = getPowerInVoltage();

  return vol_value;
}

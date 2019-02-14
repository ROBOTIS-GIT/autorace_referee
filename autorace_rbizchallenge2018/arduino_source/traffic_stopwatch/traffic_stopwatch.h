#include <OLLO.h>
#include "motor_driver.h"
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <rbiz_autorace_msgs/SensorStateStopwatch.h>
#include <rbiz_autorace_msgs/DoIt.h>

/*******************************************************************************
* Definitions
*******************************************************************************/
#define DISTANCE_THRESHOLD_PASS         500

#define DXL_ID_1                          1               // right sign
#define DXL_ID_2                          2               // left sign
#define DXL_1_UP           2048              // Dynamixel will rotate between this value
#define DXL_1_DOWN         3048              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_2_UP           2048              // (this is just for the test)
#define DXL_2_DOWN         1048              //

/*******************************************************************************
* Classes
*******************************************************************************/
ros::NodeHandle nh;

OLLO ollo;
MotorDriver motorDriver;


/*******************************************************************************
* Variables
*******************************************************************************/
// static uint16_t sensor_distance[3] = {0, 0, 0};
static uint16_t sensor_distance[3];// = {SENSOR_DISTANCE_INIT_VALUE, SENSOR_DISTANCE_INIT_VALUE, SENSOR_DISTANCE_INIT_VALUE};
static uint32_t pre_time;
double stopwatch_start_time_;
uint8_t led_turn_ = 1;

double trainig_start_time_;
double match_start_time_;

double training_time;
double match_time;
double light_loop_time_;
double random_delay;
double fail_delay;
bool mission_trigger;
bool mission_start;
bool level_trigger;
bool stopwatch_trigger;
/*******************************************************************************
* Enum
*******************************************************************************/
static enum State { MISSION, FAIL, PASS } state_;
static enum Color { LED_RED, LED_YELLOW, LED_GREEN, LED_ALL_LOW } led_color_;
static enum Mode  { READY_MODE, TRAINING_MODE, MATCH_MODE, FINISH_MODE } mode_;
static enum Mission_State { READY, TRAINING_START, TRAINING_FINISH, MATCH_START, MATCH_FINISH } mission_state_;
static enum Level { LEVEL_CLOSED, LEVEL_OPENED, LEVEL_MIDDLE } level_status_;
static enum Stopwatch_State {SETUP, TRANINIG_TIMEOUT, MATCH_TIMEOUT } stopWatchState_;
/*******************************************************************************
* Functions
*******************************************************************************/
void fnGetButtonPressed();
void fnReceiveSensorDistance();
void fnCheckVehicleStatus();
void fnInitlevel();
void fnInitStateStopwatch();

double fnGetCurrentTime();
void fnSetStopWatch();

/*******************************************************************************
* Publish function
*******************************************************************************/
void pbSensorState();
void resetCallback(const std_msgs::Bool& reset_msg);
void stateCallback(const std_msgs::Int8& state_msg);
/*******************************************************************************
* Callback function for InitStateTrafficLight msg
*******************************************************************************/
ros::Subscriber<std_msgs::Bool> reset_sub("reset", resetCallback);
ros::Subscriber<std_msgs::Int8> state_sub("state", stateCallback);

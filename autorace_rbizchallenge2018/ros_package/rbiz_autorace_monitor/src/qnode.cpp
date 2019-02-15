/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <sstream>
#include "../include/rbiz_autorace_monitor/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rbiz_autorace_monitor {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

// bool QNode::init() {
// 	ros::init(init_argc,init_argv,"rbiz_autorace_monitor");
// 	if ( ! ros::master::check() ) {
// 		return false;
// 	}
// 	ros::start(); // explicitly needed since our nodehandle is going out of scope.
// 	ros::NodeHandle n;
// 	// Add your ros communications here.
// 	// pubInitStateTrafficLight = n.advertise<std_msgs::String>("init_state/traffic_light", 1);
// 	pubInitStateTrafficLight = n.advertise<rbiz_autorace_msgs::DoIt>("init_state/traffic_light", 1);
//
// 	start();
// 	return true;
// }

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"rbiz_autorace_monitor");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
    //ros::Publisher pub_reset;

	// Publisher
	pubInitStateTrafficLight = n.advertise<rbiz_autorace_msgs::DoIt>("init_state/traffic_light", 1);
	pubInitStateLevelCrossing = n.advertise<rbiz_autorace_msgs::DoIt>("init_state/level_crossing", 1);
	pubInitStateStopwatch = n.advertise<rbiz_autorace_msgs::DoIt>("init_state/stopwatch", 1);

	pubTestStateTrafficLight = n.advertise<rbiz_autorace_msgs::DoIt>("test_state/traffic_light", 1);
	pubTestStateLevelCrossing = n.advertise<rbiz_autorace_msgs::DoIt>("test_state/level_crossing", 1);
    pub_level = n.advertise<std_msgs::Bool>("/level", 1);
    pub_reset = n.advertise<std_msgs::Bool>("/reset", 1);
    pub_state = n.advertise<std_msgs::Int8>("/state", 1);

	// Subscriber
	subSensorStateStopwatch = n.subscribe("sensor_state/stopwatch", 1, &QNode::cbReceiveSensorStateStopwatch, this);

	start();
	return true;
}

void QNode::run() {
        ros::Rate loop_rate(100);
	// int count = 0;
	while ( ros::ok() ) {

		// std_msgs::String msg;
		// std::stringstream ss;
		// ss << "hello world " << count;
		// msg.data = ss.str();
		// // pubInitStateTrafficLight.publish(msg);
		//
		// log(Info,std::string("I sent: ")+msg.data);

                process();

		ros::spinOnce();
		loop_rate.sleep();
		// ++count;
	}
	// std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

// Publish Functions

void QNode::cbReceiveSensorStateStopwatch(const rbiz_autorace_msgs::SensorStateStopwatch msgSensorStateStopwatch)
{
//	lap_time = msgSensorStateStopwatch.lap_time;
//    mission_time = msgSensorStateStopwatch.elapsed_time;

    if (msgSensorStateStopwatch.vehicle_state == 0)
    {
        stopwatchStatus = STAY;
        level_trigger = true;
    }

    else if (msgSensorStateStopwatch.vehicle_state == 1)
    {
       stopwatchStatus = READY;
    }

    else if (msgSensorStateStopwatch.vehicle_state == 2)
    {
        stopwatchStatus = START;

        start_trainging_time_ = ros::Time::now();

        if (level_trigger == true)
        {
            std_msgs::Bool level_msg;
            pub_level.publish(level_msg);
            level_trigger = false;
        }
    }

    else if (msgSensorStateStopwatch.vehicle_state == 3)
    {
        stopwatchStatus = MISSION;
        start_mission_time_ = ros::Time::now();
    }

    else if (msgSensorStateStopwatch.vehicle_state == 4)
    {
        stopwatchStatus = FINISH;
    }

    // Q_EMIT fnUpdateReadyLap();

	// std_msgs::String msg;
	// std::stringstream ss;
	// ss << "hello world " << msgSensorStateStopwatch.lap_time;
	// msg.data = ss.str();
	// // pubInitStateTrafficLight.publish(msg);
	//
	// log(Info,std::string("I sent: ")+msg.data);
}

void QNode::pbResetMsg()
{
    std_msgs::Bool reset_msg;
    reset_msg.data = true;
    stopwatchStatus = STAY;
    pub_reset.publish(reset_msg);
}

void QNode::pbStateMsg(int state)
{
    std_msgs::Int8 state_msg;
    state_msg.data = state;
    pub_state.publish(state_msg);
}

void QNode::log( const LogLevel &level, const std::string &msg) {
	// logging_model.insertRows(logging_model.rowCount(),1);
	// std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				// logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				// logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				// logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				// logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				// logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	// QVariant new_row(QString(logging_model_msg.str().c_str()));
	// logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	// Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

// void QNode::fnPassLapTime(double lap_time)
// {
// 	Q_EMIT loggingUpdated();
// }

void QNode::process()
{

    switch(stopwatchStatus)
    {



        case STAY:
        {
            start_trainging_time_ = ros::Time::now();
            Q_EMIT resetTrainingTime();
            break;
        }
        case READY:
        {
            ros::Time current_time = ros::Time::now();

            ros::Duration dur_1 = current_time - start_trainging_time_;
            ros::Duration dur = ros::Duration(10 * 60) - dur_1;
            int min = dur.sec / 60;
            int sec = dur.sec % 60;
            int m_sec = dur.nsec / 10e6;

            Q_EMIT setTrainingTime(min, sec, m_sec);

            if (dur <= ros::Duration(0))
            {
                stopwatchStatus = START;
                pbStateMsg(1);
            }

            break;
        }

        case START:
            Q_EMIT readyMissionTime();
            break;

        case MISSION:
        {
            ros::Time mission_time = ros::Time::now();
            ros::Duration dur = mission_time - start_mission_time_;

            int min = dur.sec / 60;
            int sec = dur.sec % 60;
            int m_sec = dur.nsec / 10e6;

            Q_EMIT startMissionTime(min, sec, m_sec);
            if (dur.sec >= 60 * 5)     // time out 5 min
            {
                stopwatchStatus = TIMEOUT;
            }
            break;
        }
        case FINISH:
            Q_EMIT finishMission();
            break;

        case TIMEOUT:
        {
            Q_EMIT timeOut();
            break;
        }
        default:
            break;
    }
}


}  // namespace rbiz_autorace_monitor

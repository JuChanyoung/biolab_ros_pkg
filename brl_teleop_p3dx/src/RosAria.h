//--------------- ROS --------------//
#include <ros/ros.h>
#include <Aria.h>
#include <brl_teleop_msgs/Force.h>
#include <brl_teleop_msgs/Pos.h>
#include <brl_teleop_msgs/Vel.h>
#include <brl_teleop_msgs/Energy.h>
#include <brl_teleop_msgs/Common.h>
#include <brl_teleop_msgs/Package.h>
//-------------- Common -----------//
#include <iostream>
#include <functional>
#include <algorithm>
#include <stdlib.h>
#include <vector>
#include <fstream>

const int32_t PUBLISH_RATE = 1000; // 1kHz

using namespace std;
using namespace ros;

class RosAria
{
public:
	// Ros variables
	ros::NodeHandlePtr node_;
	int32_t publish_rate_;
	int32_t time_delay_;

	// Device
	ArRobot *robot_;		// Robot instance
	ArSonarDevice sonar_sensor_;	// 	To communicate and read sonar sensor data
	ArPose Position_;				//	To read actual position of robot
	ArSimpleConnector *communication_;	//	To communicate with robot through serial port
	std::string serial_port_;
	// Slave
	double actual_velocity_;
	double desire_velocity_;
	double actual_position_;
	// Subsriber & Publisher
	string desire_velocity_src_name_;  			  	//  Source for subscribing
	string desire_velocity_topic_name_;		//	Topic for subscribing

	Subscriber velocity_sub_;
	Publisher  velocity_pub_;
	Publisher  position_pub_;


	// Methods
	RosAria(): 
		publish_rate_(PUBLISH_RATE)
	{
		init();
	}
	~RosAria(){}

	void init()
	{
		// Ros Node
		node_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));
			std::cout << "Starting with the following parameters:" << std::endl;

		// Settings
		node_->param(std::string("publish_rate"), publish_rate_, PUBLISH_RATE);
			std::cout << "\t" << "publish_rate = " << publish_rate_ << std::endl;
		// This program will communicate with p3dx through serial port at /dev/ttyUSB0
		node_->param( "port", serial_port_, std::string("/dev/ttyUSB0") );
		// Subscriber settings
		node_->param(std::string("desire_velocity_src_name"), desire_velocity_src_name_, std::string(""));
		if(!desire_velocity_src_name_.empty())
		{
			node_->param(std::string("desire_velocity_topic_name"), desire_velocity_topic_name_, std::string("force"));
			std::cout << "\t" << "desire_velocity_src_name = " << desire_velocity_src_name_ << std::endl;

			velocity_sub_ = node_->subscribe<brl_teleop_msgs::Package>(desire_velocity_src_name_ + std::string("/") + desire_velocity_topic_name_, 100,
								&RosAria::desireVelocityCallback,this);
		}
		// Publishing settings
		velocity_pub_ = node_->advertise<brl_teleop_msgs::Vel>("velocity", 100);
		position_pub_ = node_->advertise<brl_teleop_msgs::Vel>("position_", 100);
	}
	void desireVelocityCallback(const brl_teleop_msgs::PackageConstPtr& msg)
	{
		desire_velocity_ = msg->x;
		robot_->setVel(desire_velocity_);
	}

	void velocityPublish()
	{
		brl_teleop_msgs::Vel vel_out;

		vel_out.header.frame_id = ros::this_node::getName();
		vel_out.header.stamp    = ros::Time::now();

		actual_velocity_ = robot_->getVel();
		vel_out.deltaX = actual_velocity_;
		velocity_pub_.publish(vel_out);
	}
	
	void positionPublish()
	{
		brl_teleop_msgs::Pos pos_out;

		pos_out.header.frame_id = ros::this_node::getName();
		pos_out.header.stamp    = ros::Time::now();

		Position_ = robot_->getPose();
		actual_position_ = Position_.getX();

		pos_out.x = actual_position_;
		position_pub_.publish(pos_out);
	}



int Setup()
{
	ArArgumentBuilder *args;

	args = new ArArgumentBuilder();
	args->add("-rp"); //pass robot's serial port to Aria
	args->add(serial_port_.c_str());
	communication_ = new ArSimpleConnector(args);


	robot_ = new ArRobot();
	robot_->setCycleTime(1);

	ArLog::init(ArLog::File, ArLog::Verbose, "aria.log", true);

	//parse all command-line arguments
	if (!Aria::parseArgs())
	{
		Aria::logOptions();
		return 1;
	}

	// Connect to the robot
	if (!communication_->connectRobot(robot_)) {
		ArLog::log(ArLog::Terse, "rotate: Could not connect to robot! Exiting.");
		return 1;
	}

	//Sonar sensor
	robot_->addRangeDevice(&sonar_sensor_);
	robot_->enableSonar();

	// Enable the motors
	robot_->enableMotors();

	robot_->runAsync(true);

	return 0;
}

};

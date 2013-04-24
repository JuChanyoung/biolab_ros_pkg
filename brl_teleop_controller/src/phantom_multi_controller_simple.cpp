//============================================================================
// Name        : main.cpp
// Author      : Haquang
// Version     : 1.0
// Copyright   : Your copyright notice
// Description : This is a general program for multilaterla teleoperation system
//				 which can be configured for master/slave. The configuration parameters
//				 are loaded from launch file.
//				 
//============================================================================

//--------------- ROS --------------//
#include <ros/ros.h>
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

// ------------------------------------ //
const int32_t PUBLISH_RATE = 1000; // 1kHz
const int32_t TIME_DELAY = 1;	// No time delay
FILE *data;

typedef std::vector<double> vector3D; 

class controller
{
public:
	// Variable

	bool master1_;									//  Master 1 Controller
	bool master2_;									//  Master 2 Controller
	bool slave_;									//	Slave Controller
	float alpha_;									//	Scale
	
	// Device Variables
	vector3D velocity_actual_;					//	Actual Velocity
	vector3D position_actual_;					// 	Actual Position
	vector3D velocity_desired_;					//	Desired Velocity
	// Communication variables
	vector3D master1_velocity_;					// 	Velocity of Master 1 
	vector3D master2_velocity_;					//	Velocity of Master 2
	vector3D position_desire_;					//	Design Position
	vector3D force_ctrl_;						// 	Control force (in case of slave/master 2 device)
	vector3D force_feedback_master_;			// 	Force feedback from master
	vector3D force_feedback_slave_;				//	Force feebdack from slave
	
	// Ros variables
	ros::NodeHandlePtr node_;
	int32_t publish_rate_;
	int32_t time_delay_;
	// Subscriber
	std::string actual_velocity_src_name_;				//	Source for actual velocity from device
	std::string actual_velocity_topic_name_;			//	Topic for actual velocity from device

	std::string velocity_master_master_src_name_;    	//  Source for velocity command from master to master
	std::string velocity_master_master_topic_name_;		//	Topic for velocity command from master to master

	std::string force_master_master_src_name_;    		//  Source for force command from master to master
	std::string force_master_master_topic_name_;		//	Topic for force command from master to master

	std::string velocity_master1_slave_src_name_;   	//  Source for velocity command from master1 to slave
	std::string velocity_master1_slave_topic_name_;		//	Topic for force command from master1 to master

	std::string velocity_master2_slave_src_name_;   	//  Source for velocity command from master2 to slave
	std::string velocity_master2_slave_topic_name_;		//	Topic for force command from master2 to master

	std::string force_slave_master_src_name_;    		//  Source for force command from slave to master1
	std::string force_slave_master_topic_name_;			//	Topic for force command from slave to master1

	ros::Subscriber velocity_master_master_sub_;
	ros::Subscriber velocity_master1_slave_sub_;
	ros::Subscriber velocity_master2_slave_sub_;

	ros::Subscriber force_master_master_sub_;
	ros::Subscriber force_slave_master_sub_;
	
	ros::Publisher velocity_master_master_pub_;
	ros::Publisher velocity_master_slave_pub_;
	ros::Subscriber actual_velocity_sub_;
	
	ros::Publisher force_master_master_pub_;
	ros::Publisher force_slave_master1_pub_;
	ros::Publisher force_slave_master2_pub_;

	// PD Controller
	double Kp_;
	double Kd_;
	
	double dT_;			
	double beta_;

	// Method
	controller()
	{
		master1_ = false;
		master2_ = false;
		slave_ = false;
		alpha_ = 0.5;
		init();
	}

	double PdCompute(const double &Xr, const double &X,const double &velocity,const double &Kp,const double &Kd)
	{
		double pdVal,error;
		error = Xr - X; 					
		pdVal = Kp*error - Kd*velocity;
		return pdVal;
	}
	void actualVelocityCallback(const brl_teleop_msgs::VelConstPtr& msg)
	{
		velocity_actual_[0] = msg->deltaX; 		// Actual velocity
		velocity_actual_[1] = msg->deltaY;		// Actual velocity
		velocity_actual_[2] = msg->deltaZ;		// Actual velocity

		position_actual_[0] += velocity_actual_[0];
		position_actual_[1] += velocity_actual_[1];
		position_actual_[2] += velocity_actual_[2];
	}
	void velocityMasterToMasterCallback(const brl_teleop_msgs::PackageConstPtr& pkg)
	{
		velocity_desired_[0] = pkg->x;
		velocity_desired_[1] = pkg->y;
		velocity_desired_[2] = pkg->z;

		position_desire_[0] += velocity_desired_[0];
		position_desire_[1] += velocity_desired_[1];
		position_desire_[2] += velocity_desired_[2];

		for (int i = 0; i < 3; i++)
			force_ctrl_[i] = PdCompute(position_desire_[i],position_actual_[i],velocity_actual_[i],Kp_,Kd_);
	}
	
	void forceMasterToMasterCallback(const brl_teleop_msgs::PackageConstPtr& pkg)
	{
		force_feedback_master_[0] = -pkg->fx;
		force_feedback_master_[1] = -pkg->fy;
		force_feedback_master_[2] = -pkg->fz;
	}

	void velocityMaster1ToSlaveCallback(const brl_teleop_msgs::PackageConstPtr& pkg)
	{
		master1_velocity_[0] = alpha_*pkg->x;
		master1_velocity_[1] = alpha_*pkg->y;
		master1_velocity_[2] = alpha_*pkg->z;

		position_desire_[0] += master1_velocity_[0];
		position_desire_[1] += master1_velocity_[1];
		position_desire_[2] += master1_velocity_[2];

		for (int i = 0; i < 3; i++)
			force_ctrl_[i] = PdCompute(position_desire_[i],position_actual_[i],velocity_actual_[i],Kp_,Kd_);
	}
	void velocityMaster2ToSlaveCallback(const brl_teleop_msgs::PackageConstPtr& pkg)
	{
		master2_velocity_[0] = alpha_*pkg->x;
		master2_velocity_[1] = alpha_*pkg->y;
		master2_velocity_[2] = alpha_*pkg->z;

		position_desire_[0] += master2_velocity_[0];
		position_desire_[1] += master2_velocity_[1];
		position_desire_[2] += master2_velocity_[2];

		for (int i = 0; i < 3; i++)
			force_ctrl_[i] = PdCompute(position_desire_[i],position_actual_[i],velocity_actual_[i],Kp_,Kd_);
	}
	void forceSlaveToMasterCallback(const brl_teleop_msgs::PackageConstPtr& pkg)
	{
		force_feedback_slave_[0] = -pkg->fx;
		force_feedback_slave_[1] = -pkg->fy;
		force_feedback_slave_[2] = -pkg->fz;
	}

	void velocityMasterToMasterPublish()
	{
		pkg_mst_mst_[0].header.frame_id = ros::this_node::getName();
		pkg_mst_mst_[0].header.stamp    = ros::Time::now();

		velocity_master_master_pub_.publish(pkg_mst_mst_[0]);
	}

	void forceMasterToMasterPublish()
	{
		pkg_mst_mst_[0].header.frame_id = ros::this_node::getName();
		pkg_mst_mst_[0].header.stamp    = ros::Time::now();

		force_master_master_pub_.publish(pkg_mst_mst_[0]);
	}

	void velocityMasterToSlavePublish()
	{
		pkg_mst_slv_[0].header.frame_id = ros::this_node::getName();
		pkg_mst_slv_[0].header.stamp    = ros::Time::now();
	
		velocity_master_slave_pub_.publish(pkg_mst_slv_[0]);
	}
	
	void forceSlaveToMaster1Publish()
	{
		pkg_slv_mst1_[0].header.frame_id = ros::this_node::getName();
		pkg_slv_mst1_[0].header.stamp    = ros::Time::now();

		force_slave_master1_pub_.publish(pkg_slv_mst1_[0]);
	}

	void forceSlaveToMaster2Publish()
	{
		pkg_slv_mst2_[0].header.frame_id = ros::this_node::getName();
		pkg_slv_mst2_[0].header.stamp    = ros::Time::now();
		
		force_slave_master2_pub_.publish(pkg_slv_mst2_[0]);
	}
	void forceToDevice()
	{
		brl_teleop_msgs::Package pkg_out;

		pkg_out.header.frame_id = ros::this_node::getName();
		pkg_out.header.stamp    = ros::Time::now();
		
		if (popc_enable_)
		{
			if (master1_)
			{
				force_to_device_[0] = force_feedback_master_[0]+ force_feedback_slave_[0];
				force_to_device_[1] = force_feedback_master_[1]+ force_feedback_slave_[1];
				force_to_device_[2] = force_feedback_master_[2]+ force_feedback_slave_[2];
			}
			if (master2_)
				{
					force_to_device_[0] = force_ctrl_[0]+force_feedback_slave_[0];
					force_to_device_[1] = force_ctrl_[1]+force_feedback_slave_[1];
					force_to_device_[2] = force_ctrl_[2]+force_feedback_slave_[2];
				}
			if (slave_)
				force_to_device_ = force_ctrl_;
		}
		else
		{
			if (master1_)
			{
				force_to_device_[0] = force_feedback_master_[0] + force_feedback_slave_[0]; 
				force_to_device_[1] = force_feedback_master_[1] + force_feedback_slave_[1];
				force_to_device_[2] = force_feedback_master_[2] + force_feedback_slave_[2];
			}

			if (master2_)
			{
				force_to_device_[0] = force_ctrl_[0] + force_feedback_slave_[0];
				force_to_device_[1] = force_ctrl_[1] + force_feedback_slave_[1];
				force_to_device_[2] = force_ctrl_[2] + force_feedback_slave_[2];
			}

			if (slave_)
				force_to_device_ = force_ctrl_;
		}

		pkg_out.fx  = force_to_device_[0];
		pkg_out.fy  = force_to_device_[1];
		pkg_out.fz  = force_to_device_[2];	

		force_popc_pub_.publish(pkg_out);
	}
	void init()
	{
		// Ros Node
		node_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));
			std::cout << "Starting with the following parameters:" << std::endl;
		// Settings
		node_->param(std::string("publish_rate"), publish_rate_, PUBLISH_RATE);
			std::cout << "\t" << "publish_rate = " << publish_rate_ << std::endl;
		node_->param(std::string("master1"), master1_, false);
					std::cout << "\t" << "master 1 = " << master1_ << std::endl;
		node_->param(std::string("master2"), master2_, false);
					std::cout << "\t" << "master 2 = " << master2_ << std::endl;
		node_->param(std::string("slave"), slave_, false);
					std::cout << "\t" << "slave = " << slave_ << std::endl;
		node_->param(std::string("popc_enable"), popc_enable_, false);
					std::cout << "\t" << "popc_enable = " << popc_enable_ << std::endl;
		// Time delay
		node_->param(std::string("time_delay"), time_delay_, TIME_DELAY);
			std::cout << "\t" << "time_delay = " << time_delay_ << std::endl;
		// Subscriber settings
		// actual velocity
		node_->param(std::string("actual_velocity_src_name"), actual_velocity_src_name_, std::string(""));
		if (!actual_velocity_src_name_.empty())
		{
			node_->param(std::string("actual_velocity_topic_name"), actual_velocity_topic_name_, std::string("velocity"));
			std::cout << "\t" << "actual_velocity_topic_name = " << actual_velocity_topic_name_ << std::endl;

			actual_velocity_sub_ = node_->subscribe<brl_teleop_msgs::Vel>(actual_velocity_src_name_ + std::string("/") +
					actual_velocity_topic_name_, 100,&controller::actualVelocityCallback,this);
		}
		// Velocity Master - Master
		node_->param(std::string("velocity_master_master_src_name"), velocity_master_master_src_name_, std::string(""));
		if(!velocity_master_master_src_name_.empty())
		{
			node_->param(std::string("velocity_master_master_topic_name"), velocity_master_master_topic_name_, std::string("velocity_master_master"));
			std::cout << "\t" << "velocity_master_master_topic_name = " << velocity_master_master_topic_name_ << std::endl;

			velocity_master_master_sub_ = node_->subscribe<brl_teleop_msgs::Package>(velocity_master_master_src_name_ + std::string("/") + velocity_master_master_topic_name_, 100,
								&controller::velocityMasterToMasterCallback,this);
		}
		// Force Master - Master
		node_->param(std::string("force_master_master_src_name"), force_master_master_src_name_, std::string(""));
		if(!force_master_master_src_name_.empty())
		{
			node_->param(std::string("force_master_master_topic_name"), force_master_master_topic_name_, std::string("force_master_master"));
			std::cout << "\t" << "force_master_master_topic_name = " << force_master_master_topic_name_ << std::endl;

			force_master_master_sub_ = node_->subscribe<brl_teleop_msgs::Package>(force_master_master_src_name_ + std::string("/") + force_master_master_topic_name_, 100,
								&controller::forceMasterToMasterCallback,this);
		}

		// Velocity Master1 - Slave
		node_->param(std::string("velocity_master1_slave_src_name"), velocity_master1_slave_src_name_, std::string(""));
		if(!velocity_master1_slave_src_name_.empty())
		{
			node_->param(std::string("velocity_master1_slave_topic_name"), velocity_master1_slave_topic_name_, std::string("velocity_master_slave"));
			std::cout << "\t" << "velocity_master1_slave_topic_name = " << velocity_master1_slave_topic_name_ << std::endl;

			velocity_master1_slave_sub_ = node_->subscribe<brl_teleop_msgs::Package>(velocity_master1_slave_src_name_ + std::string("/") + velocity_master1_slave_topic_name_, 100,
								&controller::velocityMaster1ToSlaveCallback,this);
		}
		// Velocity Master2 - Slave
		node_->param(std::string("velocity_master2_slave_src_name"), velocity_master2_slave_src_name_, std::string(""));
		if(!velocity_master2_slave_src_name_.empty())
		{
			node_->param(std::string("velocity_master2_slave_topic_name"), velocity_master2_slave_topic_name_, std::string("velocity_master_slave"));
			std::cout << "\t" << "velocity_master2_slave_topic_name = " << velocity_master2_slave_topic_name_ << std::endl;

			velocity_master2_slave_sub_ = node_->subscribe<brl_teleop_msgs::Package>(velocity_master2_slave_src_name_ + std::string("/") + velocity_master2_slave_topic_name_, 100,
								&controller::velocityMaster2ToSlaveCallback,this);
		}

		// Force Slave - Master
		node_->param(std::string("force_slave_master_src_name"), force_slave_master_src_name_, std::string(""));
		if(!force_slave_master_src_name_.empty())
		{
			node_->param(std::string("force_slave_master_topic_name"), force_slave_master_topic_name_, std::string("force_slave_master"));
			std::cout << "\t" << "force_slave_master_topic_name = " << force_slave_master_topic_name_ << std::endl;

			force_slave_master_sub_ = node_->subscribe<brl_teleop_msgs::Package>(force_slave_master_src_name_ + std::string("/") + force_slave_master_topic_name_, 100,
								&controller::forceSlaveToMasterCallback,this);
		}
		
		//PID
		node_->param(std::string("gains/P"), Kp_, 0.1);
		node_->param(std::string("gains/D"), Kd_, 0.01);

		std::cout << "\t" << "Gains = { P: " << Kp_ << ", D: " << Kd_ << "}" << std::endl;

		// Publishing settings
		velocity_master_master_pub_ = node_->advertise<brl_teleop_msgs::Package>("velocity_master_master", 100);
		velocity_master_slave_pub_ = node_->advertise<brl_teleop_msgs::Package>("velocity_master_slave", 100);
		
		force_master_master_pub_ = node_->advertise<brl_teleop_msgs::Package>("force_master_master", 100);
		force_slave_master1_pub_ = node_->advertise<brl_teleop_msgs::Package>("force_slave_master1", 100);
		force_slave_master2_pub_ = node_->advertise<brl_teleop_msgs::Package>("force_slave_master2", 100);
		
		force_popc_pub_ = node_->advertise<brl_teleop_msgs::Package>("force_popc", 100);
	}
};

// ------------------ ROS Callback ------------------------

void *ros_publish(void *ptr)
{
   controller *Controller = (controller*) ptr;
   ros::Rate loop_rate(Controller->publish_rate_);
   ros::AsyncSpinner spinner(2);
   spinner.start();

   while(ros::ok())
   {
	   if (Controller->master1_)
	   {
		   	Controller->velocityMasterToMasterPublish();
		   	Controller->velocityMasterToSlavePublish();
		   	Controller->forceToDevice();
	   }

	   if (Controller->master2_)
	   {
	   		Controller->velocityMasterToSlavePublish();
	   		Controller->forceMasterToMasterPublish();
			Controller->forceToDevice();
	   }
	   if (Controller->slave_)
	   {
			Controller->forceSlaveToMaster1Publish();
		   	Controller->forceSlaveToMaster2Publish();
		 	Controller->forceToDevice();  	
	   }
       loop_rate.sleep();
   }
   return NULL;
}

//--------- Main Program -----------//
int main(int argc, char** argv)
{
	//-- Init ROS node
	ros::init(argc, argv, "controller");

	controller Controller;

	//-- Threads
	pthread_t publish_thread;
	pthread_create(&publish_thread,NULL,ros_publish,(void*) &Controller);
	pthread_join(publish_thread,NULL);


	ROS_INFO("Ending Session...\n");
}



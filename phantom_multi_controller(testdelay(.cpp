//============================================================================
// Name        : main.cpp
// Author      : Haquang
// Version     : 1.0
// Copyright   : Your copyright notice
// Description : This is phantom controller, using PD controller, with/without
//				 Time Delay Power Network and Time Domain Passivity Approach to
//				 stabilize system under time delay.
//				 This is source code for multi master single slave experiment 
//				 with three phantom devices in position control mode
//				 Two Master devices are coupled by position-force architecture
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
//-------------- Haptic device----------//
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>

const int32_t PUBLISH_RATE = 1000; // 1kHz
const int32_t TIME_DELAY = 1;	// No time delay
FILE *data;

class controller
{
public:
	// Variable

	bool master1_;									//  Master 1 Controller
	bool master2_;									//  Master 2 Controller
	bool slave_;									//	Slave Controller
	bool popc_enable_;

	// Device Variables
	hduVector3Dd velocity_actual_;					//	Actual Velocity
	hduVector3Dd position_actual_;					// 	Actual Position
	hduVector3Dd velocity_design_;					//	Actual Velocity
	// Communication variables
	hduVector3Dd master1_velocity_;					// 	Velocity of Master 1 
	hduVector3Dd master2_velocity_;					//	Velocity of Master 2
	hduVector3Dd position_design_;					//	Design Position
	hduVector3Dd force_ctrl_;						// 	Control force (in case of slave/master 2 device)
	hduVector3Dd force_feedback_master_;			// 	Force feedback from master
	hduVector3Dd force_feedback_slave_;				//	Force feebdack from slave
	hduVector3Dd force_ctrl_popc_;					//	Force after PoPc
	hduVector3Dd force_feedback_master_popc_;		// 	Force feedback from master
	hduVector3Dd force_feedback_slave_popc_;		//	Force feebdack from slave
	
	hduVector3Dd energy_reference_;					//	Reference Energy
	hduVector3Dd energy_mst_mst_input_;				//	Energy input from master to master
	hduVector3Dd energy_mst_mst_output_;			//	Energy output from master to master
	hduVector3Dd energy_mst_slv_intput_;			//	Energy input from master to slave
	hduVector3Dd energy_mst1_slv_output_;			//	Energy output from master 1 to slave
	hduVector3Dd energy_mst2_slv_output_;			//	Energy output from master 2 to slave
	hduVector3Dd energy_slv_mst1_input_;			//	Energy input from slave to master 1
	hduVector3Dd energy_slv_mst2_input_;			//	Energy int from slave to master 2
	hduVector3Dd energy_slv_mst_output_;			//	Energy output from slave to master
	hduVector3Dd force_to_device_;
	// Ros variables
	ros::NodeHandlePtr node_;
	int32_t publish_rate_;
	int32_t time_delay_;
	// Subsriber
	std::string actual_velocity_src_name_;
	std::string actual_velocity_topic_name_;

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

	ros::Publisher force_popc_pub_;		// To device
	
	// Delay variable
//	brl_teleop_msgs::Package* pkg_mst_mst_;		//	From master to master
//	brl_teleop_msgs::Package* pkg_mst_slv_;		//	From master to slave
//	brl_teleop_msgs::Package* pkg_slv_mst1_;	//	From slave to master 1
//	brl_teleop_msgs::Package* pkg_slv_mst2_;	//	From slave  to master 2
	// PD Controller
	double Kp_;
	double Kd_;
	
	// Mass - Spring - Damper
	double K_master_; // Mass - Spring - Damper
	double Mss_;
	hduVector3Dd Acc_;	// Acceleration
	hduVector3Dd Vm_;	// Velocity
	hduVector3Dd Xm_;	// Position

	double curTime_;
	double prvTime_;
	double dT_;			
	double beta_;

	// Method
	controller()
	{
		master1_ = false;
		master2_ = false;
		slave_ = false;
		popc_enable_=false;
		// energy_input_.set(0.0f,0.0f,0.0f);
		// energy_output_.set(0.0f,0.0f,0.0f);
		// energy_reference_.set(0.0f,0.0f,0.0f);

		Acc_.set(0.0f,0.0f,0.0f);
		Vm_.set(0.0f,0.0f,0.0f);
		Xm_.set(0.0f,0.0f,0.0f);
		K_master_ = 1;
		Mss_ = 0.0001;
		curTime_ = 0;
		prvTime_ = 0;
		dT_ = 0;
		beta_ = 2 *Mss_ *sqrt(K_master_/Mss_);
		// init
		init();
		
//		pkg_mst_slv_ = new brl_teleop_msgs::Package[time_delay_];
//		pkg_mst_mst_ = new brl_teleop_msgs::Package[time_delay_];
	
//		pkg_slv_mst1_ = new brl_teleop_msgs::Package[time_delay_];
//		pkg_slv_mst2_ = new brl_teleop_msgs::Package[time_delay_];
	}

	void MassSpringDamper(hduVector3Dd &force,const hduVector3Dd &pos,const hduVector3Dd &vel)
	{
		for (int i = 0;i < 3; i++)
		{
			Acc_[i] = (-K_master_*(Xm_[i] - pos[i]) - beta_*(Vm_[i] - vel[i]/dT_) + force[i])/Mss_;
			Vm_[i] += Acc_[i]*dT_;
			Xm_[i] += Vm_[i]*dT_;
			force[i] = K_master_*(Xm_[i] - pos[i]);
		}
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

		if ((master1_ || master2_))
		{
			for (int i=0;i<3;i++)
			{
				if (master1_)
				{
					if (velocity_actual_[i] * force_feedback_master_[i] < 0)
						energy_mst_mst_input_[i] -= velocity_actual_[i] * force_feedback_master_[i];	
				}
				
				if (velocity_actual_[i] * force_feedback_slave_[i] < 0)
					energy_mst_slv_intput_[i] -= velocity_actual_[i] * force_feedback_slave_[i];
			}
		}
	}
	void velocityMasterToMasterCallback(const brl_teleop_msgs::PackageConstPtr& pkg)
	{
		velocity_design_[0] = pkg->x;
		velocity_design_[1] = pkg->y;
		velocity_design_[2] = pkg->z;

		position_design_[0] += velocity_design_[0];
		position_design_[1] += velocity_design_[1];
		position_design_[2] += velocity_design_[2];

		energy_reference_[0] = pkg->Ex;
		energy_reference_[1] = pkg->Ey;
		energy_reference_[2] = pkg->Ez;

		for (int i = 0; i < 3; i++)
		{
			force_ctrl_[i] = PdCompute(position_design_[i],position_actual_[i],velocity_actual_[i],Kp_,Kd_);

			// Passivity Observer
			if (force_ctrl_[i]*velocity_design_[i] < 0)
				energy_mst_mst_input_[i] -= force_ctrl_[i] * velocity_design_[i];
			if (force_ctrl_[i]*velocity_design_[i] > 0)
				energy_mst_mst_output_[i] -= force_ctrl_[i] * velocity_design_[i];
			// Passivity Controller
			if (popc_enable_)
			{
				if ((energy_reference_[i] + energy_mst_mst_output_[i]<0) && (force_ctrl_[i]!=0))
				{
					// Backward 1 step
					energy_mst_mst_output_[i] += force_ctrl_[i] * velocity_design_[i];
					position_design_[i] -= velocity_design_[i];
					// Modify veloctiy
					velocity_design_[i] = (energy_reference_[i] + energy_mst_mst_output_[i])/force_ctrl_[i];
					// Update 1 step
					position_design_[i] += velocity_design_[i];	
					force_ctrl_popc_[i] = PdCompute(position_design_[i],position_actual_[i],velocity_actual_[i],Kp_,Kd_);				
					energy_mst_mst_output_[i] -= force_ctrl_popc_[i] * velocity_design_[i];
				}
				else
					force_ctrl_popc_[i] = force_ctrl_[i];	
			}
		}
	}
	
	void forceMasterToMasterCallback(const brl_teleop_msgs::PackageConstPtr& pkg)
	{
		force_feedback_master_[0] = -pkg->fx;
		force_feedback_master_[1] = -pkg->fy;
		force_feedback_master_[2] = -pkg->fz;

		energy_reference_[0] = pkg->Ex;
		energy_reference_[1] = pkg->Ey;
		energy_reference_[2] = pkg->Ez;

		for (int i = 0; i < 3; i++)
		{
			//	Passivity Observer
			if (force_feedback_master_[i] * velocity_actual_[i] > 0)
				energy_mst_mst_output_[i] -= force_feedback_master_[i] * velocity_actual_[i];
			//	Passivity Controller
			if (popc_enable_)
			{
				if ((energy_reference_[i] + energy_mst_mst_output_[i] < 0) && (velocity_actual_[i]!=0))
				{
					//	Backward 1 step
					energy_mst_mst_output_[i] += force_feedback_master_[i] * velocity_actual_[i];
					//	Force modification
					force_feedback_master_popc_[i] = (energy_reference_[i] + energy_mst_mst_output_[i])/velocity_actual_[i];
					//	Update
					energy_mst_mst_output_[i] -= force_feedback_master_popc_[i] * velocity_actual_[i];
				}
				else
					force_feedback_master_popc_[i] = force_feedback_master_[i];
			}
		}
	}

	void velocityMaster1ToSlaveCallback(const brl_teleop_msgs::PackageConstPtr& pkg)
	{
		master1_velocity_[0] = 0.5*pkg->x;
		master1_velocity_[1] = 0.5*pkg->y;
		master1_velocity_[2] = 0.5*pkg->z;

		position_design_[0] += master1_velocity_[0];
		position_design_[1] += master1_velocity_[1];
		position_design_[2] += master1_velocity_[2];

		energy_reference_[0] = pkg->Ex;
		energy_reference_[1] = pkg->Ey;
		energy_reference_[2] = pkg->Ez;

		for (int i = 0; i < 3; i++)
		{
			force_ctrl_[i] = PdCompute(position_design_[i],position_actual_[i],velocity_actual_[i],Kp_,Kd_);
			//	Passivity Observer
			if (force_ctrl_[i] * master1_velocity_[i] < 0)
				energy_slv_mst1_input_[i] -= force_ctrl_[i] * master1_velocity_[i];
			if (force_ctrl_[i] * master1_velocity_[i] > 0)
				energy_mst1_slv_output_[i] -= force_ctrl_[i] * master1_velocity_[i];

			//	Passivity Controller
			if (popc_enable_)
			{
				if ((energy_reference_[i] + energy_mst1_slv_output_[i] <0) && (force_ctrl_[i]!=0))
				{
					//	Backward 1 step
					energy_mst1_slv_output_[i]+= force_ctrl_[i] * master1_velocity_[i];
					position_design_[i] -= master1_velocity_[i];
					//	Velocity modification
					master1_velocity_[i] = (energy_reference_[i] + energy_mst1_slv_output_[i])/force_ctrl_[i];
					//	Update 1 step
					position_design_[i] += master1_velocity_[i];
					force_ctrl_popc_[i] = PdCompute(position_design_[i],position_actual_[i],velocity_actual_[i],Kp_,Kd_);
					energy_mst1_slv_output_[i]-= force_ctrl_popc_[i] * master1_velocity_[i];
				}
				else
					force_ctrl_popc_[i] = force_ctrl_[i];
			}
		}
	}

	void velocityMaster2ToSlaveCallback(const brl_teleop_msgs::PackageConstPtr& pkg)
	{
		master2_velocity_[0] = 0.5*pkg->x;
		master2_velocity_[1] = 0.5*pkg->y;
		master2_velocity_[2] = 0.5*pkg->z;

		position_design_[0] += master2_velocity_[0];
		position_design_[1] += master2_velocity_[1];
		position_design_[2] += master2_velocity_[2];

		energy_reference_[0] = pkg->Ex;
		energy_reference_[1] = pkg->Ey;
		energy_reference_[2] = pkg->Ez;

		for (int i = 0; i < 3; i++)
		{
			force_ctrl_[i] = PdCompute(position_design_[i],position_actual_[i],velocity_actual_[i],Kp_,Kd_);
			//	Passivity Observer
			if (force_ctrl_[i] * master2_velocity_[i] < 0)
				energy_slv_mst2_input_[i] -= force_ctrl_[i] * master2_velocity_[i];
			if (force_ctrl_[i] * master2_velocity_[i] > 0)
				energy_mst2_slv_output_[i] -= force_ctrl_[i] * master2_velocity_[i];

			//	Passivity Controller
			if (popc_enable_)
			{
				if ((energy_reference_[i] + energy_mst2_slv_output_[i] <0) && (force_ctrl_[i]!=0))
				{
					//	Backward 1 step
					energy_mst2_slv_output_[i]+= force_ctrl_[i] * master2_velocity_[i];
					position_design_[i] -= master2_velocity_[i];
					//	Velocity modification
					master2_velocity_[i] = (energy_reference_[i] + energy_mst2_slv_output_[i])/force_ctrl_[i];
					//	Update 1 step
					position_design_[i] += master2_velocity_[i];
					force_ctrl_popc_[i] = PdCompute(position_design_[i],position_actual_[i],velocity_actual_[i],Kp_,Kd_);
					energy_mst2_slv_output_[i]-= force_ctrl_popc_[i] * master2_velocity_[i];
				}
				else
					force_ctrl_popc_[i] = force_ctrl_[i];
			}
		}	
	}
	void forceSlaveToMasterCallback(const brl_teleop_msgs::PackageConstPtr& pkg)
	{
		force_feedback_slave_[0] = -pkg->fx;
		force_feedback_slave_[1] = -pkg->fy;
		force_feedback_slave_[2] = -pkg->fz;

		energy_reference_[0] = pkg->Ex;
		energy_reference_[1] = pkg->Ey;
		energy_reference_[2] = pkg->Ez;

		for (int i = 0; i < 3; i++)
		{
			//	Passivity Observer
			if (force_feedback_slave_[i] * velocity_actual_[i] > 0)
				energy_slv_mst_output_[i] -= force_feedback_slave_[i] * velocity_actual_[i];
			//	Passivity Controller
			if (popc_enable_)
			{
				if ((energy_reference_[i] + energy_slv_mst_output_[i] < 0) && (velocity_actual_[i]!=0))
				{
					//	Backward 1 step
					energy_slv_mst_output_[i] += force_feedback_slave_[i] * velocity_actual_[i];
					//	Force modification
					force_feedback_slave_popc_[i] = (energy_reference_[i] + energy_slv_mst_output_[i])/velocity_actual_[i];
					//	Update
					energy_slv_mst_output_[i] -= force_feedback_slave_popc_[i] * velocity_actual_[i];
				}
				else
					force_feedback_slave_popc_[i] = force_feedback_slave_[i];
			}
		}
	}
/*
	void make_delay()
	{
		int32_t i;
		if (master1_||master2_)
		{
			for(i=0; i <time_delay_-1; i++)
			{
				pkg_mst_slv_[i] = pkg_mst_slv_[i+1];
				pkg_mst_mst_[i] = pkg_mst_mst_[i+1];
			}
			if (master1_)
			{
				pkg_mst_mst_[time_delay_-1].x = velocity_actual_[0];
				pkg_mst_mst_[time_delay_-1].y = velocity_actual_[1];
				pkg_mst_mst_[time_delay_-1].z = velocity_actual_[2];

				pkg_mst_slv_[time_delay_-1].x = velocity_actual_[0];
				pkg_mst_slv_[time_delay_-1].y = velocity_actual_[1];
				pkg_mst_slv_[time_delay_-1].z = velocity_actual_[2];
			}
			else
			{
				pkg_mst_mst_[time_delay_-1].x = force_ctrl_[0];
				pkg_mst_mst_[time_delay_-1].y = force_ctrl_[1];
				pkg_mst_mst_[time_delay_-1].z = force_ctrl_[2];

				pkg_mst_slv_[time_delay_-1].x = force_ctrl_[0];
				pkg_mst_slv_[time_delay_-1].y = force_ctrl_[1];
				pkg_mst_slv_[time_delay_-1].z = force_ctrl_[2];	
			}
				pkg_mst_mst_[time_delay_-1].Ex = energy_mst_mst_input_[0];
				pkg_mst_mst_[time_delay_-1].Ey = energy_mst_mst_input_[1];
				pkg_mst_mst_[time_delay_-1].Ez = energy_mst_mst_input_[2];

				pkg_mst_slv_[time_delay_-1].Ex = energy_mst_slv_intput_[0];
				pkg_mst_slv_[time_delay_-1].Ey = energy_mst_slv_intput_[1];
				pkg_mst_slv_[time_delay_-1].Ez = energy_mst_slv_intput_[2];	
		}
		if (slave_)
		{
			for(i=0; i <time_delay_-1; i++)
			{
				pkg_slv_mst1_[i] = pkg_slv_mst1_[i+1];
				pkg_slv_mst1_[i] = pkg_slv_mst1_[i+1];
			}
				pkg_slv_mst1_[time_delay_-1].x = force_ctrl_[0];
				pkg_slv_mst1_[time_delay_-1].y = force_ctrl_[1];
				pkg_slv_mst1_[time_delay_-1].z = force_ctrl_[2];

				pkg_slv_mst1_[time_delay_-1].Ex = energy_slv_mst1_input_[0];
				pkg_slv_mst1_[time_delay_-1].Ey = energy_slv_mst1_input_[1];
				pkg_slv_mst1_[time_delay_-1].Ez = energy_slv_mst1_input_[2];

				pkg_slv_mst2_[time_delay_-1].x = force_ctrl_[0];
				pkg_slv_mst2_[time_delay_-1].y = force_ctrl_[1];
				pkg_slv_mst2_[time_delay_-1].z = force_ctrl_[2];

				pkg_slv_mst2_[time_delay_-1].Ex = energy_slv_mst2_input_[0];
				pkg_slv_mst2_[time_delay_-1].Ey = energy_slv_mst2_input_[1];
				pkg_slv_mst2_[time_delay_-1].Ez = energy_slv_mst2_input_[2];
		}

	}
	*/
	void velocityMasterToMasterPublish()
	{
		brl_teleop_msgs::Package pkg_out;

		pkg_out.header.frame_id = ros::this_node::getName();
		pkg_out.header.stamp    = ros::Time::now();

//		velocity_master_master_pub_.publish(pkg_mst_mst_[0]);
	}

	void forceMasterToMasterPublish()
	{
		brl_teleop_msgs::Package pkg_out;

		pkg_out.header.frame_id = ros::this_node::getName();
		pkg_out.header.stamp    = ros::Time::now();

//		force_master_master_pub_.publish(pkg_mst_mst_[0]);
	}

	void velocityMasterToSlavePublish()
	{
		brl_teleop_msgs::Package pkg_out;

		pkg_out.header.frame_id = ros::this_node::getName();
		pkg_out.header.stamp    = ros::Time::now();
	
//		velocity_master_slave_pub_.publish(pkg_mst_slv_[0]);
	}
	
	void forceSlaveToMaster1Publish()
	{
		brl_teleop_msgs::Package pkg_out;

		pkg_out.header.frame_id = ros::this_node::getName();
		pkg_out.header.stamp    = ros::Time::now();

//		force_slave_master1_pub_.publish(pkg_slv_mst1_[0]);
	}

	void forceSlaveToMaster2Publish()
	{
		brl_teleop_msgs::Package pkg_out;

		pkg_out.header.frame_id = ros::this_node::getName();
		pkg_out.header.stamp    = ros::Time::now();
		
//		force_slave_master2_pub_.publish(pkg_slv_mst2_[0]);
	}
	void forceToDevice()
	{
		brl_teleop_msgs::Package pkg_out;

		pkg_out.header.frame_id = ros::this_node::getName();
		pkg_out.header.stamp    = ros::Time::now();
		
		if (popc_enable_)
		{
			if (master1_)
			 	force_to_device_ = force_feedback_master_popc_+ force_feedback_slave_popc_;
			if (master2_)
				force_to_device_ = force_ctrl_popc_+force_feedback_slave_popc_;
			if (slave_)
				force_to_device_ = force_ctrl_popc_;
		}
		else
		{
			if (master1_)
				force_to_device_ = force_feedback_master_+force_feedback_slave_; 

			if (master2_)
				force_to_device_ = force_ctrl_+force_feedback_slave_;
	
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
	 //  		Controller->make_delay();
		   	Controller->velocityMasterToMasterPublish();
		   	Controller->velocityMasterToSlavePublish();
		   	Controller->forceToDevice();
	   }

	   if (Controller->master2_)
	   {
	 //  		Controller->make_delay();	
	   		Controller->velocityMasterToSlavePublish();
	   		Controller->forceMasterToMasterPublish();
			Controller->forceToDevice();
	   }
	   if (Controller->slave_)
	   {
	//   		Controller->make_delay();
			Controller->forceSlaveToMaster1Publish();
		   	Controller->forceSlaveToMaster2Publish();
		 	Controller->forceToDevice();  	
	   }


	   	// fprintf(data,"%.3f %.3f %.3f %.3f %.3f %.3f\n",Controller->energy_reference_[0],Controller->energy_output_[0],
	    			// Controller->energy_reference_[1],Controller->energy_output_[1],
	    			// Controller->energy_reference_[2],Controller->energy_output_[2]);
	 	if (Controller->master1_)
	 		fprintf(data,"%.3f %.3f %.3f\n",Controller->force_feedback_master_popc_[0]+Controller->force_feedback_slave_popc_[0],
	 										Controller->force_feedback_master_popc_[1]+Controller->force_feedback_slave_popc_[1],
	 										Controller->force_feedback_master_popc_[2]+Controller->force_feedback_slave_popc_[2]);  

	 	if (Controller->master2_)
	 		fprintf(data,"%.3f %.3f %.3f\n",Controller->force_ctrl_popc_[0]+Controller->force_feedback_slave_popc_[0],
	 										Controller->force_ctrl_popc_[1]+Controller->force_feedback_slave_popc_[1],
	 										Controller->force_ctrl_popc_[2]+Controller->force_feedback_slave_popc_[2]);
	 	if (Controller->slave_)
	 		fprintf(data,"%.3f %.3f %.3f\n",Controller->force_ctrl_popc_[0],
	 										Controller->force_ctrl_popc_[1],
	 										Controller->force_ctrl_popc_[2]);
       loop_rate.sleep();
   }
   fclose(data);
   return NULL;
}

//--------- Main Program -----------//
int main(int argc, char** argv)
{
	//-- Init ROS node
	ros::init(argc, argv, "controller");

	controller Controller;

	// data file
	data = fopen("/home/biolab/fuerte_workspace/sandbox/brl_teleop_controller/src/data.txt","w");

	if (data!=NULL)
	{
		std::cout << "\t" << "Created file success "<< std::endl;
	}
	else
	{
		std::cout << "\t" << "Fail to create file "<< std::endl;
	}
	//-- Threads
	pthread_t publish_thread;
	pthread_create(&publish_thread,NULL,ros_publish,(void*) &Controller);
	pthread_join(publish_thread,NULL);


	ROS_INFO("Ending Session...\n");
}




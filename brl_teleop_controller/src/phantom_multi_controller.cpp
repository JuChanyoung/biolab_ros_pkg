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
#include <fstream>
#include <boost/array.hpp>
// ------------------------------------ //
const int32_t PUBLISH_RATE = 1000; // 1kHz
const int32_t TIME_DELAY = 1;	// No time delay
FILE *data;

typedef boost::array<double,3> vector3D;

class controller
{
public:
	// Variable
	bool master1_;									//  Master 1 Controller
	bool master2_;									//  Master 2 Controller
	bool slave_;									//	Slave Controller
	bool popc_enable_;
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
	vector3D force_ctrl_popc_;					//	Force after PoPc
	vector3D force_feedback_master_popc_;		// 	Force feedback from master
	vector3D force_feedback_slave_popc_;		//	Force feebdack from slave
	
	vector3D energy_reference_master_;			//	Reference Energy
	vector3D energy_reference_slave_;			//	Reference Energy
	vector3D energy_reference_master1_;			//	Reference Energy
	vector3D energy_reference_master2_;			//	Reference Energy

	vector3D energy_mst_mst_input_;				//	Energy input from master to master
	vector3D energy_mst_mst_output_;			//	Energy output from master to master
	vector3D energy_mst_slv_intput_;			//	Energy input from master to slave
	vector3D energy_mst1_slv_output_;			//	Energy output from master 1 to slave
	vector3D energy_mst2_slv_output_;			//	Energy output from master 2 to slave
	vector3D energy_slv_mst1_input_;			//	Energy input from slave to master 1
	vector3D energy_slv_mst2_input_;			//	Energy int from slave to master 2
	vector3D energy_slv_mst_output_;			//	Energy output from slave to master
	vector3D force_to_device_;
	// Ros variables
	ros::NodeHandlePtr node_;
	int32_t publish_rate_;
	int32_t time_delay_;
	// Subscriber
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

	ros::Publisher force_popc_pub_;		
	
	// Delay variable
	brl_teleop_msgs::Package* pkg_mst_mst_;		//	From master to master
	brl_teleop_msgs::Package* pkg_mst_slv_;		//	From master to slave
	brl_teleop_msgs::Package* pkg_slv_mst1_;	//	From slave to master 1
	brl_teleop_msgs::Package* pkg_slv_mst2_;	//	From slave  to master 2
	// PD Controller
	double Kp_;
	double Kd_;
	
	// Mass - Spring - Damper
	double K_master_; // Mass - Spring - Damper
	double Mss_;
	vector3D Acc_;	// Acceleration
	vector3D Vm_;	// Velocity
	vector3D Xm_;	// Position

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

		alpha_ = 0.5;

		K_master_ = 1;
		Mss_ = 0.0001;
		curTime_ = 0;
		prvTime_ = 0;
		dT_ = 0.001;
		beta_ = 2 *Mss_ *sqrt(K_master_/Mss_);
		// init
		init();
		
		pkg_mst_slv_ = new brl_teleop_msgs::Package[time_delay_];
		pkg_mst_mst_ = new brl_teleop_msgs::Package[time_delay_];
	
		pkg_slv_mst1_ = new brl_teleop_msgs::Package[time_delay_];
		pkg_slv_mst2_ = new brl_teleop_msgs::Package[time_delay_];
	}
	void MassSpringDamper(vector3D &force,const vector3D &pos,const vector3D &vel)
	{
		// This is the filter to remove high frequency noise in force control
		for (int i = 0;i < 3; i++)
		{
			Acc_[i]  = (-K_master_*(Xm_[i] - pos[i]) - beta_*(Vm_[i] - vel[i]/dT_) + force[i])/Mss_;
			Vm_[i]   += Acc_[i]*dT_;
			Xm_[i]   += Vm_[i]*dT_;
			force[i] = K_master_*(Xm_[i] - pos[i]);
		}
	}

	double PdCompute(const double &Xr, const double &X,const double &velocity,const double &Kp,const double &Kd)
	{
		// Simple PD controller
		double pdVal,error;
		error = Xr - X; 
		pdVal = Kp*error - Kd*velocity;
		return pdVal;
	}
	void actualVelocityCallback(const brl_teleop_msgs::VelConstPtr& msg)
	{
		// This is the callback function which subscribing for actual velocity of device
		velocity_actual_[0] = msg->deltaX; 		
		velocity_actual_[1] = msg->deltaY;		
		velocity_actual_[2] = msg->deltaZ;		

		//	Velocity is then integrated to be position
		position_actual_[0] += velocity_actual_[0];
		position_actual_[1] += velocity_actual_[1];
		position_actual_[2] += velocity_actual_[2];

		//	In case of PoPc algorithm isn't used
		if (!popc_enable_)
			for (int i = 0; i < 3; ++i)
				Vm_[i] = velocity_actual_[i] / dT_;

		//	Calculating input energy at master side
		if ((master1_ || master2_))
		{
			for (int i=0;i<3;i++)
			{
				if (master1_)
				{
					// 	Input energy for master -> master channel
					if (Vm_[i] * force_feedback_master_[i] < 0)
						energy_mst_mst_input_[i] -= Vm_[i] * force_feedback_master_[i]*dT_;	
				}
				
					//	Input energy for master -> slave channel
				if (Vm_[i] * force_feedback_slave_[i] < 0)
					energy_mst_slv_intput_[i] -= alpha_*Vm_[i] * force_feedback_slave_[i]*dT_;
			}
		}
	}
	void velocityMasterToMasterCallback(const brl_teleop_msgs::PackageConstPtr& pkg)
	{
		//	This is the callback funtion which subscribing for the velocity command
		//	from master 1 to master 2
		velocity_desired_[0] = pkg->x;
		velocity_desired_[1] = pkg->y;
		velocity_desired_[2] = pkg->z;

		position_desire_[0] += velocity_desired_[0];
		position_desire_[1] += velocity_desired_[1];
		position_desire_[2] += velocity_desired_[2];

		energy_reference_master_[0] = pkg->Ex;
		energy_reference_master_[1] = pkg->Ey;
		energy_reference_master_[2] = pkg->Ez;

		for (int i = 0; i < 3; i++)
		{
			// Control force is calculated by a PD controller
			force_ctrl_[i] = PdCompute(position_desire_[i],position_actual_[i],velocity_actual_[i],Kp_,Kd_);

			// Passivity Observer
			if (force_ctrl_[i]*velocity_desired_[i] < 0)
				energy_mst_mst_input_[i] -= force_ctrl_[i] * velocity_desired_[i];
			if (force_ctrl_[i]*velocity_desired_[i] > 0)
				energy_mst_mst_output_[i] -= force_ctrl_[i] * velocity_desired_[i];
			// Passivity Controller
			if (popc_enable_)
			{
				if ((energy_reference_master_[i] + energy_mst_mst_output_[i]<0) && (force_ctrl_[i]!=0))
				{
					// Backward 1 step
					energy_mst_mst_output_[i] += force_ctrl_[i] * velocity_desired_[i];
					position_desire_[i] -= velocity_desired_[i];
					// Modify veloctiy
					velocity_desired_[i] = (energy_reference_master_[i] + energy_mst_mst_output_[i])/force_ctrl_[i];
					// Update 1 step
					position_desire_[i] += velocity_desired_[i];	
					force_ctrl_popc_[i] = PdCompute(position_desire_[i],position_actual_[i],velocity_actual_[i],Kp_,Kd_);				
					energy_mst_mst_output_[i] -= force_ctrl_popc_[i] * velocity_desired_[i];
				}
				else
					force_ctrl_popc_[i] = force_ctrl_[i];	
			}
		}
	}
	
	void forceMasterToMasterCallback(const brl_teleop_msgs::PackageConstPtr& pkg)
	{
		//	This is the callback function for the force feedback from master 2 to master 1
		force_feedback_master_[0] = -pkg->fx;
		force_feedback_master_[1] = -pkg->fy;
		force_feedback_master_[2] = -pkg->fz;

		energy_reference_master_[0] = pkg->Ex;
		energy_reference_master_[1] = pkg->Ey;
		energy_reference_master_[2] = pkg->Ez;

		for (int i = 0; i < 3; i++)
		{
			//	Passivity Observer
			if (force_feedback_master_[i] * Vm_[i] > 0)
				energy_mst_mst_output_[i] -= force_feedback_master_[i] * Vm_[i]*dT_;
			//	Passivity Controller
			if (popc_enable_)
			{
				if ((energy_reference_master_[i] + energy_mst_mst_output_[i] < 0) && (Vm_[i]!=0))
				{
					//	Backward 1 step
					energy_mst_mst_output_[i] += force_feedback_master_[i] * Vm_[i]*dT_;
					//	Force modification
					force_feedback_master_popc_[i] = (energy_reference_master_[i] + energy_mst_mst_output_[i])/(Vm_[i]*dT_);
					//	Update
					energy_mst_mst_output_[i] -= force_feedback_master_popc_[i] * Vm_[i]*dT_;
				}
				else
					force_feedback_master_popc_[i] = force_feedback_master_[i];
			}
		}
	}

	void velocityMaster1ToSlaveCallback(const brl_teleop_msgs::PackageConstPtr& pkg)
	{
		//	This is the calback function for velocity command from master 1 to slave
		master1_velocity_[0] = alpha_*pkg->x;
		master1_velocity_[1] = alpha_*pkg->y;
		master1_velocity_[2] = alpha_*pkg->z;

		position_desire_[0] += master1_velocity_[0];
		position_desire_[1] += master1_velocity_[1];
		position_desire_[2] += master1_velocity_[2];

		energy_reference_master1_[0] = pkg->Ex;
		energy_reference_master1_[1] = pkg->Ey;
		energy_reference_master1_[2] = pkg->Ez;

		for (int i = 0; i < 3; i++)
		{
			force_ctrl_[i] = PdCompute(position_desire_[i],position_actual_[i],velocity_actual_[i],Kp_,Kd_);
			//	Passivity Observer
			if (force_ctrl_[i] * master1_velocity_[i] < 0)
				energy_slv_mst1_input_[i] -= force_ctrl_[i] * master1_velocity_[i];
			if (force_ctrl_[i] * master1_velocity_[i] > 0)
				energy_mst1_slv_output_[i] -= force_ctrl_[i] * master1_velocity_[i];

			//	Passivity Controller
			if (popc_enable_)
			{
				if ((energy_reference_master1_[i] + energy_mst1_slv_output_[i] <0) && (force_ctrl_[i]!=0))
				{
					//	Backward 1 step
					energy_mst1_slv_output_[i] += force_ctrl_[i] * master1_velocity_[i];
					position_desire_[i] -= master1_velocity_[i];
					//	Velocity modification
					master1_velocity_[i] = (energy_reference_master1_[i] + energy_mst1_slv_output_[i])/force_ctrl_[i];
					//	Update 1 step
					position_desire_[i] += master1_velocity_[i];
					force_ctrl_popc_[i] = PdCompute(position_desire_[i],position_actual_[i],velocity_actual_[i],Kp_,Kd_);
					energy_mst1_slv_output_[i]-= force_ctrl_popc_[i] * master1_velocity_[i];
				}
				else
					force_ctrl_popc_[i] = force_ctrl_[i];
			}
		}
	}

	void velocityMaster2ToSlaveCallback(const brl_teleop_msgs::PackageConstPtr& pkg)
	{
		//	This is the calback function for velocity command from master 2 to slave
		master2_velocity_[0] = alpha_*pkg->x;
		master2_velocity_[1] = alpha_*pkg->y;
		master2_velocity_[2] = alpha_*pkg->z;

		position_desire_[0] += master2_velocity_[0];
		position_desire_[1] += master2_velocity_[1];
		position_desire_[2] += master2_velocity_[2];

		energy_reference_master2_[0] = pkg->Ex;
		energy_reference_master2_[1] = pkg->Ey;
		energy_reference_master2_[2] = pkg->Ez;

		for (int i = 0; i < 3; i++)
		{
			force_ctrl_[i] = PdCompute(position_desire_[i],position_actual_[i],velocity_actual_[i],Kp_,Kd_);
			//	Passivity Observer
			if (force_ctrl_[i] * master2_velocity_[i] < 0)
				energy_slv_mst2_input_[i] -= force_ctrl_[i] * master2_velocity_[i];
			if (force_ctrl_[i] * master2_velocity_[i] > 0)
				energy_mst2_slv_output_[i] -= force_ctrl_[i] * master2_velocity_[i];

			//	Passivity Controller
			if (popc_enable_)
			{
				if ((energy_reference_master2_[i] + energy_mst2_slv_output_[i] <0) && (force_ctrl_[i]!=0))
				{
					//	Backward 1 step
					energy_mst2_slv_output_[i]+= force_ctrl_[i] * master2_velocity_[i];
					position_desire_[i] -= master2_velocity_[i];
					//	Velocity modification
					master2_velocity_[i] = (energy_reference_master2_[i] + energy_mst2_slv_output_[i])/force_ctrl_[i];
					//	Update 1 step
					position_desire_[i] += master2_velocity_[i];
					force_ctrl_popc_[i] = PdCompute(position_desire_[i],position_actual_[i],velocity_actual_[i],Kp_,Kd_);
					energy_mst2_slv_output_[i]-= force_ctrl_popc_[i] * master2_velocity_[i];
				}
				else
					force_ctrl_popc_[i] = force_ctrl_[i];
			}
		}	
	}
	void forceSlaveToMasterCallback(const brl_teleop_msgs::PackageConstPtr& pkg)
	{
		//	This is the calback function for force feedback from slave to master1/master2 
		force_feedback_slave_[0] = -pkg->fx;
		force_feedback_slave_[1] = -pkg->fy;
		force_feedback_slave_[2] = -pkg->fz;

		energy_reference_slave_[0] = pkg->Ex;
		energy_reference_slave_[1] = pkg->Ey;
		energy_reference_slave_[2] = pkg->Ez;

		for (int i = 0; i < 3; i++)
		{
			//	Passivity Observer
			if (force_feedback_slave_[i] * Vm_[i] > 0)
				energy_slv_mst_output_[i] -= alpha_*force_feedback_slave_[i] * Vm_[i]*dT_;
			//	Passivity Controller
			if (popc_enable_)
			{
				if ((energy_reference_slave_[i] + energy_slv_mst_output_[i] < 0) && (Vm_[i]!=0))
				{
					//	Backward 1 step
					energy_slv_mst_output_[i] += alpha_*force_feedback_slave_[i] * Vm_[i]*dT_;
					//	Force modification
					force_feedback_slave_popc_[i] = (energy_reference_slave_[i] + energy_slv_mst_output_[i])/(alpha_*Vm_[i]*dT_);
					//	Update
					energy_slv_mst_output_[i] -= alpha_*force_feedback_slave_popc_[i] * Vm_[i]*dT_;
				}
				else
					force_feedback_slave_popc_[i] = force_feedback_slave_[i];
			}
		}
	}

	void make_delay()
	{
		// For experiment, artificial time delay is introduced.
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
				pkg_mst_mst_[time_delay_-1].x = Vm_[0]*dT_;
				pkg_mst_mst_[time_delay_-1].y = Vm_[1]*dT_;
				pkg_mst_mst_[time_delay_-1].z = Vm_[2]*dT_;

				pkg_mst_slv_[time_delay_-1].x = Vm_[0]*dT_;
				pkg_mst_slv_[time_delay_-1].y = Vm_[1]*dT_;
				pkg_mst_slv_[time_delay_-1].z = Vm_[2]*dT_;
			}
			else
			{
				pkg_mst_mst_[time_delay_-1].fx = force_ctrl_[0];
				pkg_mst_mst_[time_delay_-1].fy = force_ctrl_[1];
				pkg_mst_mst_[time_delay_-1].fz = force_ctrl_[2];

				pkg_mst_slv_[time_delay_-1].x = Vm_[0]*dT_;
				pkg_mst_slv_[time_delay_-1].y = Vm_[1]*dT_;
				pkg_mst_slv_[time_delay_-1].z = Vm_[2]*dT_;	
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
				pkg_slv_mst2_[i] = pkg_slv_mst2_[i+1];
			}
				pkg_slv_mst1_[time_delay_-1].fx = force_ctrl_[0];
				pkg_slv_mst1_[time_delay_-1].fy = force_ctrl_[1];
				pkg_slv_mst1_[time_delay_-1].fz = force_ctrl_[2];

				pkg_slv_mst1_[time_delay_-1].Ex = energy_slv_mst1_input_[0];
				pkg_slv_mst1_[time_delay_-1].Ey = energy_slv_mst1_input_[1];
				pkg_slv_mst1_[time_delay_-1].Ez = energy_slv_mst1_input_[2];

				pkg_slv_mst2_[time_delay_-1].fx = force_ctrl_[0];
				pkg_slv_mst2_[time_delay_-1].fy = force_ctrl_[1];
				pkg_slv_mst2_[time_delay_-1].fz = force_ctrl_[2];

				pkg_slv_mst2_[time_delay_-1].Ex = energy_slv_mst2_input_[0];
				pkg_slv_mst2_[time_delay_-1].Ey = energy_slv_mst2_input_[1];
				pkg_slv_mst2_[time_delay_-1].Ez = energy_slv_mst2_input_[2];
		}
	}
	
	// Publishing function
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
		//  This is the funtion to publish force to device nodes.
		brl_teleop_msgs::Package pkg_out;

		pkg_out.header.frame_id = ros::this_node::getName();
		pkg_out.header.stamp    = ros::Time::now();
		
		if (popc_enable_)
		{
			if (master1_)
			{
				force_to_device_[0] = force_feedback_master_popc_[0]+ force_feedback_slave_popc_[0];
				force_to_device_[1] = force_feedback_master_popc_[1]+ force_feedback_slave_popc_[1];
				force_to_device_[2] = force_feedback_master_popc_[2]+ force_feedback_slave_popc_[2];
			}
			if (master2_)
				{
					force_to_device_[0] = force_ctrl_popc_[0]+force_feedback_slave_popc_[0];
					force_to_device_[1] = force_ctrl_popc_[1]+force_feedback_slave_popc_[1];
					force_to_device_[2] = force_ctrl_popc_[2]+force_feedback_slave_popc_[2];
				}
			if (slave_)
				force_to_device_ = force_ctrl_popc_;
			
			MassSpringDamper(force_to_device_,position_actual_,velocity_actual_);
			
			force_to_device_[2] = 0.1*(0 - position_actual_[2]);	
			force_to_device_[0] = 0.1*(0 - position_actual_[0]);
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
		/*
		This is the initialize funtion to load configuration parameters from launch file
		*/
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
	   		Controller->make_delay();
		   	Controller->velocityMasterToMasterPublish();
		   	Controller->velocityMasterToSlavePublish();
		   	Controller->forceToDevice();
	   }

	   if (Controller->master2_)
	   {
	   		Controller->make_delay();	
	   		Controller->velocityMasterToSlavePublish();
	   		Controller->forceMasterToMasterPublish();
			Controller->forceToDevice();
	   }
	   if (Controller->slave_)
	   {
	   		Controller->make_delay();
			Controller->forceSlaveToMaster1Publish();
		   	Controller->forceSlaveToMaster2Publish();
		 	Controller->forceToDevice();  	
	   }
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
	//-- Threads
	pthread_t publish_thread;
	pthread_create(&publish_thread,NULL,ros_publish,(void*) &Controller);
	pthread_join(publish_thread,NULL);

	ROS_INFO("Ending Session...\n");
}



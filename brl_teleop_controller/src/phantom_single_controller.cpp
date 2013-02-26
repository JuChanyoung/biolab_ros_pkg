//============================================================================
// Name        : main.cpp
// Author      : Haquang
// Version     : 1.0
// Copyright   : Your copyright notice
// Description : This is phantom controller, using PD controller, with/without
//				 Time Delay Power Network and Time Domain Passivity Approach to
//				 stabilize system under time delay
//				 This is a single controller for Phantom - Phantom teleoperation
//				 experiment. 				 
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

FILE *data;

class controller
{
public:
	// Variable

	bool master_;								//  Master/Slave Controller
	bool popc_enable_;
	hduVector3Dd velocity_design_;				//	Design Velocity
	hduVector3Dd position_design_;				//	Design Position
	hduVector3Dd energy_input_;					//  Input energy _ corresponding with design velocity/force
	hduVector3Dd energy_output_;				//  Output energy _ to be sent
	hduVector3Dd energy_reference_;				//	Reference energy

	hduVector3Dd velocity_actual_;				//	Actual Velocity
	hduVector3Dd position_actual_;				// 	Actual Position
	hduVector3Dd force_ctrl_;					// 	Control force (in case of slave device)
	hduVector3Dd force_feedback_;				// 	Control force (in case of master device)

	hduVector3Dd force_feedback_popc_;			// 	Force feedback after PoPc
	hduVector3Dd force_ctrl_popc_;				//	Design velocity after PoPc

	ros::NodeHandlePtr node_;

	std::string design_velocity_src_name_;    	//  Source for subscribing design velocity
	std::string design_velocity_topic_name_;	//	Topic for subscribing design velocity

	std::string actual_velocity_src_name_;
	std::string actual_velocity_topic_name_;

	std::string feedback_force_src_name_; 	   	//  Source for subscribing
	std::string feedback_force_topic_name_;		//	Topic for subscribing

	ros::Subscriber design_velocity_sub_;		
	ros::Subscriber actual_velocity_sub_;
	ros::Subscriber force_sub_;

	ros::Publisher force_feedback_pub_;
	ros::Publisher force_ctrl_pub_;
	ros::Publisher force_ctrl_slv_pub_;
	ros::Publisher vel_pub;
	ros::Publisher pos_pub;

	int32_t publish_rate_;
	
	// PD Controller
	double Kp_;
	double Kd_;
	hduVector3Dd pre_error_;
	
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
		master_ = false;
		popc_enable_=false;
		energy_input_.set(0.0f,0.0f,0.0f);
		energy_output_.set(0.0f,0.0f,0.0f);
		energy_reference_.set(0.0f,0.0f,0.0f);
		pre_error_.set(0.0f,0.0f,0.0f);

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

		if (master_)
		{
			for (int i=0;i<3;i++)
			{
				if (velocity_actual_[i] * force_feedback_[i] < 0)
					energy_input_[i] -= velocity_actual_[i] * force_feedback_[i];
			}
		}
	}
	void designVelocityCallback(const brl_teleop_msgs::PackageConstPtr& pkg)
	{
		if (master_)
			return;
		
		velocity_design_[0] = pkg->x;
		velocity_design_[1] = pkg->y;
		velocity_design_[2] = pkg->z;

		position_design_[0] += velocity_design_[0];
		position_design_[1] += velocity_design_[1];
		position_design_[2] += velocity_design_[2];
	
		energy_reference_[0] = pkg->Ex;			// Corresponding Energy
		energy_reference_[1] = pkg->Ey;			// Corresponding Energy
		energy_reference_[2] = pkg->Ez;			// Corresponding Energy

		for (int i=0;i<3;i++)
		{
			force_ctrl_[i] = PdCompute(position_design_[i],position_actual_[i],velocity_actual_[i],Kp_,Kd_);
			// Passivity Observer - Force feedback channel
			if (force_ctrl_[i] * velocity_design_[i] < 0)
				energy_input_[i] -= force_ctrl_[i] * velocity_design_[i];
			// Passivity Observer - Velocity command channel
			if (force_ctrl_[i] * velocity_design_[i] > 0)
				energy_output_[i] -= force_ctrl_[i] * velocity_design_[i];
			// Passivity Controller

			if (popc_enable_)
			{
				if ((energy_output_[i] + energy_reference_[i] < 0) && (force_ctrl_[i] != 0))
				{
					// Backward 1 step
					energy_output_[i] += force_ctrl_[i] * velocity_design_[i];
					position_design_[i] -= velocity_design_[i];
					// Velocity modification
					velocity_design_[i] = (energy_output_[i] + energy_reference_[i])/force_ctrl_[i];
					// Update
					position_design_[i] += velocity_design_[i];
					force_ctrl_popc_[i] = PdCompute(position_design_[i],position_actual_[i],velocity_actual_[i],Kp_,Kd_);
					energy_output_[i] -= force_ctrl_popc_[i] * velocity_design_[i];
				}
				else
					force_ctrl_popc_[i] = force_ctrl_[i];	
			}
		}
	}
	
	void forceCallback(const brl_teleop_msgs::PackageConstPtr& pkg)
	{
		if (!master_)
			return;

		curTime_ = ros::Time::now().toSec();
		dT_ = curTime_ - prvTime_;
		prvTime_ = curTime_;

		force_feedback_[0] = -pkg->fx; 			// Design velocity
		force_feedback_[1] = -pkg->fy;			// Design velocity
		force_feedback_[2] = -pkg->fz;			// Design velocity

		energy_reference_[0] = pkg->Ex;			// Corresponding Energy
		energy_reference_[1] = pkg->Ey;			// Corresponding Energy
		energy_reference_[2] = pkg->Ez;			// Corresponding Energy

		for (int i = 0; i < 3; i++)
		{
			// Passivity Observer
			if (force_feedback_[i] * velocity_actual_[i] > 0)
				energy_output_[i] -= force_feedback_[i] * velocity_actual_[i];
			// Passivity Controller
			if (popc_enable_)
			{
				if ((energy_reference_[i] + energy_output_[i] < 0) && (velocity_actual_[i]!= 0))
				{
					// Backward 1 step
					energy_output_[i] += force_feedback_[i] * velocity_actual_[i];
					// PC
					force_feedback_popc_[i] = (energy_reference_[i] + energy_output_[i])/velocity_actual_[i];
					// Update
					energy_output_[i] -= force_feedback_popc_[i] * velocity_actual_[i];
				}
				else
					force_feedback_popc_[i] = force_feedback_[i];
			}
		}
				MassSpringDamper(force_feedback_popc_,position_actual_,velocity_actual_);	
	}

	void force_ctrl_publish()
	{
		brl_teleop_msgs::Package pkg_out;

		pkg_out.header.frame_id = ros::this_node::getName();
		pkg_out.header.stamp    = ros::Time::now();

		pkg_out.fx = force_ctrl_[0];
		pkg_out.fy = force_ctrl_[1];
		pkg_out.fz = force_ctrl_[2];

		pkg_out.Ex = energy_input_[0];
		pkg_out.Ey = energy_input_[1];
		pkg_out.Ez = energy_input_[2];

		force_ctrl_pub_.publish(pkg_out);
	}

	void force_ctrl_slv_publish()
	{
		brl_teleop_msgs::Package pkg_out;

		pkg_out.header.frame_id = ros::this_node::getName();
		pkg_out.header.stamp    = ros::Time::now();

		if (popc_enable_)
		{
			pkg_out.fx = force_ctrl_popc_[0];
			pkg_out.fy = force_ctrl_popc_[1];
			pkg_out.fz = force_ctrl_popc_[2];
		}
		else
		{
			pkg_out.fx = force_ctrl_[0];
			pkg_out.fy = force_ctrl_[1];
			pkg_out.fz = force_ctrl_[2];
		}

		pkg_out.Ex = energy_input_[0];
		pkg_out.Ey = energy_input_[1];
		pkg_out.Ez = energy_input_[2];

		force_ctrl_slv_pub_.publish(pkg_out);
	}

	void force_feedback_publish()
	{
		brl_teleop_msgs::Package pkg_out;

		pkg_out.header.frame_id = ros::this_node::getName();
		pkg_out.header.stamp    = ros::Time::now();

		if (popc_enable_)
		{
			pkg_out.fx = force_feedback_popc_[0];
			pkg_out.fy = force_feedback_popc_[1];
			pkg_out.fz = force_feedback_popc_[2];
		}
		else
		{
			pkg_out.fx = force_feedback_[0];
			pkg_out.fy = force_feedback_[1];
			pkg_out.fz = force_feedback_[2];
		}

		force_feedback_pub_.publish(pkg_out);
	}
	void velocity_publish()
	{
		brl_teleop_msgs::Package pkg_out;
		pkg_out.header.frame_id = ros::this_node::getName();
		pkg_out.header.stamp = ros::Time::now();

		pkg_out.x = velocity_actual_[0];
		pkg_out.y = velocity_actual_[1];
		pkg_out.z = velocity_actual_[2];

		pkg_out.Ex = energy_input_[0];
		pkg_out.Ey = energy_input_[1];
		pkg_out.Ez = energy_input_[2];	
		
		vel_pub.publish(pkg_out);
	}

	void init()
	{
		// Ros Node
		node_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));
			std::cout << "Starting with the following parameters:" << std::endl;
		// Settings
		node_->param(std::string("publish_rate"), publish_rate_, PUBLISH_RATE);
			std::cout << "\t" << "publish_rate = " << publish_rate_ << std::endl;
		node_->param(std::string("master"), master_, false);
					std::cout << "\t" << "master = " << master_ << std::endl;
		node_->param(std::string("popc_enable"), popc_enable_, false);
					std::cout << "\t" << "popc_enable = " << popc_enable_ << std::endl;
		// Subscriber settings
		// design velocity
		node_->param(std::string("design_velocity_src_name"), design_velocity_src_name_, std::string(""));
		if(!design_velocity_src_name_.empty())
		{
			node_->param(std::string("design_velocity_topic_name"), design_velocity_topic_name_, std::string("velocity"));
			std::cout << "\t" << "design_velocity_topic_name = " << design_velocity_topic_name_ << std::endl;

			design_velocity_sub_ = node_->subscribe<brl_teleop_msgs::Package>(design_velocity_src_name_ + std::string("/") + design_velocity_topic_name_, 100,
								&controller::designVelocityCallback,this);
		}

		// feedback force
		node_->param(std::string("feedback_force_src_name"), feedback_force_src_name_, std::string(""));
		if (!feedback_force_src_name_.empty())
		{
			std::cout << "\t" << "feedback_force_src_name = " << feedback_force_src_name_ << std::endl;
			node_->param(std::string("feedback_force_topic_name"), feedback_force_topic_name_, std::string("force"));
			std::cout << "\t" << "feedback_force_topic_name = " << feedback_force_topic_name_ << std::endl;

			force_sub_ = node_->subscribe<brl_teleop_msgs::Package>(feedback_force_src_name_ + std::string("/") +
						feedback_force_topic_name_, 100,&controller::forceCallback,this);
		}
		// actual velocity
		node_->param(std::string("actual_velocity_src_name"), actual_velocity_src_name_, std::string(""));
		if (!actual_velocity_src_name_.empty())
		{
			node_->param(std::string("actual_velocity_topic_name"), actual_velocity_topic_name_, std::string("velocity"));
			std::cout << "\t" << "actual_velocity_topic_name = " << actual_velocity_topic_name_ << std::endl;

			actual_velocity_sub_ = node_->subscribe<brl_teleop_msgs::Vel>(actual_velocity_src_name_ + std::string("/") +
					actual_velocity_topic_name_, 100,&controller::actualVelocityCallback,this);
		}
		//PID
		node_->param(std::string("gains/P"), Kp_, 0.1);
		node_->param(std::string("gains/D"), Kd_, 0.01);

		std::cout << "\t" << "Gains = { P: " << Kp_ << ", D: " << Kd_ << "}" << std::endl;

		// Publishing settings
		force_feedback_pub_ = node_->advertise<brl_teleop_msgs::Package>("force_feedback", 100);
		force_ctrl_pub_ = node_->advertise<brl_teleop_msgs::Package>("force_control", 100);
		force_ctrl_slv_pub_ = node_->advertise<brl_teleop_msgs::Package>("force_control_slv", 100);
		vel_pub = node_->advertise<brl_teleop_msgs::Package>("velocity", 100);
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
	   if (Controller->master_)
	   {
		   Controller->velocity_publish();
		   Controller->force_feedback_publish();
	   }
	   else
	   {
		   Controller->force_ctrl_publish();
		   Controller->force_ctrl_slv_publish();
	   }
	   fprintf(data,"%.3f %.3f %.3f %.3f %.3f %.3f\n",Controller->energy_reference_[0],Controller->energy_output_[0],
	   						Controller->energy_reference_[1],Controller->energy_output_[1],
	   						Controller->energy_reference_[2],Controller->energy_output_[2]);
	   // fprintf(data,"%.3f %.3f %.3f %.3f %.3f %.3f\n",Controller->force_ctrl_popc_[0],Controller->force_ctrl_popc_[1],Controller->force_ctrl_popc_[2],
	   // 												  Controller->force_ctrl_[0],Controller->force_ctrl_[1],Controller->force_ctrl_[2]);

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




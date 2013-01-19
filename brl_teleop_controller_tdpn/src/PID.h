/*
 * PID.h
 *
 *  Created on: Dec 22, 2012
 *      Author: haquang
 *  Description: This is a template for PID controller
 */

#ifndef PID_H_
#define PID_H_

template <class T>
class PID
{
private:
	double Kp_;
	double Kd_;
	double Ki_;
	T pre_error_;
	T integral_;
	int size_;
	double DT_;

public:
	PID()
	{
		Kp_= 0;
		Kd_= 0;
		Ki_= 0;
		size_= 1;
		int i;
		for (i=0;i<size_;i++)
		{
			pre_error_[i]=0;
			integral_[i]= 0;
		}
		DT_=0.001f;
	}
	PID(double Kp,double Ki,double Kd,double DT,int size):Kp_(Kp),Ki_(Ki),Kd_(Kd),DT_(DT),size_(size)
	{
		int i;
		for (i=0;i<size_;i++)
		{
			pre_error_[i]=0;
			integral_[i]= 0;
		}
	}
	PID& operator=(const PID& pid)
	{
		Kp_ = pid.Kp_;
		Ki_ = pid.Ki_;
		Kd_ = pid.Kd_;

		size_=pid.size_;
		int i;
		for (i=0;i<size_;i++)
		{
			pre_error_[i] = pid.pre_error_[i];
			integral_[i] = pid.integral_[i];
		}
		DT_ = pid.DT_;

		return *this;
	}

	PID(double Kp,double Ki,double Kd,int size):Kp_(Kp),Ki_(Ki),Kd_(Kd),size_(size)
	{
		DT_ = 0.001f;
		for (int i=0;i<size_;i++)
		{
			pre_error_[i] = 0;
			integral_[i] = 0;
		}
	}

	void configure(double Kp,double Ki,double Kd,int size)
	{
		Kp_ = Kp;
		Ki_ = Ki;
		Kd_ = Ki;
		size_ = size;
	}

	T compute(const T &Xr, const T &X)
	{
		T error, output;

		for (int i=0;i<size_;i++)
		{
			error[i] = Xr[i]  - X[i];

			integral_[i] += Ki_*error[i]*DT_;

			output[i]  = Kp_*error[i]  + Kd_*(error[i] - pre_error_[i])/DT_ + integral_[i];
			//
			pre_error_[i] = error[i];

		}
		return output;
	}
};;


#endif /* PID_H_ */

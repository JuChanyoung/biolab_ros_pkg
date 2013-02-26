/*
 * PoPc.h
 *
 *  Created on: Dec 22, 2012
 *      Author: haquang
 *  Description: This is a template for Passivity Observer - Passivity Controller
 */

#ifndef POPC_H_
#define POPC_H_

template <class T>
class PoPc{
private:
	T force_in_; 		// Input Force
	T force_out_; 		// Modified Force

	T velocity_in_; 	// Input Velocity
	T velocity_out_; 	// Modified Velocity

	T energy_input_; 	// Positive Energy
	T energy_output_; 	// Negative Energy

	bool velocity_type_; 	// force_type and velocity_type
	bool active_;		// Active PC/ inactive PC
	int Dof_;			// Degree of freedom
	bool* pc_run_;	// Track if POPC run
public:

	PoPc()
	{
		velocity_type_ =  true;
		active_ = false;
		Dof_ = 1;
	}
	PoPc(bool velocity_type,bool active,int Dof):velocity_type_(velocity_type),active_(active),Dof_(Dof) {
		pc_run_ = new bool[Dof_];

		for (int j=0;j<Dof_;j++)
		{
			force_in_[j] = 0;
			force_out_[j] = 0;

			velocity_in_[j] = 0;
			velocity_out_[j] = 0;

			energy_input_[j] = 0;
			energy_output_[j] = 0;
			pc_run_[j] = false;
		}

	}

	void configure(bool velocity_type,bool active,int Dof)
	{
		Dof_ = Dof;
		velocity_type_ = velocity_type;
		active_ = active;
	}

	void updatePoPc(const T &force_in,const T &velocity_in,bool invert)
	{
		/*
		 * invert = true: 	input energy means force and velocity has same sign
		 * invert = false: 	input energy means force and velocity has opposite sign
		 */
		force_in_ = force_in;
		velocity_in_ = velocity_in;

		for (int j=0;j<Dof_;j++)
		{
			if (!invert)
			{
				if (force_in_[j]*velocity_in_[j]>0)
				{
					if (active_) // calculate negative energy
						energy_output_[j] -= force_in_[j]*velocity_in_[j];
					else		// calculate positive energy
						energy_input_[j] += force_in_[j]*velocity_in_[j];
				}
			}
			else
			{
				if (force_in_[j]*velocity_in_[j]<0)
				{
					if (active_) // calculate negative energy
						energy_output_[j] += force_in_[j]*velocity_in_[j];
					else		// calculate positive energy
						energy_input_[j] -= force_in_[j]*velocity_in_[j];
				}
			}

		}
	}
	bool PassivityController(const T &reference_energy,bool invert)
	{
		if (!active_)
		{
			return false;
		}
		else
		{
			if (velocity_type_) //Modify Velocity
			{
				for (int j=0;j<Dof_;j++)
				{
					if (reference_energy[j] + energy_output_[j] <0)
					{
						if (!invert)
						{
							energy_output_[j] += force_in_[j]*velocity_in_[j];  //backward 1 step
							velocity_out_[j] = (reference_energy[j] + energy_output_[j])/force_in_[j];
							pc_run_[j] = true;
						//	energy_output_[j] -= force_in_[j]*velocity_out_[j];  //update 1 step
						//	in this case, can not update energy right now because of the control force modification	
						}
						else
						{
							energy_output_[j] -= force_in_[j]*velocity_in_[j];  //backward 1 step
							velocity_out_[j] = (reference_energy[j] + energy_output_[j])/force_in_[j];
							pc_run_[j] = true;
						//	energy_output_[j] += force_in_[j]*velocity_out_[j];  //update 1 step
						//	in this case, can not update energy right now because of the control force modification	
						}
					}
					else
					{
						pc_run_[j] = false;
						velocity_out_[j] = velocity_in_[j];
					}
				}
			}
			else   // Modify Force
			{
				for (int j=0;j<Dof_;j++)
				{
					if (reference_energy[j] + energy_output_[j] <0)
					{
						if (!invert)
						{
							energy_output_[j] += force_in_[j]*velocity_in_[j];  //backward 1 step
							force_out_[j] = (reference_energy[j] + energy_output_[j])/velocity_in_[j];
							energy_output_[j] -= force_out_[j]*velocity_in_[j];  //update 1 step
						}
						else
						{
							energy_output_[j] -= force_in_[j]*velocity_in_[j];  //backward 1 step
							force_out_[j] = (reference_energy[j] + energy_output_[j])/velocity_in_[j];
							energy_output_[j] += force_out_[j]*velocity_in_[j];  //update 1 step
						}
					}
					else
					{
						force_out_[j] = force_in_[j];
					}
				}
			}
		}
		return true;
	}

	bool updateModifiedEnergy (const T &force_modified,bool invert)
	{
		if (velocity_type_)
		{
			for (int j=0;j<Dof_;j++)
			{
				if (pc_run_[j])
				{
					if (!invert)
						energy_output_[j] -= force_modified[j]*velocity_out_[j];  //update 1 step
					else
						energy_output_[j] += force_modified[j]*velocity_out_[j];  //update 1 step	
				}
			}
		}
	}
	T getForce()
	{
		return force_out_;
	}
	T getVel()
	{
		return velocity_out_;
	}
	T getInputEnergy()
	{
		return energy_input_;
	}

	T getOutputEnergy()
	{
		return energy_output_;
	}

	~PoPc() {
		// TODO Auto-generated destructor stub
	}
};



#endif /* POPC_H_ */

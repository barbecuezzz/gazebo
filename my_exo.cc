#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <math.h>


#include <gazebo/physics/Model.hh>

namespace gazebo
{
  class My_exo : public WorldPlugin
  {
  	

    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->world_ = _parent;
      this->model_ = this->world_->ModelByName("exo");
      this->left_lower_leg = this->model_->GetLink("left_lower_leg");
      this->left_knee_joint = this->model_->GetJoint("left_knee_joint");
      this->left_hip_joint = this->model_->GetJoint("left_hip_joint");
      this->right_hip_joint = this->model_->GetJoint("right_hip_joint");
      this->right_knee_joint = this->model_->GetJoint("right_knee_joint");
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&My_exo::OnUpdate, this));

    }

    // Called by the world update start event
    public: void OnUpdate()
    {	static int i = 0;
    	
    	time2 = this->world_->SimTime();
    	time1 = time2.Float();
    // 	if(i<=29)
    // 	{
	   //  	if(time1 != old_time1 && i%)
	   //  	{
	   //  		this->left_hip_joint->SetPosition(0,hip_joint[i]);
				// this->left_knee_joint->SetPosition(0,knee_joint[i]);
				// i++;
	   //  	}
    // 	}	
    // 	old_time1 = time1;
    	if(time1<=5.0)
    	{
	    	left_hip_angle = -0.01255*pow(time1,3)+0.1819*pow(time1,2)-0.6532*time1-0.08356;
	    	left_knee_angle = 0.06553*pow(time1,3)-0.6638*pow(time1,2)+1.626*time1+0.146;

	    	this->left_hip_joint->SetPosition(0,left_hip_angle);
	    	this->left_knee_joint->SetPosition(0,left_knee_angle);
    	}
    	else if(time1>5.0 && time1<=10.0)
    	{
    		time3 = time1-5.0;
    		left_hip_angle = 0.074162*time1-0.74162;
    		right_hip_angle = -0.01255*pow(time3,3)+0.1819*pow(time3,2)-0.6532*time3-0.08356;
    		right_knee_angle = 0.06553*pow(time3,3)-0.6638*pow(time3,2)+1.626*time3+0.146;
    		this->left_hip_joint->SetPosition(0,left_hip_angle);
    		this->right_hip_joint->SetPosition(0,right_hip_angle);
    		this->right_knee_joint->SetPosition(0,right_knee_angle);
    	}
    	// while(time1<30)
    	// {
    	// 	if((float)time1%0.1==0)
    	// 	{
    	// 		this->left_hip_joint->SetPosition(0,hip_joint[i]);
    	// 		this->left_knee_joint->SetPosition(0,knee_joint[i]);
    	// 		i++;
    	// 	}
    	// }
    	// std::cout << time1 << std::endl;
      // Apply a small linear velocity to the model.
      // this->left_knee_joint->SetJointPosition("left_knee_joint",5);
      // this->left_lower_leg->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
      // this->left_lower_leg->AddForce(ignition::math::Vector3d(5,0,0));
      // this->model->SetJointPosition("left_knee_joint",(1,0,1));
      // p = this->left_knee_joint->GetChild();
      // std::cout << &p << std::endl;
    	// this->left_hip_joint->SetPosition(0,-1);
    	// this->left_knee_joint->SetPosition(0,1);
     	// this->left_knee_joint->SetForce(0,50.0);
      // this->left_lower_leg->AddForce(ignition::math::Vector3d(-50,0,0));
    }

    // Pointer to the model
	public:  physics::WorldPtr world_;		
    private: physics::ModelPtr model_;
    private: physics::LinkPtr left_lower_leg;
    private: physics::LinkPtr p;
    private: physics::JointPtr left_knee_joint;
    private: physics::JointPtr left_hip_joint;
    private: physics::JointPtr right_knee_joint;
    private: physics::JointPtr right_hip_joint;
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
	public: common::Time time2;
	public: float time1;
	public: float time3;
	public: float hip_joint[100] = {0};
	public: float knee_joint[100] = {0.11,0.12,0.13,0.14,0.15,0.16,0.17,0.18,0.19,0.2,0.21,0.22,0.23,0.24,0.25,0.25,0.27,0.28,0.29,0.3,0.31,0.32,0.33,0.34,0.35,0.36,0.37,0.38,0.39,0.4};
	public: common::Time old_time1=0;
	public: float left_hip_angle;
	public: float left_knee_angle;
	public: float right_hip_angle;
	public: float right_knee_angle;	
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(My_exo)
}
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include "build_robot.h"


#include <gazebo/physics/Model.hh>
extern ListNode I1
extern ListNode I2
extern ListNode I3
extern ListNode I4
extern ListNode I5
extern ListNode I6
extern ListNode I7
extern ListNode I8
extern ListNode I9
extern ListNode I10
extern ListNode I11
extern ListNode I12
extern ListNode I13

 
  


namespace gazebo
{
  class My_exo : public WorldPlugin
  {
  	public:  physics::WorldPtr world_;   
    private: physics::ModelPtr model_;
    private: physics::LinkPtr left_lower_leg;
    private: physics::LinkPtr p;
    private: physics::JointPtr left_knee_joint_p;
    private: physics::JointPtr left_hip_joint_p;
    private: physics::JointPtr right_knee_joint_p;
    private: physics::JointPtr right_hip_joint_p;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    public: common::Time time2;
    public: int i;
    public: float time1;
    public: float time3;
    public: float hip_joint[100] = {0};
    public: float knee_joint[100] = {0};
    public: common::Time old_time1=0;
    public: float left_hip_angle;
    public: float left_knee_angle;
    public: float right_hip_angle;
    public: float right_knee_angle; 

    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->world_ = _parent;
      this->model_ = this->world_->ModelByName("exo");
      this->left_lower_leg = this->model_->GetLink("left_lower_leg");
      this->left_knee_joint_p = this->model_->GetJoint("left_knee_joint_p");
      this->left_hip_joint_p = this->model_->GetJoint("left_hip_joint_p");
      this->right_hip_joint_p = this->model_->GetJoint("right_hip_joint_p");
      this->right_knee_joint_p = this->model_->GetJoint("right_knee_joint_p");


      build_robot_model();
      
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&My_exo::OnUpdate, this));

    }

    // Called by the world update start event
    public: void OnUpdate()
    {	static int i = 0;
      i++;
      if(i>1000)
      {
    	
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

  	    	this->left_hip_joint_p->SetPosition(0,left_hip_angle);
  	    	this->left_knee_joint_p->SetPosition(0,left_knee_angle);
      	}
      	else if(time1>5.0 && time1<=10.0)
      	{
      		time3 = time1-5.0;
      		left_hip_angle = 0.074162*time1-0.74162;
      		right_hip_angle = -0.01255*pow(time3,3)+0.1819*pow(time3,2)-0.6532*time3-0.08356;
      		right_knee_angle = 0.06553*pow(time3,3)-0.6638*pow(time3,2)+1.626*time3+0.146;
      		this->left_hip_joint_p->SetPosition(0,left_hip_angle);
      		this->right_hip_joint_p->SetPosition(0,right_hip_angle);
      		this->right_knee_joint_p->SetPosition(0,right_knee_angle);
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
  }

    // Pointer to the model
  
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(My_exo)
}
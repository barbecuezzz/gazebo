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



// using namespace Eigen;
extern double q2, q3, q4, q5, q6, q7;
extern int build_robot_model();
extern void IK_leg(ListNode *E, ListNode *S);
extern ListNode I1;
extern ListNode I2;
extern ListNode I3;
extern ListNode I4;
extern ListNode I5;
extern ListNode I6;
extern ListNode I7;
extern ListNode I8;
extern ListNode I9;
extern ListNode I10;
extern ListNode I11;
extern ListNode I12;
extern ListNode I13;

 
  

using namespace std;
namespace gazebo
{

  class My_exo : public WorldPlugin
  {
  	public:  physics::WorldPtr world_;   
    private: physics::ModelPtr model_;
    private: physics::LinkPtr body;
    private: physics::LinkPtr left_lower_leg;
    private: physics::LinkPtr p;
    private: physics::JointPtr left_knee_joint_p;
    private: physics::JointPtr left_hip_joint_p;
    private: physics::JointPtr left_ankle_joint_p;
    private: physics::JointPtr right_knee_joint_p;
    private: physics::JointPtr right_hip_joint_p;
    private: physics::JointPtr right_ankle_joint_p;

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
    public: float left_ankle_angle;
    public: float right_hip_angle;
    public: float right_knee_angle; 
    public: float right_ankle_angle;

    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->world_ = _parent;
      this->model_ = this->world_->ModelByName("exo");
      this->left_lower_leg = this->model_->GetLink("left_lower_leg");
      this->body = this->model_->GetLink("body");
      this->left_knee_joint_p = this->model_->GetJoint("left_knee_joint_p");
      this->left_hip_joint_p = this->model_->GetJoint("left_hip_joint_p");
      this->left_ankle_joint_p = this->model_->GetJoint("left_ankle_joint_p");
      this->right_hip_joint_p = this->model_->GetJoint("right_hip_joint_p");
      this->right_knee_joint_p = this->model_->GetJoint("right_knee_joint_p");
      this->right_ankle_joint_p = this->model_->GetJoint("right_ankle_joint_p");


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
    	    // this->body->SetForce(ignition::math::Vector3d(10, 0, 0));
         //  this->body->SetForce(ignition::math::Vector3d(-5, 0, 0));
          
          I7.data.p<<0.4,0.3,0.8;
          I1.IK_leg(&I7,&I1);
          I1.ForwardKinematics(&I1);
          // I1.ForwardKinematics(&I1);
          left_hip_angle = q4;
          left_knee_angle = q5;
          left_ankle_angle = q6;
          this->left_hip_joint_p->SetPosition(0,left_hip_angle);
          this->left_knee_joint_p->SetPosition(0,left_knee_angle);
          this->left_ankle_joint_p->SetPosition(0,left_ankle_angle);
          cout<<left_hip_angle<<","<<left_knee_angle<<","<<left_ankle_angle<<endl;


    //   	time2 = this->world_->SimTime();
    //   	time1 = time2.Float();
   
    //   	if(time1<=5.0)
    //   	{
  	 //    	left_hip_angle = -0.01255*pow(time1,3)+0.1819*pow(time1,2)-0.6532*time1-0.08356;
  	 //    	left_knee_angle = 0.06553*pow(time1,3)-0.6638*pow(time1,2)+1.626*time1+0.146;

  	 //    	this->left_hip_joint_p->SetPosition(0,left_hip_angle);
  	 //    	this->left_knee_joint_p->SetPosition(0,left_knee_angle);
    //   	}
    //   	else if(time1>5.0 && time1<=10.0)
    //   	{
    //   		time3 = time1-5.0;
    //   		left_hip_angle = 0.074162*time1-0.74162;
    //   		right_hip_angle = -0.01255*pow(time3,3)+0.1819*pow(time3,2)-0.6532*time3-0.08356;
    //   		right_knee_angle = 0.06553*pow(time3,3)-0.6638*pow(time3,2)+1.626*time3+0.146;
    //   		this->left_hip_joint_p->SetPosition(0,left_hip_angle);
    //   		this->right_hip_joint_p->SetPosition(0,right_hip_angle);
    //   		this->right_knee_joint_p->SetPosition(0,right_knee_angle);
    //   	}
      	
    }
  }

    // Pointer to the model
  
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(My_exo)
}
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <gazebo/sensors/sensors.hh>
#include "build_robot.h"
#include <gazebo/physics/Model.hh>





using namespace Eigen;
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
	private: sensors::SensorPtr imu;
	private:sensors::SensorManager *pMgr = sensors::SensorManager::Instance();
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
    public: double model_position_x;
    public: double model_position_y;
    public: double model_position_z;
    public: double model_position_quaternions_x;
    public: double model_position_quaternions_y;
    public: double model_position_quaternions_z;
    public: double model_position_quaternions_w; 


	public: double acc;
	public: ignition::math::Vector3d linearAcceleration;
	public: ignition::math::Vector3d angularVelocity;
  public: ros::Publisher joint_pub;
  
  public: geometry_msgs::TransformStamped odom_trans;
   
  
  public: ros::Rate loop_rate();
    public: math::Angle hip_joint_position;
    // public: physics::ModelState::GetPose();
	  public: math::Pose model_position;

    
    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->world_ = _parent;
      this->model_ = this->world_->GetModel("exo");
      this->left_lower_leg = this->model_->GetLink("left_lower_leg");
      this->body = this->model_->GetLink("body");
      this->left_knee_joint_p = this->model_->GetJoint("left_knee_joint_p");
      this->left_hip_joint_p = this->model_->GetJoint("left_hip_joint_p");
      this->left_ankle_joint_p = this->model_->GetJoint("left_ankle_joint_p");
      this->right_hip_joint_p = this->model_->GetJoint("right_hip_joint_p");
      this->right_knee_joint_p = this->model_->GetJoint("right_knee_joint_p");
      this->right_ankle_joint_p = this->model_->GetJoint("right_ankle_joint_p");
      
      
      build_robot_model();
      
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "joint_control");
      ros::NodeHandle n;
      joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "ExoBody";
      // tf::TransformBroadcaster broadcaster;
      // ros::Rate loop_rate(30);


      

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&My_exo::OnUpdate, this));

    }

    // Called by the world update start event
    public: void OnUpdate()
    {	
      static int i = 0;
      static float x = 0;
      static float z = 0;
      
      i++;
  //     if(pMgr == nullptr)
		// {
		//    /*ERROR*/
		// }
		// else
		// {
		//     sensors::SensorPtr pSensor = pMgr->GetSensor("imu");
		//     if(pSensor == nullptr)
		//     {
		//        /*ERROR*/
		//     } 
		//     else
		//     {
		//        sensors::ImuSensorPtr pImuSensor = dynamic_pointer_cast<sensors::ImuSensor, sensors::Sensor>(pSensor);
		//        if(pImuSensor == nullptr)
		//        {
		//            /*ERROR*/
		//        }
		//        else
		//        {
		//             /*Do something ... for example to read the linear accel from the IMU:*/
		//             linearAcceleration = pImuSensor->LinearAcceleration();
		//             angularVelocity = pImuSensor->AngularVelocity();
		//        }
		//     }
		// }
      if(i>1000)
      {
          x += 0.1;
          z += 0.1;
    	    // this->body->SetForce(ignition::math::Vector3d(10, 0, 0));
         //  this->body->SetForce(ignition::math::Vector3d(-5, 0, 0));
          // if(angularVelocity[1]>1)
          // {
          // 	this-body->SetForce(ignition::math::Vector3d(10, 0, 0));
          // }
      	  
          I7.data.p<<x,0.3,z;
          I1.IK_leg(&I7,&I1);
          I1.ForwardKinematics(&I1);
          
          left_hip_angle = q4;
          left_knee_angle = q5;
          left_ankle_angle = q6;
          this->left_hip_joint_p->SetPosition(0,left_hip_angle);
          this->left_knee_joint_p->SetPosition(0,left_knee_angle);
          this->left_ankle_joint_p->SetPosition(0,left_ankle_angle);
          // cout<<left_hip_angle<<","<<left_knee_angle<<","<<left_ankle_angle<<endl;
          hip_joint_position = this->left_hip_joint_p->GetAngle(1);
          // model_position = this->model_->physics::ModelState::GetPose();
          model_position = this->model_->GetWorldPose();
          // cout<<model_position<<endl;
          model_position_x = model_position.pos.x;
          model_position_y = model_position.pos.y;
          model_position_z = model_position.pos.z;
          model_position_quaternions_x = model_position.rot.x;
          model_position_quaternions_y = model_position.rot.y;
          model_position_quaternions_z = model_position.rot.z;
          model_position_quaternions_w = model_position.rot.w;
          // cout<<model_position_x<<""<<model_position_roll<<endl;
          // model_position_z = model_position->z;
          // cout<<model_position_z<<endl;


          if(x>0.5)
          {
            x = 0;
            z = 0;
          }
          // cout<<"加速度"<<linearAcceleration<<endl;
          // cout<<"---------------------"<<endl;
          // cout<<"角速度"<<angularVelocity<<endl;

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
      	
    

        if(ros::ok())
        {   
            
            static tf::TransformBroadcaster broadcaster;
            sensor_msgs::JointState joint_state;
            joint_state.header.stamp = ros::Time::now();
            joint_state.name.resize(18);
            joint_state.position.resize(18);
            joint_state.name[0] ="R_HIP_YAW";
            joint_state.position[0] = 0;
            joint_state.name[1] ="R_HIP_ROLL";
            joint_state.position[1] = 0;
            joint_state.name[2] ="R_HIP_PITCH";
            joint_state.position[2] = 0;
            joint_state.name[3] ="R_KNEE_PITCH";
            joint_state.position[3] = 0;
            joint_state.name[4] ="R_ANKLE_ROLL";
            joint_state.position[4] = 0;
            joint_state.name[5] ="R_ANKLE_PITCH";
            joint_state.position[5] = 0;
            joint_state.name[6] ="R_SHOULDER_ROLL";
            joint_state.position[6] = 0;
            joint_state.name[7] ="R_SHOULDER_PITCH";
            joint_state.position[7] = 0;
            joint_state.name[8] ="R_ELBOW_PITCH";
            joint_state.position[8] = 0;
            joint_state.name[9] ="L_HIP_YAW";
            joint_state.position[9] = 0;
            joint_state.name[10] ="L_HIP_ROLL";
            joint_state.position[10] = 0;
            joint_state.name[11] ="L_HIP_PITCH";
            joint_state.position[11] = hip_joint_position.Radian();
            joint_state.name[12] ="L_KNEE_PITCH";
            joint_state.position[12] = 0;
            joint_state.name[13] ="L_ANKLE_ROLL";
            joint_state.position[13] = 0;
            joint_state.name[14] ="L_ANKLE_PITCH";
            joint_state.position[14] = 0;
            joint_state.name[15] ="L_SHOULDER_ROLL";
            joint_state.position[15] = 0;
            joint_state.name[16] ="L_SHOULDER_PITCH";
            joint_state.position[16] = 0;
            joint_state.name[17] ="L_ELBOW_PITCH";
            joint_state.position[17] = 0;

            // joint_state.name[0] ="R_HIP_PITCH";
            // joint_state.position[0] = 
            odom_trans.header.stamp = ros::Time::now();
            odom_trans.transform.translation.x = model_position_x;
            odom_trans.transform.translation.y = model_position_y;
            odom_trans.transform.translation.z = model_position_z;
            // odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0.57);
            odom_trans.transform.rotation.x = model_position_quaternions_x;
            odom_trans.transform.rotation.y = model_position_quaternions_y;
            odom_trans.transform.rotation.z = model_position_quaternions_z;
            odom_trans.transform.rotation.w = model_position_quaternions_w;
            
            cout<<odom_trans.transform.rotation.w<<"------------"<<endl;
            cout<<model_position_quaternions_w<<endl;
            // odom_trans.transform.rotation.x = 0.57;
            // odom_trans.transform.rotation.y = 0.57;
            // odom_trans.transform.rotation.z = 0.57;
            // odom_trans.transform.rotation.w = sqrt(1-3*0.57*0.57);
            joint_pub.publish(joint_state);
            broadcaster.sendTransform(odom_trans);
            // loop_rate.sleep();
        }

    }

  }

    // Pointer to the model
  
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(My_exo)
}
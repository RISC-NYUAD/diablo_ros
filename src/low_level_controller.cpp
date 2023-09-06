#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <time.h>
#include <sstream>
#include <Eigen/Dense>


namespace gazebo
{
class DiabloControlPlugin : public ModelPlugin
{

public: ros::NodeHandle nh;
public: ros::Subscriber velocity_cmd;
public: ros::Publisher left_cmd, right_cmd;
public: double LIM_, smooth_x_vel;
private: double lin_x_cmd, ang_z_cmd, integral, Kp, Kd, Ki, vel_int;
private: physics::LinkPtr base_link;
private: physics::ModelPtr model;
private: event::ConnectionPtr updateConnection;

public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
	{
	  	this->model = _parent;	
	  	smooth_x_vel = 0.0;		
	  	this->lin_x_cmd = 0.0;
	  	this->ang_z_cmd = 0.0;
	  	this->integral = 0.0;
	  	LIM_ = 10.0;
	  	this->Kp = 250.0;
	  	this->Kd = 10.0;
	  	this->Ki = 70.0;
	  	this->vel_int = 0.0;

		std::string ns;
		if(_sdf->HasElement("namespace"))
			ns = _sdf->GetElement("namespace")->Get<std::string>();							
			
		this->base_link = this->model->GetLink(ns+"::Body");		
		
		std::string vel_topic = ns + "/vel_cmd" ;

		velocity_cmd = nh.subscribe(vel_topic, 1, &DiabloControlPlugin::vel_cmd_callback, this); 		
		left_cmd = nh.advertise<std_msgs::Float64>(ns+"/Motor2_L/vel_cmd", 1); 			
		right_cmd = nh.advertise<std_msgs::Float64>(ns+"/Motor2_R/vel_cmd", 1); 			
	
	  	this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&DiabloControlPlugin::onUpdate, this));
		
	}  

public: void onUpdate()
	{
		ignition::math::Pose3d pose = (this->base_link)->WorldCoGPose();
		ignition::math::Quaternion orientation = pose.Rot();
		ignition::math::Vector3d lin_vel = (this->base_link)->WorldLinearVel();
		ignition::math::Vector3d ang_vel = (this->base_link)->RelativeAngularVel();
				
		smooth_x_vel = 0.95*smooth_x_vel + 0.05*(this->lin_x_cmd);
		double vel_error = lin_vel.X() - smooth_x_vel ;
		this->vel_int += 0.009 * vel_error;
		double des_pitch = -1.0*vel_error - 0.1*(this->vel_int);
		
		double ang_vel_error = ang_vel.Z() - (this->ang_z_cmd) ;
		double des_ang_z_rate = -1.2*ang_vel_error ;
		
		double pitch_error = orientation.Y() - des_pitch ;
		this->integral += 0.009 * pitch_error;
		if((this->integral)>LIM_){
			this->integral = LIM_;
		}if((this->integral)<-LIM_){
			this->integral = -LIM_;
		}
		double rate = ang_vel.Y();
		
		double des_vel = (this->Kp)*pitch_error + (this->Ki)*(this->integral) + (this->Kd)*rate ;
		std_msgs::Float64 r_msg, l_msg;
		r_msg.data = 0.5*des_vel + 0.5*des_ang_z_rate;
		l_msg.data = 0.5*des_vel - 0.5*des_ang_z_rate;
		left_cmd.publish(l_msg);
		right_cmd.publish(r_msg);

	}

public: void vel_cmd_callback(const geometry_msgs::Twist& msg)
	{
		this->lin_x_cmd = msg.linear.x;
		this->ang_z_cmd = 2.0*msg.angular.z;
		return;		
	}



};
GZ_REGISTER_MODEL_PLUGIN(DiabloControlPlugin)
}
























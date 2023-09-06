#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
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
public: double LIM_;
private: double cmd, integral, Kp, Kd, Ki, vel_int;
private: physics::LinkPtr base_link;
private: physics::ModelPtr model;
private: event::ConnectionPtr updateConnection;

public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
	{
	  	this->model = _parent;			
	  	this->cmd = 0.0;
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
				
		double vel_error = lin_vel.X() - (this->cmd);
		this->vel_int += 0.009 * vel_error;
		double des_pitch = -1.0*vel_error - 0.1*(this->vel_int);
		
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
		r_msg.data = 0.5*des_vel;
		l_msg.data = 0.5*des_vel;
		left_cmd.publish(l_msg);
		right_cmd.publish(r_msg);

	}

public: void vel_cmd_callback(const std_msgs::Float64& msg)
	{
		this->cmd = msg.data;
		return;		
	}



};
GZ_REGISTER_MODEL_PLUGIN(DiabloControlPlugin)
}
























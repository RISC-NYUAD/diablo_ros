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
class GenericMotorPlugin : public ModelPlugin
{

public: ros::NodeHandle nh;
public: ros::Subscriber position_cmd, velocity_cmd;
public: double LIM_;
private: int mode;
private: double cmd, integral, Kp, Kd, Ki;
private: physics::JointPtr motor_joint;
private: physics::ModelPtr model;
private: event::ConnectionPtr updateConnection;

public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
	{
	  	this->model = _parent;			
		this->mode = 0; //default mode is position
		this->cmd = 0.0;
		this->integral = 0.0;
		this->Kp = 0.5;
		this->Kd = 0.05;
		this->Ki = 0.1;
		LIM_ = 0.5;

		std::string name;
		std::string ns;
		if(_sdf->HasElement("namespace"))
			ns = _sdf->GetElement("namespace")->Get<std::string>();							
		if(_sdf->HasElement("joint_name"))
			name = _sdf->GetElement("joint_name")->Get<std::string>();		
		if(_sdf->HasElement("Kp"))
			this->Kp = _sdf->GetElement("Kp")->Get<double>();				
		if(_sdf->HasElement("Kd"))
			this->Kd = _sdf->GetElement("Kd")->Get<double>();				
		if(_sdf->HasElement("Ki"))
			this->Ki = _sdf->GetElement("Ki")->Get<double>();				
			
		this->motor_joint = this->model->GetJoint(ns+"::"+name);		
		
		std::string pos_topic = ns + "/" + name + "/pos_cmd" ;
		std::string vel_topic = ns + "/" + name + "/vel_cmd" ;

		position_cmd = nh.subscribe(pos_topic, 1, &GenericMotorPlugin::pos_cmd_callback, this); 
		velocity_cmd = nh.subscribe(vel_topic, 1, &GenericMotorPlugin::vel_cmd_callback, this); 		
		
	  	this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&GenericMotorPlugin::onUpdate, this));
		
	}  

public: void onUpdate()
	{
		double torque = 0.0;
		double angle = this->motor_joint->Position(0);
		double rate = this->motor_joint->GetVelocity(0);

		if(mode==0){
			this->integral += 0.005 * (angle - this->cmd) ; 
			if(this->integral > LIM_){
				this->integral = LIM_;
			}
			if(this->integral < -LIM_){
				this->integral = -LIM_;
			}
			torque = -(this->Kp)*(angle - this->cmd) - 2.0*rate - 0.3*this->integral;			
			this->motor_joint->SetForce(0,torque);
			return;
		}
		if(mode==1){
			this->integral += 0.005 * (rate - this->cmd) ; 
			if(this->integral > LIM_){
				this->integral = LIM_;
			}
			if(this->integral < -LIM_){
				this->integral = -LIM_;
			}
			torque = -(this->Kd)*(rate - this->cmd) - (this->Ki)*this->integral;
			this->motor_joint->SetForce(0,torque);
			return;
		}

	}

public: void pos_cmd_callback(const std_msgs::Float64& msg)
	{
		if(this->mode != 0){
			this->integral = 0.0;
		}
		this->mode = 0;
		this->cmd = msg.data;
		return;		
	}

public: void vel_cmd_callback(const std_msgs::Float64& msg)
	{
		if(this->mode != 1){
			this->integral = 0.0;
		}
		this->mode = 1;
		this->cmd = msg.data;
		return;		
	}



};
GZ_REGISTER_MODEL_PLUGIN(GenericMotorPlugin)
}
























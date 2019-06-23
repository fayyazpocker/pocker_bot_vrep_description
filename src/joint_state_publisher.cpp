#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include "pocker_bot_vrep_description/wheel_vel.h"

class Joint_state_publisher
{
	ros::Publisher joint_state_pub;
	ros::Subscriber wheel_vel_sub;
	sensor_msgs::JointState joint_state_msg;
	float joint_position_left;
	float joint_position_right;

public:
	double last_time;
	Joint_state_publisher(){};
	Joint_state_publisher(ros::NodeHandle*, std::string, std::string, int);
	void wheel_vel_callback(const pocker_bot_vrep_description::wheel_vel::ConstPtr& );
	void publish_joint_state();
};

Joint_state_publisher::Joint_state_publisher(ros::NodeHandle *node_handle, std::string joint_state_pub, std::string wheel_vel_sub, int queue_size)
{
	this->joint_state_pub = node_handle->advertise<sensor_msgs::JointState>(joint_state_pub,queue_size);

    this->joint_state_msg.name.resize(2);
    // this->joint_state_msg.velocity.resize(2);
    this->joint_state_msg.position.resize(2);
    this->joint_state_msg.header.frame_id = "wheel";
    this->joint_state_msg.name[0]="joint_left_wheel_link";
    this->joint_state_msg.name[1]="joint_right_wheel_link";

    this->wheel_vel_sub = node_handle->subscribe(wheel_vel_sub,queue_size,&Joint_state_publisher::wheel_vel_callback,this);
}

void Joint_state_publisher::wheel_vel_callback(const pocker_bot_vrep_description::wheel_vel::ConstPtr& wheel_vel)
{
	double current_time = ros::Time::now().toSec();
	double dt = current_time - this->last_time;
	float ldelta_angle = wheel_vel->left_vel * dt;
	float rdelta_angle = wheel_vel->right_vel * dt;
	float langle = this->joint_position_left + ldelta_angle;
	float rangle = this->joint_position_right + rdelta_angle;

	// while(langle <= -3.14) langle +=3.14; //restricting angle betweeen -3.14 to 3.14 radians
	// while(langle > 3.14) langle -=3.14;

	// while(rangle <= -3.14) rangle +=3.14;
	// while(rangle > 3.14) rangle -=3.14;

	this->joint_position_left = langle;
	this->joint_position_right = rangle;
	this->last_time = current_time;
	// ROS_INFO("ldelta_angle : %.3f, langle : %.3f",ldelta_angle,langle);
}

void Joint_state_publisher::publish_joint_state()
{
	this->joint_state_msg.header.stamp = ros::Time::now();
	this->joint_state_msg.position[0] = this->joint_position_left;
	this->joint_state_msg.position[1] = this->joint_position_right;
	this->joint_state_pub.publish(this->joint_state_msg);	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "joint_state_publisher");
	ros::NodeHandle node_handle;
	ros::NodeHandle node_private("~");

	int frequency;
	node_private.param<int>("pub_rate",frequency,5);
	ros::Rate loop_rate(frequency);
	Joint_state_publisher joint_state(&node_handle, "joint_states", "wheel_vel", 1000);

	while(ros::ok()){

		joint_state.publish_joint_state();
		ros::spinOnce();
		loop_rate.sleep();
	}
	
}
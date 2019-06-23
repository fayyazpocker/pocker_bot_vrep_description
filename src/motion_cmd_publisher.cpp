#include<ros/ros.h>
#include<std_msgs/Float32.h>

void cmd_vel_callback(const std_msgs::Float32::ConstPtr& msg)
{
	
}

int main(int argc, char**argv)
{
	ros::init(argc,argv,"motion_cmd_publisher");
	ros::NodeHandle node_handle;
	ros::Publisher lcmd_pub = node_handle.advertise<std_msgs::Float32>("/left_wheel_vel",1000);
	ros::Publisher lcmd_pub = node_handle.advertise<std_msgs::Float32>("/left_wheel_vel",1000);
	ros::Subscriber cmd_vel_sub = node_handle.subscribe("/cmd_vel",1000,cmd_vel_callback);
}
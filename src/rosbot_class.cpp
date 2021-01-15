#include "rosbot/rosbot_class.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "unistd.h"

// rosbot constructor
RosbotClass::RosbotClass(){
	nh = ros::NodeHandle("~"); // create private Nodehandle
	//laser
	laser_topic = "/scan";
	laser_sub = nh.subscribe(laser_topic, 10, &RosbotClass::laser_callback, this);
	//velocity
	vel_topic = "/cmd_vel";
	vel_pub = nh.advertise<geometry_msgs::Twist>(nh.resolveName(vel_topic),1);
	//odometry
	odom_topic = "/odom";
	odom_sub = nh.subscribe(odom_topic, 10, &RosbotClass::odom_callback, this);

	//print of screen/terminal
	ROS_INFO("Initializing ROSbot's Node....");
	usleep(2000000); // sleep for 2 seconds
}

// define all methods

// laser callback method
void RosbotClass::laser_callback(const sensor_msgs::LaserScan::ConstPtr &laser_msg){
	laser_range = laser_msg->ranges;
}

// odom callback method
void RosbotClass::odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg){
	x_pos = odom_msg->pose.pose.position.x;
	y_pos = odom_msg->pose.pose.position.y;
	z_pos = odom_msg->pose.pose.position.z;
	ROS_INFO_STREAM("Odometry: x=" << x_pos << " y=" << y_pos << " z=" << z_pos);
}

// moves robot for 2 seconds
void RosbotClass::move()
{
	ros::Rate rate(10); //rate of publishing (Hz)
	ros::Time start_time = ros::Time::now();
	ros::Duration timeout(2.0); // timeout for 2 seconds
	while(ros::Time::now() - start_time < timeout){
		ros::spinOnce();
		vel_msg.linear.x += 0.2;
		vel_msg.angular.z = 0.0;
		vel_pub.publish(vel_msg);
		rate.sleep();
	}
	vel_msg.linear.x = 0.0;
	vel_msg.angular.z = 0.0;
	vel_pub.publish(vel_msg);
}

// moves robot forward for given seconds
void RosbotClass::move_forward(int seconds)
{
	ros::Rate rate(10); //rate of publishing (Hz)
	ros::Time start_time = ros::Time::now();
	ros::Duration timeout(seconds); // timeout for certain seconds
	while(ros::Time::now() - start_time < timeout){
		ROS_INFO_STREAM("Moving Forward....");
		ros::spinOnce();
		vel_msg.linear.x = 0.5;
		vel_msg.angular.z = 0.0;
		vel_pub.publish(vel_msg);
		rate.sleep();
	}
	vel_msg.linear.x = 0.0;
	vel_msg.angular.z = 0.0;
	vel_pub.publish(vel_msg);
}

//moves robot backwards for given seconds
void RosbotClass::move_backward(int seconds)
{
	ros::Rate rate(10); //rate of publishing (Hz)
	ros::Time start_time = ros::Time::now();
	ros::Duration timeout(seconds); // timeout for certain seconds
	while(ros::Time::now() - start_time < timeout){
		ROS_INFO_STREAM("Moving Backward....");
		ros::spinOnce();
		vel_msg.linear.x = -0.5;
		vel_msg.angular.z = 0.0;
		vel_pub.publish(vel_msg);
		rate.sleep();
	}
	vel_msg.linear.x = 0.0;
	vel_msg.angular.z = 0.0;
	vel_pub.publish(vel_msg);
}

// turns robot either clockwise and counterclockwise
void RosbotClass::turn(std::string direction,int seconds){
	ros::Rate rate(10); //rate of publishing (Hz)
	ros::Time start_time = ros::Time::now();
	ros::Duration timeout(seconds); // timeout for certain seconds	
	double wz = 0.0;
	if (direction == "clockwise"){
		ROS_INFO_STREAM("Turning clockwise...");
		wz = -2.5;
	}
	else if (direction == "counterclockwise"){
		ROS_INFO_STREAM("Turning counterclockwise...");
		wz = 2.5;
	}
	while(ros::Time::now() - start_time < timeout){
		ros::spinOnce();
		vel_msg.linear.x = 0.5;
		vel_msg.angular.z = wz;
		vel_pub.publish(vel_msg);
		rate.sleep();
	}
	vel_msg.linear.x = 0.0;
	vel_msg.angular.z = 0.0;
	vel_pub.publish(vel_msg);
}

// stop the robot
void RosbotClass::stop_moving(){
	ROS_INFO_STREAM("Stopping robot...");
	vel_msg.linear.x = 0.0;
	vel_msg.angular.z = 0.0;
	vel_pub.publish(vel_msg);
}

// get robot position
float RosbotClass::get_position(int coord){
	if (coord == 1) return this->x_pos;
	else if (coord == 2) return this->y_pos;
	else if (coord == 3) return this->z_pos;
	return 0;
}

// get system time in seconds
double RosbotClass::get_time(){
	double sec = ros::Time::now().toSec();
	return sec;
}

// get laser value at particular index
float RosbotClass::get_laser(int index){
	return this->laser_range[index];
}

// get entire laser array
float* RosbotClass::get_laser_full(){
	float *laser_pointer = laser_range.data();
	return laser_pointer;
}

// get entire position as list
std::list<float> RosbotClass::get_position_full() {
  std::list<float> coordinates({this->x_pos, this->y_pos, this->z_pos});
  return coordinates;
}

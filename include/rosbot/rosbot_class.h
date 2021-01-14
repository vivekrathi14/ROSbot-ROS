#pragma once
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include <string>
#include <list>
#include <vector>


class RosbotClass {
	private:
		// Node handle to communicate with ROS nodes
		ros::NodeHandle nh;
		// Subscriber for laser scan topic
		ros::Subscriber laser_sub;
		std::string laser_topic;
		std::vector<float> laser_range;

		// Subscriber for odometry topic
		ros::Subscriber odom_sub;
		std::string odom_topic;
		float x_pos,y_pos,z_pos;

		// Publisher for velocity data
		ros::Publisher vel_pub;
		std::string vel_topic;
		geometry_msgs::Twist vel_msg;

		// Subscriber callback functions
		void laser_callback(const sensor_msgs::LaserScan::ConstPtr &laser_msg);
		void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg);

	public:
		// Class constructor
		RosbotClass();
		// Various methods for robot interaction

		void move(); //moves for 2 seconds
		void move_forward(int seconds); //move forward for certain seconds
		void move_backward(int seconds); // move backward for certain seconds
		void turn(std::string direction,int seconds); // turn either clockwise/anticlockwise for certain seconds
		void stop_moving(); // stop the robot
		double get_time(); // get the system time
		float get_position(int coord); // get position at particular coordinate
		float get_laser(int index); // get laser scan value at particular index (out of 0..718)
		float *get_laser_full(); // get entire laser array
		std::list<float> get_position_full(); // get all x,y,z position as list
};
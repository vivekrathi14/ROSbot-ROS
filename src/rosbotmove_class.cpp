#include "rosbot/rosbotmove_class.h"

// define methods
void RosbotMove::get_exit(){
	rosbot.move_forward(1); // move forward for 1 second
	while (rosbot.get_laser(0) > 1.75){ // move forward untill you reach close to the wall
		ROS_INFO_STREAM("Laser reading: " << rosbot.get_laser(0));
		rosbot.move_forward(1);
	}

	// turn right
	rosbot.turn("clockwise",3);
	rosbot.move_forward(1);
	// turn left
	rosbot.turn("counterclockwise",3);

	// validate the distance using distance formula
	float x0 = rosbot.get_position(1); // initial position
	float y0 = rosbot.get_position(2);
	float x1=x0;
	float y1=y0;
	float dist = this->calc_distance(x0,y0,x1,y1);
	while (dist < 8.00){
		x1 = rosbot.get_position(1);
		y1 = rosbot.get_position(2);
		dist = this->calc_distance(x0,y0,x1,y1);
		ROS_INFO_STREAM("Distance travelled: " << dist);
		//keep moving
		rosbot.move_forward(1);
		x0 = x1;
		y0 = y1;
    }
    rosbot.turn("clockwise", 3);
  	rosbot.move_forward(5);
  	ROS_INFO_STREAM("Success!!!");
}

float RosbotMove::calc_distance(float x0, float y0, float x1, float y1) {
  return std::sqrt(std::pow((x0 - x1), 2) + std::pow((x0 - x1), 2));
}

#include "rosbot/rosbotmove_class.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "rosbot_move_node");
  RosbotMove rosbot_moves;
  rosbot_moves.get_exit();
}
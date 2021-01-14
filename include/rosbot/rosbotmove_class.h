#pragma once
#include "rosbot/rosbot_class.h"
#include <math.h>

class RosbotMove{
	public:
		RosbotClass rosbot;
		void get_exit();
		float calc_distance(float x0,float y0, float x1, float y1);
};
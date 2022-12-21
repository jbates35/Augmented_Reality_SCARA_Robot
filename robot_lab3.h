#pragma once

#include "robot_base.h"

//In this we draw a simple set of boxes that we can then manipulate with a camera
//The point of this lab is the camera, not the boxes, really

class CRobot_3 : public CRobot_base 
{
public:
	CRobot_3();
	~CRobot_3();

	/**
	 ** @brief Creates simple robot
	 ***/
	void create();
	
	/**
	 ** @brief Draws the robot
	 ***/
	void draw();
	
private:
	//Box struct for easy iteration later on
	struct box {
		vector<Mat> shape;
		Scalar color;
	};
	vector<box> _simple_robot;	
	
	
};
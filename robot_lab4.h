#pragma once

#include "robot_base.h"

//In this we take the robot from last lab and just place it on an ChAuRco board

class CRobot_4 : public CRobot_base 
{
public:
	CRobot_4();
	~CRobot_4();

	/**
	 ** @brief Creates augmented reality robot
	 ***/
	void create();
	
	/**
	 ** @brief Draws the augmented reality robot
	 ***/
	void draw();
	
private:
	//Box struct for easy iteration later on
	struct box {
		vector<Mat> shape;
		Scalar color;
	};
	vector<box> _simple_robot;	
	
	double turn_timer;	
};
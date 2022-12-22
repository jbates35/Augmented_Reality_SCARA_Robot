#pragma once

#include "robot_base.h"

//In this lab we build a SCARA robot and use forward kinematics

class CRobot_5 : public CRobot_base
{
public:
	CRobot_5();
	~CRobot_5();
	
	/**
	** @brief Creates augmented reality robot
	***/
	void create();
	
	/**
	 ** @brief Change robot position, detect aruco, etc.
	 ***/
	void update();
	
	/**
	 ** @brief Draws the augmented reality robot
	 ***/
	void draw();
	
protected:
	//Initializes important variables
	void init();

	//Holds the coordinates and various info for the robot components
	struct _box_struct {
		vector<Mat> shape;
		Scalar color;
		Mat transpose;
		Mat rotate;
	};
	vector<_box_struct> _robot;	
	
	//Joint angles and z
	vector<int> _joint;
	
	//Min and maxes of the joint angles and z
	vector<int> _joint_min, _joint_max;
	
	//Names for joints
	vector<string> _joint_names;
	
	//For state machine
	int _stage, _count, _do_animate;	
	
	//Draws trackbars, etc.
	void update_fkine();
	
	//Calculates framerate, updates image
	void frame_rate();
	
	//For keeping track of frame time
	double frame_time_beg, frame_time_end, frame_time, frame_freq;
	vector<double> frame_time_vec;
	
	//The current view at the end of all translations
	Mat _current_view;
};
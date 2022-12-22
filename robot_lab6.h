#pragma once

#include "robot_lab5.h"

//Same as lab 5 but implementing ikine 

class CRobot_6 : public CRobot_5
{
public:
	CRobot_6();
	~CRobot_6();
	
	/**
	 ** @brief Change robot position, detect aruco, etc.
	 ***/
	void update();

protected:
	//Initializes important variables
	void init();

	//Draws trackbars, etc.
	void update_ikine();
	
	//Calculates joint positions based on input x and y positions
	vector<int> ikine_calculate(int x_in = 0, int y_in = 0, bool positive = true);
	
	//Checks to see if which equation we should be choosing
	void check_make_positive();
	
	//Fills xyz/R values when kin_select is fkine
	void fkine();
	
	//Fills q1 q2 qz q3 values when kin_select is ikine
	void ikine();
	
	//Names of ikine coordinates
	vector<string> _ikine_names;
	
	//Value of ikine coordinates
	vector<int> _icoord;
	
	//Min and max of ikine coordinates (xyz and one rotation)
	vector<int> _icoord_min, _icoord_max;	
	
	//Which one of the equations to choose for ikine calculate
	int _equation_select;
	
	// false for froward kinematics, true for inverse kinematics
	bool _kin_select; 
	
	//For checking start of ikine animation
	Point2i _start_point;
	
	//Increment amount for speed
	Point2i _increment;
	
	//Corner points to go from and to with ikine animation
	vector<Point2i> _corner_point;
	vector<Point2i> _corner_incs;
	
	//Increment amount based on unit vector
	Point2f _diff_norm;
	
	//State machine variables for ikine animation
	int _do_animate_inv;
	int _istage, _icount;
	int _istart;
	

};
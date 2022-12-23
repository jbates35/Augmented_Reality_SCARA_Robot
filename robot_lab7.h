#pragma once

#include "robot_lab6.h"

//In this lab we use trajectories to traverse the robot between markers

class CRobot_7 : public CRobot_6
{
public:
	CRobot_7();
	~CRobot_7();

	/**
	 ** @brief Change robot position, detect aruco, etc.
	 ** @brief Change robot posi tion, detect aruco, etc.
	 ***/	
	void update();
	
private:
	//Initialize all variables	
	void init();
	
	//Create buttons for jtraj and ctraj
	void update_traj();
	
	//Gives a vector of incrementing positions in a vector
	vector<float> jtraj(float s0, float sT, float v0 = 0, float vT = 0, int steps = 50, float T = TOTAL_TIME);
	
	//Takes care of figuring out the next marker position
	void ctraj();
	
	//Decides whether ctraj or jtraj will be on
	bool _ctraj_on, _jtraj_on;
	
	//Keeps track of current position in jtraj vector
	int pose_counter;
	
	//Arrays for each joint that will be passed through jtraj
	vector<float> jtraj_vec_q1, jtraj_vec_q2, jtraj_vec_q3, jtraj_vec_z;
	
	//Arrays for each coordinate that will be passed through jtraj
	vector<float> ctraj_vec_x, ctraj_vec_y, ctraj_vec_z, ctraj_vec_yaw;
	
	//Used to hold current and next marker place
	vector<vector<int>> _ctraj_pose;
	
	//Used to signify we haven't gotten any markers yet
	bool _ctraj_begin;
	
	//Direction of jtraj 
	int dir;
	
	//Ctraj state machine
	int ctraj_state;

};
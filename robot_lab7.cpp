#include "robot_lab7.h"

CRobot_7::CRobot_7()
{
	//Static jtrajectories
	jtraj_vec_q1 = jtraj(0, -180, 0, 0, STEP_COUNT);
	jtraj_vec_q2 = jtraj(0, 90, 0, 0, STEP_COUNT);
	jtraj_vec_q3 = jtraj(0, 90, 0, 0, STEP_COUNT);
	jtraj_vec_z = jtraj(0, 150, 0, 0, STEP_COUNT);
	
	init();
}


CRobot_7::~CRobot_7()
{
}


void CRobot_7::init()
{
	// reset state machine variables for fkine animation
	_do_animate = 0;	
	_stage = 0;
	_count = 0;

	//Reset robot joint positions
	_joint.clear();
	for (int i = 0; i < 4; i++) _joint.push_back(0);
	_joint[2] = 75;	
	
	//Reset inverse kinematic coordinates
	_icoord.clear();
	for (int i = 0; i < 4; i++) _icoord.push_back(0);
	_icoord[0] = 300;

	//Inverse kinematics animation state machine
	_istart = 0;
	_istage = 0;
	_icount = 0;
	
	//Which eq to choose from for inverse kinematics calculate
	_equation_select = true;

	//Starts off with forward kinematics
	_kin_select = false;

	//Inverse kinematics state machine
	_do_animate_inv = 0;
	
	//Follow aruco marker instead of setting trackbars
	_follow_marker = false;
	
	//Whether jtraj or ctraj is enabled
	_jtraj_on = false;
	_ctraj_on = false;
	
	//Place in vector, reset
	pose_counter = 0;
	dir = 1;
	
	//Empty ctraj array and tell rest of code that it has been cleared
	_ctraj_pose.clear();
}


void CRobot_7::update()
{
	//Detect aruco to see charuco board
	if (_worldview) 
	{
		_virtualcam.detect_aruco(_canvas, _canvas_copy);
	}
	
	//Draw trackbars and animation for fkine/ikine
	update_fkine();	
	update_ikine();
	update_traj();
	
	//JtraJ if selected (move robot joints)
	if (_jtraj_on) {
		//Store new values into pose
		_joint[0] = jtraj_vec_q1[pose_counter];
		_joint[1] = jtraj_vec_q2[pose_counter];
		_joint[3] = jtraj_vec_q3[pose_counter];
		_joint[2] = jtraj_vec_z[pose_counter];

		//increment or decrement counter
		pose_counter = pose_counter + dir;

		//Change direction of counter when they reach max and min
		if (pose_counter >= STEP_COUNT - 1)
			dir = -1;
		if (pose_counter <= 0)
			dir = 1;

		fkine();
	}

	//Ctraj if selected
	if (_ctraj_on) {
		if (ctraj_state == -1 && _virtualcam.markers_found()) {			
			//Initiate first two marker pose
			ctraj_state = 0;
			_ctraj_pose.push_back(_virtualcam.get_pose(ctraj_state));
			
			ctraj();
		}

		if (ctraj_state != -1) {
			//Dump vals into xyz and yaw
			_icoord[0] = ctraj_vec_x[pose_counter];
			_icoord[1] = ctraj_vec_y[pose_counter];
			_icoord[2] = ctraj_vec_z[pose_counter];
			_icoord[3] = ctraj_vec_yaw[pose_counter];

			pose_counter++;

			//Reached end of step count
			if (pose_counter >= STEP_COUNT) {
				pose_counter = 0;
				//If we have no markers found, don't go into algorithm
				if (!_virtualcam.markers_found()) {
					_ctraj_on = false;
				}
				else {
					ctraj();
				}
			}
		}
		ikine();
	}
	
	
}

void CRobot_7::update_traj()
{
	Point _setting_window;

	_setting_window.x = _canvas_copy.size().width - 220;
	_setting_window.y = 585;
	
	cvui::window(_canvas_copy, _setting_window.x, _setting_window.y, 220, 80, "Trajectory Settings");
	
	_setting_window.x += 5;
	_setting_window.y += 25;

	//Button for jtraj
	if (cvui::button(_canvas_copy, _setting_window.x, _setting_window.y, 100, 30, "jtraj")) {
		_jtraj_on = !_jtraj_on;
		_ctraj_on = false;
		dir = 1;
		pose_counter = 0;
	}

	//Button for ctraj
	if (cvui::button(_canvas_copy, _setting_window.x + 110, _setting_window.y, 100, 30, "ctraj")) {
		_ctraj_on = !_ctraj_on;
		_jtraj_on = false;
		ctraj_state = -1;
		pose_counter = 0;
	}
	
	_setting_window.y += 40;

	Scalar traj_color = RED;
	if (_ctraj_on) traj_color = GREEN;

	//If ctraj or jtraj on, RED or GREEN, else make this biatch gray
	if (_ctraj_on || _jtraj_on)
		circle(_canvas_copy, Point2i(_setting_window.x + 100, _setting_window.y), 8, traj_color, -1);
	else
		circle(_canvas_copy, Point2i(_setting_window.x + 100, _setting_window.y), 8, Scalar(144, 144, 144), -1);
	

}

vector<float> CRobot_7::jtraj(float s0, float sT, float v0 /* = 0 */, float vT /* = 0 */, int steps /* = 50 */, float T /* = TOTAL_TIME */)
{
	//Goal time is 10Hz, i.e. 50 frames.
	vector<float> pos_vec;

	//Get amount of time per step
	float time_per_step = T / steps;

	double T5 = T * T * T * T * T;
	double T4 = T * T * T * T;
	double T3 = T * T * T;
	double T2 = T * T;

	//From the notes, coef mat will be calculated from this
	Mat T_mat = (Mat1f(6, 6) <<
		0, 0, 0, 0, 0, 1,
		T5, T4, T3, T2, T, 1,
		0, 0, 0, 0, 1, 0,
		5*T4, 4*T3, 3*T2, 2*T, 1, 0,
		0, 0, 0, 2, 0, 0,
		20*T3, 12*T2, 6*T, 2, 0, 0
		);
	
	//ABCDEF are calculated with this as well
	Mat traj_mat = (Mat1f(6, 1) <<
		s0, sT, v0, vT, 0, 0
		);

	//Calculate mat and store values in array
	Mat coef_mat = T_mat.inv() * traj_mat;
	vector<float> coefs;

	for (int i = 0; i < coef_mat.rows; i++) {
		coefs.push_back(coef_mat.at<float>(i));
	}

	//With those coefficients, we now need to calculate the time steps
	//We can use just the position, time_per_step, store to vector and return it
	vector<double> time_vec;
	float pos;

	for (int i = 0; i < steps; i++) {
		
		//Clear variables we need
		time_vec.clear();
		pos = 0;

		//Get current time through iterative process
		float t = time_per_step * i;

		//Get values to get multiplied by coefs
		time_vec.push_back(t * t * t * t * t);
		time_vec.push_back(t * t * t * t);
		time_vec.push_back(t * t * t);
		time_vec.push_back(t * t);
		time_vec.push_back(t);
		time_vec.push_back(1);

		//Mult coefs by time equation
		for (int j = 0; j < coefs.size(); j++)
			pos += coefs[j] * time_vec[j];

		//Push back to position vector
		pos_vec.push_back(pos);
	}

	return pos_vec;
}


void CRobot_7::ctraj()
{
	//Store current pose into starting pos	
	if (_ctraj_pose.size()>1)
	{
		_ctraj_pose.erase(_ctraj_pose.begin()); 
	}
	
	//increment ctraj and reset if it reaches max marker count
	ctraj_state++;
	if (ctraj_state >= _virtualcam.marker_count())
		ctraj_state = 0;

	//Get next pose to compare with to jtraj
	_ctraj_pose.push_back(_virtualcam.get_pose(ctraj_state));

	//Get jtraj of these two markers (rpy,xyz)
	ctraj_vec_x = jtraj((float)_ctraj_pose[0][3], (float)_ctraj_pose[1][3], 0, 0, STEP_COUNT);
	ctraj_vec_y = jtraj((float)_ctraj_pose[0][4], (float)_ctraj_pose[1][4], 0, 0, STEP_COUNT);
	ctraj_vec_z = jtraj((float)_ctraj_pose[0][5], (float)_ctraj_pose[1][5], 0, 0, STEP_COUNT);
	ctraj_vec_yaw = jtraj((float)_ctraj_pose[0][2], (float)_ctraj_pose[1][2], 0, 0, STEP_COUNT);
}


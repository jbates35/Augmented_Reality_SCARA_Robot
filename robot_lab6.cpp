#include "robot_lab6.h" 

CRobot_6::CRobot_6()
{
	
	//Boundaries of joint angles or distances (4th entry)
	_icoord_min = { -325, -325, 0, -180 };
	_icoord_max = { 325, 325, 200, 180 };
	
	//Names of inverse kinematics	
	_ikine_names = { "X", "Y", "Z", "R" };
	
	init();	
}


CRobot_6::~CRobot_6()
{
}

void CRobot_6::init()
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
}

//Update trackbars
void CRobot_6::update()
{
	//Detect aruco to see charuco board
	if (_worldview) 
	{
		_virtualcam.detect_aruco(_canvas, _canvas_copy);
	}
	
	//Draw trackbars and animation for fkine/ikine
	update_fkine();	
	update_ikine();
}


void CRobot_6::calculate()
{	
	//Either find joint angles or find xyz coordinates
	if (_kin_select)
	{
		ikine();		
	}
	else
	{
		fkine();
	}
}


//Change trackbars for ikine
void CRobot_6::update_ikine()
{
	Point _setting_window;

	_setting_window.x = _canvas_copy.size().width - 195;
	_setting_window.y = 170;
	
	//Inverse kinematic parameters
	if (_lab >= 6) 
	{
		//Create trackbars and label them for inverse kinematics
		for (int i = 0; i < _joint.size(); i++) {
			cvui::trackbar(_canvas_copy, _setting_window.x, _setting_window.y, 180, &_icoord[i], _icoord_min[i], _icoord_max[i]);
			cvui::text(_canvas_copy, _setting_window.x + 180, _setting_window.y + 20, _ikine_names[i]);

			_setting_window.y += 45;
		}
	}
	
	//Button for inverse kinematics animate
	if (cvui::button(_canvas_copy, _setting_window.x, _setting_window.y, 100, 30, "IAnimate")) {
		_kin_select = true;
		_do_animate_inv = 1;
		_istage = 0;
		_icount = 0;
	}

	//Variables for kinematic type indicator
	string kin_type;
	Scalar kin_color;

	//If inverse kinematics, circle will be green
	if (_kin_select) 
	{
		kin_type = "inverse";
		kin_color = GREEN;
	}
	//If forward kinematics, circle will be red
	else
	{
		kin_type = "forward";
		kin_color = RED;		
	}

	if (cvui::button(_canvas_copy, _setting_window.x + 110, _setting_window.y, 100, 30, kin_type)) {
		_kin_select = !_kin_select;
		check_make_positive();
	}

	_setting_window.y += 35;

	//Indicating color for kinematic type
	circle(_canvas_copy, Point2i(_setting_window.x + 160, _setting_window.y + 8), 8, kin_color, -1);	
	
	//Inverse kinematics animation state machine
	if (_do_animate_inv != 0) 
	{		
		//Points for difference between current and next point, plus the magnitude (to get unit vector)
		Point2i diff;
		float diff_mag;

		//Different values for temporary x y values
		int x;
		int y;
		
		//Current part of state machine
		int i;
		
		//Next part of state machine
		int inext;

		switch (_do_animate_inv) 
		{
		case 1:
			//Find out the closest point
			x = _icoord[0];
			y = _icoord[1];
			_start_point = Point2i(x, y);

			//Figure out which quadrant we are in and move accordingly
			if (x >= 0) 
			{
				if (y >= 0) _istart = 0;
				else _istart = 3;
			}
			else 
			{
				if (y >= 0) _istart = 1;
				else _istart = 2;
			}

			//Calculate unit vector 
			diff = _corner_point[_istart] - _start_point;
			diff_mag = sqrt(diff.x * diff.x + diff.y * diff.y);
			_diff_norm = Point2f((float)diff.x / diff_mag, (float)diff.y / diff_mag);

			_icount = 1;
			_istage = 0;
			_do_animate_inv++;
			break;
		case 2:
			//Move towards the goal
			_icoord[0] = (int)round(_start_point.x + _icount * ANIMATE_INCREMENT * _diff_norm.x);
			_icoord[1] = (int)round(_start_point.y + _icount * ANIMATE_INCREMENT * _diff_norm.y);

			//If we are already at our point in either x or y, don't increment further
			if (abs(_icoord[0] - _corner_point[_istart].x) < 5)
				_icoord[0] = _corner_point[_istart].x;			
			if (abs(_icoord[1] - _corner_point[_istart].y) < 5)
				_icoord[1] = _corner_point[_istart].y;

			_icount++;
			
			//If both coordinates are at position, move to next part of state machine
			if (_icoord[0] == _corner_point[_istart].x && _icoord[1] == _corner_point[_istart].y) {
				_do_animate_inv++;
				_icount = 1;
			}
			break;
		case 3:
			i = _istart + _istage;
			inext = i + 1;

			//Move towards the goal
			_icoord[0] = _icoord[0] + _corner_incs[i].x;
			_icoord[1] = _icoord[1] + _corner_incs[i].y;

			//If we are already at our point in either x or y, don't increment further
			if (abs(_icoord[0] - _corner_point[inext].x) <= 5)
				_icoord[0] = _corner_point[inext].x;			
			if (abs(_icoord[1] - _corner_point[inext].y) <= 5)
				_icoord[1] = _corner_point[inext].y;

			_icount++;

			//If both coordinates are at position, move to next part of state machine
			if (abs(_icoord[0] - _corner_point[inext].x) < 5 && abs(_icoord[1] - _corner_point[inext].y) < 5) 
			{
				_istage++;
				_icount = 1;
			}

			if (_istage == 4) 
			{
				_do_animate_inv++;				
				
				//Get new point and increase stage
				diff = _start_point - _corner_point[_istage + _istart];
				diff_mag = sqrt(diff.x * diff.x + diff.y * diff.y);
				_diff_norm = Point2f((float)diff.x / diff_mag, (float)diff.y / diff_mag);
				_icount = 1;
			}
			break;
		case 4:
			//Move towards the goal
			_icoord[0] = (int)round(_corner_point[_istage + _istart].x + _icount * ANIMATE_INCREMENT * _diff_norm.x);
			_icoord[1] = (int)round(_corner_point[_istage + _istart].y + _icount * ANIMATE_INCREMENT * _diff_norm.y);

			//If we are already at our point in either x or y, don't increment further
			if (abs(_icoord[0] - _start_point.x) < 5)
				_icoord[0] = _start_point.x;
			if (abs(_icoord[1] - _start_point.y) < 5)
				_icoord[1] = _start_point.y;

			_icount++;
			
			//If both coordinates are at position, we are done the state machine so reset
			if (abs(_icoord[0] - _start_point.x) < 5 && abs(_icoord[1] - _start_point.y) < 5)
				init();
			break;
		default:
			_do_animate_inv = 0;

		} // end switch _do_animate_inv
	} // end if _do_animate_inv
}


vector<int> CRobot_6::ikine_calculate(int x_in /* = 0 */, int y_in /* = 0 */, bool positive /* = true */)
{
	//Change ints to floats (maybe don't need this, might be because I'm used to embedded? T_T)
	float x = (float)x_in;
	float y = (float)y_in;

	//Joint angles in radians
	float q1, q2;

	//Avoid infinity solutions
	if (x == 0) x = 0.000001;
	if (y == 0) y = 0.000001;

	//Inverse kinematic num and denoms 1 (correlates to 'true')
	float num1 = sqrt(-66015625 + 106250 * y * y - y * y * y * y + 106250 * x * x - 2 * x * x * y * y - x * x * x * x);
	float denom1 = -8125 + y * y + 300 * x + x * x;
	
	//Inverse kinematic num and denoms 2 (correlates to 'false')
	float num2 = sqrt(-1 * (-625 + y * y + x * x) * (-105625 + y * y + x * x));
	float denom2 = -625 + y * y + x * x;
	
	//Calculating inverse kinematics with atan
	if (positive) 
	{
		q1 = round(360 / PI * atan2(300 * y + num1, denom1));
		q2 = round(-360 / PI * atan2(num2, denom2));
	}
	else 
	{
		//Alternate equations
		q1 = round(360 / PI * atan2(300 * y - num1, denom1));
		q2 = round(360 / PI * atan2(num2, denom2));
	}

	//Avoid non-sensical equations
	if (q1 > 180) q1 -= 360;
	if (q1 < -180) q1 += 360;
	if (q2 > 180) q2 -= 360;
	if (q2 < -180) q2 += 360;

	//Return joint values
	vector<int> q = { 
		(int)q1, 
		(int)q2 
		};
	return q;
}

//Checks to which equation correlates to the current angles, and then switches equation selector accordingly
void CRobot_6::check_make_positive()
{
	//Match current joints moving forward
	vector<int> q = ikine_calculate(_icoord[0], _icoord[1], true);
	if (q[0] == _joint[0] && q[1] == _joint[1]) 
	{	
		_equation_select = true;
	}

	//Redo equations and see if the new equations match matches
	q.clear();
	
	q = ikine_calculate(_icoord[0], _icoord[1], false);
	if (q[0] == _joint[0] && q[1] == _joint[1]) 
	{
		_equation_select = false;
	}
}


//Fills xyz/R values when kin_select is fkine
void CRobot_6::fkine()
{
	//Extract important matrix values from _current_view
	float r11, r21, r31, r32, r33;

	r11 = _current_view.at<float>(0, 0);
	r21 = _current_view.at<float>(1, 0);
	r31 = _current_view.at<float>(2, 0);
	r32 = _current_view.at<float>(2, 1);
	r33 = _current_view.at<float>(2, 2);

	//Calculate roll/pitch/yaw from extracted values
	float pitch = atan2(-1 * r31, sqrt(r11 * r11 + r21 * r21));
	float roll = 180 / PI * atan2(r32 / cos(pitch), r33 / cos(pitch));
	float angle = (int)round(roll);
	if (angle > 180) angle -= 360;
	if (angle <= -180) angle += 360;

	//Temporary placeholders for x and y
	float x, y;
	float q1 = PI / 180 * _joint[0];
	float q2 = PI / 180 * _joint[1];

	//Calculate x and y from first two joint angles
	x = 175 * cos(q1 + q2) + 150 * cos(q1);
	y = 175 * sin(q1 + q2) + 150 * sin(q1);

	//Store values
	_icoord[0] = (int)round(x);
	_icoord[1] = (int)round(y);
	_icoord[2] = _joint[2] + 25;
	_icoord[3] = angle;	
}

//Fills q1 q2 qz q3 values when kin_select is ikine
void CRobot_6::ikine()
{
	//Capture aruco marker on board, if there is one!
	if (_virtualcam.can_draw_ikine() && _virtualcam.get_valid_pose()) 
	{		
		//Aruco marker position
		_icoord[0] = _virtualcam.box.x;
		_icoord[1] = _virtualcam.box.y;
		_icoord[2] = _virtualcam.box.z;
		_icoord[3] = _virtualcam.box.yaw;
	
	//Figure out the magnitude of the x and y
	float radius = sqrt(_icoord[0] * _icoord[0] + _icoord[1] * _icoord[1]);

	//Make sure requested x y values are possible in first place1
	if (radius <= MAX_ICOORD && radius > MIN_ICOORD) 
	{
		// a1, a2 from x, y
		vector<int> q;
		q = ikine_calculate(_icoord[0], _icoord[1], _equation_select);
		_joint[0] = q[0];
		_joint[1] = q[1];
	}

	//Z
	_joint[2] = _icoord[2] - 25;

	//Theta
	_joint[3] = _icoord[3] - _joint[0] - _joint[1];				
	}
}
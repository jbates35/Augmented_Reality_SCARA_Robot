#include "robot_lab5.h"

CRobot_5::CRobot_5()
{
	//Turn on camera and use charuco as worldview
	disable_worldview();
	
	//Zero-stuff frame time vec
	for (int i = 0; i < 5; i++) frame_time_vec.push_back(0);
	
	//Boundaries of joint angles or distances (4th entry)
	_joint_min = { -180, -180, -25, -180 };
	_joint_max = { 180, 180, 175, 180 };
	
	//Names of joints
	_joint_names = { "Q1", "Q2", "QZ", "Q3" };
	
	//Initialize important vars
	init();
}


CRobot_5::~CRobot_5()
{
}


void CRobot_5::init()
{
	// reset state machine variables for fkine animation
	_do_animate = 0;	
	_stage = 0;
	_count = 0;

	//Reset robot joint positions
	_joint.clear();
	for (int i = 0; i < 4; i++) _joint.push_back(0);
	_joint[2] = 75;	
}


void CRobot_5::create()
{
	//Empty vector
	_robot.clear();
	
	//Colors for each box
	vector<Scalar> colors = { 
		WHITE, 
		RED, 
		GREEN, 
		BLUE 
		};

	//Empty canvas
	_canvas = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);
	_canvas_copy = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);

	//Translations for each box
	vector<Mat> transpose_box = {
		extrinsic(0, 0, 90, 0, 0, 0),
		extrinsic(0, 0, 90, 0.175, 0, 0, false),
		extrinsic(0, 0, 0, 0.15, 0, 0, false),
		extrinsic(0, 0, 90, 0.15, -1*(float)_joint[2] / 1000, 0, false)
		};

	//Rotations for each box
	vector<Mat> rotate_box = {
		extrinsic(),
		extrinsic(0, _joint[0]),
		extrinsic(0, _joint[1]),
		extrinsic(_joint[3])
		};

	//Keeps track of the different translations and rotations per box
	Mat current_view = extrinsic();

	for (int i = 0; i < colors.size(); i++) 
	{
		_box_struct _box;
		
		if (i == 3)
		{
			_box.shape = createBox(0.2, 0.05, 0.05);
		}
		else
		{	
			_box.shape = createBox(0.15, 0.05, 0.05);
		}
		
		_box.color = colors[i];
		_box.transpose = transpose_box[i];
		_box.rotate = rotate_box[i];

		transformPoints(_box.shape, extrinsic(0, 0, 0, 0.075));

		_robot.push_back(_box);
	}
}


void CRobot_5::update()
{
	//Detect aruco to see charuco board
	if (_worldview) 
	{
		_virtualcam.detect_aruco(_canvas, _canvas_copy);
	}
	
	update_fkine();
}


void CRobot_5::draw()
{
	//Draw trackbars for camera
	_virtualcam.update_settings(_canvas_copy);
	
	//Show frame rate
	frame_rate();
	
	//Matrix for keeping track of where each thing should be drawn
	Mat current_view = extrinsic();
	
	for (auto x : _robot) 
	{
		//Change origin
		current_view = current_view * x.transpose * x.rotate;

		//Transform box
		transformPoints(x.shape, current_view);

		//Create worldview
		std::vector<Mat> O = createCoord();
		transformPoints(O, current_view);

		//Draw box + worldview (if feasible)
		if (_virtualcam.get_pose_seen() || _worldview == false) 
		{
			drawCoord(_canvas_copy, O);
			drawBox(_canvas_copy, x.shape, x.color);
		}
	}

	//Draw last worldview (end effector)
	Mat effector_translate = extrinsic(0, 0, 0, 0.15, 0, 0, false);
	current_view = current_view * effector_translate;

	//Store into member variable
	_current_view = current_view;

	//FLIP for drawing last coordinate upside down
	current_view *= extrinsic(0, 0, 180);
	std::vector<Mat> O = createCoord();
	transformPoints(O, current_view);

	//Update button enabling/disabling worldview
	update_worldview();
		
	//Update all trackbars
	cvui::update();
	
	//Show image
	cv::imshow(CANVAS_NAME, _canvas_copy);
}


void CRobot_5::update_fkine()
{
	Point _setting_window;

	//Create underlay for trackbars
	_setting_window.x = _canvas_copy.size().width - 220;
	cvui::window(_canvas_copy, _setting_window.x, _setting_window.y, 220, 250, "Forward Kin Settings");

	//First trackbar position
	_setting_window.x += 15;
	_setting_window.y += 25;
	
	//Create trackbars for joint angles	
	for (int i = 0; i < _joint.size(); i++) 
	{
		cvui::trackbar(_canvas_copy, _setting_window.x, _setting_window.y, 180, &_joint[i], _joint_min[i], _joint_max[i]);
		cvui::text(_canvas_copy, _setting_window.x + 180, _setting_window.y + 20, _joint_names[i]);

		_setting_window.y += 45;
	}
	
	//Creat button for animate
	if (cvui::button(_canvas_copy, _setting_window.x-10, _setting_window.y+5, 100, 30, "Animate")) 
	{
		init();
		_do_animate = 1;
		_stage = 0;
	}

	//Create button for resetting robot position
	if (cvui::button(_canvas_copy, _setting_window.x + 100, _setting_window.y+5, 100, 30, "reset")) 
	{
		init();
	}

	//Animate forward kinematics
	if (_do_animate != 0) 
	{
		//How fast the robot moves, essentially
		int step_size = 10;
		
		//Index i.e. which part of the state machine is it in
		int i = _do_animate - 1;

		//Stage of which part of the motion it is
		switch (_stage) {
		case 0: // cw to -180
			_joint[i] -= step_size;
			if (_joint[i] <= _joint_min[i]) _stage = 1;
			break;
		case 1: // ccw to 180
			_joint[i] += step_size;
			if (_joint[i] >= _joint_max[i]) _stage = 2;
			break;
		case 2: // cw to 0
			_joint[i] -= step_size;
			if (_joint[i] <= (_joint_max[i] + _joint_min[i]) / 2) _stage = 3;
			break;
		case 3: // move on
			_joint[i] = (_joint_max[i] + _joint_min[i]) / 2;
			_stage = 0;
			_do_animate++;
			break;
		default:
			init();
		}

		//Reset when state machine finishes
		if (_do_animate == 5) 
		{
			_do_animate = 0;
			init();
		}
	}
}


void CRobot_5::frame_rate()
{
	//Calculate frame time since last frame
	frame_time_end = cv::getTickCount() / cv::getTickFrequency();
	
	//Erase first entry in vector, add last entry into vector
	frame_time_vec.erase(frame_time_vec.begin());
	frame_time_vec.push_back(frame_time_end - frame_time_beg);
	
	//Calculate average frame time
	double frame_temp = 0;
	for (auto i : frame_time_vec) frame_temp += i;
	frame_time = frame_temp / frame_time_vec.size();
	
	//Inverse for frame rate
	frame_freq = 1 / frame_time;
	
	//Log time for next frame
	frame_time_beg = cv::getTickCount() / cv::getTickFrequency();
	
	//Create strings to represent frame rate and frame time
	string frame_string_1 = "Frame time is: " + to_string(frame_time_vec[0]) + "\ts";
	string frame_string_2 = "Frame frequency is : " + to_string(1 / frame_time_vec[0]) + "\tHz";

	//Put frame time/rate strings on canvas
	putText(_canvas_copy, frame_string_1, cv::Point(300, 40), 0, 0.5, Scalar(100, 150, 0), 2);
	putText(_canvas_copy, frame_string_2, cv::Point(300, 60), 0, 0.5, Scalar(100, 0, 150), 2);
}

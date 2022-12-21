#include "robot_lab4.h"

CRobot_4::CRobot_4()
{
	//Set camera on and use charuco board as worldview
	set_worldview();
	
	//Initial timer
	turn_timer = getTickCount();
}


CRobot_4::~CRobot_4()
{
}


void CRobot_4::create()
{
	//Offsets to create robot box
	vector<Point2f> translate = { 
		Point2f(0.0, 0.0), 
		Point2f(0.0, 0.05), 
		Point2f(0.05, 0.1), 
		Point2f(-0.05, 0.1), 
		Point2f(0.0, 0.15) 
		};
	
	//Colors of each box
	vector<Scalar> colors = { 
		RED, 
		RED, 
		GREEN, 
		BLUE, 
		RED 
		};

	//Create robots and translate per box to give it its robot-ty shape
	for (int i = 0; i < translate.size(); i++) 
	{
		box _box;
		_box.shape = createBox(0.05, 0.05, 0.05);
		_box.color = colors[i];
		
		//Translate each box
		transformPoints(_box.shape, extrinsic(0, 0, 0, translate[i].x, translate[i].y + 0.025));

		_simple_robot.push_back(_box);
	}
}


void CRobot_4::draw()
{
	//Detect charuco and apply transformations of worldview
	_virtualcam.detect_aruco(_canvas, _canvas_copy);
	_virtualcam.update_settings(_canvas_copy);
	
	//Move robot
	Vec3d tvec = _virtualcam.get_tvec();

	//Only draw robot if we have seen charuco board
	for (auto x : _simple_robot) 
	{
		if (_virtualcam.get_pose_seen())
		{
			drawBox(_canvas_copy, x.shape, x.color);
		}
	}

	//Rotate robot every 10ms
	if ((getTickCount() - turn_timer) / getTickFrequency() >= 0.01) {
		turn_timer = getTickCount();

		//Get rotation matrix - roll=0 pitch=4 yaw=0
		Mat T = extrinsic(0, 4, 0);
		
		//Rotate robot for next frame
		for (auto x : _simple_robot)
			transformPoints(x.shape, T);
	}
	
	//cv::imshow("7825 Canvas (test1)", _canvas);
	cv::imshow("7825 Canvas Lab 4", _canvas_copy);	
}

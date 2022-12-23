#include "robot_lab3.h"

CRobot_3::CRobot_3()
{
}

CRobot_3::~CRobot_3()
{
}

void CRobot_3::create()
{
	//Offsets to create robot box
	vector<Point2f> translate = { Point2f(0, 0), Point2f(0, 0.05), Point2f(0.05, 0.1), Point2f(-0.05, 0.1), Point2f(0, 0.15) };
	
	//Colors of diff boxes
	vector<Scalar> colors = { RED, RED, GREEN, BLUE, RED };

	for (int i = 0; i < translate.size(); i++) 
	{
		//First create box normally
		box _box;
		_box.shape = createBox(0.05, 0.05, 0.05);
		_box.color = colors[i];

		//Translate each box
		transformPoints(_box.shape, extrinsic(0, 0, 0, translate[i].x, translate[i].y + 0.025));

		//Add it to robot
		_simple_robot.push_back(_box);
	}
}


void CRobot_3::draw()
{
	//Reset canvas
	_canvas = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);

	//Apply camera angles
	_virtualcam.update_settings(_canvas);

	//Create base coordinate for worldview
	std::vector<Mat> O = createCoord();
	drawCoord(_canvas, O);

	//Draw robot
	for (auto x : _simple_robot) 
	{
		drawBox(_canvas, x.shape, x.color);
	}

	//Update trackbars
	cvui::update();
	
	//Show final product
	cv::imshow(CANVAS_NAME, _canvas);
}

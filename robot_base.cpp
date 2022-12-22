#include "robot_base.h"


CRobot_base::CRobot_base(int lab /* = 0 */)
{
	//Keep track of which lab
	set_lab(lab);
	
	//Default camera off
	_worldview = false;

	//////////////////////////////////////
	// Create image and window for drawing
	_image_size = Size(1000, 600);
	_canvas = cv::Mat::zeros(_image_size, CV_8UC3);
	cv::namedWindow(CANVAS_NAME);
	
	//Initialize trackbar system	
	cvui::init(CANVAS_NAME);
}


CRobot_base::~CRobot_base()
{
}


void CRobot_base::set_worldview()
{
	//Turn camera on and change worldview to detected charuco board
	_worldview = true;
	_virtualcam.enable_worldview();
}

void CRobot_base::transformPoints(std::vector<Mat>& points, Mat T)
{
	//Change points by applying transformation matrix
	for (int i = 0; i < points.size(); i++)
	{
		points.at(i) = T * points.at(i);
	}
}


vector<cv::Mat> CRobot_base::createBox(float w, float h, float d)
{
	//Return variable
	std::vector <Mat> box;

	// The 8 vertexes, origin at the center of the box
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, -h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, -h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, -h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, -h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, h / 2, d / 2, 1)));

	return box;
}


void CRobot_base::drawBox(Mat& im, std::vector<Mat> box3d, Scalar colour)
{
	std::vector<Point2f> box2d;

	// The 12 lines connecting all vertexes 
	float draw_box1[] = { 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3 };
	float draw_box2[] = { 1, 2, 3, 0, 5, 6, 7, 4, 4, 5, 6, 7 };

	//Change from 3d to 2d
	if (_worldview == false) 
	{
		_virtualcam.transform_to_image(box3d, box2d);
	}
	else 
	{
		_virtualcam.transform_to_image_real(box3d, box2d);
	}
	
	//Draw each line constituting a box
	for (int i = 0; i < 12; i++)
	{
		Point pt1 = box2d.at(draw_box1[i]);
		Point pt2 = box2d.at(draw_box2[i]);
		line(im, pt1, pt2, colour, 1);
	}
}


vector<cv::Mat> CRobot_base::createCoord()
{
	//Return variable
	std::vector <Mat> coord;

	//Length is 5cm
	float axis_length = 0.05;

	//Push lines composing coordinate
	coord.push_back((Mat1f(4, 1) << 0, 0, 0, 1)); // O
	coord.push_back((Mat1f(4, 1) << axis_length, 0, 0, 1)); // X
	coord.push_back((Mat1f(4, 1) << 0, axis_length, 0, 1)); // Y
	coord.push_back((Mat1f(4, 1) << 0, 0, axis_length, 1)); // Z

	return coord;
}


void CRobot_base::drawCoord(Mat& im, std::vector<Mat> coord3d)
{
	Point2f O, X, Y, Z;

	//Change 3d points to 2d 
	if (_worldview == false) 
	{
		_virtualcam.transform_to_image(coord3d.at(0), O);
		_virtualcam.transform_to_image(coord3d.at(1), X);
		_virtualcam.transform_to_image(coord3d.at(2), Y);
		_virtualcam.transform_to_image(coord3d.at(3), Z);
	}
	else 
	{
		_virtualcam.transform_to_image_real(coord3d.at(0), O);
		_virtualcam.transform_to_image_real(coord3d.at(1), X);
		_virtualcam.transform_to_image_real(coord3d.at(2), Y);
		_virtualcam.transform_to_image_real(coord3d.at(3), Z);
	}

	//Draw lines composing coordinate
	line(im, O, X, CV_RGB(255, 0, 0), 1); // X=RED
	line(im, O, Y, CV_RGB(0, 255, 0), 1); // Y=GREEN
	line(im, O, Z, CV_RGB(0, 0, 255), 1); // Z=BLUE
}



void CRobot_base::set_lab(int lab)
{
	_lab = lab;
	_virtualcam.set_lab(lab);
}


Mat CRobot_base::extrinsic(int roll /* = 0 */, int pitch /* = 0 */, int yaw /* = 0 */, float x /* = 0 */, float y /* = 0 */, float z /* = 0 */, bool normal /* = true */)
{
	//Calculate angles
	float sx = sin((float)roll * PI / 180);
	float cx = cos((float)roll * PI / 180);
	float sy = sin((float)pitch * PI / 180);
	float cy = cos((float)pitch * PI / 180); 
	float sz = sin((float)yaw * PI / 180);
	float cz = cos((float)yaw * PI / 180);

	//Create rotation matrix from angles
	Mat rotate = (Mat1f(4, 4) <<
		cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx, 0,
		sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx, 0,
		-1 * sy, cy * sx, cy * cx, 0,
		0, 0, 0, 1);

	//Translate matrix to multiply with rotation matrix
	Mat translate = (Mat1f(4, 4) <<
		1, 0, 0, x,
		0, 1, 0, y,
		0, 0, 1, z,
		0, 0, 0, 1
		);

	//Normally, we want translate to be affected by rotate
	if (normal) 
	{
		return rotate * translate;
	}
	//Sometimes we want translate unaffected by the rotation
	else 
	{
		return translate * rotate;	
	}
}

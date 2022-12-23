#pragma once

#include <opencv2/opencv.hpp>

#include "Camera.h"
#include "cvui.h"

using namespace std;
using namespace cv;

#define CANVAS_NAME "7825 Project"

#define ANIMATE_INCREMENT 5 //I.e. speed of animation
#define MAX_ANIMATE_RANGE 200 //Max it animates to (distance)

#define TOTAL_TIME 3.5 //Amount of time for trajectory
#define STEP_COUNT 35 //i.e. size of vector of positions

#define ARM_LENGTH 0.15 // Length of robot arm (a1, a2)
#define MAX_ICOORD 325 // So robot doesn't go out of range (i.e. ikine makes complex number)
#define MIN_ICOORD 50.5 // So robot doesn't jam into itself

//Colors for cv::Scalar()
#define WHITE Scalar(255, 255, 255)
#define RED Scalar(0, 0, 255)
#define GREEN Scalar(0, 255, 0)
#define BLUE Scalar(255, 0, 0)
#define YELLOW Scalar(0, 255, 255)
#define MAGENTA Scalar(255, 0, 255)
#define TEAL Scalar(255, 255, 0)

class CRobot_base
{
public:
	CRobot_base(int lab = 0);
	~CRobot_base();
	
protected:
	//Virtual and real camera
	CCamera _virtualcam;
	
	//Default size of canvas
	Size _image_size;
	
	//Canvas' that will be shown to user
	Mat _canvas, _canvas_copy;
	
	//Transpose set of coordinates with mat T
	void transformPoints(std::vector<Mat>& points, Mat T);
	
	//Create box that will be drawn
	vector<Mat> createBox(float w, float h, float d);
	
	//Draw box
	void drawBox(Mat& im, std::vector<Mat> box3d, Scalar colour);	
	
	//Create coordinate system
	vector<Mat> createCoord();
	
	//Draw coordinates
	void drawCoord(Mat& im, std::vector<Mat> coord3d);
	
	//Concerned with whether worldview is virtual or augmented
	bool _worldview;
	
	// Turns camera on and uses board as worldview
	void set_worldview();
	
	// Turns camera off and uses virtual camera
	void disable_worldview();
	
	// Draws button for turning on/off worldview
	void update_worldview();
	
	//Lab number currently on
	int _lab;
	
	//Set camera lab and robot lab
	void set_lab(int lab);
	
	//Create matrix from rpy and xyz
	Mat extrinsic(int roll = 0, int pitch = 0, int yaw = 0, float x = 0, float y = 0, float z = 0, bool normal = true);
};
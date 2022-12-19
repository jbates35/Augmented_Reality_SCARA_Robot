#pragma once

//Opencv imports
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>

//For arctan
#include <cmath>

//For slider bars
#include "cvui.h"

//Namespaces used here
using namespace std;
using namespace cv;

//Definitions for project
#define REFRESH_INT 1
#define PI 3.14159265359
#define MODEL_SCALE 1

class CCamera
{
public:
	CCamera();
	~CCamera();
	
private:
	//Init vars
	bool testing;
	
	//Method for initalizing vars
	
	
	// Virtual Camera
	float _pixel_size;
	Point2f _principal_point;
	Mat _cam_virtual_intrinsic;
	Mat _cam_virtual_extrinsic;

	void calculate_intrinsic();
	void calculate_extrinsic();
	
		
	void calculate_real_extrinsic();	
};

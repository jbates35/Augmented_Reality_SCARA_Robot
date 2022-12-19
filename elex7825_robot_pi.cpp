/**
 * @function findContours_Demo.cpp
 * @brief Demo code to find contours in an image
 * @author OpenCV team
 */

#include <pigpio.h>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <opencv2/opencv.hpp>

// Add simple GUI elements
#define CVUI_DISABLE_COMPILATION_NOTICES
#define CVUI_IMPLEMENTATION
#include "cvui.h"

//Namespaces used for project
using namespace std;	// standard lib
using namespace cv;		// opencv

#include "Camera.h"

void lab1()
{
	CCamera _virtualcam;
	//_virtualcam.createChArUcoBoard();
}

void lab2()
{
	CCamera _virtualcam;
	//_virtualcam.calibrate_board();
}

void lab3(int cam_id)
{

}

void lab4(int cam_id)
{

}

void lab5(int cam_id)
{

}

void lab6(int cam_id)
{

}

void lab7(int cam_id)
{

}

int main(int argc, char* argv[])
{
	int sel = -1;
	int cam_id = 0;

	while (sel != 0)
	{
		cout << "\n*****************************************************";
		cout << "\n(3) Lab 3 - Virtual Camera";
		cout << "\n(4) Lab 4 - Camera Calibration";
		cout << "\n(5) Lab 5 - Forward Kinematics";
		cout << "\n(6) Lab 6 - Inverse Kinematics";
		cout << "\n(7) Lab 7 - Trajectories";
		cout << "\n(0) Exit";
		cout << "\n\n(1) Create Charuco board";
		cout << "\n(2) Calibrate Camera";
		cout << "\n>> ";

		cin >> sel;
		switch (sel)
		{
		case 1: lab1(); break;
		case 2: lab2(); break;
		case 3: lab3(cam_id); break;
		case 4: lab4(cam_id); break;
		case 5: lab5(cam_id); break;
		case 6: lab6(cam_id); break;
		case 7: lab7(cam_id); break;
		}
	}

	return 1;
}
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

//Robot scripts
#include "robot_lab3.h"
#include "robot_lab4.h"
#include "robot_lab5.h"
#include "robot_lab6.h"

void lab1()
{
	CCamera _virtualcam;
	_virtualcam.createChArUcoBoard();
}

void lab2()
{
	CCamera _virtualcam;
	_virtualcam.calibrate_board();
}

void lab3(int cam_id)
{
	char exit_key = -1;
	CRobot_3 robot;

	//Initially create robot
	robot.create();

	//Continuously draw robot
	while (exit_key != 'q')
	{
		robot.draw();
		exit_key = waitKey(10);
	}
}

void lab4(int cam_id)
{
	char exit_key = -1;
	CRobot_4 robot;
   
	//Initially create robot
	robot.create();

	//Continuously draw robot
	while (exit_key != 'q') 
	{
		robot.draw();
		exit_key = waitKey(5);   
	}
}

void lab5(int cam_id)
{
	char exit_key = -1;
	CRobot_5 robot;
	
	while (exit_key != 'q')
	{
		//Continuously create robot
		robot.create();
		
		//Update settings
		robot.update();
		
		//Draw robot
		robot.draw();
		
		exit_key = waitKey(10);
	}
}

void lab6(int cam_id)
{
	char exit_key = -1;
	CRobot_6 robot;
	
	while (exit_key != 'q')
	{
		//Continuously create robot
		robot.create();
		
		//Update settings
		robot.update();
		
		//Draw robot
		robot.draw();
		
		exit_key = waitKey(10);
	}
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
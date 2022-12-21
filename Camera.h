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
	
	/**
	 ** @brief Initializes variables in camera
	 **
	 ** @param image_size cv::Size of initial mat image
	 ** @param cam_id camera being chosen
	 ***/
	void init(Size image_size, int cam_id = 0);
	
	/**
	 ** @brief Saves camera parameters to file to be loaded later
	 ***/	
	bool save_camparam(string filename, Mat& cam, Mat& dist);
	
	/**
	 ** @brief Loads camera parameters from file to be used for intrinsic mat
	 ***/
	bool load_camparam(string filename, Mat& cam, Mat& dist);	
	
	/**
	 ** @brief Creates charuco board which is saved to file therefore printed later
	 ***/	
	void createChArUcoBoard();
	
	/**
	 ** @brief Loads camera parameters from file to be used for intrinsic mat
	 ***/
	void calibrate_board();
	
	/**
	 ** @brief Detect aruco board and its markers
	 **
	 ** @param im pointer to main image mat 
	 ** @param im pointer to secondary image mat
	 ***/	
	void detect_aruco(Mat& im, Mat& im_cpy);

	/**
	 ** @brief Transforms a single point from a 3d coordinate to a virtual 2d image coordinate
	 **
	 ** @param pt3d_mat 3d point to be converted
	 ** @param pt 2d converted point for image
	 ***/	
	void transform_to_image(Mat pt3d_mat, Point2f& pt);
	
	/**
	** @brief Transforms a vector of points from a 3d coordinate to a virtual 2d image coordinate
	**
	** @param pt3d_mat 3d points to be converted
	** @param pts2d 2d converted points for image
	***/	
	void transform_to_image(std::vector<Mat> pts3d_mat, std::vector<Point2f>& pts2d);

	/**
	** @brief Transforms a vector of points from a 3d coordinate to a real 2d image coordinate
	**
	** @param pt3d_mat 3d points to be converted
	** @param pts2d 2d converted points for image
	***/		
	void transform_to_image_real(Mat pt3d_mat, Point2f& pt);

	/**
	** @brief Transforms a vector of points from a 3d coordinate to a real 2d image coordinate
	**
	** @param pt3d_mat 3d points to be converted
	** @param pts2d 2d converted points for image
	***/		
	void transform_to_image_real(std::vector<Mat> pts3d_mat, std::vector<Point2f>& pts2d);

	/**
	** @brief Transforms a vector of points from a 3d coordinate to a real 2d image coordinate
	**
	** @param pt3d_mat 3d points to be converted
	** @param pts2d 2d converted points for image
	***/		
	void set_rvec(Point3i rvec_local);
		
	/**
	 ** @brief Enable real camera worldview if main file permits
	 ***/
	void enable_worldview();
	
	/**
	 ** @brief Allow robot.cpp to change the lab
	 ** 
	 ** @param lab Lab number to be set
	 ***/
	void set_lab(int lab);
	
	/**
	 ** @brief Return if markers have been found, so we can change trajectories
	 ***/
	bool markers_found();
	
	/**
	 ** @brief Return amount of markers found
	 ***/
	int marker_count();
	
	/**
	 ** @brief Get parameters of the current marker robot end effector is at
	 **
	 ** @param curr Current ID the robot end effector has reached
	 ***/
	vector<int> get_pose(int curr);
	
	/**
	 ** @brief Draws tracker bars, updates all extrinsic and intrinsic matrices
	 **
	 ** @param im Main im file that will have tracker bars implemented
	 ***/
	void update_settings(Mat &im);

	
private:
	//If true, use camera as basis for worldview, otherwise use virtual worldview
	bool _worldview;
	
	//Lab number
	int _lab;
	
	//Init vars
	bool testing;
	
	//Turn on if board has first been seen
	bool _pose_seen;
	
	//Turn on if there's a current board X and Y coordinate
	bool _can_detect;
	
	//Rvec that precedes setting trackbars
	Point3i _rvec_prime;
	
	//Tells update_settings that we can calculate new angle
	bool _update_angle;
	
	//Signify that we have detected aruco boxes
	bool _pose_detected;
	
	//Cam id from initializer
	int _cam_id;

	//Checks if markers associated with marker_id are found
	std::vector<bool> _marker_found;
	
	//Markers that we need to detect to manipulate robot to
	std::vector<int> _marker_id;
	
	//Vectors for storing marker tvec and rvec
	std::vector<cv::Vec3d> _marker_tvec, _marker_rvec;
	
	//Aruco marker box poses
	vector<vector<int>> _box_poses;
	
	// Virtual Camera
	float _pixel_size;
	Point2f _principal_point;
	Mat _cam_virtual_intrinsic;
	Mat _cam_virtual_extrinsic;

	// Virtual cam matrices
	void calculate_intrinsic();
	void calculate_extrinsic();
	
	// Real cam matrix
	void calculate_real_extrinsic();	
	
	//Changes camera angles from weird cv format to rpy
	Point3i convert_to_angle(Mat rotate);
	
	//Easy way of keeping parameters organized for robot boxes
	struct box_pos
	{
		int x;
		int y;
		int z;
		int roll;
		int pitch;
		int yaw;
	};
	struct box_pos box;
	
	// CVUI setting variables
	int _cam_setting_f;
	int _cam_setting_x;
	int _cam_setting_y;
	int _cam_setting_z;
	int _cam_setting_roll;
	int _cam_setting_pitch;
	int _cam_setting_yaw;
	
	// Mats for keeping track of live camera parameters 
	Mat _cam_real_intrinsic;
	Mat _cam_real_extrinsic;
	Mat _cam_real_dist_coeff;
	Mat _trans_factor;	
	
	//For detecting charuco and creating such
	Size board_size;
	int dictionary_id;
	
	//ChArUco aruco square and mark sizes
	float size_aruco_square, size_aruco_mark;
	
	//Filename for camera calibration
	std::string _filename;
	
	//Camera parameters that _filename loads params into
	cv::Mat cameraMatrix, distCoeffs;
	
	//Vectors holding location of square and mark coordinates on charuco board
	cv::Vec3d rvec, tvec;
	
	//Parameters for detecting board
	Ptr<aruco::DetectorParameters> detectorParams;
	
	//Dictionary holding parameters of charuco board
	Ptr<aruco::Dictionary> dictionary;

	//Charuco board to set coordinates upon
	Ptr<aruco::CharucoBoard> charucoboard;
	
	//Aruco markers
	Ptr<aruco::Board> board;

	//Camera capture (HD cam or FLIR cam)
	VideoCapture inputVideo;
		
	//Make extrinsic matrix from given rpy and xyz coordinates
	Mat extrinsic(int roll = 0, int pitch = 0, int yaw = 0, float x = 0, float y = 0, float z = 0, bool normal = true);
};

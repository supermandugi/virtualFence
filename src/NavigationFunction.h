#pragma once

#define NOMINMAX


struct Engine
{
	int state = 0;
};
struct Cam_640480
{
	bool imshow = true;
	unsigned char data[640 * 480 * 3];
};
struct Robot
{
	int is_on = 0;

	double x = 0;
	double y = 0;
	double th = 0;

	double set_vel = 0.0;
	double set_rotvel = 0.0;

	double get_vel = 0.0;
	double get_rotvel = 0.0;

	double KeyFrame[3][3] = {}; // [idx][val] -> [val] : {Key, t, r}

	int front_prox = 0;
};
struct ObservationData
{
	//////////////////////////// input ////////////////////////////
	int mode = 0; // 0:localize, 1:save, 2:saveDB, 3:loadDB, 9:reset
	bool draw_flag = 1;
	int row = 480;
	int col = 640;
	unsigned char RGB_data[640 * 480 * 3];

	//////////////////////////// output ////////////////////////////
	int num_detected_feature = 0;
	int num_DB_size = 0;
	float score[2048];
	float likelihood[2048];
	cv::Mat line_plot;
};
struct Kinect1Data
{
	bool draw_grid = 1;

	int        cgridWidth = 500;
	int        cgridHeight = 500;
	int		   cRobotCol = 250;
	int		   cRobotRow = 350;

	double mm2grid = 50.0 / 10000.0;

	unsigned char gridData[500 * 500];
	unsigned char freeData[500 * 500];
	unsigned char occupyData[500 * 500];
};
struct Kinect2Data
{
	bool draw_color = 1;
	int        cColorWidth = 960; // 1920 / 2;
	int        cColorHeight = 540; // 1080 / 2;
	unsigned char data[960 * 540 * 3];
};
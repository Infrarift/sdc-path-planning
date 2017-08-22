#pragma once
#include <vector>

struct CarState
{
	double x;
	double y;
	double s;
	double d;
	double yaw;
	double speed;
	double end_path_s;
	double end_path_d;
	std::vector<double> previous_path_x;
	std::vector<double> previous_path_y;
};

struct DetectedCarState
{
	int id;
	double x;
	double y;
	double vx;
	double vy;
	double s;
	double d;
	double speed;
};

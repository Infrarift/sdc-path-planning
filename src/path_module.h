#pragma once
#include "car_state.h"
#include "spline.h"
using std::vector;

struct PathWaypoints
{
	vector<double> x;
	vector<double> y;
	double ref_x, ref_y, ref_yaw;
};

class PathModule
{
public:

	void Process(const CarState& state);

	vector<double> path_x_;
	vector<double> path_y_;
	vector<double> map_wp_s_, map_wp_x_, map_wp_y_;
	double current_v_ = 0, tar_v_ = 49, current_lane_ = 1.0, tar_lane_ = 1.0;

private:

	void ShiftLane();
	void ProcessAcceleration();
	void PopulateWithPreviousPath(const CarState& state);
	void AddWaypointNode(PathWaypoints path_waypoints, tk::spline s, double& prev_x);
	void TransformToLocal(vector<double>& path_x, vector<double>& path_y, double ref_x, double ref_y, double ref_yaw) const;

	vector<double> TransformToGlobal(double x_point, double y_point, double ref_x, double ref_y, double ref_yaw) const;
	vector<double> ConvertFrenetToXY(double s, double d) const;

	static double GetDForLane(double lane);
	PathWaypoints GenerateAnchorPoints(const CarState& state, PathWaypoints& path_waypoints);
	
	double spline_increaser_ = 37.5;
	double spline_node_count_ = 3;
	double waypoint_node_count_ = 35;
	double lane_change_rate_ = 0.02;
	double acceleration_ = 0.25;

};


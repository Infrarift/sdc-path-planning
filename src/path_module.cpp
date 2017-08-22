#define _USE_MATH_DEFINES
#include <cmath>
#include "path_module.h"
#include "spline.h"


void PathModule::Process(const CarState & state)
{
	// Generate the desired waypoints.
	PathWaypoints path_waypoints;
	GenerateAnchorPoints(state, path_waypoints);
	TransformToLocal(path_waypoints.x, path_waypoints.y, path_waypoints.ref_x, path_waypoints.ref_y, path_waypoints.ref_yaw);

	// Create the spline.
	tk::spline spline;
	spline.set_points(path_waypoints.x, path_waypoints.y);

	// Populate the waypoints with the previously used path.
	PopulateWithPreviousPath(state);

	double prev_x = 0;

	for (auto i = 1; i < waypoint_node_count_ - state.previous_path_x.size(); i++)
		AddWaypointNode(path_waypoints, spline, prev_x);
}

void PathModule::AddWaypointNode(PathWaypoints path_waypoints, tk::spline s, double& prev_x)
{
	ShiftLane();
	ProcessAcceleration();

	auto km_v = 0.0088 * current_v_;
	prev_x += km_v;

	auto global_points = TransformToGlobal(prev_x, s(prev_x), path_waypoints.ref_x, path_waypoints.ref_y, path_waypoints.ref_yaw);
	path_x_.push_back(global_points[0]);
	path_y_.push_back(global_points[1]);
}

void PathModule::ProcessAcceleration()
{
	if (current_v_ < tar_v_)
		current_v_ += acceleration_;

	if (current_v_ > tar_v_)
		current_v_ -= acceleration_;
}

void PathModule::ShiftLane()
{
	if (current_lane_ < tar_lane_)
		current_lane_ += lane_change_rate_;

	if (current_lane_ > tar_lane_)
		current_lane_ -= lane_change_rate_;
}

PathWaypoints PathModule::GenerateAnchorPoints(const CarState& state, PathWaypoints& path_waypoints)
{
	auto prev_path_size = state.previous_path_x.size();
	double prev_x, prev_y;

	if (prev_path_size > 2)
	{
		// Use the previous two points.
		int i1 = prev_path_size - 1;
		int i2 = prev_path_size - 2;

		path_waypoints.ref_x = state.previous_path_x[i1];
		path_waypoints.ref_y = state.previous_path_y[i1];

		prev_x = state.previous_path_x[i2];
		prev_y = state.previous_path_y[i2];

		path_waypoints.ref_yaw = atan2(path_waypoints.ref_y - prev_y, path_waypoints.ref_x - prev_x);
	}
	else
	{
		// Use car's current point.
		path_waypoints.ref_x = state.x;
		path_waypoints.ref_y = state.y;
		path_waypoints.ref_yaw = state.yaw * 0.017453292;

		prev_x = path_waypoints.ref_x - cos(path_waypoints.ref_yaw);
		prev_y = path_waypoints.ref_y - sin(path_waypoints.ref_yaw);
	}

	path_waypoints.x.push_back(prev_x);
	path_waypoints.y.push_back(prev_y);
	path_waypoints.x.push_back(path_waypoints.ref_x);
	path_waypoints.y.push_back(path_waypoints.ref_y);

	for (auto i = 1; i <= spline_node_count_; i++)
	{
		auto new_point = ConvertFrenetToXY(state.s + spline_increaser_ * i, GetDForLane(current_lane_));
		path_waypoints.x.push_back(new_point[0]);
		path_waypoints.y.push_back(new_point[1]);
	}

	return path_waypoints;
}

void PathModule::TransformToLocal(vector<double>& path_x, vector<double>& path_y, double ref_x, double ref_y, double ref_yaw) const
{
	// Convert waypoint positions to local car co-ordinates using the referenced x, y, and yaw.
	for (int i = 0, m = path_x.size(); i < m; i++)
	{
		auto shift_x = path_x[i] - ref_x;
		auto shift_y = path_y[i] - ref_y;

		path_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
		path_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
	}
}

vector<double> PathModule::TransformToGlobal(double x_point, double y_point, double ref_x, double ref_y, double ref_yaw) const
{
	// Convert waypoint posiions from local to global co-ordinates.
	auto new_x = x_point * cos(ref_yaw) - y_point * sin(ref_yaw);
	auto new_y = x_point * sin(ref_yaw) + y_point * cos(ref_yaw);
	new_x += ref_x;
	new_y += ref_y;
	return { new_x, new_y };
}

vector<double> PathModule::ConvertFrenetToXY(double s, double d) const
{
	auto prev_wp = -1;
	while (s > map_wp_s_[prev_wp + 1] && (prev_wp < static_cast<int>(map_wp_s_.size() - 1)))
		prev_wp++;
	int wp2 = (prev_wp + 1) % map_wp_x_.size();
	auto heading = atan2((map_wp_y_[wp2] - map_wp_y_[prev_wp]), (map_wp_x_[wp2] - map_wp_x_[prev_wp]));
	auto seg_s = (s - map_wp_s_[prev_wp]);
	auto seg_x = map_wp_x_[prev_wp] + seg_s * cos(heading);
	auto seg_y = map_wp_y_[prev_wp] + seg_s * sin(heading);
	auto perp_heading = heading - M_PI / 2;
	auto x = seg_x + d * cos(perp_heading);
	auto y = seg_y + d * sin(perp_heading);
	return { x,y };
}

void PathModule::PopulateWithPreviousPath(const CarState& state)
{
	path_x_.clear();
	path_y_.clear();

	for(auto i = 0; i < state.previous_path_x.size(); i++)
	{
		path_x_.push_back(state.previous_path_x[i]);
		path_y_.push_back(state.previous_path_y[i]);
	}
}

double PathModule::GetDForLane(double lane)
{
	return 2.0 + 4.0 * lane;
}

#pragma once
#include "car_state.h"
#include "cost.h"
using std::vector;

class LaneAnalyzer
{
public:

	LaneAnalyzer();
	void SetLane(int lane);
	bool CarIsInLane(const CarState& state) const;
	void Analyze(const CarState& state, vector<DetectedCarState>& detected_cars);
	void AddAdjacentLane(LaneAnalyzer* lane);

	int lane_id_ = 0;
	double lane_d_ = 0;

	float cost_ = 0;
	float max_speed_ = 0;
	float base_cost_ = 0.5;

	vector<LaneAnalyzer*> adjacent_lanes_;

private:
	
	Cost cost_forward_speed_;
	Cost cost_forward_distance_;
	Cost cost_back_distance_;
	Cost cost_back_speed_;

	double forward_bound_start_ = 120;
	double forward_bound_end_ = 0;
	double back_bound_start_ = 10;
	double back_bound_end_ = -20;

	double speed_limit_distance_ = 25;
	double stop_distance_ = 15;
	double lane_detect_width_ = 3.0;
	
};

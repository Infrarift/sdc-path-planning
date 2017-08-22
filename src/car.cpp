#include "car.h"
#include <cstdio>
#include <complex>
using std::abs;

Car::Car()
{
	printf("Initialize Car\n");
	printf("Initialize Cost");

	lane_analyzer_1_.SetLane(0);
	lane_analyzer_2_.SetLane(1);
	lane_analyzer_3_.SetLane(2);

	lane_analyzer_1_.base_cost_ = 0.5;
	lane_analyzer_2_.base_cost_ = 0.0;
	lane_analyzer_3_.base_cost_ = 0.5;

	lane_analyzer_1_.AddAdjacentLane(&lane_analyzer_2_);
	lane_analyzer_2_.AddAdjacentLane(&lane_analyzer_1_);
	lane_analyzer_2_.AddAdjacentLane(&lane_analyzer_3_);
	lane_analyzer_3_.AddAdjacentLane(&lane_analyzer_2_);

	current_lane_ = &lane_analyzer_2_;
	target_lane_ = nullptr;
}

void Car::Process(const CarState& state, vector<DetectedCarState>& detected_cars)
{
	printf(" ----- Processing ----- \n");
	ProcessLanes(state, detected_cars);
	ProcessPath(state);
}

void Car::ProcessLanes(const CarState& state, vector<DetectedCarState>& detected_cars)
{
	lane_analyzer_1_.Analyze(state, detected_cars);
	lane_analyzer_2_.Analyze(state, detected_cars);
	lane_analyzer_3_.Analyze(state, detected_cars);

	printf("Lane 1 Max Speed: %0.2f, Cost: %0.2f\n", lane_analyzer_1_.max_speed_, lane_analyzer_1_.cost_);
	printf("Lane 2 Max Speed: %0.2f, Cost: %0.2f\n", lane_analyzer_2_.max_speed_, lane_analyzer_2_.cost_);
	printf("Lane 3 Max Speed: %0.2f, Cost: %0.2f\n", lane_analyzer_3_.max_speed_, lane_analyzer_3_.cost_);

	if (target_lane_ != nullptr)
		CheckLaneCompletion(state);
	else
		SurveyLanes();
}

void Car::CheckLaneCompletion(const CarState& state)
{
	if (abs(state.d - target_lane_->lane_d_) < 0.1)
	{
		current_lane_ = target_lane_;
		target_lane_ = nullptr;
	}
}

void Car::SurveyLanes()
{
	// Find the best adjacent lane to switch to.
	LaneAnalyzer* adjacentLane = nullptr;
	for (auto i = 0; i < current_lane_->adjacent_lanes_.size(); i++)
	{
		auto new_lane = current_lane_->adjacent_lanes_[i];
		if (adjacentLane == nullptr || adjacentLane->cost_ > new_lane->cost_)
			adjacentLane = new_lane;
	}

	// If the cost is acceptable, change the lane.
	if (adjacentLane->cost_ < lane_change_max_cost_ && current_lane_->cost_ - adjacentLane->cost_ >= lane_change_min_cost_)
		target_lane_ = adjacentLane;
}
void Car::ProcessPath(const CarState& state)
{
	if (target_lane_ == nullptr)
	{
		path_module_.tar_v_ = current_lane_->max_speed_;
		path_module_.tar_lane_ = current_lane_->lane_id_ ;
	} else
	{
		path_module_.tar_v_ = (current_lane_->max_speed_ + target_lane_->max_speed_)/2;
		path_module_.tar_lane_ = target_lane_->lane_id_;
	}

	path_module_.tar_v_ = current_lane_->max_speed_;
	path_module_.tar_lane_ = target_lane_ == nullptr ? current_lane_->lane_id_ : target_lane_->lane_id_;
	path_module_.Process(state);
}



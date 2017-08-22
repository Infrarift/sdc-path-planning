#include "lane_analyzer.h"
#include <complex>

LaneAnalyzer::LaneAnalyzer()
{
	cost_forward_speed_.ApplyRange(50, 25).ApplyWeight(2);
	cost_forward_distance_.ApplyRange(50, 5).ApplyWeight(3);
	cost_back_distance_.ApplyRange(5, 0).ApplyWeight(25);
	cost_back_speed_.ApplyRange(-5, 5).ApplyWeight(25);
}

void LaneAnalyzer::SetLane(int lane)
{
	lane_id_ = lane;
	lane_d_ = 2.0 + 4.0 * lane;
}

bool LaneAnalyzer::CarIsInLane(const CarState& state) const
{
	return abs(state.d - lane_d_) < 0.5;
}

void LaneAnalyzer::Analyze(const CarState& state, vector<DetectedCarState>& detected_cars)
{
	DetectedCarState* forward_car = nullptr;
	DetectedCarState* back_car = nullptr;
	double best_forward_distance = 9999;
	double best_back_distance = 9999;
	max_speed_ = 50;

	for (auto i = 0; i < detected_cars.size(); i++)
	{
		auto d_car = &detected_cars[i];
		auto lane_distance = abs(d_car->d - lane_d_);
		d_car->speed = sqrt(d_car->vx * d_car->vx + d_car->vy * d_car->vy) * 2.23694;

		if (lane_distance < lane_detect_width_)
		{
			auto s_diff = d_car->s - state.s;
			if (s_diff > forward_bound_end_ && s_diff < forward_bound_start_ && best_forward_distance > s_diff)
			{
				best_forward_distance = s_diff;
				forward_car = d_car;
			}
			
			if (s_diff < back_bound_start_ && s_diff > back_bound_end_ && std::abs(s_diff) < best_back_distance)
			{
				best_back_distance = std::abs(s_diff);
				back_car = d_car;
			}
		}
	}

	auto distance_to_forward = 9999;
	auto distance_to_back = 9999;
	auto back_var_speed_difference = -9999;
	auto foward_car_speed = 50;

	if (forward_car != nullptr)
	{
		distance_to_forward = std::abs(forward_car->s - state.s);
		foward_car_speed = forward_car->speed;

		if (distance_to_forward < speed_limit_distance_)
			max_speed_ = distance_to_forward < stop_distance_ ? foward_car_speed * 0.7 : foward_car_speed;
	}

	if (back_car != nullptr && !CarIsInLane(state))
	{
		distance_to_back = best_back_distance;
		back_var_speed_difference = back_car->speed - state.speed;
	}

	cost_ = base_cost_;
	cost_ += cost_forward_speed_.AdjustedValue(foward_car_speed);
	cost_ += cost_forward_distance_.AdjustedValue(distance_to_forward);
	cost_ += cost_back_distance_.AdjustedValue(distance_to_back);
	cost_ += cost_back_speed_.AdjustedValue(back_var_speed_difference);
}

void LaneAnalyzer::AddAdjacentLane(LaneAnalyzer* lane)
{
	adjacent_lanes_.push_back(lane);
}

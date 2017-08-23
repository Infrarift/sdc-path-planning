#include "safety_module.h"
#include <valarray>

void SafetyModule::Process(const CarState& state, vector<DetectedCarState>& detected_cars)
{
	max_speed_ = 50;
	for (auto i = 0; i < detected_cars.size(); i++)
	{
		auto d_car = &detected_cars[i];
		auto forward_distance = d_car->s - state.s;

		if (forward_distance < 0 || forward_distance > braking_range_)
			continue;

		auto lane_distance = std::abs(d_car->d - state.d);
		if (lane_distance > braking_width_)
			continue;

		max_speed_ = braking_speed_;
		break;
	}
}

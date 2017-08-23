#pragma once
#include "car_state.h"
using std::vector;

class SafetyModule
{
public:
	void Process(const CarState& state, vector<DetectedCarState>& detected_cars);
	double max_speed_ = 50;

private:
	double braking_range_ = 10;
	double braking_speed_ = 20;
	double braking_width_ = 3;
};

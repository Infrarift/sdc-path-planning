#pragma once
#include "path_module.h"
#include "car_state.h"
#include "lane_analyzer.h"
#include "safety_module.h"

class Car
{
public:
	Car();
	void ProcessLanes(const CarState& state, vector<DetectedCarState>& detected_cars);
	void ProcessPath(const CarState& state);
	void SurveyLanes(const CarState& state);
	void Process(const CarState& state, vector<DetectedCarState>& detected_cars);
	void CheckLaneCompletion(const CarState& state);

	PathModule path_module_;
	SafetyModule safety_module_;
private:

	float lane_change_min_cost_ = 0.5;
	float lane_change_max_cost_ = 8.5;
	float lane_change_min_speed_ = 37.5;

	LaneAnalyzer lane_analyzer_1_;
	LaneAnalyzer lane_analyzer_2_;
	LaneAnalyzer lane_analyzer_3_;
	LaneAnalyzer* current_lane_;
	LaneAnalyzer* target_lane_;
};

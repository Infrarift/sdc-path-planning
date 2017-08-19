#pragma once
#include "StateModule.h"
#include "PathModule.h"
#include <vector>

struct CarState
{
	double x;
	double y;
	double s;
	double d;
	double yaw;
	double speed;
	double endPathS;
	double endPathD;

	std::vector<double> previousPathX;
	std::vector<double> previousPathY;
};

class Car
{
public:
	Car();
	~Car();
	void Process(const ::CarState& state);
private:
	StateModule state_module_;
	PathModule path_module_;
};


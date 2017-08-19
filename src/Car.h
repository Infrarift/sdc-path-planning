#pragma once
#include "StateModule.h"
#include "PathModule.h"

class Car
{
public:
	Car();
	~Car();
private:
	StateModule state_module_;
	PathModule path_module_;
};

struct CarState
{
	double x;
	double y;
};

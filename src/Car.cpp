#include "Car.h"
#include <cstdio>

Car::Car()
{
	printf("Initialize Car\n");
}

Car::~Car()
{
}

void Car::Process(const ::CarState& state)
{
	printf("Processing x: %f", state.x);
}

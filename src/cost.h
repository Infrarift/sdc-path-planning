#pragma once

class Cost
{
public:
	Cost();
	Cost(const float& low , const float& high, const float& weight);
	~Cost();

	Cost& ApplyWeight(const float& weight);
	Cost& ApplyRange(const double& low, const double& high);

	double AdjustedValue(double val) const;

private:
	const double kMinE = -3.75;
	const double kMaxE = 3.75;
	float e_range_ = 0;
	double v_low_ = 0;
	double v_high_ = 0;
	double weight_ = 1;
};
#include "cost.h"
#include <complex>

Cost::Cost()
{
	e_range_ = kMaxE - kMinE;
}

Cost::Cost(const float& low, const float& high, const float& weight)
{
	e_range_ = kMaxE - kMinE;
	ApplyWeight(weight);
	ApplyRange(low, high);
}

Cost::~Cost()
{
}

Cost& Cost::ApplyWeight(const float& weight)
{
	weight_ = weight;
	return *this;
}

Cost& Cost::ApplyRange(const double& low, const double& high)
{
	v_low_ = low;
	v_high_ = high;
	return *this;
}

double Cost::AdjustedValue(double val) const
{
	auto v_range = v_low_ - v_high_;
	auto v_ref = (val - v_high_) / v_range;
	auto s_val = kMinE + (v_ref * e_range_);
	auto adjusted_value = 1 / (1 + std::exp(s_val));
	return adjusted_value * weight_;
}

#pragma once

#include <cstdint>
#include <matrix/matrix/math.hpp>

namespace eso_common
{

enum class ModelProfile : int32_t {
	UavArmV4 = 0,
	UamV5 = 1,
};

struct DynamicsProfile {
	float mass_total;
	float com[3];
	float arm_inertia_diag[3];
	float body_inertia_diag[3];
	bool use_dynamic_arm_model;

	matrix::Vector3f comVector() const
	{
		return {com[0], com[1], com[2]};
	}

	matrix::Matrix3f armInertiaMatrix() const
	{
		return matrix::diag(matrix::Vector3f{arm_inertia_diag[0], arm_inertia_diag[1], arm_inertia_diag[2]});
	}

	matrix::Matrix3f bodyInertiaMatrix() const
	{
		return matrix::diag(matrix::Vector3f{body_inertia_diag[0], body_inertia_diag[1], body_inertia_diag[2]});
	}

	matrix::Matrix3f staticSystemInertiaMatrix() const
	{
		return armInertiaMatrix() + bodyInertiaMatrix();
	}
};

static constexpr DynamicsProfile kUavArmV4Profile{
	1.289f,                         //mass_total
	{-0.01f, 0.0f, 0.161f},         //com
	{0.016f, 0.016f, 0.016f},	//arm_inertia_diag
	{0.045f, 0.045f, 0.08f},	//body_inertia_diag
	true				//use_dynamic_arm_model
};

static constexpr DynamicsProfile kUamV5Profile{
	4.8548f,
	{0.00405203f, 0.0f, 0.06556744f},// UAM V5 SDF is FLU/Z-up; controller model uses PX4 body FRD.
	{0.00842308f, 0.04709706f, 0.04006098f},
	{0.049436f, 0.052665f, 0.086619f},
	true
};

inline const DynamicsProfile &getDynamicsProfile(ModelProfile profile)
{
	switch (profile) {
	case ModelProfile::UamV5:
		return kUamV5Profile;

	case ModelProfile::UavArmV4:
	default:
		return kUavArmV4Profile;
	}
}

inline ModelProfile resolveModelProfile(int32_t raw_value)
{
	return (raw_value == static_cast<int32_t>(ModelProfile::UamV5))
	       ? ModelProfile::UamV5
	       : ModelProfile::UavArmV4;
}

} // namespace eso_common

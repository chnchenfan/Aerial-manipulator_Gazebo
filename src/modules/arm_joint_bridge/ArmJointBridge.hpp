#pragma once

#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/arm_joint_states.h>
#include <uORB/topics/debug_key_value.h>

using namespace time_literals;

extern "C" __EXPORT int arm_joint_bridge_main(int argc, char *argv[]);

class ArmJointBridge : public ModuleBase<ArmJointBridge>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	ArmJointBridge();
	~ArmJointBridge() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	static constexpr hrt_abstime PUBLISH_INTERVAL{20_ms};
	static constexpr hrt_abstime FIELD_TIMEOUT{200_ms};

	void Run() override;
	void processDebugMessage(const debug_key_value_s &msg);
	void publishJointStates();
	bool fieldFresh(hrt_abstime timestamp, hrt_abstime now) const;

	uORB::SubscriptionCallbackWorkItem _debug_key_value_sub{this, ORB_ID(debug_key_value)};
	uORB::Publication<arm_joint_states_s> _arm_joint_states_pub{ORB_ID(arm_joint_states)};

	float _q[4]{};
	float _dq[4]{};
	hrt_abstime _q_timestamp[4]{};
	hrt_abstime _dq_timestamp[4]{};
	float _valid_value{0.0f};
	hrt_abstime _valid_timestamp{0};
	hrt_abstime _last_publish{0};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, "arm_joint_bridge")};
};

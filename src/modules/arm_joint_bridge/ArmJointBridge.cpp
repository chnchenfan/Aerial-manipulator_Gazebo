#include "ArmJointBridge.hpp"

#include <cstring>

#include <px4_platform_common/log.h>

ArmJointBridge::ArmJointBridge() :
	ModuleParams(nullptr),
	ScheduledWorkItem("arm_joint_bridge", px4::wq_configurations::nav_and_controllers)
{
}

ArmJointBridge::~ArmJointBridge()
{
	ScheduleClear();
	_debug_key_value_sub.unregisterCallback();
	perf_free(_loop_perf);
}

bool ArmJointBridge::init()
{
	if (!_debug_key_value_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	ScheduleOnInterval(PUBLISH_INTERVAL);
	return true;
}

int ArmJointBridge::task_spawn(int argc, char *argv[])
{
	ArmJointBridge *instance = new ArmJointBridge();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	if (!instance->init()) {
		delete instance;
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;
	return PX4_OK;
}

int ArmJointBridge::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ArmJointBridge::print_status()
{
	const hrt_abstime now = hrt_absolute_time();
	PX4_INFO("running, valid=%s q=[%.3f %.3f %.3f %.3f] dq=[%.3f %.3f %.3f %.3f]",
		 (fieldFresh(_valid_timestamp, now) && _valid_value > 0.5f) ? "true" : "false",
		 (double)_q[0], (double)_q[1], (double)_q[2], (double)_q[3],
		 (double)_dq[0], (double)_dq[1], (double)_dq[2], (double)_dq[3]);
	return 0;
}

bool ArmJointBridge::fieldFresh(hrt_abstime timestamp, hrt_abstime now) const
{
	return timestamp != 0 && ((now - timestamp) <= FIELD_TIMEOUT);
}

void ArmJointBridge::processDebugMessage(const debug_key_value_s &msg)
{
	const char *key = msg.key;

	if ((strncmp(key, "AJQ", 3) == 0) && (key[3] >= '0') && (key[3] <= '3') && key[4] == '\0') {
		const int index = key[3] - '0';
		_q[index] = msg.value;
		_q_timestamp[index] = msg.timestamp;
		return;
	}

	if ((strncmp(key, "AJD", 3) == 0) && (key[3] >= '0') && (key[3] <= '3') && key[4] == '\0') {
		const int index = key[3] - '0';
		_dq[index] = msg.value;
		_dq_timestamp[index] = msg.timestamp;
		return;
	}

	if (strcmp(key, "AJVAL") == 0) {
		_valid_value = msg.value;
		_valid_timestamp = msg.timestamp;
	}
}

void ArmJointBridge::publishJointStates()
{
	const hrt_abstime now = hrt_absolute_time();
	bool fresh = fieldFresh(_valid_timestamp, now) && (_valid_value > 0.5f);

	for (int i = 0; i < 4; ++i) {
		fresh &= fieldFresh(_q_timestamp[i], now);
		fresh &= fieldFresh(_dq_timestamp[i], now);
	}

	arm_joint_states_s msg{};
	msg.timestamp = now;

	for (int i = 0; i < 4; ++i) {
		msg.q[i] = _q[i];
		msg.dq[i] = _dq[i];
	}

	msg.valid = fresh;
	_arm_joint_states_pub.publish(msg);
	_last_publish = now;
}

void ArmJointBridge::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	debug_key_value_s debug_msg{};
	if (_debug_key_value_sub.update(&debug_msg)) {
		processDebugMessage(debug_msg);
	}

	const hrt_abstime now = hrt_absolute_time();
	if ((_last_publish == 0) || ((now - _last_publish) >= PUBLISH_INTERVAL)) {
		publishJointStates();
	}

	perf_end(_loop_perf);
}

int ArmJointBridge::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Aggregates incoming MAVLink NAMED_VALUE_FLOAT keys into arm_joint_states.

The module listens to debug_key_value values produced by mavlink_receiver and
publishes arm_joint_states at 50 Hz with a 200 ms freshness timeout.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("arm_joint_bridge", "module");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

int arm_joint_bridge_main(int argc, char *argv[])
{
	return ArmJointBridge::main(argc, argv);
}

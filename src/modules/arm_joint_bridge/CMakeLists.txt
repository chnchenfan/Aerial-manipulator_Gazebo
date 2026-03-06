px4_add_module(
	MODULE modules__arm_joint_bridge
	MAIN arm_joint_bridge
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		ArmJointBridge.cpp
		ArmJointBridge.hpp
	DEPENDS
		px4_work_queue
	)

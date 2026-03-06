
# uav_control

PX4 SITL + MAVROS 的自定义控制包，用于按论文流程依次实现：位置环 ESO（扰动估计）、位置环控制器、姿态环 ESO/控制器。当前已补充 **PositionESO** 类，其余模块仍为占位。

## 目录结构
- `include/uav_control/`：头文件（Position/Attitude ESO & Controller）。
- `src/`：对应实现和主控制节点（目前仅 PositionESO 完整，其余占位）。
- `scripts/`：Python 画图/数据处理占位脚本。
- `launch/`：启动文件占位，后续用于单独测试 ESO。
- `test/`：实验/数据采集占位，例如只跑 ESO 的节点。

## 计划实现顺序
1) **位置环 ESO**：订阅位姿/速度，估计位置与扰动，发布话题供测试。
2) **位置环控制器**：按论文式(16)(17)(22)(24) 生成加速度/力指令，经 MAVROS 下发。
3) **姿态环 ESO + 控制器**：按论文式(33)(38)(39)(42)。
4) **数据采集与绘图**：`test/eso_data_collector.cpp` 发布估计结果，`scripts/plot_eso_results.py` 绘图，`scripts/test_data_analysis.py` 处理日志。

## 构建提示
- 需要已配置好的 catkin 工作空间（本目录上层已包含 `CMakeLists.txt`）。
- 添加源码后，在工作空间根目录执行：`catkin_make`（或 `catkin build`）并 `source devel/setup.bash`。
- Python 脚本已通过 `catkin_install_python` 安装，可使用 `rosrun uav_control plot_eso_results.py <bag>` 读取 rosbag 画图。

## 下一步
- 在其它 `.hpp/.cpp` 补充控制逻辑，并在 CMake 中添加可执行/库。
- 在 `launch/test_eso.launch` 填入 MAVROS + 控制节点的组合，便于单步调试位置 ESO。
- 补全 `package.xml` 中维护者信息，按需增加新依赖。


2025.11.22日 位置环ESO类初步完成，并测试。测试成功
	测试步骤：
	cd ~/PX4_Firmware/ESO_paper_reproduction
	source devel/setup.bash
	source ~/PX4_Firmware/Tools/setup_gazebo.bash ~/PX4_Firmware ~/PX4_Firmware/build/px4_sitl_default
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware:~/PX4_Firmware/Tools/sitl_gazebo
	roslaunch uav_arm_top arm_pid_SITL_Gazebo.launch |& tee ~/PX4_Firmware/ESO_paper_reproduction/px4_logs/px4_$(date +%F_%H%M%S).log
	5.新建终端运行：rosrun uav_control eso_data_collector（可能需要source一下）【启动ESO估计】
	6.启动QGC使得飞机飞行
	7.新建终端运行：rosrun arm_controller joint_position_commander.py【机械臂开始运动】
	8.新建终端运行：rqt_plot【绘图工具】
	9.配置绘图：
		在 rqt_plot 上方的输入框里，输入话题名并回车：
		/eso/debug/disturbance/vector/x (看X轴干扰)
		/eso/debug/disturbance/vector/y (看Y轴干扰)
		/eso/debug/position/vector/x (看位置估计)
		/mavros/local_position/pose/pose/position/x (看真实位置)
	10.保存数据：
	在实验开始前，运行：
	rosbag record /mavros/local_position/pose /eso/debug/disturbance /eso/debug/position -O eso_test_01.bag 【录制真实位置和 ESO 的输出】
	做实验（起飞、动机械臂、降落）。
	Ctrl+C 结束录制。
	这个 .bag 文件就是你的原始数据，之后可以用 Python 读取它，用 matplotlib 画出出版级的图表。

	如何看ESO估计是否正确？
	1.位置估计：是否与真实位置一致。
	2.干扰估计：
	（1）稳定时看z轴扰动：应该始终保持为一个定值——因为无人机受到了重力
	   例如：
		代码逻辑：ESO的控制器输入为u_input(0, 0, 9.81)，这意味着你在告诉 ESO：“我认为现在的推力抵消了重力，产生了一个向上的 9.81 加速度。”
		物理事实：飞机悬停，实际垂直加速度在0左右。
		观测数据：eso/debug/disturbance/vector/z 保持在-10左右。
		根据公式
			z2_dot = u_input + z3_est_dist_ + beta2_.cwiseProduct(err)
		因为
			z2_dot = 0
		所以
			u_input = -z3_est_dist_ - beta2_.cwiseProduct(err) = -9.81 - 误差 ≈ -10
	（2）手动添加扰动：
		根据F=ma，已知质量m=xxx,手动添加F=4N：
		rosservice call /gazebo/apply_body_wrench "{body_name: 'uav_arm_v4::base_link', reference_frame: 'base_link', wrench: { force: { x: 4.0, y: 0.0, z: 0.0 } }, start_time: 0, duration: 5000000000}"
		记录扰动a，带入式子m=F/a
		观察计算之后的m与实际m是否一致。

	11.如何根据保存的数据作图？
	chmod +x plot_eso_results.py
	python3 plot_eso_result.py eso_test_01.bag
	注意：bag文件和plot_eso_results.py文件需要放在同一个文件夹下。一般默认是生成在工作空间下，需要移动一下位置

2025.11.23日 位置环控制器初步完成，并测试。测试未完成
	测试步骤：
	1.cd ~/PX4_Firmware/ESO_paper_reproduction
	2.source devel/setup.bash
	3.source ~/PX4_Firmware/Tools/setup_gazebo.bash ~/PX4_Firmware ~/PX4_Firmware/build/px4_sitl_default
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware:~/PX4_Firmware/Tools/sitl_gazebo
	4.roslaunch uav_arm_top arm_pid_SITL_Gazebo.launch
	5.新建终端运行：rosrun uav_control position_control_test（可能需要source一下）【启动位置环控制器，稳定在2米位置】

    测试结果1：无法稳定，飞机乱飞炸机
	检查：
	1.开环推力测试：验证推力和重力方向
		目标：确保飞机能老实待在地上，或者缓慢起飞，而不是“火箭发射”
		// 临时注释掉控制器计算
		// Eigen::Vector3d force_cmd = ctrl.calculateControl(...);

		// --- 强制测试 1: 只给重力 (理论上应该悬停或缓慢升降) ---
		// 假设质量 1.25kg，重力约 12.25N
		Eigen::Vector3d force_cmd(0, 0, 12.25);

		// --- 映射 ---
		// 这里的 30.0 是关键！如果飞机直接冲顶，说明 30.0 太小了，要改大！
		double max_thrust = 30.0;
		auto att_thrust_pair = ControlUtils::forceToAttitudeThrust(force_cmd, 0.0, max_thrust);
	结果：发现飞机无法起飞，一直加到19.25N，才起飞。发现是ControlUtils::forceToAttitudeThrust函数中，推力代码为线性映射，实际为非线性映射
		// 修改 src/ControlUtils.cpp
		// 原来是：double thrust_norm = thrust_force / max_thrust_newton;
		// 改成：
		double thrust_norm = sqrt(thrust_force / max_thrust_newton);
	结果：12.25N可以起飞

	2.纯位置环控制器控制
		目标：验证参数是否会导致震荡，以及方向是否正确（负反馈）。
		禁用 ESO：注释原ESO代码，将Eigen::Vector3d disturbance = Eigen::Vector3d::Zero();设置为0。
		禁用前馈：注释原af代码，将Eigen::Vector3d sys_com_pos = Eigen::Vector3d::Zero();设置为0。

		调参log：
		   1.第一次调参
			Lambda=[1.5, 1.5, 1.5], Kv=[2.0, 2.0, 2.0]
			现象：飞机一启动就侧翻
			原因：积分项在地面“憋炸了” (Integral Windup)
			场景还原：
				代码一运行，target_z = 2.0，实际 z = 0.0。
				误差 e_z = 2.0。同时，如果 GPS 稍微有点漂移（比如 x = 0.1），那么 e_x = -0.1。// 文件路径: /home/cf/PX4_Firmware/ESO_paper_reproduction/src/arm_controller/src/move_arm_z.cpp

				#include <ros/ros.h>
				#include <trajectory_msgs/JointTrajectory.h>
				#include <trajectory_msgs/JointTrajectoryPoint.h>
				#include <cmath>
				#include <vector>
				#include <string>

				int main(int argc, char** argv) {
				    // 1. 初始化ROS节点
				    ros::init(argc, argv, "arm_z_disturbance_generator_cpp");
				    ros::NodeHandle nh;

				    // 2. 创建发布者，发布到与Gazebo控制器匹配的话题
				    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("/joint_trajectory_controller/command", 10);
				    ros::Rate rate(50.0); // 以50Hz频率发布

				    // 3. 定义关节名称
				    std::vector<std::string> joint_names = {"arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4"};

				    ros::Time start_time = ros::Time::now();
				    ROS_INFO("C++ Z轴扰动脚本启动: 机械臂将开始上下运动。");

				    while (ros::ok()) {
					double elapsed_time = (ros::Time::now() - start_time).toSec();

					// --- 核心运动逻辑 ---
					// 频率: 0.2 Hz (5秒一个周期), 振幅: 0.5 弧度 (约30度)
					double angle_offset = 0.5 * std::sin(2 * M_PI * 0.2 * elapsed_time);

					// 创建轨迹消息
					trajectory_msgs::JointTrajectory traj;
					traj.header.stamp = ros::Time::now();
					traj.joint_names = joint_names;

					// 创建轨迹点
					trajectory_msgs::JointTrajectoryPoint point;
					// 关节2 和 关节 3 协同运动产生上下效果
					point.positions.resize(joint_names.size());
					point.positions[0] = 0.0; // 保持基座不动
					point.positions[1] = -0.5 + angle_offset; // 大臂上下摆动
					point.positions[2] = 0.5 - angle_offset;  // 小臂配合运动
					point.positions[3] = 0.0; // 保持手腕不动
					point.time_from_start = ros::Duration(0.5); // 要求在0.5秒内到达

					traj.points.push_back(point);

					// 发布消息
					pub.publish(traj);

					ros::spinOnce();
					rate.sleep();
				    }

				    return 0;
				}

				在你**未解锁（Disarmed）**的那几秒钟里，integral_pos_err_（积分项）一直在疯狂累加。
				解锁瞬间：积分项已经累积了一个巨大的值。控制器瞬间输出一个“向后猛拉 + 向上猛冲”的指令。
				由于电机响应非线性，或者地面的摩擦力，飞机可能瞬间向前或向后翻滚。
			解决办法：在解锁的一瞬间，强制清零控制器
				// 全局变量记录上一次的状态
				bool last_armed_state = false
				while(ros::ok()) {
				// ...
				// 【新增】检测到从 "未解锁" 变为 "解锁" 的瞬间 -> 重置控制器
				if (g_current_state.armed && !last_armed_state) {
					ROS_WARN("Vehicle ARMED! Resetting Controllers...");
					ctrl.resetIntegral();          // 清零 PID 积分
					// 如果 ESO 有重置接口，最好也重置 ESO，或者给一个初值
					// eso.reset();
					last_u_nominal << 0, 0, 9.81;  // 重置 ESO 的输入记忆
				}
				last_armed_state = g_current_state.armed;
				// ... 后面的计算逻辑 ...
				}
			结果：飞机能够正常起飞，但是xyz都在晃动，像一个弹簧一样在目标位置附近疯狂来回弹跳
		   	原因：控制参数太“硬”了，或者内环（速度环）不够快，跟不上外环（位置环）的需求。
			解决办法：
				第一刀：大幅降低外环 Lambda_p（让它变“懒”）现在的飞机太“急躁”了，想瞬间到达目标，结果冲过头了。我们要让它慢下来。
				第二刀：保持或微调内环 K_v（增加阻尼）K_v 相当于阻尼。阻尼越大，越粘稠，越不容易晃。

2025.11.24日 位置环控制器测试。基本完成
	调参log：
	   2.第二次调参
	   	调小外环，调大内环，结果：飞机能够正常起飞，但是xyz都在晃动，像一个弹簧一样在目标位置附近疯狂来回弹跳
	   3.第三次调参
	   	PositionController ctrl(M_TOTAL, 0.3, 0.8, 1.0, 1.5);
		把内环也调小，结果：飞机能够正常起飞，并且稳定
		原因：因为微分项过大，所以噪声严重，导致飞机抖动。
	ESO和前馈都加上，悬停时，机械臂运动，飞机存在0.1米左右的误差

2025.11.24日 位置环控制器测试。目的——提升精度到0.02米，失败
	测试步骤：
	1.cd ~/PX4_Firmware/ESO_paper_reproduction
	2.source devel/setup.bash
	3.source ~/PX4_Firmware/Tools/setup_gazebo.bash ~/PX4_Firmware ~/PX4_Firmware/build/px4_sitl_default
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware:~/PX4_Firmware/Tools/sitl_gazebo
	4.roslaunch uav_arm_top arm_pid_SITL_Gazebo.launch
	5.新建终端运行：rosbag record /mavros/local_position/pose /eso/debug/position /eso/debug/disturbance /uav_arm/joint_states -O data_position_test.bag（可能需要source一下）【录制数据】
	6.新建终端运行：rosrun uav_control position_control_test（可能需要source一下）【启动位置环控制器，稳定在2米位置】
	7.新建终端运行：rosrun arm_controller joint_position_commander.py（可能需要source一下）【运行机械臂】
	8.新建终端运行：
	# 默认从第 10 秒开始画
	python3 src/uav_control/scripts/plot_positionTest.py data_position_test.bag
	# 如果你想看从第 5 秒开始
	python3 src/uav_control/scripts/plot_positionTest.py data_position_test.bag --start 5.0（可能需要source一下）【绘制轨迹】
	调参log：
	   1.取消ESO和模型前馈补偿，只测试控制器，先保证控制器精度
	   	第一次调参：
		    Z轴在开始时，会有超调（2.25m），之后会在2m附近来回震荡，误差0.04m,后续增大到【-0.075,+0.05】,整体基本在0.04左右
		    xy轴：开始20秒，误差最大达到-0.4(x轴)和-0.25(y轴)，50秒左右0.25(y轴)，后续0.15m
		第二次调参：取消积分
		    Z轴，没有超调，但是有0.1m左右的静差，上下震荡[0.05m]，五十秒左右震荡到0.1m
		    x轴，静差在-0.25左右，震荡0.1m
		    y轴，静差在-0.2左右，震荡0.1m
		第三次调参：积分为2
		    没有静差
		    Z轴正在开始时出现超调，0.16左右，后续误差在0.05左右
		    X轴在开始时，误差最大达到0.45，后续0.1m
		    Y轴在开始时，误差最大达到0.25，后续0.1~0.2m
		第四次调参：增大微分项 从(0.3, 0.8, 1, 2);到(0.3, 0.8, 2, 2);
		    噪声太大，明显不行
		第五次调参：减小微分项(M_TOTAL, 0.3, 0.8, 1.5, 2);
		[1. 真实位置控制精度 (Real Position Control Accuracy)]
			Axis   | Mean (m)     | Std Dev (m)  | Peak-to-Peak (m) | Max Deviation (m)
			---------------------------------------------------------------------------
			X      |      -0.0106 |       0.0549 |           0.4260 |           0.3368
			Y      |       0.0053 |       0.0764 |           0.3548 |           0.2125
			---------------------------------------------------------------------------
			* Std Dev (标准差): 反映飞行平稳度，越小越好。
			* Max Deviation (最大偏离): 反映是否达到 0.02m 精度目标的关键指标。
			Z轴正在开始时出现超调，0.35左右，后续误差在0.05左右，后续误差在0.05左右
		第六次调参：增大微分项，找到噪声的平衡点(M_TOTAL, 0.3, 0.8, 1.8, 2);
		    噪声太大，明显不行
		第七次调参：减小微分项，找到噪声的平衡点(M_TOTAL, 0.3, 0.8, 1.65, 2);
		    噪声太大，明显不行
		第八次调参：减小微分项，找到噪声的平衡点(M_TOTAL, 0.3, 0.8, 1.55, 2);
		    噪声太大，明显不行
		第九次调参：减小微分项，找到噪声的平衡点(M_TOTAL, 0.3, 0.8, 1.0, 3.0);
		    中间调了几次，微分项是存在耦合的，调大Z的同时，要调小XY
		Axis   | Mean (m)     | Std Dev (m)  | Peak-to-Peak (m) | Max Deviation (m)
		---------------------------------------------------------------------------
		X      |      -0.0450 |       0.0822 |           0.5572 |           0.4160
		Y      |       0.0085 |       0.0979 |           0.5142 |           0.2763
		   效果不如第五次的


2025.11.28日 姿态环控制器测试，怀疑可能是姿态环也需要调整
	1.lockstep问题：
		ERROR [mavlink] Please disable lockstep for actuator offboard control:
		ERROR [mavlink] https://dev.px4.io/master/en/simulation/#disable-lockstep-simulation
	解决方法：
		1.注释掉对应if块：src/modules/mavlink/mavlink_receiver.cpp，handle_message_set_actuator_control_target 开头有一个 #if defined(ENABLE_LOCKSTEP_SCHEDULER) 分支会直接打印。仓库当前版本的位置大约是 src/modules/mavlink/mavlink_receiver.cpp (line 1263) 左右。
		2.清理缓存：make clean
		3.重新编译仿真：make px4_sitl gazebo_uav_arm_v4
	上实物时不建议，实物建议onboard模式

# ESO vs PX4-PID Plot Data

This folder contains the raw data, plotting scripts, and generated figures for
the UAM V5 PX4 SITL comparison after arm motion starts.

- `raw/`: ROS bag files, recorder metadata, and computed metric summaries.
- `scripts/`: scripts used to record the bags and regenerate the plots.
- `figure/`: generated PNG figures.

## Fresh PX4 SITL comparison

The fresh comparison uses:

- `exp1_hover_disturbance_uam_v5` as Simulink `model1` / mode 1, the disturbance
  rejection test.
- `exp4_square_tracking_uam_v5` as Simulink mode 2, the square trajectory
  tracking test.
- `paper_eso`: the current paper ESO parameter set.
- `px4_pid`: the PX4 native multicopter/PID baseline from the same `uam_v5`
  PX4 SITL launch, with no ESO parameters injected by the data script.

Each fresh run is written to its own dataset folder under `raw/` and `figure/`
so it does not mix with legacy data. The current dataset is:

- raw data: `raw/px4_sitl_comparison_20260513_193944/`
- MATLAB figures: `figure/px4_sitl_comparison_20260513_193944/`

The canonical MATLAB inputs inside the dataset folder are:

- `px4_sitl_mode1_paper_eso.mat`
- `px4_sitl_mode2_paper_eso.mat`
- `px4_sitl_mode1_px4_pid.mat`
- `px4_sitl_mode2_px4_pid.mat`

Each `.mat` file contains a `result` struct compatible with
`scripts/plot_px4_pid_comparison.m`. The recorded segment starts at the first
`/experiment/arm_motion_enabled=true` marker. Mode 1 is cropped to 60 seconds;
mode 2 is cropped after the square trajectory returns to the origin.

To rerun the fresh SITL flights and regenerate the `.mat` files:

```bash
cd /home/cf/PX4_Firmware_clean
source ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh
python3 ESO_paper_reproduction/data/scripts/run_px4_sitl_comparison_for_matlab.py
```

To generate the MATLAB figures from the current dataset:

```matlab
cd('/home/cf/PX4_Firmware_clean')
addpath('ESO_paper_reproduction/data/scripts')
raw_dir = 'ESO_paper_reproduction/data/raw/px4_sitl_comparison_20260513_193944';
figure_dir = 'ESO_paper_reproduction/data/figure/px4_sitl_comparison_20260513_193944';
plot_px4_pid_comparison( ...
    fullfile(raw_dir, 'px4_sitl_mode1_paper_eso.mat'), ...
    fullfile(raw_dir, 'px4_sitl_mode2_paper_eso.mat'), ...
    fullfile(raw_dir, 'px4_sitl_mode1_px4_pid.mat'), ...
    fullfile(raw_dir, 'px4_sitl_mode2_px4_pid.mat'), ...
    figure_dir)
```

## Legacy ESO vs MC figures

To regenerate the older Python figures from `raw/`:

```bash
source /opt/ros/noetic/setup.bash
python3 scripts/plot_current_default_eso_vs_mc_after_arm_motion.py
```

# ESO vs MC Plot Data

This folder contains the raw data, plotting scripts, and generated figures for
the current-default ESO vs MC comparison after arm motion starts.

- `raw/`: ROS bag files, recorder metadata, and computed metric summaries.
- `scripts/`: scripts used to record the bags and regenerate the plots.
- `figure/`: generated PNG figures.

To regenerate the figures from `raw/`:

```bash
source /opt/ros/noetic/setup.bash
python3 scripts/plot_current_default_eso_vs_mc_after_arm_motion.py
```


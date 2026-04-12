# me5413_slam_comparison

Minimal evaluation workflow for the ME5413 SLAM task.

For the final A-LOAM submission, the intended comparison is:

- reference: `/gazebo/ground_truth/state`
- estimate: `/aft_mapped_to_init`

## 1. Record a bag during one mapping run

```bash
cd ~/ros_ws/src/ME5413_Final_Project_Group11/src/me5413_slam_comparison/scripts
./record_trajectory_aloam.sh
```

Default output:

- `../bags/aloam_eval.bag`

Recorded topics:

- `/gazebo/ground_truth/state`
- `/aft_mapped_to_init`
- `/laser_odom_to_init`
- `/aft_mapped_path`
- `/tf`
- `/tf_static`
- `/mid/points`

## 2. Extract TUM trajectories

```bash
cd ~/ros_ws/src/ME5413_Final_Project_Group11/src/me5413_slam_comparison/scripts
./extract_trajectory_aloam.sh
```

Default outputs:

- `../bags/ground_truth_trajectory.tum`
- `../bags/aloam_mapped_trajectory.tum`
- `../bags/aloam_laser_odom_trajectory.tum`
- `../bags/aloam_path_trajectory.tum`

## 3. Run evo

Install `evo` first if it is not already available:

```bash
pip3 install --user evo
```

Then run:

```bash
cd ~/ros_ws/src/ME5413_Final_Project_Group11/src/me5413_slam_comparison/scripts
./evo_compare_aloam.sh
```

Default comparison:

- reference: `ground_truth_trajectory.tum`
- estimate: `aloam_mapped_trajectory.tum`

Outputs:

- raw APE:
  - `../results/aloam_ape_raw.zip`
  - `../results/plots/aloam_vs_ground_truth_raw.pdf`
- aligned APE:
  - `../results/aloam_ape_aligned.zip`
  - `../results/plots/aloam_vs_ground_truth_aligned.pdf`

## Suggested report content

- one trajectory overlay figure
- one APE summary table with RMSE / mean / max
- report both:
  - raw error
  - aligned error
- a short note that the estimate is evaluated against `/gazebo/ground_truth/state`

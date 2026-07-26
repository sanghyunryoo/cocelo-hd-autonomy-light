# autonomy_light

Minimal pipeline only:

`/livox/lidar` + `/livox/imu` → Point-LIO → `/cloud_registered` + `/aft_mapped_to_init` → elevation map.

Run after building the workspace:

```bash
./build.sh
source ../../install/setup.bash
ros2 launch autonomy_light basic_pipeline.launch.py use_sim_time:=true
```

The elevation output is `/autonomy_light/elevation_map` (`sensor_msgs/PointCloud2`) in `hd/base`.
Legacy autonomy-light functionality is retained, but not built, in `temp/legacy_20260726`.

Build products always live in the workspace root: `ros2_ws/build`, `ros2_ws/install`, and `ros2_ws/log`.

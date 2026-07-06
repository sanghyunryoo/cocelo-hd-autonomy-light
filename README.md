# cocelo-hd-autonomy-light

## Jetson Livox setup

This repository vendors `livox_ros_driver2` under `third_party/livox_ros_driver2`.
On a new Jetson, prepare the Livox SDK, link the vendored driver into the ROS
workspace, and build it with:

```bash
cd ~/ros2_ws/src/cocelo-hd-autonomy-light
./build.sh
```

If the Jetson already has Livox-SDK2 installed:

```bash
./build.sh --skip-sdk
```

The script creates:

```text
~/ros2_ws/src/livox_ros_driver2 -> ~/ros2_ws/src/cocelo-hd-autonomy-light/third_party/livox_ros_driver2
```

Then run:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
./launch.sh --real
```

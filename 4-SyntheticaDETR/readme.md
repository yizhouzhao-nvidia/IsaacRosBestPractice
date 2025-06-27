# SyntheticaDETR

https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_object_detection/index.html

Start Docker
```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
./scripts/run_dev.sh
```

Build
```bash
cd isaac_ros_examples
colcon build --symlink-install  --packages-select isaac_ros_examples
colcon build --symlink-install --packages-up-to isaac_ros_rtdetr --base-paths ${ISAAC_ROS_WS}/src/isaac_ros_object_detection/isaac_ros_rtdetr
```
Or

```bash
sudo apt update
sudo apt-get install -y ros-humble-isaac-ros-examples
rosdep update && rosdep install --from-paths ${ISAAC_ROS_WS}/src/isaac_ros_object_detection/isaac_ros_rtdetr --ignore-src -y
colcon build --symlink-install --packages-up-to isaac_ros_rtdetr
source install/setup.bash
```

Launch
```bash
ros2 launch isaac_ros_examples isaac_ros_examples.launch.py launch_fragments:=rtdetr interface_specs_file:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_rtdetr/quickstart_interface_specs.json engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr/sdetr_grasp.plan confidence_threshold:=0.1
```

Detection output:
```bash
ros2 topic info /detections_output
ros2 interface show vision_msgs/msg/Detection2DArray
```

Test
```bash
colcon test --packages-select isaac_ros_rtdetr --event-handlers console_direct+
# or
python -m pytest src/isaac_ros_object_detection/isaac_ros_rtdetr/test/isaac_ros_rtdetr_pol_test.py -v
```

Visualization
```bash
ros2 run isaac_ros_rtdetr isaac_ros_rtdetr_visualizer.py
ros2 run rqt_image_view rqt_image_view /rtdetr_processed_image
```

# Modification

1. Modify `interface_specs_file` at: `${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_rtdetr/quickstart_interface_specs_realsense.json`

```json
{
    "camera_resolution": {
        "width": 1280,
        "height": 720
    }
}
```

ros2 launch isaac_ros_examples isaac_ros_examples.launch.py launch_fragments:=rtdetr interface_specs_file:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_rtdetr/quickstart_interface_specs.json engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr/sdetr_grasp.plan

2. Modify `${ISAAC_ROS_WS}/isaac_ros_object_detection/isaac_ros_rtdetr/launch/isaac_ros_rtdetr_core.launch.py` Line 64

```python
remappings=[
                    ('image', ' '),
                    ('camera_info', '/camera/camera/color/camera_info')
                ]
            ),
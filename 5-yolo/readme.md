# Yolo

Edit `image_input_topic` and `camera_info_input_topic` at `isaac_ros_object_detection/isaac_ros_yolov8/launch/isaac_ros_yolov8_core.launch.py`

```
            'image_input_topic': DeclareLaunchArgument(
                'image_input_topic',
                default_value='/camera/camera/color/image_raw',
                description='Input image topic name'
            ),
            'camera_info_input_topic': DeclareLaunchArgument(
                'camera_info_input_topic',
                default_value='/camera/camera/color/camera_info',
                description='Input camera info topic name'
            ),
```

Build
```bash
sudo apt-get update
rosdep update && rosdep install --from-paths ${ISAAC_ROS_WS}/src/isaac_ros_object_detection/isaac_ros_yolov8 --ignore-src -y
colcon build --symlink-install --packages-up-to isaac_ros_yolov8
source install/setup.bash
```

Run
```bash
ros2 launch isaac_ros_examples isaac_ros_examples.launch.py launch_fragments:=yolov8 interface_specs_file:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_yolov8/quickstart_interface_specs.json model_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8/yolov8s.onnx engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8/yolov8s.plan
```

Visualizer
```bash
source install/setup.bash
ros2 run isaac_ros_yolov8 isaac_ros_yolov8_visualizer.py
```
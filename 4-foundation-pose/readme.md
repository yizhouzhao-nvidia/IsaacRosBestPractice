```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
./scripts/run_dev.sh
```

https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_pose_estimation/isaac_ros_foundationpose/index.html


Build
```bash
cd isaac_ros_examples
colcon build --symlink-install  --packages-select isaac_ros_examples
colcon build --symlink-install --packages-up-to isaac_ros_rtdetr --base-paths ${ISAAC_ROS_WS}/src/isaac_ros_object_detection/isaac_ros_rtdetr
```

Launch
```
ros2 launch isaac_ros_examples isaac_ros_examples.launch.py launch_fragments:=rtdetr interface_specs_file:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_rtdetr/quickstart_interface_specs.json engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr/sdetr_grasp.plan
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
ros2 run rqt_image_view rqt_image_view /rtdetr_processed_image
```
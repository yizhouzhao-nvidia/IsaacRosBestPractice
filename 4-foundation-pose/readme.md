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
```

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
ros2 run isaac_ros_yolov8 isaac_ros_yolov8_visualizer.py
```

# Foundation pose
```bash
cd ${ISAAC_ROS_WS}/src && \
   git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation.git isaac_ros_pose_estimation
   
cd ${ISAAC_ROS_WS}/src/isaac_ros_pose_estimation
```

Update
```bash
sudo apt-get update
rosdep update && rosdep install --from-paths ${ISAAC_ROS_WS}/src/isaac_ros_pose_estimation/isaac_ros_foundationpose --ignore-src -y
```

Build
```bash
colcon build --symlink-install --packages-up-to isaac_ros_foundationpose
source install/setup.bash
```

Model download and TensorRT conversion
```bash
# SyntheticaDETR 
mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr && \
cd ${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr && \
   wget 'https://api.ngc.nvidia.com/v2/models/nvidia/isaac/synthetica_detr/versions/1.0.0_onnx/files/sdetr_grasp.onnx'
/usr/src/tensorrt/bin/trtexec --onnx=${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr/sdetr_grasp.onnx --saveEngine=${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr/sdetr_grasp.plan

# Foundation pose
mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose && \
   cd ${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose && \
   wget 'https://api.ngc.nvidia.com/v2/models/nvidia/isaac/foundationpose/versions/1.0.0_onnx/files/refine_model.onnx' -O refine_model.onnx && \
   wget 'https://api.ngc.nvidia.com/v2/models/nvidia/isaac/foundationpose/versions/1.0.0_onnx/files/score_model.onnx' -O score_model.onnx
/usr/src/tensorrt/bin/trtexec --onnx=${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose/refine_model.onnx --saveEngine=${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose/refine_trt_engine.plan --minShapes=input1:1x160x160x6,input2:1x160x160x6 --optShapes=input1:1x160x160x6,input2:1x160x160x6 --maxShapes=input1:42x160x160x6,input2:42x160x160x6
/usr/src/tensorrt/bin/trtexec --onnx=${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose/score_model.onnx --saveEngine=${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose/score_trt_engine.plan --minShapes=input1:1x160x160x6,input2:1x160x160x6 --optShapes=input1:1x160x160x6,input2:1x160x160x6 --maxShapes=input1:252x160x160x6,input2:252x160x160x6
```

Run
```bash
## may need to run
# sudo apt-get install -y ros-humble-isaac-ros-examples
ros2 launch isaac_ros_examples isaac_ros_examples.launch.py launch_fragments:=foundationpose interface_specs_file:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_foundationpose/quickstart_interface_specs.json mesh_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_foundationpose/Mustard/textured_simple.obj texture_path:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_foundationpose/Mustard/texture_map.png score_engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose/score_trt_engine.plan refine_engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose/refine_trt_engine.plan rt_detr_engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr/sdetr_grasp.plan
```

Visualization
```bash
rviz2 -d  $(ros2 pkg prefix isaac_ros_foundationpose --share)/rviz/foundationpose.rviz
```
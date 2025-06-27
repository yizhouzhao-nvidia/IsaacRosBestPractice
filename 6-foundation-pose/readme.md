Start Docker
```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
./scripts/run_dev.sh
```

https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_pose_estimation/isaac_ros_foundationpose/index.html


# Foundation pose
```bash
cd ${ISAAC_ROS_WS}/src && \
   git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation.git isaac_ros_pose_estimation
   
cd ${ISAAC_ROS_WS}/src/isaac_ros_pose_estimation
```

Update
```bash
sudo apt-get update
sudo apt-get install -y ros-humble-isaac-ros-examples
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
rviz2 -d isaac_ros_foundationpose/rviz/foundationpose.rviz
```

## Utils

topic relay
```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-topic-tools

ros2 run topic_tools relay camera/camera/color/image_raw image_rect
ros2 run topic_tools relay camera/camera/depth/image_rect_raw depth
ros2 run topic_tools relay camera/camera/color/camera_info camera_info_rect
```

# Banana
```bash
ros2 launch isaac_ros_examples isaac_ros_examples.launch.py launch_fragments:=foundationpose interface_specs_file:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_foundationpose/quickstart_interface_specs.json mesh_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_foundationpose/banana/banana.obj texture_path:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_foundationpose/banana/banana.png score_engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose/score_trt_engine.plan refine_engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose/refine_trt_engine.plan rt_detr_engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr/sdetr_grasp.plan
```

# soup can
```bash
ros2 launch isaac_ros_examples isaac_ros_examples.launch.py launch_fragments:=foundationpose interface_specs_file:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_foundationpose/quickstart_interface_specs.json mesh_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_foundationpose/soup_can/soup_can.obj texture_path:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_foundationpose/soup_can/baked_mesh_tex0.png score_engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose/score_trt_engine.plan refine_engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose/refine_trt_engine.plan rt_detr_engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr/sdetr_grasp.plan
```

# Debug
realsense depth
```
header:
  stamp:
    sec: 1750982989
    nanosec: 190113525
  frame_id: camera_color_optical_frame
height: 480
width: 640
encoding: 16UC1
is_bigendian: 0
step: 1280

```

Need to be
```
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: tf_camera
height: 480
width: 640
encoding: 32FC1
is_bigendian: 0
step: 2560
```
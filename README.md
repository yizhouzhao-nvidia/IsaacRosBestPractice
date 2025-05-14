# Isaac Ros Best Practice [Updated 05/14/2025]

Best Practices for [Isaac ROS](https://developer.nvidia.com/isaac/ros)

### Testing systems:

x86_64 Platform, Ubuntu 24.04 LTS, GPU RTX A6000, Driver 550, CUDA 12.4, Isaac Sim 4.5 

## Environment set up

1. Follow [Developer Environment Setup](https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html), and verify that you have `ISAAC_ROS_WS` env variable:

```bash
echo $ISAAC_ROS_WS
```
2. Clone isaac_ros_common under `${ISAAC_ROS_WS}/src`.

```bash
cd ${ISAAC_ROS_WS}/src && \
   git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git isaac_ros_common
```

3. Start the docker container:

```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
   ./scripts/run_dev.sh
```

## Tutorials

1. [Controlling ur10e robot in Isaac Sim via ROS2 topic publishing](./1-ros2+isaac%20sim+ur10e/README.md)

2. [Controlling ur10e robot via Moveit and CuMotion](./2-ur10e+CuMotion/README.md)

3. [Connecting to Realsense] (Coming soon)

4. [FoundationPose/Dope ur10e robot in Isaac Sim MoveIt CuMotion Loop] (Coming soon)

5. [Real-Robot Practice: Putting everything together] (Coming soon)

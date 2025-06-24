# RealSense

## 1. Install RealSense

Update dependencies
```bash
sudo apt-get update
sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
```

Build librealsense from source, or see this [tutorial](https://dev.intelrealsense.com/docs/compiling-librealsense-for-linux-ubuntu-guide)
```bash
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
mkdir build && cd build
cmake .. -DBUILD_TOOLS=ON
make -j$(nproc)
sudo make install
```

## Docker start

Add
```
    --device=/dev/bus/usb \
    --device=/dev/video0 \
    --device=/dev/video1 \
    --device=/dev/video2 \
    --device=/dev/video3 \
    --device=/dev/video4 \
    --device=/dev/video5 \
    --device=/dev/media0 \
    --device=/dev/media1 \
```


[Update Fireware](https://dev.intelrealsense.com/docs/firmware-update-tool) if necessary


# 2. With Docker publish image:

See [Realsense Ros Github](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file)

```bash
sudo apt update
sudo apt install -y ros-humble-librealsense2*
sudo apt install -y ros-humble-realsense2-*
```

# Debug
```bash
sudo apt install v4l-utils
v4l2-ctl --list-devices
```

# 3. Start Node
```bash
ros2 launch realsense2_camera rs_launch.py
ros2 launch realsense2_camera rs_launch.py rgb_camera.color_profile:=640x480x30 depth_module.depth_profile:=640x480x30 pointcloud.enable:=false
```




List ros2 topics, 

```bash
ros2 topic list
```

And you should see

```
/camera/camera/color/camera_info
/camera/camera/color/image_raw
/camera/camera/color/image_raw/compressed
/camera/camera/color/image_raw/compressedDepth
/camera/camera/color/image_raw/theora
/camera/camera/color/metadata
/camera/camera/depth/camera_info
/camera/camera/depth/image_rect_raw
/camera/camera/depth/image_rect_raw/compressed
/camera/camera/depth/image_rect_raw/compressedDepth
/camera/camera/depth/image_rect_raw/theora
/camera/camera/depth/metadata
/camera/camera/extrinsics/depth_to_color
/parameter_events
/rosout
/tf_static
```

# Visualization

```bash
ros2 run rqt_image_view rqt_image_view 
```
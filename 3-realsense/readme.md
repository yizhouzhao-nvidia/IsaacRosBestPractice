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

[Update Fireware](https://dev.intelrealsense.com/docs/firmware-update-tool) if necessary


# 2. With Docker publish image:

See [Realsense Ros Github](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file)

```
sudo apt install ros-humble-librealsense2*
sudo apt install ros-humble-realsense2-*
```
# pycuvslam

## Realsense best practice

Refer to: https://github.com/yizhouzhao-nvidia/IsaacRosBestPractice/tree/main/3-realsense
```
# Clone the repository
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense

# Create and enter build directory
mkdir build && cd build

# replace your DPYTHON_EXECUTABLE with <path to your python>
cmake ../ -DCMAKE_BUILD_TYPE=Release -DFORCE_RSUSB_BACKEND=ON -DBUILD_EXAMPLES=true -DBUILD_PYTHON_BINDINGS=true  -DCHECK_FOR_UPDATES=false -DPYTHON_EXECUTABLE=/home/yizhou/Projects/pycuvslam/.venv/bin/python

# Build (using all available cores)
make -j$(nproc)

# Install system-wide
sudo make install
```

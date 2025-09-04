# pycuvslam

## Env set up

Install uv: 
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
uv venv --python=3.10
source .venv/bin/activate
```

Install pycuvslam: https://github.com/NVlabs/PyCuVSLAM/tree/main
```
git clone https://github.com/NVlabs/pycuvslam.git
cd pycuvslam

uv pip install -e bin/x86_64
uv pip install -r examples/requirements.txt
```

**RUN EVERYTIME!**
```bash
# replace with you python3.10 python lib
export LD_LIBRARY_PATH=/home/yizhou/.local/share/uv/python/cpython-3.10.18-linux-x86_64-gnu/lib:$LD_LIBRARY_PATH 
```

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

Run an example:
```
# enter pycuvslam/examples/realsense
python run_stereo.py

```


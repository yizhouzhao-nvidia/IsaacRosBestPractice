colcon build --packages-up-to isaac_ros_ess --cmake-args -DBUILD_TESTING=OFF

uv pip install --upgrade pip
uv pip install wheel
uv pip install tensorrt==10.3.0
uv pip install pycuda
uv pip install matplotlib

export LD_LIBRARY_PATH=/home/yizhou/Projects/IsaacRosBestPractice/7-depth/.venv/lib/python3.10/site-packages/nvidia/cudnn/lib/:$LD_LIBRARY_PATH

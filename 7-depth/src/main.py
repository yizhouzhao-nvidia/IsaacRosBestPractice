import numpy as np
import os
import pycuda.driver as cuda
import pycuda.autoinit
import tensorrt as trt
import ctypes

import matplotlib.pyplot as plt
from PIL import Image

TRT_LOGGER = trt.Logger()

# Filenames of TensorRT plan file and input/output images.
engine_file = "/home/yizhou/workspaces/isaac_ros-dev/isaac_ros_assets/models/dnn_stereo_disparity/dnn_stereo_disparity_v4.1.0_onnx/ess.engine"
plugin_path = "/home/yizhou/workspaces/isaac_ros-dev/isaac_ros_assets/models/dnn_stereo_disparity/dnn_stereo_disparity_v4.1.0_onnx/plugins/x86_64/ess_plugins.so"
input_file  = "./image/left.png"
output_file = "./image/right.png"

def load_engine(engine_file_path, plugin_path=None):
    assert os.path.exists(engine_file_path)

    # Load the plugin library
    if plugin_path:
        assert os.path.exists(plugin_path), f"Plugin library not found: {plugin_path}"
        ctypes.CDLL(plugin_path)
        print(f"Plugin library {plugin_path} loaded.")

    print("Reading engine from file {}".format(engine_file_path))
    with open(engine_file_path, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
        return runtime.deserialize_cuda_engine(f.read())

def main():
    print("Running TensorRT inference for FCN-ResNet101")
    with load_engine(engine_file, plugin_path) as engine:
        print("Engine loaded successfully")
        # context = engine.create_execution_context()
        # print("Execution context created successfully")
        # input_data = np.random.randn(1, 3, 224, 224).astype(np.float32)
        # output_data = np.empty((1, 1000), dtype=np.float32)
        # bindings = [int(input_data.device), int(output_data.device)]
        # stream = cuda.Stream()
    

if __name__ == "__main__":
    main()
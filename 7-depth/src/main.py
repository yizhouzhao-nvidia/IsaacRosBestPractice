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

output_file = "./image/disparity.png"

def load_and_resize_image(image_path, target_size=(960, 576)):
    """
    Load an image from path and resize it to the target size.
    
    Args:
        image_path (str): Path to the image file
        target_size (tuple): Target size as (width, height)
    
    Returns:
        PIL.Image: Resized image
    """
    image = Image.open(image_path)
    print(f"Original image size: {image.size}")
    
    # Resize the image
    resized_image = image.resize(target_size, Image.Resampling.LANCZOS)
    print(f"Resized image size: {resized_image.size}")
    
    return np.moveaxis(np.array(resized_image), 2, 0)


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
    
    # Example: Load and resize images
    print("\n--- Image Loading and Resizing Example ---")
    
    # Load and resize the left image
    left_image = load_and_resize_image("./image/left.png", target_size=(960, 576))
    print(f"Successfully loaded and resized left image to {left_image.size}")
    # You can save the resized image if needed
    # left_image.save("./image/left_resized.png")
    
    # Load and resize the right image
    right_image = load_and_resize_image("./image/right.png", target_size=(960, 576))
    print(f"Successfully loaded and resized right image to {right_image.size}")
    # You can save the resized image if needed
    # right_image.save("./image/right_resized.png")
    
    print("\n--- TensorRT Inference ---")
    with load_engine(engine_file, plugin_path) as engine:
        print("Engine loaded successfully")
        with engine.create_execution_context() as context:
            tensor_names = [engine.get_tensor_name(i) for i in range(engine.num_io_tensors)]

            input_buffer = {}
            input_memory = {}
            output_buffer = {}
            output_memory = {}
            
            for tensor in tensor_names:
                size = trt.volume(context.get_tensor_shape(tensor))
                dtype = trt.nptype(engine.get_tensor_dtype(tensor))

                
                if engine.get_tensor_mode(tensor) == trt.TensorIOMode.INPUT:
                    context.set_input_shape(tensor, (1, 3, 576, 960))
                    input_image = left_image if tensor == "input_left" else right_image
                    input_buffer[tensor] = np.ascontiguousarray(input_image)
                    input_memory[tensor] = cuda.mem_alloc(input_image.nbytes)
                    context.set_tensor_address(tensor, int(input_memory[tensor]))
                else:
                    output_buffer[tensor] = cuda.pagelocked_empty(size, dtype)
                    output_memory[tensor] = cuda.mem_alloc(output_buffer[tensor].nbytes)
                    context.set_tensor_address(tensor, int(output_memory[tensor]))

            stream = cuda.Stream()

            

            # Transfer input data to the GPU.
            cuda.memcpy_htod_async(input_memory["input_left"], input_buffer["input_left"], stream)
            cuda.memcpy_htod_async(input_memory["input_right"], input_buffer["input_right"], stream)
            
            # Run inference
            context.execute_async_v3(stream_handle=stream.handle)

            import ipdb; ipdb.set_trace()

            # Transfer prediction output from the GPU.
            cuda.memcpy_dtoh_async(output_buffer["output_left"], output_memory["output_left"], stream)
            cuda.memcpy_dtoh_async(output_buffer["output_conf"], output_memory["output_conf"], stream)
            
            import ipdb; ipdb.set_trace()
            

            stream.synchronize()
            output_d64 = np.array(output_buffer["output_left"], dtype=np.int64)
            # np.savetxt('test.out', output_d64.astype(int), fmt='%i', delimiter=' ', newline=' ')
            img = np.reshape(output_d64, (576, 960))
            print("Writing output image to file {}".format(output_file))
            # transfer the numpy array to uint8 and save it as grayscale png
            Image.fromarray(img.astype(np.uint8)).save(output_file)
            
        # print("Execution context created successfully")
        # input_data = np.random.randn(1, 3, 224, 224).astype(np.float32)
        # output_data = np.empty((1, 1000), dtype=np.float32)
        # bindings = [int(input_data.device), int(output_data.device)]
        # stream = cuda.Stream()
    

if __name__ == "__main__":
    main()
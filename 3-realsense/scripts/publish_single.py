#!/usr/bin/env python3

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import json
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory


class SingleImagePublisher(Node):
    """ROS2 Node to publish single test image and camera info at 10 FPS."""
    
    def __init__(self):
        super().__init__('single_image_publisher')
        
        # Publishers
        self.image_pub = self.create_publisher(
            Image, 
            '/image_rect', 
            10
        )
        self.camera_info_pub = self.create_publisher(
            CameraInfo, 
            '/camera_info_rect', 
            10
        )
        
        # CV Bridge for image conversion
        self.cv_bridge = CvBridge()
        
        # Timer for 10 FPS (0.1 seconds)
        self.timer = self.create_timer(0.1, self.publish_data)
        
        # Load test data
        self.load_test_data()
        
        self.get_logger().info('Single Image Publisher started - Publishing at 10 FPS')
        self.get_logger().info(f'Image topic: /image_rect')
        self.get_logger().info(f'Camera info topic: /camera_info_rect')
    
    def load_test_data(self):
        """Load image and camera info from test data."""
        try:
            # Get package path
            package_path = get_package_share_directory('isaac_ros_rtdetr')
            test_data_path = os.path.join(
                package_path, 
                '..', '..', '..', '..', 
                'isaac_ros_rtdetr', 
                'test', 
                'test_cases', 
                'single_detection'
            )
            test_data_path = os.path.abspath(test_data_path)
            
            self.get_logger().info(f'Loading test data from: {test_data_path}')
            
            # Load camera info
            camera_info_path = os.path.join(test_data_path, 'camera_info.json')
            with open(camera_info_path, 'r') as f:
                self.camera_info_data = json.load(f)
            
            # Load image metadata
            image_json_path = os.path.join(test_data_path, 'image.json')
            with open(image_json_path, 'r') as f:
                image_metadata = json.load(f)
            
            # Load image
            image_path = os.path.join(test_data_path, image_metadata['image'])
            self.image = cv2.imread(image_path)
            
            if self.image is None:
                raise FileNotFoundError(f'Could not load image: {image_path}')
            
            # Convert BGR to RGB (OpenCV loads as BGR, ROS expects RGB)
            self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
            
            self.get_logger().info(f'Loaded image: {image_path}')
            self.get_logger().info(f'Image shape: {self.image.shape}')
            self.get_logger().info(f'Image encoding: {image_metadata["encoding"]}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load test data: {str(e)}')
            raise
    
    def create_camera_info_msg(self):
        """Create CameraInfo message from JSON data."""
        camera_info = CameraInfo()
        
        # Header
        camera_info.header = Header()
        camera_info.header.stamp = self.image_time_stamp #self.get_clock().now().to_msg()
        camera_info.header.frame_id = self.camera_info_data['header']['frame_id']
        
        # Image dimensions
        camera_info.width = self.camera_info_data['width']
        camera_info.height = self.camera_info_data['height']
        
        # Distortion model and parameters
        camera_info.distortion_model = self.camera_info_data['distortion_model']
        camera_info.d = self.camera_info_data['D']
        
        # Camera matrices
        camera_info.k = self.camera_info_data['K']
        camera_info.r = self.camera_info_data['R']
        camera_info.p = self.camera_info_data['P']
        
        return camera_info
    
    def create_image_msg(self):
        """Create Image message from loaded image."""
        # Create header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        self.image_time_stamp = header.stamp
        header.frame_id = self.camera_info_data['header']['frame_id']
        
        # Convert OpenCV image to ROS Image message
        image_msg = self.cv_bridge.cv2_to_imgmsg(self.image, encoding='rgb8')
        image_msg.header = header
        
        return image_msg
    
    def publish_data(self):
        """Timer callback to publish image and camera info."""
        try:
            # Create and publish image message
            image_msg = self.create_image_msg()
            self.image_pub.publish(image_msg)
            
            # Create and publish camera info message
            camera_info_msg = self.create_camera_info_msg()
            self.camera_info_pub.publish(camera_info_msg)
            
            # Log every 50 publishes (5 seconds at 10 FPS)
            if hasattr(self, 'publish_count'):
                self.publish_count += 1
            else:
                self.publish_count = 1
                
            if self.publish_count % 50 == 0:
                self.get_logger().info(f'Published {self.publish_count} frames')
        
        except Exception as e:
            self.get_logger().error(f'Error publishing data: {str(e)}')


def main(args=None):
    """Main function to run the node."""
    rclpy.init(args=args)
    
    try:
        node = SingleImagePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import os
import math
import yaml
import numpy as np
from PIL import Image

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        # 'map' 토픽으로 OccupancyGrid 메시지를 퍼블리시하는 퍼블리셔 생성
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', 10)
        
        # YAML 파일 및 이미지 경로 설정 (필요에 따라 경로 수정)
        map_dir = os.path.expanduser('~/map')
        yaml_file = os.path.join(map_dir, 'a3.yaml')
        self.get_logger().info(f"Loading map metadata from: {yaml_file}")
        
        with open(yaml_file, 'r') as f:
            self.map_metadata = yaml.safe_load(f)
        
        # 이미지 파일 경로 (YAML 파일 내 'image' 필드 사용)
        image_file = os.path.join(map_dir, self.map_metadata['image'])
        self.get_logger().info(f"Loading map image from: {image_file}")
        
        self.image = Image.open(image_file)
        
        # 이미지 데이터를 OccupancyGrid 데이터로 변환
        self.grid_data = self.convert_image_to_occupancy(self.image, self.map_metadata)
        
        # OccupancyGrid 메시지 생성
        self.map_msg = OccupancyGrid()
        self.map_msg.header = Header()
        self.map_msg.header.frame_id = self.map_metadata.get('frame', 'map')
        
        # MapMetaData 설정
        meta = MapMetaData()
        meta.resolution = self.map_metadata['resolution']
        meta.width = self.image.width
        meta.height = self.image.height
        
        # YAML의 origin은 [x, y, yaw]
        origin = self.map_metadata['origin']
        meta.origin.position.x = float(origin[0])
        meta.origin.position.y = float(origin[1])
        meta.origin.position.z = 0.0
        
        yaw = float(origin[2])
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        meta.origin.orientation.x = 0.0
        meta.origin.orientation.y = 0.0
        meta.origin.orientation.z = qz
        meta.origin.orientation.w = qw
        
        self.map_msg.info = meta
        self.map_msg.data = self.grid_data
        
        # 1초마다 맵 퍼블리시하는 타이머 생성
        self.timer = self.create_timer(1.0, self.publish_map)
        self.get_logger().info("Map publisher node started.")

    def convert_image_to_occupancy(self, image, metadata):
        image = image.convert('L')
        data = np.array(image, dtype=np.float32)
        norm_data = data / 255.0
        
        negate = metadata.get('negate', 0)
        occupied_thresh = metadata.get('occupied_thresh', 0.65)
        free_thresh = metadata.get('free_thresh', 0.196)
        
        occupancy = np.empty(norm_data.shape, dtype=np.int8)
        if negate:
            occupancy[norm_data < free_thresh] = 100
            occupancy[norm_data > occupied_thresh] = 0
        else:
            occupancy[norm_data > occupied_thresh] = 100
            occupancy[norm_data < free_thresh] = 0
        occupancy[(norm_data >= free_thresh) & (norm_data <= occupied_thresh)] = -1
        
        return occupancy.flatten().tolist()

    def publish_map(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.map_msg)
        self.get_logger().info("Published map.")

def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import yaml
import numpy as np
from PIL import Image
import os

class MapServer(Node):
  def __init__(self):
    super().__init__('map_server')
    self.publisher_ = self.create_publisher(OccupancyGrid, 'map', 10)
    self.timer = self.create_timer(1, self.publish_map)

    self.declare_parameter('yaml_filename', '')  # パラメータを宣言
    map_yaml_path = self.get_parameter('yaml_filename').get_parameter_value().string_value  # パラメータ値を取得

    self.map_msg = self.load_map(map_yaml_path)

  def load_map(self, map_yaml_path):
    with open(map_yaml_path, 'r') as file:
        map_data = yaml.safe_load(file)
    yaml_directory = os.path.dirname(map_yaml_path)
    map_image_path = os.path.join(yaml_directory, map_data['likelihood_image'])
    image = Image.open(map_image_path)
    image = image.transpose(Image.FLIP_TOP_BOTTOM)  # 画像と地図の座標系が違うためy軸方向を反転
    map_image = np.array(image)
    
    # 尤度マップの値をOccupancyGridの範囲（0-100）に変換
    occupancy_grid_data = np.interp(map_image, [0, 255], [100, 0]).astype(int)

    map_msg = OccupancyGrid()
    map_msg.header.frame_id = "map"
    map_msg.info.width = map_image.shape[1]
    map_msg.info.height = map_image.shape[0]
    map_msg.info.resolution = map_data['resolution']
    map_msg.info.origin.position.x = map_data['origin'][0]
    map_msg.info.origin.position.y = map_data['origin'][1]
    map_msg.info.origin.position.z = 0.0
    map_msg.info.origin.orientation.w = 1.0
    map_msg.data = occupancy_grid_data.ravel().tolist()
    return map_msg

  def publish_map(self):
    self.publisher_.publish(self.map_msg)

def main(args=None):
  rclpy.init(args=args)
  map_server = MapServer()
  rclpy.spin(map_server)
  map_server.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import yaml
from pathlib import Path
from typing import Dict, Any, Optional
import os


class LikelihoodMapGenerator(Node):

    def __init__(self):
        super().__init__('likelihood_map_generator')
        script_directory: Path = Path(__file__).resolve().parent
        map_dir: Path = script_directory / "../../config/maps"

        self.declare_parameter('map_yaml_path', str(map_dir / 'iscas_museum_map.yaml'))
        self.declare_parameter('output_likelihood_map_path', str(map_dir / 'iscas_museum_map_likelihood.pgm'))
        self.declare_parameter('kernel_size', 7)
        self.declare_parameter('sigma', 10.0)

        
        self.map_yaml_path: str = self.get_parameter('map_yaml_path').value
        self.output_likelihood_map_path: str = self.get_parameter('output_likelihood_map_path').value

        self.kernel_size: int = self.get_parameter('kernel_size').value
        self.sigma: float = self.get_parameter('sigma').value
        
        self.map_image: Optional[np.ndarray] = None
        self.likelihood_map: Optional[np.ndarray] = None
        self.resolution: float = 0.0
        self.origin: list[float] = [0.0, 0.0, 0.0]
        
        self.load_map(self.map_yaml_path)
        self.create_likelihood_map()
        self.display_maps()
        

    def load_map(self, yaml_path: str) -> None:
        with open(yaml_path, 'r') as file:
            map_data: Dict[str, Any] = yaml.safe_load(file)
        
        image_path: Path = Path(yaml_path).parent / map_data['image']
        self.map_image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
        if self.map_image is None:
            self.get_logger().error(f'Failed to load image: {image_path}')
            return

        self.resolution = map_data['resolution']
        self.origin = map_data['origin']
        
        self.get_logger().info(f'Map loaded successfully: {image_path}')
        self.get_logger().info(f'Map shape: {self.map_image.shape}')

    def create_likelihood_map(self) -> None:
        if self.map_image is None:
            self.get_logger().error('Map image is not loaded')
            return

        zero_regions: np.ndarray = (self.map_image == 0)  # 値が0の領域を抽出

        blurred: np.ndarray = cv2.GaussianBlur(zero_regions.astype(np.float32), 
                                               (self.kernel_size, self.kernel_size), 
                                               self.sigma)

        normalized: np.ndarray = (blurred * 254).astype(np.uint8)  # 正規化して0-254の範囲に変換

        self.likelihood_map = 254 - normalized

        self.likelihood_map[zero_regions] = 0  # 元画像の0であった部分を尤度マップでも0にする


        output_path: str = self.get_parameter('output_likelihood_map_path').value
        if output_path:
            self.get_logger().info(f'Likelihood map saved to: {output_path}')

    def display_maps(self) -> None:
        if self.map_image is not None and self.likelihood_map is not None:
            cv2.namedWindow('Original Map', cv2.WINDOW_NORMAL)
            cv2.imshow('Original Map', self.map_image)

            cv2.namedWindow('Likelihood Map', cv2.WINDOW_NORMAL)
            cv2.imshow('Likelihood Map', self.likelihood_map)

            self.get_logger().info('Displaying maps. Press any key to continue...')
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        else:
            self.get_logger().warn('Maps are not available for display')

def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = LikelihoodMapGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
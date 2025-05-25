import rclpy 
from rclpy.node import Node
from rclpy.serialization import serialize_message

import pykitti
import numpy as np

from tf_transformations import quaternion_from_matrix

class Cat2Traj(Node):
    def __init__(self):
        super().__init__('cat2traj')

        self.finished = False 

        # Parameters
        self.basedir = self.declare_parameter('dataset_path', '/ros2_ws/KITTI/odometry').value
        self.lidar_topic_name = self.declare_parameter('lidar_topic_name', '/kitti/velodyne_points').value
        self.pose_topic_name = self.declare_parameter('pose_topic_name', '/kitti/pose').value
        self.sequence = int(self.declare_parameter('sequence', '0').value)
            
        self.output_file = self.declare_parameter('output_file', f'{self.sequence}_kitti_pose.tum').value
        
        self.get_logger().info(f"Reading KITTI sequence {self.sequence} from {self.basedir}")
        self.get_logger().info(f"Saving TUM format to {self.output_file}")
        
        # Load KITTI dataset
        self.load_dataset()

        self.convert_kitti_to_tum(self.output_file)
        self.get_logger().info("Conversion complete.")

    def load_dataset(self):
        self.sequence = f'{self.sequence:02d}'
        self.dataset = pykitti.odometry(self.basedir, self.sequence)

    def convert_kitti_to_tum(self, output_file):
        with open(output_file, 'w') as f:
            for pose, timestamp in zip(self.dataset.poses, self.dataset.timestamps):
                t = pose[0:3, 3]
                quat = quaternion_from_matrix(pose)
                ts = timestamp.total_seconds()

                f.write(f"{ts:.6f} {t[0]:.6f} {t[1]:.6f} {t[2]:.6f} "
                        f"{quat[0]:.6f} {quat[1]:.6f} {quat[2]:.6f} {quat[3]:.6f}\n")


def main(args=None):
    rclpy.init(args=args)
    node = Cat2Traj()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 

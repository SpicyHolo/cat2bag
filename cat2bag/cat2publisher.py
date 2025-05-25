import rclpy
from rclpy.node import Node

import sensor_msgs.msg as sensor_msgs
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_matrix

import std_msgs.msg

import pykitti
import numpy as np
import struct

class KittiVelodynePublisher(Node):
    def __init__(self):
        super().__init__('kitti_velodyne_publisher')

        # Parameters
        self.basedir = self.declare_parameter('dataset_path', '/ros2_ws/KITTI/odometry').value
        self.lidar_topic_name = self.declare_parameter('lidar_topic_name', '/kitti/velodyne_points').value
        self.pose_topic_name = self.declare_parameter('pose_topic_name', '/kitti/pose').value
        self.sequence = self.declare_parameter('sequence', 0).value

        self.lidar_publisher = self.create_publisher(PointCloud2, self.lidar_topic_name, 10)
        self.pose_publisher = self.create_publisher(PoseStamped, self.pose_topic_name, 10)

        # Load dataset for current sequence
        self.load_dataset()

        # Timer to publish frames at ~10Hz (adjust as needed)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def load_dataset(self):
        self.sequence = f'{self.sequence:02d}'  # format like '01', '02', etc.
        self.dataset = pykitti.odometry(self.basedir, self.sequence)
        self.frame_idx = 0
        if self.dataset.frames is not None:
            self.num_frames = len(self.dataset.frames)
        else:
            # fallback: use timestamps length
            self.num_frames = len(self.dataset.timestamps)
        self.get_logger().info(f'Loaded KITTI sequence {self.sequence}, total frames: {self.num_frames}')

    def create_pointcloud2_msg(self, points, timestamp, frame_id='velodyne'):
        # points: Nx4 numpy array (x,y,z,intensity)
        header = std_msgs.msg.Header()
        header.stamp = timestamp
        header.frame_id = frame_id

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        # Flatten points as bytes
        cloud_data = points.astype(np.float32).tobytes()

        pointcloud_msg = PointCloud2()
        pointcloud_msg.header = header
        pointcloud_msg.height = 1
        pointcloud_msg.width = points.shape[0]
        pointcloud_msg.fields = fields
        pointcloud_msg.is_bigendian = False
        pointcloud_msg.point_step = 16  # 4 floats * 4 bytes each
        pointcloud_msg.row_step = pointcloud_msg.point_step * points.shape[0]
        pointcloud_msg.is_dense = True
        pointcloud_msg.data = cloud_data

        return pointcloud_msg

    def create_pose_msg(self, pose, timestamp):
        translation = pose[0:3, 3]
        quat = quaternion_from_matrix(pose)

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'world'
        pose_msg.header.stamp = timestamp

        pose_msg.pose.position.x = translation[0]
        pose_msg.pose.position.y = translation[1]
        pose_msg.pose.position.z = translation[2]

        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        return pose_msg

    def timer_callback(self):
        # Finished sequence

        if self.frame_idx >= self.num_frames:
            self.get_logger().info('Completed all sequences 1 to 10. Shutting down.')
            rclpy.shutdown()
            return

        timestamp = self.get_clock().now().to_msg()

        # Get velodyne scan for current frame (subsampled 10 times)
        velo_points = self.dataset.get_velo(self.frame_idx)#[::10, :]  # Nx4 (x,y,z,intensity)

        # Create PointCloud2 msg
        pc_msg = self.create_pointcloud2_msg(velo_points, timestamp)
    
        # Get pose
        pose = self.dataset.poses[self.frame_idx]
        pose_msg = self.create_pose_msg(pose, timestamp)

        # Publish
        self.lidar_publisher.publish(pc_msg)
        self.pose_publisher.publish(pose_msg)
        self.get_logger().info(f'Published frame {self.frame_idx} of sequence {self.sequence}, points: {velo_points.shape[0]}')
        self.frame_idx += 1


def main(args=None):
    rclpy.init(args=args)
    node = KittiVelodynePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

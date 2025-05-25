import rclpy 
from rclpy.node import Node
from rclpy.serialization import serialize_message

import sensor_msgs.msg as sensor_msgs
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg

import pykitti
import numpy as np

from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_matrix

# Import rosbag2 python writer API
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata

class KittiVelodyneBagger(Node):
    def __init__(self):
        super().__init__('kitti_velodyne_bagger')

        self.finished = False 

        # Parameters
        self.basedir = self.declare_parameter('dataset_path', '/ros2_ws/KITTI/odometry').value
        self.lidar_topic_name = self.declare_parameter('lidar_topic_name', '/kitti/velodyne_points').value
        self.pose_topic_name = self.declare_parameter('pose_topic_name', '/kitti/pose').value
        self.sequence = int(self.declare_parameter('sequence', '0').value)

        # Initialize rosbag2 writer
        self.writer = SequentialWriter()
        storage_options = StorageOptions(uri='kitti_velodyne_bag', storage_id='sqlite3')
        converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        self.writer.open(storage_options, converter_options)
        
        # Create lidar topic
        lidar_topic_info = TopicMetadata(
                name = self.lidar_topic_name,
                type = 'sensor_msgs/msg/PointCloud2',
                serialization_format = 'cdr')
        self.writer.create_topic(lidar_topic_info) 
        
        # Create ground truth (pose) topic
        pose_topic_info = TopicMetadata(
                name = self.pose_topic_name,
                type = 'geometry_msgs/msg/PoseStamped',
                serialization_format = 'cdr')
        self.writer.create_topic(pose_topic_info) 

        self.load_dataset()

    def load_dataset(self):
        self.sequence = f'{self.sequence:02d}'
        self.dataset = pykitti.odometry(self.basedir, self.sequence)
        self.frame_idx = 0

        if self.dataset.frames is not None:
            self.num_frames = len(self.dataset.frames)
        else:
            # fallback: use timestamps length
            self.num_frames = len(self.dataset.timestamps)
        self.get_logger().info(f'Loaded KITTI sequence {self.sequence}, total frames: {self.num_frames}')

    def create_pointcloud2_msg(self, points, timestamp_nsec):
        header = std_msgs.msg.Header()
        header.stamp.sec = timestamp_nsec // 1_000_000_000
        header.stamp.nanosec = timestamp_nsec % 1_000_000_000
        header.frame_id = 'velodyne'

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_data = points.astype(np.float32).tobytes()

        pc_msg = PointCloud2()
        pc_msg.header = header
        pc_msg.height = 1
        pc_msg.width = points.shape[0]
        pc_msg.fields = fields
        pc_msg.is_bigendian = False
        pc_msg.point_step = 16
        pc_msg.row_step = pc_msg.point_step * points.shape[0]
        pc_msg.is_dense = True
        pc_msg.data = cloud_data

        return pc_msg
        
    def create_pose_msg(self, pose, timestamp_nsec):
        translation = pose[0:3, 3]
        quat = quaternion_from_matrix(pose)

        pose_msg = PoseStamped()
        pose_msg.header.stamp.sec = timestamp_nsec // 1_000_000_000
        pose_msg.header.stamp.nanosec = timestamp_nsec % 1_000_000_000
        pose_msg.header.frame_id = 'world'

        pose_msg.pose.position.x = translation[0]
        pose_msg.pose.position.y = translation[1]
        pose_msg.pose.position.z = translation[2]

        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        return pose_msg

    def write_pointcloud_to_bag(self):
        while not self.finished:
            if self.frame_idx >= self.num_frames:
                self.finished = True
                self.get_logger().info('Completed all sequences, closing bag and shutting down.')
                return

            # Get timestamp for this frame in nanoseconds (KITTI timestamps are timedelta)
            timestamp = self.dataset.timestamps[self.frame_idx]
            timestamp_nsec = int(timestamp.total_seconds() * 1e9)

            # Get velodyne points, subsample to speed up (optional)
            points = self.dataset.get_velo(self.frame_idx)#[::10, :]
            pose = self.dataset.poses[self.frame_idx]

            pc_msg = self.create_pointcloud2_msg(points, timestamp_nsec)
            pose_msg = self.create_pose_msg(pose, timestamp_nsec)

            # Serialize ROS2 message to CDR bytes
            serialized_pc_msg = serialize_message(pc_msg)
            serialized_pose_msg = serialize_message(pose_msg)
            
            # Write serialized message to bag
            self.writer.write(self.lidar_topic_name, serialized_pc_msg, timestamp_nsec)
            self.writer.write(self.pose_topic_name, serialized_pose_msg, timestamp_nsec)

                
            if (self.frame_idx % 100 == 0):
                self.get_logger().info(f"Processing frame {self.frame_idx}")
            self.frame_idx += 1


def main(args=None):
    rclpy.init(args=args)
    node = KittiVelodyneBagger()

    node.write_pointcloud_to_bag()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

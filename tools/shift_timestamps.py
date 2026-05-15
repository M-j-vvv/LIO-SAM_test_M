#!/usr/bin/env python3
"""
Simple ROS2 helper to shift header timestamps of common sensor topics.
Usage examples:
  # shift IMU timestamps forward by -1.08 seconds (i.e., subtract 1.08s)
  python3 tools/shift_timestamps.py --topic /imu/data --type imu --offset -1.08

  # shift PointCloud2 timestamps
  python3 tools/shift_timestamps.py --topic /velodyne_points --type pc2 --offset -1.08

Run this node while you play the bag (with --clock). It republishes on the same topic name with suffix _shifted (or you can remap LIO-SAM to subscribe to the shifted topic).
"""
import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2
import copy
import math

# velodyne_msgs may not be available in all environments; import lazily when needed
VelodyneScan = None


class ShiftTimestamps(Node):
    def __init__(self, topic, msg_type, offset_s, output_topic=None):
        super().__init__('shift_timestamps_' + msg_type.replace('/', '_'))
        self.topic = topic
        self.offset = float(offset_s)
        self.msg_type = msg_type
        # last published stamp in seconds (float) to keep monotonicity
        self._last_pub_stamp = None
        self.output_topic = output_topic if output_topic else topic + '_shifted'
        qos = QoSProfile(depth=10)

        if msg_type == 'imu':
            self.pub = self.create_publisher(Imu, self.output_topic, qos)
            self.sub = self.create_subscription(Imu, topic, self.callback_imu, qos)
            self.get_logger().info(f'Subscribed IMU {topic} -> publishing {self.output_topic} offset={self.offset}s')
        elif msg_type == 'pc2':
            self.pub = self.create_publisher(PointCloud2, self.output_topic, qos)
            self.sub = self.create_subscription(PointCloud2, topic, self.callback_pc2, qos)
            self.get_logger().info(f'Subscribed PointCloud2 {topic} -> publishing {self.output_topic} offset={self.offset}s')
        elif msg_type == 'velodyne':
            # lazy import
            try:
                from velodyne_msgs.msg import VelodyneScan as _Vel
                global VelodyneScan
                VelodyneScan = _Vel
            except Exception as e:
                self.get_logger().error('velodyne_msgs not available: ' + str(e))
                raise

            self.pub = self.create_publisher(VelodyneScan, self.output_topic, qos)
            self.sub = self.create_subscription(VelodyneScan, topic, self.callback_velodyne, qos)
            self.get_logger().info(f'Subscribed VelodyneScan {topic} -> publishing {self.output_topic} offset={self.offset}s')
        else:
            self.get_logger().error('Unsupported msg_type. Use imu|pc2|velodyne')
            raise RuntimeError('Unsupported msg_type')

    def shift_stamp(self, stamp):
        # stamp is builtin_interfaces.msg.Time
        # Use float seconds then split with floor to handle negatives correctly.
        total = float(stamp.sec) + float(stamp.nanosec) / 1e9
        total += float(self.offset)

        # Ensure monotonicity: if we have a last published stamp, bump forward slightly
        if self._last_pub_stamp is not None and total <= self._last_pub_stamp:
            # bump by 1 microsecond
            total = self._last_pub_stamp + 1e-6

        sec = math.floor(total)
        frac = total - sec
        # round to nearest nanosecond to avoid negative nanosec from floating errors
        nanosec = int(round(frac * 1e9))
        # clamp nanosec to [0, 999999999]
        if nanosec < 0:
            nanosec = 0
        if nanosec >= 1000000000:
            sec += 1
            nanosec -= 1000000000

        # mutate a copy-safe time object
        stamp.sec = int(sec)
        stamp.nanosec = int(nanosec)
        # remember last published stamp
        self._last_pub_stamp = float(stamp.sec) + float(stamp.nanosec) / 1e9
        return stamp

    def callback_imu(self, msg: Imu):
        # deepcopy to avoid accidental shared references
        out = copy.deepcopy(msg)
        try:
            out.header.stamp = self.shift_stamp(out.header.stamp)
        except Exception as e:
            self.get_logger().error('Failed to shift IMU stamp: ' + str(e))
            return
        self.pub.publish(out)

    def callback_pc2(self, msg: PointCloud2):
        out = copy.deepcopy(msg)
        try:
            out.header.stamp = self.shift_stamp(out.header.stamp)
        except Exception as e:
            self.get_logger().error('Failed to shift PointCloud2 stamp: ' + str(e))
            return
        self.pub.publish(out)

    def callback_velodyne(self, msg: VelodyneScan):
        out = copy.deepcopy(msg)
        try:
            out.header.stamp = self.shift_stamp(out.header.stamp)
        except Exception as e:
            self.get_logger().error('Failed to shift Velodyne stamp: ' + str(e))
            return
        self.pub.publish(out)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', required=True, help='Input topic to shift')
    parser.add_argument('--type', required=True, choices=['imu', 'pc2', 'velodyne'], help='Message type')
    parser.add_argument('--offset', required=True, type=float, help='seconds to add to header.stamp (can be negative)')
    parser.add_argument('--output', help='output topic (default: <topic>_shifted)')
    args = parser.parse_args()

    rclpy.init()
    node = ShiftTimestamps(args.topic, args.type, args.offset, args.output)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

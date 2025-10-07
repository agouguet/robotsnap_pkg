#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class MapProxyNode(Node):
    def __init__(self):
        super().__init__('unity_map_proxy')

        qos_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.qos_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # Subscriber Unity
        self.sub = self.create_subscription(
            OccupancyGrid,
            '/map_unity',
            self.map_callback,
            qos_sub
        )

        # Publisher Nav2
        self.pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            self.qos_pub
        )

        self.get_logger().info("Unity Map Proxy Node started.")

    def map_callback(self, msg: OccupancyGrid):
        msg.header.frame_id = "map"
        self.pub.publish(msg)
        self.get_logger().debug("Republished map to /map")


def main(args=None):
    rclpy.init(args=args)
    node = MapProxyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

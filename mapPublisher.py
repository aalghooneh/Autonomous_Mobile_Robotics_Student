import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

from mapUtilities import *


def timerCallback():
    Publisher.publish(occGrid)

if __name__ == "__main__":

    rclpy.init()

    map_ = mapManipulator()

    node = Node("mapPublisher")

    Publisher = node.create_publisher(OccupancyGrid, "/customMap", 10)

    # Create a static transform broadcaster
    static_broadcaster = StaticTransformBroadcaster(node)

    # Create an identity transform (no translation or rotation)
    static_transform_stamped = TransformStamped()
    static_transform_stamped.header.stamp = node.get_clock().now().to_msg()
    static_transform_stamped.header.frame_id = "map"
    static_transform_stamped.child_frame_id = "odom"

    static_transform_stamped.transform.translation.x = 0.0
    static_transform_stamped.transform.translation.y = 0.0
    static_transform_stamped.transform.translation.z = 0.0

    # Identity rotation quaternion
    static_transform_stamped.transform.rotation.x = 0.0
    static_transform_stamped.transform.rotation.y = 0.0
    static_transform_stamped.transform.rotation.z = 0.0
    static_transform_stamped.transform.rotation.w = 1.0

    # Send the transform
    static_broadcaster.sendTransform(static_transform_stamped)

    map_.make_likelihood_field()

    occGrid = map_.to_message()

    node.create_timer(1.0, timerCallback)

    rclpy.spin(node)

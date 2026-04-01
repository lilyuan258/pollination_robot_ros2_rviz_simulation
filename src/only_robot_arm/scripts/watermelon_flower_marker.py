#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray


class WatermelonFlowerMarker(Node):
    def __init__(self):
        super().__init__('watermelon_flower_marker')

        # 这里的 anchor_frame 要改成 joint_1 对应的 child link frame。
        # 如果你的 TF 里真的有 joint_1 这个 frame，也可以直接写 joint_1。
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('anchor_frame', 'link_1')

        # 需求：相对 joint_1 原点，world -Y 方向 0.8m，world +X 方向 0.1m
        self.declare_parameter('dx_world', 0.1)
        self.declare_parameter('dy_world', -0.8)

        # 花蕊离地 0.1m
        self.declare_parameter('flower_center_z', 0.1)

        self.world_frame = self.get_parameter('world_frame').value
        self.anchor_frame = self.get_parameter('anchor_frame').value
        self.dx_world = float(self.get_parameter('dx_world').value)
        self.dy_world = float(self.get_parameter('dy_world').value)
        self.flower_center_z = float(self.get_parameter('flower_center_z').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(MarkerArray, '/watermelon_flower', 10)
        self.timer = self.create_timer(0.2, self.publish_flower)

        self.get_logger().info(
            f'Watermelon flower marker started. world_frame={self.world_frame}, '
            f'anchor_frame={self.anchor_frame}'
        )

    def _make_marker(self, marker_id, marker_type, x, y, z, sx, sy, sz, r, g, b, a=1.0):
        m = Marker()
        m.header.frame_id = self.world_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'watermelon_flower'
        m.id = marker_id
        m.type = marker_type
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation.w = 1.0
        m.scale.x = sx
        m.scale.y = sy
        m.scale.z = sz
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = a
        m.lifetime = Duration(seconds=0.0).to_msg()
        return m

    def publish_flower(self):
        try:
            # 查询 world -> anchor_frame 的变换
            t = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.anchor_frame,
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(
                f'Cannot find transform {self.world_frame} -> {self.anchor_frame}: {ex}',
                throttle_duration_sec=2.0
            )
            return

        anchor_x = t.transform.translation.x
        anchor_y = t.transform.translation.y

        # 你的目标位置：按 world 方向偏移
        flower_x = anchor_x + self.dx_world
        flower_y = anchor_y + self.dy_world
        flower_z = self.flower_center_z

        markers = MarkerArray()

        # 0) 花茎：从地面到花蕊中心下方
        stem_height = max(0.02, flower_z)
        stem = self._make_marker(
            marker_id=0,
            marker_type=Marker.CYLINDER,
            x=flower_x,
            y=flower_y,
            z=stem_height / 2.0,
            sx=0.012,
            sy=0.012,
            sz=stem_height,
            r=0.15, g=0.65, b=0.15, a=1.0
        )
        markers.markers.append(stem)

        # 1) 花蕊
        core = self._make_marker(
            marker_id=1,
            marker_type=Marker.SPHERE,
            x=flower_x,
            y=flower_y,
            z=flower_z,
            sx=0.035,
            sy=0.035,
            sz=0.035,
            r=0.95, g=0.80, b=0.10, a=1.0
        )
        markers.markers.append(core)

        # 2) 花瓣，简单用 6 个小球近似
        petal_radius = 0.045
        petal_size = 0.040
        for i in range(6):
            ang = i * (2.0 * math.pi / 6.0)
            px = flower_x + petal_radius * math.cos(ang)
            py = flower_y + petal_radius * math.sin(ang)
            pz = flower_z

            petal = self._make_marker(
                marker_id=10 + i,
                marker_type=Marker.SPHERE,
                x=px,
                y=py,
                z=pz,
                sx=petal_size,
                sy=petal_size,
                sz=0.018,
                r=1.0, g=0.97, b=0.75, a=1.0
            )
            markers.markers.append(petal)

        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    node = WatermelonFlowerMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
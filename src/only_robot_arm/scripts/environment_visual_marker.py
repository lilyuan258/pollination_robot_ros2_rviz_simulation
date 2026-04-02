#!/usr/bin/env python3
import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def normalize_list(values: Optional[List[float]], expected_len: int, fallback: List[float]) -> List[float]:
    if values is None or len(values) != expected_len:
        return fallback
    return [float(v) for v in values]


class EnvironmentVisualMarker(Node):
    def __init__(self) -> None:
        super().__init__("environment_visual_marker")

        self.declare_parameter("world_frame", "world")
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("use_embedded_materials", True)

        self._parts = ("rail", "column", "platform")
        for part in self._parts:
            self.declare_parameter(f"{part}_enabled", True)
            self.declare_parameter(f"{part}_mesh_resource", "")
            self.declare_parameter(f"{part}_xyz", [0.0, 0.0, 0.0])
            self.declare_parameter(f"{part}_rpy", [0.0, 0.0, 0.0])
            self.declare_parameter(f"{part}_scale", [1.0, 1.0, 1.0])
            self.declare_parameter(f"{part}_rgba", [1.0, 1.0, 1.0, 1.0])

        self.world_frame = self.get_parameter("world_frame").value
        self.use_embedded_materials = bool(self.get_parameter("use_embedded_materials").value)

        publish_rate_hz = max(0.2, float(self.get_parameter("publish_rate_hz").value))
        self.marker_pub = self.create_publisher(MarkerArray, "/environment_visual", 10)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_markers)

        self.get_logger().info(
            "Environment visual marker started. "
            "This node only publishes display markers and does not affect MoveIt planning."
        )

    def _build_mesh_marker(self, marker_id: int, part_name: str) -> Optional[Marker]:
        enabled = bool(self.get_parameter(f"{part_name}_enabled").value)
        mesh_resource = str(self.get_parameter(f"{part_name}_mesh_resource").value).strip()
        if not enabled or not mesh_resource:
            return None

        xyz = normalize_list(self.get_parameter(f"{part_name}_xyz").value, 3, [0.0, 0.0, 0.0])
        rpy = normalize_list(self.get_parameter(f"{part_name}_rpy").value, 3, [0.0, 0.0, 0.0])
        scale = normalize_list(self.get_parameter(f"{part_name}_scale").value, 3, [1.0, 1.0, 1.0])
        rgba = normalize_list(self.get_parameter(f"{part_name}_rgba").value, 4, [1.0, 1.0, 1.0, 1.0])

        qx, qy, qz, qw = euler_to_quaternion(rpy[0], rpy[1], rpy[2])

        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "environment_visual"
        marker.id = marker_id
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = mesh_resource
        marker.mesh_use_embedded_materials = self.use_embedded_materials

        marker.pose.position.x = xyz[0]
        marker.pose.position.y = xyz[1]
        marker.pose.position.z = xyz[2]
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw

        marker.scale.x = max(1e-6, scale[0])
        marker.scale.y = max(1e-6, scale[1])
        marker.scale.z = max(1e-6, scale[2])

        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.color.a = rgba[3]
        marker.lifetime = Duration(seconds=0.0).to_msg()
        return marker

    def publish_markers(self) -> None:
        marker_array = MarkerArray()
        marker_id = 0
        for part_name in self._parts:
            marker = self._build_mesh_marker(marker_id, part_name)
            if marker is not None:
                marker_array.markers.append(marker)
                marker_id += 1

        self.marker_pub.publish(marker_array)


def main() -> None:
    rclpy.init()
    node = EnvironmentVisualMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

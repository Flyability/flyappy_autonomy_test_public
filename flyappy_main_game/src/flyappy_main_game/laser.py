import math
from collections.abc import Sequence

import numpy as np
from numpy import typing as npt
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class Laser:
    def __init__(
        self,
        fov: float,
        resolution: int,
        scaling: float,
        ros_node: Node,
        screen_width: int,
        screen_height: int,
    ) -> None:
        self.ros_node = ros_node
        self.fov = fov  # degrees
        self.angle_max = math.radians(fov / 2.0)  # radians
        self.angle_min = -math.radians(fov / 2.0)  # radians
        self.angle_increment = math.radians(fov / (resolution - 1.0))  # radians
        self.resolution = resolution
        self.range = 355  # pixels
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.laser_scan_publisher = ros_node.create_publisher(
            LaserScan, "/flyappy_laser_scan", 10
        )
        self.scaling = scaling

    def scan(
        self, start_point: tuple[float, float], bitmap: npt.NDArray[np.uint8]
    ) -> Sequence[tuple[int, int, int]]:
        pointcloud = []
        rays_to_cast = range(self.resolution)

        for i in rays_to_cast:
            # calc endpoint from angle and range
            angle = self.angle_min + (i * self.angle_increment)
            end_point = (
                int(math.cos(angle) * self.range + start_point[0]),
                int(-math.sin(angle) * self.range + start_point[1]),
            )
            # endpoint
            pointcloud.append(self._raycast(start_point, end_point, bitmap))
        # publish the scan
        self._publish_laser_scan(pointcloud, start_point)
        return pointcloud

    def _raycast(
        self,
        p0: tuple[float, float],
        p1: tuple[float, float],
        bitmap: npt.NDArray[np.uint8],
    ) -> tuple[int, int, int]:
        x0 = int(p0[0])
        y0 = int(p0[1])
        x1 = int(p1[0])
        y1 = int(p1[1])
        # calculate end point from angle and bitmap
        # bresenhams algorithm
        dx = x1 - x0
        dy = y1 - y0

        xsign = 1 if dx > 0 else -1
        ysign = 1 if dy > 0 else -1

        dx = abs(dx)
        dy = abs(dy)

        if dx > dy:
            xx, xy, yx, yy = xsign, 0, 0, ysign
        else:
            dx, dy = dy, dx
            xx, xy, yx, yy = 0, ysign, xsign, 0

        d = 2 * dy - dx
        y = 0

        pixel_pos = (x0, y0)
        for x in range(dx + 1):
            pixel_pos = (x0 + x * xx + y * yx, y0 + x * xy + y * yy)
            # break if out of bounds
            if not (0 <= pixel_pos[0] < self.screen_width) or not (
                0 <= pixel_pos[1] < self.screen_height
            ):
                return (*pixel_pos, 1)
            # save pixel_val
            pixel_val = bitmap[pixel_pos[0]][pixel_pos[1]]

            if pixel_val > 127:
                return (*pixel_pos, 1)
            # save val
            if d >= 0:
                y += 1
                d -= 2 * dx
            d += 2 * dy
        return (*pixel_pos, 0)

    def _publish_laser_scan(
        self, pointcloud: Sequence[tuple[int, int, int]], start_point: tuple[float, float]
    ) -> None:
        scan = LaserScan()
        scan.header.stamp = self.ros_node.get_clock().now().to_msg()
        scan.header.frame_id = "laser_frame"
        scan.range_min = 0.0
        scan.range_max = self.range * self.scaling
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment

        scan.ranges = []
        scan.intensities = []
        for i in range(len(pointcloud)):
            if pointcloud[i][2] == 1:
                dist = math.hypot(
                    pointcloud[i][0] - start_point[0], pointcloud[i][1] - start_point[1]
                )
                scan.ranges.append(self.scaling * dist)
                scan.intensities.append(pointcloud[i][2])
            else:
                scan.ranges.append(scan.range_max)
                scan.intensities.append(pointcloud[i][2])
        self.laser_scan_publisher.publish(scan)

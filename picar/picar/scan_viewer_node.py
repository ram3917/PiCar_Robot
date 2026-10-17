"""Throwaway: builds a 2D occupancy grid from the ultrasonic sweep and publishes it on /map.

Subscribes to /controller/scan and publishes nav_msgs/OccupancyGrid on /map in the scan's own
frame (ultrasonic_link), with the grid centred on the sensor. The robot is assumed to stay still,
since there is no odometry or TF yet. Each beam marks cells along a cone (``cone_deg`` wide) as
free, and the cell at the hit as occupied. Log-odds are clamped so the map can change over time.

The grid logic is pure Python, so it is easy to check by hand. This whole file is a first step and
will be removed later.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from picar.interface import SCAN_TOPIC

MAP_TOPIC = "/map"
CONE_STEP_DEG = 1.0  # angle between the rays that make up one beam's cone


class OccupancyGridBuilder:
    """Log-odds occupancy grid, centred on the sensor. Pure logic, no ROS.

    Cell (row, col) covers x in [origin_x + col * res, +res) and y in [origin_y + row * res, +res),
    with x forward and y left, as in the scan. Cells that were never observed stay at log-odds 0.
    """

    def __init__(
        self,
        resolution: float,
        width_m: float,
        height_m: float,
        cone_deg: float,
        l_occ: float,
        l_free: float,
        l_min: float,
        l_max: float,
    ) -> None:
        self.resolution = resolution
        self.width = round(width_m / resolution)
        self.height = round(height_m / resolution)
        self.origin_x = -width_m / 2.0
        self.origin_y = -height_m / 2.0
        self.cone_deg = cone_deg
        self.l_occ = l_occ
        self.l_free = l_free
        self.l_min = l_min
        self.l_max = l_max
        self._log_odds = [0.0] * (self.width * self.height)

    def add_scan(
        self,
        angle_min: float,
        angle_increment: float,
        ranges: list[float],
        range_min: float,
        range_max: float,
    ) -> None:
        for i, r in enumerate(ranges):
            if math.isnan(r) or r < range_min:
                continue
            angle = angle_min + i * angle_increment
            if r == math.inf or r >= range_max:
                self._trace_beam(angle, range_max, hit=False)
            else:
                self._trace_beam(angle, r, hit=True)

    def _trace_beam(self, angle: float, r: float, hit: bool) -> None:
        sensor_cell = self._cell(0.0, 0.0)
        half = self.cone_deg / 2.0
        n = round(half / CONE_STEP_DEG)
        # Symmetric about the beam, always including the centre ray (offset 0).
        offsets = [k * half / n for k in range(-n, n + 1)] if n > 0 else [0.0]
        for offset in offsets:
            a = angle + math.radians(offset)
            end = self._cell(r * math.cos(a), r * math.sin(a))
            cells = _line(sensor_cell, end)
            if hit and offset == 0.0:
                # The last cell of the centre ray is the hit; every other cell is free.
                for cell in cells[:-1]:
                    self._update(cell, self.l_free)
                self._update(cells[-1], self.l_occ)
            else:
                # Edge rays do not mark a hit, so their end cell is free too.
                for cell in cells[:-1] if hit else cells:
                    self._update(cell, self.l_free)

    def _cell(self, x: float, y: float) -> tuple[int, int]:
        col = math.floor((x - self.origin_x) / self.resolution)
        row = math.floor((y - self.origin_y) / self.resolution)
        return row, col

    def _update(self, cell: tuple[int, int], delta: float) -> None:
        row, col = cell
        if not (0 <= row < self.height and 0 <= col < self.width):
            return
        i = row * self.width + col
        self._log_odds[i] = min(self.l_max, max(self.l_min, self._log_odds[i] + delta))

    def occupancy_values(self) -> list[int]:
        """int8 values for OccupancyGrid.data: 0-100 occupancy, -1 for unknown, row-major."""
        values = []
        for l in self._log_odds:
            if l == 0.0:
                values.append(-1)
            else:
                values.append(round(100.0 / (1.0 + math.exp(-l))))
        return values


def _line(start: tuple[int, int], end: tuple[int, int]) -> list[tuple[int, int]]:
    """Bresenham's line between two cells, both ends included."""
    r0, c0 = start
    r1, c1 = end
    dr, dc = abs(r1 - r0), abs(c1 - c0)
    sr = 1 if r1 >= r0 else -1
    sc = 1 if c1 >= c0 else -1
    err = dc - dr
    cells = []
    r, c = r0, c0
    while True:
        cells.append((r, c))
        if r == r1 and c == c1:
            return cells
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            c += sc
        if e2 < dc:
            err += dc
            r += sr


class ScanViewerNode(Node):
    def __init__(self) -> None:
        super().__init__("scan_viewer")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("width_m", 4.0)
        self.declare_parameter("height_m", 4.0)
        self.declare_parameter("cone_deg", 15.0)
        self.declare_parameter("l_occ", 0.85)
        self.declare_parameter("l_free", -0.4)
        self.declare_parameter("l_min", -2.0)
        self.declare_parameter("l_max", 3.5)

        self._builder = OccupancyGridBuilder(
            resolution=self.get_parameter("resolution").value,
            width_m=self.get_parameter("width_m").value,
            height_m=self.get_parameter("height_m").value,
            cone_deg=self.get_parameter("cone_deg").value,
            l_occ=self.get_parameter("l_occ").value,
            l_free=self.get_parameter("l_free").value,
            l_min=self.get_parameter("l_min").value,
            l_max=self.get_parameter("l_max").value,
        )
        # Latched, so RViz shows the map even when it starts after the first scan.
        self._pub = self.create_publisher(
            OccupancyGrid,
            MAP_TOPIC,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.create_subscription(LaserScan, SCAN_TOPIC, self._on_scan, 10)

    def _on_scan(self, scan: LaserScan) -> None:
        self._builder.add_scan(
            scan.angle_min, scan.angle_increment, scan.ranges, scan.range_min, scan.range_max
        )
        self._pub.publish(self._to_message(scan))

    def _to_message(self, scan: LaserScan) -> OccupancyGrid:
        b = self._builder
        msg = OccupancyGrid()
        msg.header.stamp = scan.header.stamp
        msg.header.frame_id = scan.header.frame_id
        msg.info.resolution = b.resolution
        msg.info.width = b.width
        msg.info.height = b.height
        msg.info.origin = Pose()
        msg.info.origin.position.x = b.origin_x
        msg.info.origin.position.y = b.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = b.occupancy_values()
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanViewerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()  # SIGINT has usually shut the context down already

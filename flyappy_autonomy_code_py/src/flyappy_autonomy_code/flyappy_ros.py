from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool


class FlyappyRos:
    def __init__(self, node: Node):
        self._logger = node.get_logger()

        # Publisher for sending acceleration commands to Flyappy
        self._pub_acc_cmd = node.create_publisher(
            Vector3,
            "/flyappy_acc",
            1
        )

        # Subscribers to topics from Flyappy game
        self._sub_vel = node.create_subscription(
            Vector3,
            "/flyappy_vel",
            self.velocity_callback,
            10
        )
        self._sub_laser_scan = node.create_subscription(
            LaserScan,
            "/flyappy_laser_scan",
            self.laser_scan_callback,
            10
        )
        self._sub_game_ended = node.create_subscription(
            Bool,
            "/flyappy_game_ended",
            self.game_ended_callback,
            5
        )

    def velocity_callback(self, msg: Vector3) -> None:
        # Example of publishing acceleration command to Flyappy
        x = 0.5  # move and accelerate Flyappy forward
        y = 0
        self._pub_acc_cmd.publish(Vector3(x=float(x), y=float(y), z=float(0)))

    def laser_scan_callback(self, msg: LaserScan) -> None:
        # Example of printing laser angle and range
        self._logger.info(
            f"Laser range: {msg.ranges[0]}, angle: {msg.angle_min}", throttle_duration_sec=1

        )

    def game_ended_callback(self, msg: Bool) -> None:
        if msg.data:
            self._logger.info("Crash detected.")
        else:
            self._logger.info("End of countdown.")

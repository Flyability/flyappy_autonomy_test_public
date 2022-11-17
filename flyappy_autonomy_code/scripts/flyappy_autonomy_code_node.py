#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool


class FlyappyRos:
    def __init__(self):
        # Publisher for sending acceleration commands to Flyappy
        self._pub_acc_cmd = rospy.Publisher(
            "/flyappy_acc",
            Vector3,
            queue_size=1
        )

        # Subscribers to topics from Flyappy game
        self._sub_vel = rospy.Subscriber(
            "/flyappy_vel",
            Vector3,
            self.velocity_callback
        )
        self._sub_laser_scan = rospy.Subscriber(
            "/flyappy_laser_scan",
            LaserScan,
            self.laser_scan_callback
        )
        self._sub_game_ended = rospy.Subscriber(
            "/flyappy_game_ended",
            Bool,
            self.game_ended_callback
        )

    def velocity_callback(self, msg: Vector3) -> None:
        # Example of publishing acceleration command to Flyappy
        x = 0
        y = 0
        self._pub_acc_cmd.publish(Vector3(x, y, 0))

    def laser_scan_callback(self, msg: LaserScan) -> None:
        # Example of printing laser angle and range
        rospy.loginfo_throttle(
            1,
            f"Laser range: {msg.ranges[0]}, angle: {msg.angle_min}"
        )

    def game_ended_callback(self, msg: Bool) -> None:
        if msg.data:
            rospy.loginfo("Crash detected.")
        else:
            rospy.loginfo("End of countdown.")


def main() -> None:
    rospy.init_node('flyappy_autonomy_code', anonymous=True)
    flyappy_ros = FlyappyRos()  # noqa: F841
    rospy.spin()


if __name__ == '__main__':
    main()

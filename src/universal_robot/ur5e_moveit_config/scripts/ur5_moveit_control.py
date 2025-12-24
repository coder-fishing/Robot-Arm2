#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import random
import math

from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler


class UR5eTwoObstacles:
    def __init__(self):
        rospy.init_node("ur5e_two_obstacles", anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)

        self.arm = moveit_commander.MoveGroupCommander("manipulator")
        self.scene = moveit_commander.PlanningSceneInterface()

        rospy.sleep(2)

        # ===== CẤU HÌNH CHẠY CHẬM + ỔN ĐỊNH =====
        self.arm.set_planning_time(10.0)
        self.arm.set_num_planning_attempts(10)
        self.arm.set_max_velocity_scaling_factor(0.05)
        self.arm.set_max_acceleration_scaling_factor(0.05)

        self.reference_frame = self.arm.get_planning_frame()

        self.marker_pub = rospy.Publisher(
            "/visualization_marker", Marker, queue_size=10
        )

        rospy.sleep(1)

        # HOME / TARGET (an toàn hơn)
        self.home_position = [0.3, 0.0, 0.4]
        self.target_position = [0.45, 0.0, 0.35]

        rospy.loginfo("UR5e controller READY")

    # --------------------------------------------------
    def clear_all(self):
        for obj in self.scene.get_known_object_names():
            self.scene.remove_world_object(obj)

        marker = Marker()
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(marker)

        rospy.sleep(1)

    # --------------------------------------------------
    def draw_sphere(self, position, color, label, mid):
        m = Marker()
        m.header.frame_id = self.reference_frame
        m.header.stamp = rospy.Time.now()
        m.ns = "points"
        m.id = mid
        m.type = Marker.SPHERE
        m.action = Marker.ADD

        m.pose.position.x = position[0]
        m.pose.position.y = position[1]
        m.pose.position.z = position[2]
        m.pose.orientation.w = 1.0

        m.scale.x = 0.12
        m.scale.y = 0.12
        m.scale.z = 0.12

        m.color.r, m.color.g, m.color.b, m.color.a = color

        self.marker_pub.publish(m)

        # text
        t = Marker()
        t.header.frame_id = self.reference_frame
        t.header.stamp = rospy.Time.now()
        t.ns = "labels"
        t.id = mid + 100
        t.type = Marker.TEXT_VIEW_FACING
        t.action = Marker.ADD

        t.pose.position.x = position[0]
        t.pose.position.y = position[1]
        t.pose.position.z = position[2] + 0.15

        t.scale.z = 0.08
        t.color.r = t.color.g = t.color.b = 1.0
        t.color.a = 1.0
        t.text = label

        self.marker_pub.publish(t)

    # --------------------------------------------------
    def go_to_position(self, pos, label="TARGET"):
        rospy.loginfo(f"Moving to {label}...")

        pose = Pose()
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]

        # orientation an toàn hơn (hơi nghiêng xuống)
        q = quaternion_from_euler(-math.pi / 2, 0, 0)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        self.arm.set_pose_target(pose)

        success = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()

        if success:
            rospy.loginfo(f"✓ Reached {label}")
        else:
            rospy.logwarn(f"✗ Failed to reach {label}")

        return success

    # --------------------------------------------------
    def create_two_random_obstacles(self):
        rospy.loginfo("Creating 2 random obstacles...")

        for obj in self.scene.get_known_object_names():
            self.scene.remove_world_object(obj)

        for i in range(2):
            x = random.uniform(0.25, 0.45)
            y = random.uniform(-0.2, 0.2)
            z = random.uniform(0.2, 0.35)
            size = random.uniform(0.08, 0.12)

            pose = PoseStamped()
            pose.header.frame_id = self.reference_frame
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0

            name = f"obstacle_{i}"
            self.scene.add_box(name, pose, (size, size, size))

        rospy.sleep(1)

    # --------------------------------------------------
    def run_demo(self):
        self.clear_all()

        self.draw_sphere(self.home_position, (0, 0, 1, 1), "HOME", 1)
        self.draw_sphere(self.target_position, (1, 0, 0, 1), "TARGET", 2)

        self.create_two_random_obstacles()

        input("\nEnter → move HOME")
        self.go_to_position(self.home_position, "HOME")

        input("\nEnter → move TARGET")
        self.go_to_position(self.target_position, "TARGET")

        input("\nEnter → back HOME")
        self.go_to_position(self.home_position, "HOME")


def main():
    try:
        app = UR5eTwoObstacles()

        print("\nCommands:")
        print("1 - Run demo")
        print("q - Quit")

        while not rospy.is_shutdown():
            cmd = input(">> ").strip()
            if cmd == "1":
                app.run_demo()
            elif cmd in ["q", "quit"]:
                break

    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

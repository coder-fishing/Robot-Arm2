#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from moveit_commander import PlanningSceneInterface


class UR5MoveItController:
    def __init__(self):
        rospy.init_node("ur5_moveit_controller", anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)

        # Điều khiển cánh tay UR5
        self.arm = moveit_commander.MoveGroupCommander("manipulator")
        self.arm.set_planning_time(5)
        self.arm.set_num_planning_attempts(5)
        self.arm.set_max_velocity_scaling_factor(0.5)
        self.arm.set_max_acceleration_scaling_factor(0.5)

        # Planning scene để thêm vật cản
        self.scene = PlanningSceneInterface()
        rospy.sleep(2)

        rospy.loginfo("UR5 MoveIt Controller initialized")

    # =========================
    # BASIC MOTIONS
    # =========================
    def go_home(self):
        rospy.loginfo("Moving UR5 to HOME position")
        self.arm.set_named_target("home")
        self.arm.go(wait=True)
        self.arm.stop()

    def move_to_pose_xyz(self, x, y, z):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0

        rospy.loginfo(f"Moving to pose x={x}, y={y}, z={z}")
        self.arm.set_pose_target(pose)
        success = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()

        if not success:
            rospy.logwarn("Failed to reach target pose")

    # =========================
    # COLLISION OBJECT
    # =========================
    def add_collision_box(self):
        box_pose = Pose()
        box_pose.position.x = 0.4
        box_pose.position.y = 0.0
        box_pose.position.z = 0.2
        box_pose.orientation.w = 1.0

        self.scene.add_box(
            "obstacle_box",
            box_pose,
            size=(0.15, 0.15, 0.15)
        )

        rospy.loginfo("Collision box added")
        rospy.sleep(2)

    # =========================
    # DEMO SCENARIO
    # =========================
    def demo_motion(self):
        # 1. Về HOME
        self.go_home()
        rospy.sleep(1)

        # 2. Thêm vật cản
        self.add_collision_box()

        # 3. Di chuyển né vật cản
        self.move_to_pose_xyz(0.35, -0.25, 0.35)
        rospy.sleep(1)

        self.move_to_pose_xyz(0.35, 0.25, 0.35)
        rospy.sleep(1)

        # 4. Quay về HOME
        self.go_home()


if __name__ == "__main__":
    try:
        controller = UR5MoveItController()
        rospy.sleep(2)
        controller.demo_motion()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

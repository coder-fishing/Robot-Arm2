k#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from moveit_commander import PlanningSceneInterface


class UR5FullDemo:
    def __init__(self):
        rospy.init_node("ur5_full_demo", anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)

        # Move group
        self.arm = moveit_commander.MoveGroupCommander("manipulator")
        self.arm.set_planning_time(5)
        self.arm.set_num_planning_attempts(5)
        self.arm.set_max_velocity_scaling_factor(0.5)
        self.arm.set_max_acceleration_scaling_factor(0.5)

        # Planning scene
        self.scene = PlanningSceneInterface()
        rospy.sleep(2)

        rospy.loginfo("UR5 Full Demo READY")
        rospy.loginfo("Type 'start' or '1' to run demo, 'q' to quit")

    # -------------------------
    # COLLISION OBJECT
    # -------------------------
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
        rospy.sleep(1)

    def clear_collision(self):
        self.scene.remove_world_object("obstacle_box")
        rospy.sleep(1)

    # -------------------------
    # BASIC MOTIONS
    # -------------------------
    def go_home(self):
        rospy.loginfo("Moving to HOME")
        self.arm.set_named_target("home")
        self.arm.go(wait=True)
        self.arm.stop()

    def move_to_pose(self, x, y, z):
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
            rospy.logwarn("Motion failed")

    # -------------------------
    # DEMO
    # -------------------------
    def run_demo(self):
        rospy.loginfo("=== DEMO START ===")

        self.clear_collision()
        self.go_home()
        rospy.sleep(1)

        self.add_collision_box()

        self.move_to_pose(0.35, -0.25, 0.35)
        rospy.sleep(1)

        self.move_to_pose(0.35, 0.25, 0.35)
        rospy.sleep(1)

        self.go_home()

        rospy.loginfo("=== DEMO FINISHED ===")


if __name__ == "__main__":
    try:
        demo = UR5FullDemo()

        while not rospy.is_shutdown():
            cmd = input("\n>>> Enter command (start / 1 / q): ").strip().lower()

            if cmd in ["start", "1"]:
                demo.run_demo()
            elif cmd in ["q", "quit", "exit"]:
                rospy.loginfo("Exiting demo node")
                break
            else:
                print("Unknown command")

    except rospy.ROSInterruptException:
        pass


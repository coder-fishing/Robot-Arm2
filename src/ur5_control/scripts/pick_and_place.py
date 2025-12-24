#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def main():
    rospy.init_node("pick_and_place")

    pub = rospy.Publisher(
        "/eff_joint_traj_controller/command",
        JointTrajectory,
        queue_size=10
    )

    rospy.sleep(2)

    traj = JointTrajectory()
    traj.joint_names = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    ]

    point = JointTrajectoryPoint()
    point.positions = [0.0, -1.2, 1.2, 0.0, 0.0, 0.0]
    point.time_from_start = rospy.Duration(3)

    traj.points.append(point)

    rospy.loginfo("Sending trajectory to UR5...")
    pub.publish(traj)

    rospy.spin()

if __name__ == "__main__":
    main()

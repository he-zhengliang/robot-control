#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from geometry_msgs.msg import PointStamped, PoseStamped
import moveit_commander
from tf.transformations import quaternion_from_euler, quaternion_matrix
from threading import Lock

# Shared point with lock
latest_point = None 
point_lock = Lock()

def callback(point_msg):
    global latest_point
    with point_lock:
        latest_point = point_msg
    rospy.loginfo(f"[pose_receiver] Received new target: ({point_msg.point.x:.3f}, {point_msg.point.y:.3f}, {point_msg.point.z:.3f})")

def create_pose(point_msg, offset_front=0.0, offset_lr=0.0, offset_tool=-0.1):
    # Fixed orientation: gripper pointing down, X forward
    q = quaternion_from_euler(0, np.pi/2, 0)

    # Tool offset in local tool frame
    tool_offset = np.array([0.0, offset_lr, offset_tool, 1])
    T = quaternion_matrix(q)
    offset = T @ tool_offset

    pose_target = PoseStamped()
    pose_target.header.frame_id = "base"
    pose_target.header.stamp = rospy.Time.now()
    pose_target.pose.orientation.x = q[0]
    pose_target.pose.orientation.y = q[1]
    pose_target.pose.orientation.z = q[2]
    pose_target.pose.orientation.w = q[3]
    pose_target.pose.position.x = point_msg.point.x + offset[0] + offset_front
    pose_target.pose.position.y = point_msg.point.y + offset[1]
    pose_target.pose.position.z = point_msg.point.z + offset[2]
    return pose_target

def move_to_pose(group, pose):
    group.set_pose_target(pose)
    success = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    return success

def return_home(group):
    home_joints = [0, 0, 0, 0, 0, 0]
    group.set_joint_value_target(home_joints)
    success = group.go(wait=True)
    group.stop()
    return success

def main():
    global latest_point

    rospy.init_node('pose_receiver')
    moveit_commander.roscpp_initialize([])

    group = moveit_commander.MoveGroupCommander("manipulator")
    rospy.Subscriber("/detected_point_base", PointStamped, callback)

    rospy.loginfo("[pose_receiver] Robot ready with vertical tool orientation...")

    rate = rospy.Rate(2)
    last_stamp = None

    while not rospy.is_shutdown():
        current_point = None

        with point_lock:
            if latest_point and latest_point.header.stamp != last_stamp:
                current_point = latest_point
                last_stamp = latest_point.header.stamp

        if current_point:
            rospy.loginfo("[pose_receiver] Moving to approach point first...")

            # Step 1: Move to approach point
            approach_pose = create_pose(current_point, offset_front=-0.20, offset_lr=-0.08)
            if move_to_pose(group, approach_pose):
                rospy.loginfo("[pose_receiver] Reached approach point. Waiting 1 second...")
                rospy.sleep(0.1)

                # Step 2: Move to actual target
                rospy.loginfo("[pose_receiver] Moving to actual target...")
                target_pose = create_pose(current_point, offset_front=0.025, offset_lr=0.015)
                if move_to_pose(group, target_pose):
                    rospy.loginfo("[pose_receiver] Reached target. Waiting 6 seconds...")
                    rospy.sleep(6)

                    # Step 3: Move back to approach point
                    rospy.loginfo("[pose_receiver] Moving back to approach point before next...")
                    if move_to_pose(group, approach_pose):
                        rospy.loginfo("[pose_receiver] Reached approach again. Waiting 0.5 second...")
                        rospy.sleep(0.5)

                        # Step 4: Move to specific point 1
                        rospy.loginfo("[pose_receiver] Moving to specific point 1...")
                        specific_pose_1 = PoseStamped()
                        specific_pose_1.header.frame_id = "base"
                        specific_pose_1.header.stamp = rospy.Time.now()
                        specific_pose_1.pose.position.x = 0.65
                        specific_pose_1.pose.position.y = 0.32 + 0.09
                        specific_pose_1.pose.position.z = 0.25 - 0.03
                        q = quaternion_from_euler(0, np.pi/2, 0)
                        specific_pose_1.pose.orientation.x = q[0]
                        specific_pose_1.pose.orientation.y = q[1]
                        specific_pose_1.pose.orientation.z = q[2]
                        specific_pose_1.pose.orientation.w = q[3]

                        if move_to_pose(group, specific_pose_1):
                            rospy.loginfo("[pose_receiver] Reached specific point 1. Waiting 3 seconds...")
                            rospy.sleep(1)

                            # Step 5: Move to specific point 
                            rospy.loginfo("[pose_receiver] Moving to specific point 2...")
                            specific_pose_2 = PoseStamped()
                            specific_pose_2.header.frame_id = "base"
                            specific_pose_2.header.stamp = rospy.Time.now()
                            specific_pose_2.pose.position.x = 0.5
                            specific_pose_2.pose.position.y = 0.09
                            specific_pose_2.pose.position.z = 0.30
                            specific_pose_2.pose.orientation = specific_pose_1.pose.orientation

                            if move_to_pose(group, specific_pose_2):
                                rospy.loginfo("[pose_receiver] Reached specific point 2. Waiting 2 seconds...")
                                rospy.sleep(2)

                                # Step 6: Return to home
                                rospy.loginfo("[pose_receiver] Returning to home...")
                                if return_home(group):
                                    rospy.loginfo("[pose_receiver] Returned to home. Waiting 5 seconds...")
                                    rospy.sleep(2)
                                else:
                                    rospy.logwarn("[pose_receiver] Failed to return to home.")
                            else:
                                rospy.logwarn("[pose_receiver] Failed to reach specific point 2 before home.")
                        else:
                            rospy.logwarn("[pose_receiver] Failed to reach specific point 1.")
                    else:
                        rospy.logwarn("[pose_receiver] Failed to return to approach point before specific move.")
                else:
                    rospy.logwarn("[pose_receiver] Failed to reach target.")
            else:
                rospy.logwarn("[pose_receiver] Failed to reach approach point.")

        rate.sleep()

if __name__ == "__main__":
    main()

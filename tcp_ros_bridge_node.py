#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import socket
import rospy
import numpy as np
from geometry_msgs.msg import PointStamped
from tf.transformations import quaternion_matrix

# === Calibration transform: camera_link ? robot base ===
CALIB_Q = [
    -0.018925117294074128,  # qx
    -0.1865101908584339,    # qy
    0.11973996395524347,   # qz
     0.9749451931638787     # qw
]
CALIB_T = [
    0.06602739401849941,    # x
    -0.43988361911425317,    # y
    0.060559593526417266    # z
]

def transform_point(x_opt, y_opt, z_opt):
    # Step 1: Convert from RealSense optical frame to engineering (ABB base-style)
    # RealSense optical: X right, Y down, Z forward
    # Engineering frame (ABB): X forward, Y left, Z up
    v_eng = np.array([z_opt, -x_opt, -y_opt])

    # Step 2: Apply rotation + translation from camera_link ? base
    T = quaternion_matrix(CALIB_Q)
    R_cb = T[0:3, 0:3]
    t_cb = np.array(CALIB_T)

    # Transform point
    return R_cb.dot(v_eng) + t_cb

def main():
    rospy.init_node('transform_point_node')
    pub = rospy.Publisher('/detected_point_base', PointStamped, queue_size=10)

    HOST, PORT = '0.0.0.0', 65432
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)
    rospy.loginfo(f"[transform_point_node] Listening on {HOST}:{PORT}")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            conn, addr = server.accept()
            data = conn.recv(1024)
            conn.close()
            if not data:
                continue

            # Parse input as comma-separated x,y,z in optical frame
            x_s, y_s, z_s = data.decode().strip().split(',')
            x_opt, y_opt, z_opt = map(float, (x_s, y_s, z_s))

            # Transform point
            xb, yb, zb = transform_point(x_opt, y_opt, z_opt)

            # Create and publish ROS message
            msg = PointStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'base'
            msg.point.x, msg.point.y, msg.point.z = xb, yb, zb
            pub.publish(msg)

            rospy.loginfo(f"[transform_point_node] Optical: ({x_opt:.3f}, {y_opt:.3f}, {z_opt:.3f}) ? Base: ({xb:.3f}, {yb:.3f}, {zb:.3f})")

        except Exception as e:
            rospy.logerr(f"[transform_point_node] Error: {e}")
        rate.sleep()

    server.close()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import matplotlib.animation as animation
import tf.transformations

# Globals to store data
x_data, y_data = [], []
cov_data = []
roll = pitch = yaw = 0.0

def quaternion_to_euler(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return tf.transformations.euler_from_quaternion([x, y, z, w])

def rotation_matrix_from_euler(roll, pitch, yaw):
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]
    ])

    Ry = np.array([
        [ np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0, 0, 1]
    ])

    return Rz @ Ry @ Rx

def draw_rover_box(ax, roll, pitch, yaw):
    ax.cla()

    # Box size
    l, w, h = 1.0, 0.5, 0.3

    # Vertices of box centered at origin
    vertices = np.array([
        [-l/2, -w/2, -h/2],
        [ l/2, -w/2, -h/2],
        [ l/2,  w/2, -h/2],
        [-l/2,  w/2, -h/2],
        [-l/2, -w/2,  h/2],
        [ l/2, -w/2,  h/2],
        [ l/2,  w/2,  h/2],
        [-l/2,  w/2,  h/2]
    ])

    R = rotation_matrix_from_euler(roll, pitch, yaw)
    rotated_vertices = vertices @ R.T

    faces = [
        [rotated_vertices[j] for j in [0,1,2,3]],  # bottom
        [rotated_vertices[j] for j in [4,5,6,7]],  # top
        [rotated_vertices[j] for j in [0,1,5,4]],  # front
        [rotated_vertices[j] for j in [2,3,7,6]],  # back
        [rotated_vertices[j] for j in [1,2,6,5]],  # right
        [rotated_vertices[j] for j in [4,7,3,0]]   # left
    ]

    box = Poly3DCollection(faces, facecolors='cyan', linewidths=1, edgecolors='r', alpha=0.7)
    ax.add_collection3d(box)

    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-0.5, 0.5])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title("Rover Orientation")
    ax.view_init(elev=20, azim=30)
    # Comment out if matplotlib version <3.3:
    # ax.set_box_aspect([2,2,1])

def plot_covariance_ellipse(ax, x, y, cov, n_std=2.0, **kwargs):
    # Extract 2x2 covariance matrix for position x,y
    cov_2d = np.array([[cov[0], cov[1]],
                       [cov[6], cov[7]]])

    # Eigenvalues and eigenvectors
    vals, vecs = np.linalg.eigh(cov_2d)

    # Sort by largest eigenvalue first
    order = vals.argsort()[::-1]
    vals = vals[order]
    vecs = vecs[:, order]

    # Calculate ellipse parameters
    width, height = 2 * n_std * np.sqrt(vals)
    angle = np.degrees(np.arctan2(vecs[1, 0], vecs[0, 0]))

    ellipse = Ellipse((x, y), width, height, angle, **kwargs)
    ax.add_patch(ellipse)

def odom_callback(msg):
    global x_data, y_data, roll, pitch, yaw, cov_data
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    if msg.pose.covariance[35]==1:
        x_data.append(x)
        y_data.append(y)
        roll, pitch, yaw = quaternion_to_euler(msg.pose.pose.orientation)
        cov_data.append(msg.pose.covariance)
        print(f"\n[+] Rover's pose udpate!\n|___ x:   {x:.4f}\n|___ y:   {y:.4f}\n|___ yaw: {yaw:.4f}")

def animate(i):
    ax1.cla()
    ax2.cla()

    # Plot 2D path points
    ax1.plot(x_data, y_data, 'b.-')
    ax1.set_xlim(-5, 5)
    ax1.set_ylim(-5, 5)
    ax1.set_title("2D Rover Path")
    ax1.set_xlabel("X")
    ax1.set_ylabel("Y")
    ax1.grid()
    ax1.legend()

    # Plot covariance ellipses
    for x, y, cov in zip(x_data, y_data, cov_data):
        plot_covariance_ellipse(ax1, x, y, cov, n_std=2, edgecolor='red', alpha=0.3, facecolor='none')

    # Plot 3D orientation box
    draw_rover_box(ax2, roll, pitch, yaw)

if __name__ == "__main__":
    rospy.init_node("live_plot_3d_orientation")

    rospy.Subscriber("/filtered_state", Odometry, odom_callback)

    fig = plt.figure(figsize=(5, 8))
    fig.canvas.manager.set_window_title("ROAR Localization")
    ax1 = fig.add_subplot(211)
    ax2 = fig.add_subplot(212, projection='3d')

    ani = animation.FuncAnimation(fig, animate, interval=100)
    plt.show()

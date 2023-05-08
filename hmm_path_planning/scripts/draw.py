"""
draw_rrt.py
Author: Chang-Hong Chen
Email: longhongc@gmail.com
"""

import matplotlib.pyplot as plt
import os
import sys

# Create plot
fig, ax = plt.subplots(dpi=300)
ax.set_aspect('equal')

# Set limits and axis label
ax.set_xlim((-50, 50))
ax.set_ylim((-50, 50))
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')

start = (0, 0)
# goal = (30, 30)
goal = (16, -2)
goal_tolerance = 2

# Draw start and goal
start_circle = plt.Circle(start, 1, color='lime', fill=False)
goal_circle = plt.Circle(goal, goal_tolerance, color='gold', fill=False)
ax.add_patch(start_circle)
ax.add_patch(goal_circle)

# Draw obstacles
obstacles = []
obstacles_file_path = '../data/obstacles.txt'
line_count = 0
for line in open(obstacles_file_path, 'r'):
    line_count += 1
    if (line_count == 1):
        continue
    obstacles.append([float(s) for s in line.split(',')])

for obstacle in obstacles:
    x, y, r = obstacle
    circle = plt.Circle((x, y), r, color='r')
    ax.add_patch(circle)



def draw_robot(base_pose, joint_poses):
    base_circle = plt.Circle(base_pose, 3, color='blue', fill=False)
    ax.add_patch(base_circle)
    x0, y0 = base_pose
    j1, j2 = joint_poses
    x1, y1 = j1
    x2, y2 = j2
    plt.scatter([x1], [y1], marker='o', color='black', s=0.3)
    plt.scatter([x2], [y2], marker='o', color='darkorange', s=0.4)
    plt.plot([x0, x1, x2], [y0, y1, y2], color='black', linewidth=0.2)

for line in open('path3.txt', 'r'):
    coords = [float(s) for s in line.split(',')]
    base_x, base_y, _, \
    j1_x, j1_y, _, \
    j2_x, j2_y, _ = coords
    base_pose = (base_x, base_y)
    joint_poses = [(j1_x, j1_y), (j2_x, j2_y)]
    draw_robot(base_pose, joint_poses)

fig.savefig("test4.png")

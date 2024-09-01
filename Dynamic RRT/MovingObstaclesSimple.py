import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.animation as animation
import plotly.graph_objects as go
from TrajectorySmoothing import gradient_descent_smoothing
from RRT3DstarObstacles import pathPlanning
from time import sleep

############################## Plotting #####################################################

# Function to draw obstacles
def draw_obstacles(ax, obstacles):
    for o in obstacles:
        x = o[0] - o[3] / 2
        y = o[1] - o[4] / 2
        z = o[2] - o[5] / 2
        dx = o[3]
        dy = o[4]
        dz = o[5]
        xx = [x, x + dx, x + dx, x, x, x + dx, x + dx, x]
        yy = [y, y, y + dy, y + dy, y, y, y + dy, y + dy]
        zz = [z, z, z, z, z + dz, z + dz, z + dz, z + dz]
        vertices = [[xx[0], yy[0], zz[0]],
                    [xx[1], yy[1], zz[1]],
                    [xx[2], yy[2], zz[2]],
                    [xx[3], yy[3], zz[3]],
                    [xx[4], yy[4], zz[4]],
                    [xx[5], yy[5], zz[5]],
                    [xx[6], yy[6], zz[6]],
                    [xx[7], yy[7], zz[7]]]
        faces = [[vertices[j] for j in [0, 1, 2, 3]],  # bottom face
                 [vertices[j] for j in [4, 5, 6, 7]],  # top face
                 [vertices[j] for j in [0, 3, 7, 4]],  # left face
                 [vertices[j] for j in [1, 2, 6, 5]],  # right face
                 [vertices[j] for j in [0, 1, 5, 4]],  # front face
                 [vertices[j] for j in [2, 3, 7, 6]]]  # back face
        ax.add_collection3d(Poly3DCollection(faces, facecolors='k', linewidths=1, edgecolors='black', alpha=.25))

############################## Initialisation ###################################################

start = np.array([-400, -400, 180])  # Start location in 3D
goal = np.array([400, 400, 0])  # Goal location in 3D
y = -350
obstacles = [
    np.array([0, y, 70, 100, 100, 140]),
    ]
  # cuboids parametrized by [x, y, z, dx, dy, dz]
obstacle_y_velocity = 300
dt = 1
time = np.arange(0, 3, 1)
previous_paths = []
previous_starts = [start.copy()]
node_array, parent_array = None, None

############################## Static Plot ###################################################
def init_static_plot(og, previous_paths, previous_starts, start, goal, t):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    # Add a title
    ax.set_title(f'3D RRT* Dynamic Path Planning (t={t})')
    # Label the axes
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    # Set the boundary
    bounds_x = np.array([-500, 500])  # Bounds for x and y axes
    bounds_y = np.array([-500, 500])  # Bounds for x and y axes
    bounds_z = np.array([0, 200])  # Bounds for z axis
    ax.set_xlim(bounds_x[0], bounds_x[1])
    ax.set_ylim(bounds_y[1], bounds_y[0])
    ax.set_zlim(bounds_z[0], bounds_z[1])

    # plot
    draw_obstacles(ax, obstacles)

    # Plot previous paths
    for path in previous_paths:
        ax.plot(path[:, 0], path[:, 1], path[:, 2], 'r', linewidth=2, label='Drone Travelled Path' if path is previous_paths[0] else "")

    # Plot previous start points
    for idx, s in enumerate(previous_starts):
        ax.scatter(s[0], s[1], s[2], c='g', marker='^', s=40)
        # ax.text(s[0], s[1], s[2], f'start {idx}', color='black')

    # Plot New Generated RRT* Path
    ax.plot(og[:, 0], og[:, 1], og[:, 2], linewidth=2, label=f"New Generated Path")
    ax.scatter(start[0], start[1], start[2], c='g', marker='^', s=40, label="Start")
    ax.text(start[0], start[1], start[2], f'start {t}', color='black')

    # Plot Goal
    ax.scatter(goal[0], goal[1], goal[2], c='r', marker='x', s=40, label="Goal")
    ax.text(goal[0], goal[1], goal[2], f'Goal', color='black')

    ax.legend(loc='upper left')
    plt.tight_layout()
    plt.show()

############################### Simulation ###################################################
for t in time:
    # generate path
    og, _= pathPlanning(start, goal, obstacles)
    og_smooth = gradient_descent_smoothing(og)
    rows = og.shape[0]

    # plot
    init_static_plot(og_smooth, previous_paths, previous_starts, start, goal, t)

    # update start and obstacle positions 
    start = np.array([og[int(rows)-7, 0], og[int(rows)-7, 1], og[int(rows)-7, 2]])
    y = y + obstacle_y_velocity*dt
    obstacles = [np.array([0, y, 70, 100, 100, 140])]
    current_path = og_smooth[int(rows)-7:, :]
    previous_paths.append(current_path)
    previous_starts.append(start.copy())




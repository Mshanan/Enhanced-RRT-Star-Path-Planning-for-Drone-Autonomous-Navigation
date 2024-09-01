# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d.art3d import Poly3DCollection
# import matplotlib.animation as animation
# import plotly.graph_objects as go
# from TrajectorySmoothing import gradient_descent_smoothing
# from RRT3DstarReuse2 import pathPlanning
# from time import sleep

# ############################## Plotting #####################################################

# # Function to draw obstacles
# def draw_obstacles(ax, obstacles,a):
#     for o in obstacles:
#         o = o/100
#         x = o[0] - o[3] / 2
#         y = o[1] - o[4] / 2
#         z = o[2] - o[5] / 2
#         dx = o[3]
#         dy = o[4]
#         dz = o[5]
#         xx = [x, x + dx, x + dx, x, x, x + dx, x + dx, x]
#         yy = [y, y, y + dy, y + dy, y, y, y + dy, y + dy]
#         zz = [z, z, z, z, z + dz, z + dz, z + dz, z + dz]
#         vertices = [[xx[0], yy[0], zz[0]],
#                     [xx[1], yy[1], zz[1]],
#                     [xx[2], yy[2], zz[2]],
#                     [xx[3], yy[3], zz[3]],
#                     [xx[4], yy[4], zz[4]],
#                     [xx[5], yy[5], zz[5]],
#                     [xx[6], yy[6], zz[6]],
#                     [xx[7], yy[7], zz[7]]]
#         faces = [[vertices[j] for j in [0, 1, 2, 3]],  # bottom face
#                  [vertices[j] for j in [4, 5, 6, 7]],  # top face
#                  [vertices[j] for j in [0, 3, 7, 4]],  # left face
#                  [vertices[j] for j in [1, 2, 6, 5]],  # right face
#                  [vertices[j] for j in [0, 1, 5, 4]],  # front face
#                  [vertices[j] for j in [2, 3, 7, 6]]]  # back face
#         ax.add_collection3d(Poly3DCollection(faces, facecolors='k', linewidths=1, edgecolors='black', alpha=a))

# ############################## Static Plot ###################################################
# def plot_test(start, goal, og, t, all_prev_obs, merged= None, previous_merged=None):

#     start = start / 100
#     goal = goal / 100
#     og = og/100
    
#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')
#     # Add a title
#     # ax.set_title(f'3D RRT* Dynamic Path Planning at t = {t}')
#     # Label the axes
#     ax.set_xlabel('X position (m)')
#     ax.set_ylabel('Y position (m)')
#     ax.set_zlabel('Z position (m)')
#     # Set the boundary
#     bounds_x = np.array([-5, 5])  # Bounds for x and y axes
#     bounds_y = np.array([-5, 5])  # Bounds for x and y axes
#     bounds_z = np.array([0, 2])  # Bounds for z axis
#     ax.set_xlim(bounds_x[0], bounds_x[1])
#     ax.set_ylim(bounds_y[1], bounds_y[0])
#     ax.set_zlim(bounds_z[0], bounds_z[1])

#     # plot start, goal and obstacles
#     ax.scatter(start[0], start[1], start[2], c='g', marker='^', s=40, label="Start")
#     ax.text(start[0], start[1], start[2], f'start', color='black')
#     ax.scatter(goal[0], goal[1], goal[2], c='r', marker='x', s=40, label="Goal")
#     ax.text(goal[0], goal[1], goal[2], f'Goal', color='black')
#     a = 0.02
#     for prev_obs in all_prev_obs:
#         draw_obstacles(ax, prev_obs, a)
#         a = a + 0.1

#     # Plot RRT Paths
#     if previous_merged is not None:
#         previous_merged = previous_merged/100
#         ax.plot(previous_merged[:, 0], previous_merged[:, 1], previous_merged[:, 2], "--r", linewidth=1 , label=f"Previous Path (t = {t-1})")

#     if merged is not None:
#         merged = merged/100
#         ax.plot(merged[:, 0], merged[:, 1], merged[:, 2], "r", label=f"New Merged (t = {t})", linewidth=2)

#     if t == 0:
#         ax.plot(og[:, 0], og[:, 1], og[:, 2], "g", linewidth=2, label=f"Initial Path")
#     else:
#         ax.plot(og[:, 0], og[:, 1], og[:, 2], "g", linewidth=2, label=f"Replanned Section")

#     ax.legend(loc='upper right')
#     plt.tight_layout()
#     plt.show()

# ############################## Initialisation ###################################################

# start = np.array([-400, -400, 180])  # Start location in 3D
# goal = np.array([400, 400, 0])  # Goal location in 3D
# y = -350
# obstacles = [
#     np.array([-200, -200, 100, 100, 100, 200]),
#     ]
#   # cuboids parametrized by [x, y, z, dx, dy, dz]
# obstacle_y_velocity = 300
# time = np.arange(0, 3, 1)
# dt = 1
# previous_starts = [start.copy()]
# og = None
# full_merged = None
# previous_merged = None
# all_prev_obs = [obstacles]

# ############################### Simulation ###################################################
# for t in time:
#     # generate path
#     if t==2:
#         og, _, full_merged = pathPlanning(start, goal, obstacles, full_merged)
#     else:
#         og, _, full_merged = pathPlanning(start, goal, obstacles, og)
    
#     og_smooth = gradient_descent_smoothing(og)
    
#     if full_merged is not None:
#         full_merged_smooth = gradient_descent_smoothing(full_merged)
#         plot_test(start, goal, og_smooth, t, all_prev_obs, full_merged_smooth, previous_merged)
#     else:
#         plot_test(start, goal, og_smooth, t, all_prev_obs, full_merged, previous_merged)

#     if t in [1,2]:
#         previous_merged = full_merged
#     else:
#         previous_merged = og
        
#     # prev_obs = obstacles
#     # update start and obstacle positions 
#     if t == 0:
#         y = y + obstacle_y_velocity*dt
#         obstacles = [np.array([0, y, 70, 100, 100, 140])]
#     if t == 1:
#         y = y + 200*dt
#         obstacles = [np.array([200, y, 70, 100, 100, 140])]

#     all_prev_obs.append(obstacles)

  

import numpy as np
import plotly.graph_objects as go
from TrajectorySmoothing import gradient_descent_smoothing
from RRT3DstarReuse2 import pathPlanning

############################## Plotting #####################################################

# Function to draw obstacles
def draw_obstacles(fig, obstacles, alpha):
    for o in obstacles:
        o = o / 100
        x = o[0] - o[3] / 2
        y = o[1] - o[4] / 2
        z = o[2] - o[5] / 2
        dx = o[3]
        dy = o[4]
        dz = o[5]
        fig.add_trace(go.Mesh3d(
            x=[x, x + dx, x + dx, x, x, x + dx, x + dx, x],
            y=[y, y, y + dy, y + dy, y, y, y + dy, y + dy],
            z=[z, z, z, z, z + dz, z + dz, z + dz, z + dz],
            color='olive',
            opacity=alpha,
            alphahull=0
        ))

############################## Static Plot ###################################################
def plot_test(start, goal, og, t, all_prev_obs, merged=None, previous_merged=None):

    start = start / 100
    goal = goal / 100
    og = og / 100
    
    fig = go.Figure()

    # Set the boundary
    bounds_x = np.array([-5, 5])
    bounds_y = np.array([-5, 5])
    bounds_z = np.array([0, 2])

    # Plot start, goal, and obstacles
    fig.add_trace(go.Scatter3d(x=[start[0]], y=[start[1]], z=[start[2]], mode='markers', marker=dict(size=5, color='green'), showlegend=False))
    fig.add_trace(go.Scatter3d(x=[goal[0]], y=[goal[1]], z=[goal[2]], mode='markers', marker=dict(size=5, color='red'), showlegend=False))

    # Plot obstacles
    alpha = 0.1  # Higher opacity for better visibility
    for prev_obs in all_prev_obs:
        draw_obstacles(fig, prev_obs, alpha)
        alpha += 0.1

    # Plot RRT Paths
    if previous_merged is not None:
        previous_merged = previous_merged / 100
        fig.add_trace(go.Scatter3d(x=previous_merged[:, 0], y=previous_merged[:, 1], z=previous_merged[:, 2], mode='lines', line=dict(color='red', dash='dash', width=4), name=f"Previous Path (t = {t-1})"))

    if merged is not None:
        merged = merged / 100
        fig.add_trace(go.Scatter3d(x=merged[:, 0], y=merged[:, 1], z=merged[:, 2], mode='lines', line=dict(color='red', width=6), name=f"New Merged Path (t = {t})"))

    if t == 0:
        fig.add_trace(go.Scatter3d(x=og[:, 0], y=og[:, 1], z=og[:, 2], mode='lines', line=dict(color='green', width=6), name=f"Initial Path (t=0)"))
    else:
        fig.add_trace(go.Scatter3d(x=og[:, 0], y=og[:, 1], z=og[:, 2], mode='lines', line=dict(color='green', width=6), name=f"Replanned Section"))

    # Update layout
    fig.update_layout(
        scene=dict(
            xaxis=dict(range=[bounds_x[0], bounds_x[1]], title='X Position (m)'),
            yaxis=dict(range=[bounds_y[0], bounds_y[1]], title='Y Position (m)'),
            zaxis=dict(range=[bounds_z[0], bounds_z[1]], title='Z Position (m)')
        ),
        legend=dict(
            x=0.55,  # Adjust the x position (closer to 1 moves it to the right)
            y=0.9,  # Adjust the y position (closer to 1 moves it up)
            xanchor="left",
            yanchor="top",
            font=dict(
                size=14  # Change this to your desired font size
            )
        ),
        margin=dict(l=0, r=0, b=0, t=0)
    )

    fig.show()

############################## Initialisation ###################################################

start = np.array([-400, -400, 180])  # Start location in 3D
goal = np.array([400, 400, 0])  # Goal location in 3D
y = -350
obstacles = [
    np.array([-200, -200, 100, 100, 100, 200]),
]
# cuboids parametrized by [x, y, z, dx, dy, dz]
obstacle_y_velocity = 300
time = np.arange(0, 3, 1)
dt = 1
previous_starts = [start.copy()]
og = None
full_merged = None
previous_merged = None
all_prev_obs = [obstacles]

############################### Simulation ###################################################
for t in time:
    # generate path
    if t == 2:
        og, _, full_merged = pathPlanning(start, goal, obstacles, full_merged)
    else:
        og, _, full_merged = pathPlanning(start, goal, obstacles, og)
    
    og_smooth = gradient_descent_smoothing(og)
    
    if full_merged is not None:
        full_merged_smooth = gradient_descent_smoothing(full_merged)
        plot_test(start, goal, og_smooth, t, all_prev_obs, full_merged_smooth, previous_merged)
    else:
        plot_test(start, goal, og_smooth, t, all_prev_obs, full_merged, previous_merged)

    if t in [1, 2]:
        previous_merged = full_merged
    else:
        previous_merged = og

    # update start and obstacle positions 
    if t == 0:
        y = y + obstacle_y_velocity * dt
        obstacles = [np.array([0, y, 70, 100, 100, 140])]
    if t == 1:
        y = y + 200 * dt
        obstacles = [np.array([200, y, 70, 100, 100, 140])]

    all_prev_obs.append(obstacles)



# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d.art3d import Poly3DCollection
# import matplotlib.animation as animation
# import plotly.graph_objects as go
# from TrajectorySmoothing import gradient_descent_smoothing
# from RRT3DstarTarget import pathPlanning

# ############################## Initialisation ###################################################

# start = np.array([0, 0, 100])  # Start location in 3D
# initial_start = start.copy()
# goal = np.array([100, 0, 0])  # Goal location in 3D
# obstacles = [
#     np.array([450, 87.5, 90, 1100, 25, 180]),
#     np.array([450, -87.5, 90, 1100, 25, 180]),
# ]  # cuboids parametrized by [x, y, z, dx, dy, dz]
# time = np.arange(0, 9, 1)
# drone_path = []

# ############################## Plotting #########################################################
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# # Function to draw obstacles
# def draw_obstacles(ax, obstacles):
#     for o in obstacles:
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
#         ax.add_collection3d(Poly3DCollection(faces, facecolors='k', linewidths=1, edgecolors='black', alpha=.25))

# draw_obstacles(ax, obstacles)

# # Set up plot limits and labels
# ax.set_title('3D RRT* Dynamic Path Planning')
# ax.set_xlabel('X axis')
# ax.set_ylabel('Y axis')
# ax.set_zlabel('Z axis')
# bounds_x = np.array([-100, 1000])  # Bounds for x 
# bounds_y = np.array([-100, 100])  # Bounds for y
# bounds_z = np.array([0, 180])  # Bounds for z 
# ax.set_xlim(bounds_x[0], bounds_x[1])
# ax.set_ylim(bounds_y[0], bounds_y[1])
# ax.set_zlim(bounds_z[0], bounds_z[1])

# initial_start_scatter = ax.scatter(initial_start[0], initial_start[1], initial_start[2], c='g', marker='^', label='Initial Start', s=40)
# start_scatter = ax.scatter(start[0], start[1], start[2], c='b', marker='x', label='Start', s=40)
# goal_scatter = ax.scatter(goal[0], goal[1], goal[2], c='r', marker='*', label='Goal', s=40)
# path_line, = ax.plot([], [], [], linewidth=2, c='g')
# drone_path_line, = ax.plot([], [], [], linewidth=1, c='b', label='Drone Path')

# def init():
#     path_line.set_data([], [])
#     path_line.set_3d_properties([], zdir='z')
#     start_scatter.set_offsets([start[0], start[1]])
#     start_scatter.set_3d_properties([start[2]], zdir='z')
#     goal_scatter.set_offsets([goal[0], goal[1]])
#     goal_scatter.set_3d_properties([goal[2]], zdir='z')
#     drone_path_line.set_data([], [])
#     drone_path_line.set_3d_properties([], zdir='z')
#     return path_line, start_scatter, goal_scatter, drone_path_line

# def update(t):
#     global start, goal, drone_path
#     og, path = pathPlanning(start, goal, obstacles)
#     og = gradient_descent_smoothing(og)
    
#     # Update the drone path
#     drone_path.append(start)
#     drone_path_np = np.array(drone_path)
    
#     # Plot the complete drone path
#     drone_path_line.set_data(drone_path_np[:, 0], drone_path_np[:, 1])
#     drone_path_line.set_3d_properties(drone_path_np[:, 2], zdir='z')
    
#     # Plot the new path from the current start to the current goal
#     path_line.set_data(og[:, 0], og[:, 1])
#     path_line.set_3d_properties(og[:, 2], zdir='z')
    
#     # Clear and redraw the plot
#     ax.cla()
#     draw_obstacles(ax, obstacles)
#     ax.set_xlim(bounds_x[0], bounds_x[1])
#     ax.set_ylim(bounds_y[0], bounds_y[1])
#     ax.set_zlim(bounds_z[0], bounds_z[1])
#     ax.set_title('3D RRT* Dynamic Path Planning')
#     ax.set_xlabel('X axis')
#     ax.set_ylabel('Y axis')
#     ax.set_zlabel('Z axis')
    
#     ax.plot(drone_path_np[:, 0], drone_path_np[:, 1], drone_path_np[:, 2], linewidth=1, c='b', label='Drone Path')
#     ax.plot(og[:, 0], og[:, 1], og[:, 2], linewidth=2, c='g')
    
#     ax.scatter(initial_start[0], initial_start[1], initial_start[2], c='g', marker='^', s=40, label='Start')
#     ax.scatter(start[0], start[1], start[2], c='b', marker='x', s=40)
#     ax.scatter(goal[0], goal[1], goal[2], c='r', marker='*', s=40, label='Goal')
#     ax.legend(loc='upper left')
    
#     # Update start and goal positions
#     start = np.array([og[3, 0], og[3, 1], og[3, 2]])
#     goal = goal + np.array([100, 0, 0])
#     print(og)
#     print(start)
    
#     return path_line, start_scatter, goal_scatter, drone_path_line

# ani = animation.FuncAnimation(fig, update, frames=time, init_func=init, blit=False, repeat=False)

# plt.tight_layout()
# plt.show()

import numpy as np
import plotly.graph_objects as go
from TrajectorySmoothing import gradient_descent_smoothing
from RRT3DstarTarget import pathPlanning

############################## Initialisation ###################################################

start = np.array([0, 0, 100])  # Start location in 3D
initial_start = start.copy()
goal = np.array([100, 0, 0])  # Goal location in 3D
obstacles = [
    np.array([450, 65.5, 90, 1100, 25, 180]),
    np.array([450, -65.5, 90, 1100, 25, 180]),
]  # cuboids parametrized by [x, y, z, dx, dy, dz]
time = np.arange(0, 9, 1)
drone_path = []
goal_path = []

############################## Path Planning ####################################################

for t in time:
    og, path = pathPlanning(start, goal, obstacles)
    og = gradient_descent_smoothing(og)
    
    # Update the drone path
    drone_path.append(start)
    
    # Save the goal position for each step
    goal_path.append(goal.copy())
    
    # Update start and goal positions
    start = np.array([og[3, 0], og[3, 1], og[3, 2]])
    goal = goal + np.array([100, 0, 0])

drone_path_np = np.array(drone_path)/100
goal_path_np = np.array(goal_path)/100
initial_start = initial_start/100

############################## Plotting #########################################################

# Create Plotly figure
fig = go.Figure()

# Plot the drone path
fig.add_trace(go.Scatter3d(
    x=drone_path_np[:, 0],
    y=drone_path_np[:, 1],
    z=drone_path_np[:, 2],
    mode='lines',
    line=dict(color='blue', width=2),
    name='Drone Path'
))

# Plot the goal path
fig.add_trace(go.Scatter3d(
    x=goal_path_np[:, 0],
    y=goal_path_np[:, 1],
    z=goal_path_np[:, 2],
    mode='lines',
    line=dict(color='red', width=2, dash='dash'),
    name='Goal Path'
))

# Plot initial start point
fig.add_trace(go.Scatter3d(
    x=[initial_start[0]],
    y=[initial_start[1]],
    z=[initial_start[2]],
    mode='markers',
    marker=dict(color='green', size=5, symbol='square'),
    name='Initial Start'
))

# Plot final goal position
fig.add_trace(go.Scatter3d(
    x=[goal_path_np[-1, 0]],
    y=[goal_path_np[-1, 1]],
    z=[goal_path_np[-1, 2]],
    mode='markers',
    marker=dict(color='red', size=5, symbol='circle'),
    name='Final Goal'
))

# Function to draw obstacles
def draw_obstacles(fig, obstacles):
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
            opacity=0.07,
            alphahull=0
        ))

draw_obstacles(fig, obstacles)

# Update plot layout
fig.update_layout(
    title='3D RRT* Static Path Planning',
    scene=dict(
        xaxis_title='X Position (m)',
        yaxis_title='Y Position (m)',
        zaxis_title='Z Position (m)',
        xaxis=dict(range=[-1, 10]),
        yaxis=dict(range=[-0.8, 0.8]),
        zaxis=dict(range=[0, 1.1])
    ),
    margin=dict(l=0, r=0, b=0, t=40)
)

# Show plot
fig.show()

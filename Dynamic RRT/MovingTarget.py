import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.animation as animation
import plotly.graph_objects as go
from TrajectorySmoothing import gradient_descent_smoothing
from RRT3DstarTarget import pathPlanning
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

# def update_plot():  # <- ADDED THIS FUNCTION
#     global drone_path, platform_path, og, t, obstacles

#     path_x_drone = [pos[0] for pos in drone_path]
#     path_y_drone = [pos[1] for pos in drone_path]
#     path_z_drone = [pos[2] for pos in drone_path]
    
#     path_x_rrt = [pos[0] for pos in og]
#     path_y_rrt = [pos[1] for pos in og]
#     path_z_rrt = [pos[2] for pos in og]

#     path_x_plat = [pos[0] for pos in platform_path]
#     path_y_plat = [pos[1] for pos in platform_path]
#     path_z_plat = [pos[2] for pos in platform_path]

#     # Plot the complete drone path
#     drone_path_line.set_data(path_x_drone, path_y_drone)
#     drone_path_line.set_3d_properties(path_z_drone, zdir='z')

#     # Plot the complete platform path
#     platform_path_line.set_data(path_x_plat, path_y_plat)
#     platform_path_line.set_3d_properties(path_z_plat, zdir='z')
    
#     # Plot the new path from the current start to the current goal
#     current_path_line.set_data(path_x_rrt, path_y_rrt)
#     current_path_line.set_3d_properties(path_z_rrt, zdir='z')

#     # Clear and redraw the plot
#     ax.cla()
#     draw_obstacles(ax, obstacles)
#     ax.set_xlim(bounds_x[0], bounds_x[1])
#     ax.set_ylim(bounds_y[1], bounds_y[0])
#     ax.set_zlim(bounds_z[0], bounds_z[1])
#     ax.set_title('Drone Position - IMU vs Model')
#     ax.set_xlabel('X axis')
#     ax.set_ylabel('Y axis')
#     ax.set_zlabel('Z axis')
    
#     # ax.plot(path_x_drone, path_y_drone, path_z_drone, linewidth=2, c='g', label='Drone Path')
#     ax.plot(path_x_drone, path_y_drone, path_z_drone, linewidth=2, c='g')
#     ax.plot(path_x_plat, path_y_plat, path_z_plat, linewidth=2, c='black', label='platform Path')
#     ax.scatter(path_x_plat, path_y_plat, path_z_plat, linewidth=2, c='black', marker='x')
#     ax.plot(path_x_rrt, path_y_rrt, path_z_rrt, linewidth=2, c='red', label='Current RRT Path')
#     # ax.scatter(start[0], start[1], start[2], c='b', marker='*', s=40)
#     ax.scatter(goal[0], goal[1], goal[2], c='r', marker='x', s=40)
#     ax.text(start[0], start[1], start[2], f'{t+1}', color='black')
#     ax.text(goal[0], goal[1], goal[2], f'{t+1}', color='black')
    
#     ax.legend(loc='upper left')

#     # ax.relim()
#     # ax.autoscale_view()
#     fig.canvas.draw()
#     fig.canvas.flush_events()

############################## Initialisation ###################################################

start = np.array([-1800, 0, 100])  # Start location in 3D
goal = np.array([-1500, 0, 0])  # Goal location in 3D
goal_x_velocity = np.array([600, 0, 0])
dt = 1
predicted_goal = goal + goal_x_velocity*dt
# Open sea
y = 0
obstacles = [
    # np.array([27.5, 8, 0.75, 55, 200, 1.5]),
    ]
  # cuboids parametrized by [x, y, z, dx, dy, dz]
obstacle_y_velocity = 250
time = np.arange(0, 5, 1)
drone_path = np.array([start])
# drone_path = np.delete(drone_path, -1, axis=0)
platform_path = [goal]

############################# Live Plot Initialisation #########################################

# plt.ion()
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# draw_obstacles(ax, obstacles)

# # Set up plot limits and labels
# ax.set_title('Drone Autonomous Landing on Moving Platform')
# ax.set_xlabel('X axis')
# ax.set_ylabel('Y axis')
# ax.set_zlabel('Z axis')
# bounds_x = np.array([-2000, 2000])  # Bounds for x 
# bounds_y = np.array([-1000, 1000])  # Bounds for y
# bounds_z = np.array([0, 200])  # Bounds for z 
# ax.set_xlim(bounds_x[0], bounds_x[1])
# ax.set_ylim(bounds_y[1], bounds_y[0])
# ax.set_zlim(bounds_z[0], bounds_z[1])

# drone_path_line, = ax.plot([], [], [], linewidth=2, c='g', label='Drone Path')
# platform_path_line, = ax.plot([], [], [], linewidth=2, c='b', label='Current RRT Path')
# current_path_line, = ax.plot([], [], [], linewidth=2, c='b', label='Current RRT Path')

# drone_path_line.set_data([], [])
# drone_path_line.set_3d_properties([], zdir='z')
# current_path_line.set_data([], [])
# current_path_line.set_3d_properties([], zdir='z')

# ############################### Simulation ###################################################
# for t in time:
#     # Generate new path from current drone position to predicted goal position
#     og, _ = pathPlanning(start, goal, obstacles)
#     og = gradient_descent_smoothing(og)

#     # Update the drone and goal path - Plot
#     drone_path = np.delete(drone_path, -1, axis=0)
#     drone_path = np.vstack((drone_path, og[2:,:]))
#     platform_path.append(goal)
#     update_plot()
#     sleep(dt)

#     # Get current position of drone and goal
#     start = np.array([og[3, 0], og[3, 1], og[3, 2]])
#     goal = goal + goal_x_velocity*dt
#     predicted_goal = goal + goal_x_velocity*2*dt
#     # y += obstacle_y_velocity
#     # obstacles = [
#     # np.array([-500, y, 25, 100, 200, 50]),]

############################## Static Plot ###################################################
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# # Add a title
# # ax.set_title('3D RRT* Dynamic Path Planning')
# # Label the axes
# ax.set_xlabel('X axis')
# ax.set_ylabel('Y axis')
# ax.set_zlabel('Z axis')
# # Set the boundary
# bounds_x = np.array([-20, 35])  # Bounds for x 
# bounds_y = np.array([-10, 10])  # Bounds for y
# bounds_z = np.array([0, 2])  # Bounds for z 
# ax.set_xlim(bounds_x[0], bounds_x[1])
# ax.set_ylim(bounds_y[1], bounds_y[0])
# ax.set_zlim(bounds_z[0], bounds_z[1])
# # Plot obstacles as cuboids
# for o in obstacles:
#     x = o[0] - o[3] / 2
#     y = o[1] - o[4] / 2
#     z = o[2] - o[5] / 2
#     dx = o[3]
#     dy = o[4]
#     dz = o[5]
#     xx = [x, x + dx, x + dx, x, x, x + dx, x + dx, x]
#     yy = [y, y, y + dy, y + dy, y, y, y + dy, y + dy]
#     zz = [z, z, z, z, z + dz, z + dz, z + dz, z + dz]
#     vertices = [[xx[0], yy[0], zz[0]],
#                 [xx[1], yy[1], zz[1]],
#                 [xx[2], yy[2], zz[2]],
#                 [xx[3], yy[3], zz[3]],
#                 [xx[4], yy[4], zz[4]],
#                 [xx[5], yy[5], zz[5]],
#                 [xx[6], yy[6], zz[6]],
#                 [xx[7], yy[7], zz[7]]]
#     faces = [[vertices[j] for j in [0, 1, 2, 3]],  # bottom face
#             [vertices[j] for j in [4, 5, 6, 7]],  # top face
#             [vertices[j] for j in [0, 3, 7, 4]],  # left face
#             [vertices[j] for j in [1, 2, 6, 5]],  # right face
#             [vertices[j] for j in [0, 1, 5, 4]],  # front face
#             [vertices[j] for j in [2, 3, 7, 6]]]  # back face
#     ax.add_collection3d(Poly3DCollection(faces, facecolors='k', linewidths=1, edgecolors='black', alpha=.25))

# ax.scatter(start[0], start[1], start[2], c='b', marker='x', label='Start', s=40)
# ax.scatter(goal[0], goal[1], goal[2], c='r', marker='*', label='Goal', s=40)

# # ############################## Simulation ###################################################
# for t in time:

#     if t>2:
#         predicted_goal = goal + goal_x_velocity*(t-2)
#         platform_path.append(predicted_goal)
#         og, path = pathPlanning(start, predicted_goal, obstacles)
#         ax.scatter(predicted_goal[0]/100, predicted_goal[1]/100, predicted_goal[2]/100, c='g', marker='x', s=100)
#         # ax.text(predicted_goal[0]/100, predicted_goal[1]/100, predicted_goal[2]/100, f'{t+1}', color='green')
#         # ax.scatter(goal[0], goal[1], goal[2], c='r', marker='x', s=40)
#     else:
#         og, path = pathPlanning(start, goal, obstacles)
#         # ax.scatter(goal[0], goal[1], goal[2], c='r', marker='x', s=40)
#         ax.text(goal[0]/100, goal[1]/100, goal[2]/100, f'{t}', color='black')

#     og = gradient_descent_smoothing(og)
#     ax.plot(og[:, 0]/100, og[:, 1]/100, og[:, 2]/100, linewidth=2)
#     ax.scatter(start[0]/100, start[1]/100, start[2]/100, c='b', marker='^', s=40)
#     ax.text(start[0]/100, start[1]/100, start[2]/100, f'{t}', color='black')
#     # ax.scatter(goal[0], goal[1], goal[2], c='r', marker='x', s=40)
#     ax.text(goal[0]/100, goal[1]/100, goal[2]/100, f'{t}', color='black')
    
#     start = np.array([og[3, 0], og[3, 1], og[3, 2]])
#     goal = goal + goal_x_velocity*dt
#     platform_path.append(goal)


# path_x_plat = [pos[0]/100 for pos in platform_path]
# path_y_plat = [pos[1]/100 for pos in platform_path]
# path_z_plat = [pos[2]/100 for pos in platform_path]  
# ax.plot(path_x_plat, path_y_plat, path_z_plat, c='black', linewidth=2, marker="x", label="Goal Path")
# ax.scatter(predicted_goal[0]/100, predicted_goal[1]/100, predicted_goal[2]/100, c='g', marker='x', s=100, label="Predicted Goal") 
# ax.legend(loc='upper left')
# plt.tight_layout()
# plt.show()


# Define the boundaries
bounds_x = np.array([-20, 25])
bounds_y = np.array([-2.5, 2.5])
bounds_z = np.array([0, 1.5])

# Initialize the 3D plot
fig = go.Figure()

# Plot start and goal points
fig.add_trace(go.Scatter3d(x=[start[0]], y=[start[1]], z=[start[2]], mode='markers', marker=dict(color='blue', size=5, symbol='circle'), name='Current Drone Position'))

# Define a list of colours to cycle through
colours = ['green', 'purple', 'orange', 'cyan', 'magenta']

# Simulation loop
for i, t in enumerate(time):
    if t > 1:
        predicted_goal = goal + goal_x_velocity * (t - 2)
        platform_path.append(predicted_goal)
        og, path = pathPlanning(start, predicted_goal, obstacles)
        fig.add_trace(go.Scatter3d(x=[predicted_goal[0] / 100], y=[predicted_goal[1] / 100], z=[predicted_goal[2] / 100],
                                   mode='markers', marker=dict(color='green', size=6, symbol='diamond'), showlegend=False))
    else:
        og, path = pathPlanning(start, goal, obstacles)
        fig.add_trace(go.Scatter3d(x=[goal[0] / 100], y=[goal[1] / 100], z=[goal[2] / 100],
                                   mode='text', text=[f'{t}'], textposition="top center", showlegend=False))

    fig.add_trace(go.Scatter3d(x=[goal[0] / 100], y=[goal[1] / 100], z=[goal[2] / 100],
                                   mode='text', text=[f'{t}'], textposition="top center", showlegend=False))
    og = gradient_descent_smoothing(og)
    # Use the current index to select a colour from the list
    fig.add_trace(go.Scatter3d(x=og[:, 0] / 100, y=og[:, 1] / 100, z=og[:, 2] / 100,
                               mode='lines', line=dict(width=4, color="red"), showlegend=False))
    fig.add_trace(go.Scatter3d(x=og[3:, 0] / 100, y=og[3:, 1] / 100, z=og[3:, 2] / 100,
                               mode='lines', line=dict(width=4, color="blue"), showlegend=False))

    fig.add_trace(go.Scatter3d(x=[start[0] / 100], y=[start[1] / 100], z=[start[2] / 100],
                               mode='markers+text', marker=dict(color='blue', size=5, symbol='circle'),
                               text=[f'{t}'], textposition="top center", showlegend=False))

    start = np.array([og[3, 0], og[3, 1], og[3, 2]])
    goal = goal + goal_x_velocity * dt
    platform_path.append(goal)

fig.add_trace(go.Scatter3d(x=[predicted_goal[0] / 100], y=[predicted_goal[1] / 100], z=[predicted_goal[2] / 100],
                                   mode='markers', marker=dict(color='green', size=6, symbol='diamond'), name="Predicted Targets", showlegend=True))
fig.add_trace(go.Scatter3d(x=og[:, 0] / 100, y=og[:, 1] / 100, z=og[:, 2] / 100,
                            mode='lines', line=dict(width=4, color="red"), name="Invalid Path", showlegend=True))
fig.add_trace(go.Scatter3d(x=og[3:, 0] / 100, y=og[3:, 1] / 100, z=og[3:, 2] / 100,
                               mode='lines', line=dict(width=4, color="blue"), name="Generated Path", showlegend=True))
# Plot the entire platform's path from the start
path_x_plat = [pos[0] / 100 for pos in platform_path]
path_y_plat = [pos[1] / 100 for pos in platform_path]
path_z_plat = [pos[2] / 100 for pos in platform_path]
fig.add_trace(go.Scatter3d(x=path_x_plat, y=path_y_plat, z=path_z_plat, mode='lines+markers',
                           line=dict(color='black', width=3), marker=dict(symbol='diamond', size=3), text=[f'{t}'], name="Target Path"))

def draw_obstacles(fig, obstacles):
    for o in obstacles:
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
            opacity=0.1,
            alphahull=0
        ))
obstacles = [
    np.array([20, 2, 0.75, 75, 1, 1.5]),
    ]
draw_obstacles(fig, obstacles)

# Set axis labels and limits
fig.update_layout(
    scene=dict(
    xaxis_title='X Position (m)',
    yaxis_title='Y Position (m)',
    zaxis_title='Z Position (m)',
    xaxis=dict(range=[bounds_x[0], bounds_x[1]]),
    yaxis=dict(range=[bounds_y[0], bounds_y[1]]),
    zaxis=dict(range=[bounds_z[0], bounds_z[1]])
),
legend=dict(
        x=0.55,  # Move the legend closer to the plot
        y=0.8,
        xanchor='left',
        yanchor='top'
    ),
)

# Show the plot
fig.show()



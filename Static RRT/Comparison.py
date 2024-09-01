# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d.art3d import Poly3DCollection
# from RRT3DstarSimple import pathPlanning as pps
# from RRT3DstarObstaclesSafe import pathPlanning as ppo
# from RRT3DstarMaxAngle import pathPlanning as ppm
# from RRT3Dstar import pathPlanning as ppf
# from TrajectorySmoothing import gradient_descent_smoothing

# ############################################ Plotting ############################################

# def plotting3D(start, goal, obstacles, path1, path2, path3, path4):
#     """Plot the generated RRT* tree and the final path"""
#     fig = plt.figure()
#     # fi1 = plt.figure()

#     # ax1 = fi1.add_subplot(111, projection='3d')
#     ax2 = fig.add_subplot(111, projection='3d')

#     start = start/100
#     goal = goal/100
#     path1, path2, path3, path4 = path1/100, path2/100, path3/100, path4/100
#     # Function to plot obstacles
#     def plot_obstacles(ax):
#         i = 0
#         for o in obstacles:
#             o = o/100
#             if i in [1, 2]:
#                 a = 0.0
#             else:
#                 a = 0.0
#             x = o[0] - o[3] / 2
#             y = o[1] - o[4] / 2
#             z = o[2] - o[5] / 2
#             dx = o[3]
#             dy = o[4]
#             dz = o[5]
#             xx = [x, x + dx, x + dx, x, x, x + dx, x + dx, x]
#             yy = [y, y, y + dy, y + dy, y, y, y + dy, y + dy]
#             zz = [z, z, z, z, z + dz, z + dz, z + dz, z + dz]
#             vertices = [[xx[0], yy[0], zz[0]],
#                         [xx[1], yy[1], zz[1]],
#                         [xx[2], yy[2], zz[2]],
#                         [xx[3], yy[3], zz[3]],
#                         [xx[4], yy[4], zz[4]],
#                         [xx[5], yy[5], zz[5]],
#                         [xx[6], yy[6], zz[6]],
#                         [xx[7], yy[7], zz[7]]]
#             faces = [[vertices[j] for j in [0, 1, 2, 3]],  # bottom face
#                      [vertices[j] for j in [4, 5, 6, 7]],  # top face
#                      [vertices[j] for j in [0, 3, 7, 4]],  # left face
#                      [vertices[j] for j in [1, 2, 6, 5]],  # right face
#                      [vertices[j] for j in [0, 1, 5, 4]],  # front face
#                      [vertices[j] for j in [2, 3, 7, 6]]]  # back face
#             ax.add_collection3d(Poly3DCollection(faces, facecolors='olive', linewidths=1, edgecolors='grey', alpha=a))
#             i+=1

#     # Function to plot start and goal points
#     def plot_start_goal(ax):
#         ax.scatter(start[0], start[1], start[2], c='b', marker='*', s=100, label='Start')
#         ax.scatter(goal[0], goal[1], goal[2], c='r', marker='x', s=100, label='Goal')

#     # First subplot with path1 and path2
#     # plot_obstacles(ax1)
#     # plot_start_goal(ax1)
#     # ax1.plot(path1[:, 0], path1[:, 1], path1[:, 2], label="RRT* Simple", linewidth=2, color='red')
#     # ax1.plot(path2[:, 0], path2[:, 1], path2[:, 2], label="RRT* with obstacle avoidance", linewidth=2, color='green')
#     # ax1.set_xlim([-5, 5])
#     # ax1.set_ylim([-5, 5])
#     # ax1.set_zlim([0, 2])
#     # ax1.set_xlabel('X position (m)')
#     # ax1.set_ylabel('Y position (m)')
#     # ax1.set_zlabel('Z position (m)')
#     # ax1.legend(loc='upper left')
#     # ax1.set_title('3D RRT* Path Planning with Obstacle Avoidance')

#     # Second subplot with path3 and path4
#     plot_start_goal(ax2)
#     ax2.plot(path1[:, 0], path1[:, 1], path1[:, 2], label="RRT* Base", linewidth=2, color='black', linestyle="--")
#     ax2.plot(path2[:, 0], path2[:, 1], path2[:, 2], label="RRT* with with Obstacle Avoidance", linewidth=2, color='blue')
#     ax2.plot(path3[:, 0], path3[:, 1], path3[:, 2], label="RRT* with with Turning Constraints", linewidth=2, color='red')
#     ax2.plot(path4[:, 0], path4[:, 1], path4[:, 2], label="RRT* with Ground Clearance", linewidth=2, color='green')
#     plot_obstacles(ax2)
#     ax2.set_xlim([-5, 5])
#     ax2.set_ylim([-5, 5])
#     ax2.set_zlim([0, 2])
#     ax2.set_xlabel('X position (m)')
#     ax2.set_ylabel('Y position (m)')
#     ax2.set_zlabel('Z position (m)')
#     ax2.legend(loc='upper right')
#     ax2.set_title('Improved 3D RRT* Path Planning')

#     plt.tight_layout()
#     plt.show()

# ############################# Final #####################################

# def finalrrt(start, goal, obstacles, path1):
#     """Plot the generated RRT* tree and the final path"""
#     fig = plt.figure()

#     ax = fig.add_subplot(111, projection='3d')

#     # Function to plot obstacles
#     def plot_obstacles(ax):
#         for o in obstacles:
#             x = o[0] - o[3] / 2
#             y = o[1] - o[4] / 2
#             z = o[2] - o[5] / 2
#             dx = o[3]
#             dy = o[4]
#             dz = o[5]
#             xx = [x, x + dx, x + dx, x, x, x + dx, x + dx, x]
#             yy = [y, y, y + dy, y + dy, y, y, y + dy, y + dy]
#             zz = [z, z, z, z, z + dz, z + dz, z + dz, z + dz]
#             vertices = [[xx[0], yy[0], zz[0]],
#                         [xx[1], yy[1], zz[1]],
#                         [xx[2], yy[2], zz[2]],
#                         [xx[3], yy[3], zz[3]],
#                         [xx[4], yy[4], zz[4]],
#                         [xx[5], yy[5], zz[5]],
#                         [xx[6], yy[6], zz[6]],
#                         [xx[7], yy[7], zz[7]]]
#             faces = [[vertices[j] for j in [0, 1, 2, 3]],  # bottom face
#                      [vertices[j] for j in [4, 5, 6, 7]],  # top face
#                      [vertices[j] for j in [0, 3, 7, 4]],  # left face
#                      [vertices[j] for j in [1, 2, 6, 5]],  # right face
#                      [vertices[j] for j in [0, 1, 5, 4]],  # front face
#                      [vertices[j] for j in [2, 3, 7, 6]]]  # back face
#             ax.add_collection3d(Poly3DCollection(faces, facecolors='khaki', linewidth=3, edgecolors='black', alpha=0.3))

#     # Function to plot start and goal points
#     def plot_start_goal(ax):
#         ax.scatter(start[0], start[1], start[2], c='b', marker='*', s=100, label='Start')
#         ax.scatter(goal[0], goal[1], goal[2], c='r', marker='x', s=100, label='Goal')

#     # Second subplot with path3 and path4
#     plot_start_goal(ax)
#     ax.plot(path1[:, 0], path1[:, 1], path1[:, 2], linewidth=2, color='steelblue')
#     plot_obstacles(ax)
#     ax.set_xlim([-500, 500])
#     ax.set_ylim([-500, 500])
#     ax.set_zlim([0, 200])
#     ax.set_xlabel('X axis')
#     ax.set_ylabel('Y axis')
#     ax.set_zlabel('Z axis')
#     ax.legend(loc='upper right')
#     ax.set_title('Improved 3D RRT* Path Planning')

#     plt.tight_layout()
#     plt.show()


# ############################# run ########################################

# start = np.array([-400, 400, 0])  # Start location in 3D
# goal = np.array([400, -400, 0])  # Goal location in 3D
# obstacles = [ 
#     np.array([-15, 450, 50, 30, 100, 50]),
#     np.array([-15, 210, 45, 30, 370, 90]),
#     np.array([-15, 210, 155, 30, 370, 90]),
#     np.array([-150, 15, 100, 300, 30, 200]),
#     np.array([-280, -100, 100, 30, 200, 200]),
#     np.array([-385, -220, 100, 235, 30, 200]),

#     np.array([200, 200, 100, 100, 100, 200]),
#     np.array([400, 0, 100, 100, 100, 200]),
#     np.array([200, -200, 100, 100, 100, 200]),
#     np.array([0, -400, 100, 100, 100, 200]),
#     ]  # cuboids parametrized by [x, y, z, dx, dy, dz]
# path1, _ = pps(start, goal, obstacles)
# path2, _ = ppo(start, goal, obstacles)
# path3, _ = ppm(start, goal, obstacles)
# path4, _ = ppf(start, goal, obstacles)
# # finalrrt(start, goal, obstacles, gradient_descent_smoothing(path4))
# plotting3D(start, goal, obstacles, gradient_descent_smoothing(path1), gradient_descent_smoothing(path2), gradient_descent_smoothing(path3), gradient_descent_smoothing(path4))



import numpy as np
import plotly.graph_objects as go
from RRT3DstarSimple import pathPlanning as pps
from RRT3DstarObstacles import pathPlanning as ppo
from RRT3DstarMaxAngle import pathPlanning as ppm
from RRT3Dstar import pathPlanning as ppf
from TrajectorySmoothing import gradient_descent_smoothing

############################# run ########################################

start = np.array([-400, 400, 0])  # Start location in 3D
goal = np.array([400, -400, 0])  # Goal location in 3D
obstacles = [ 
    np.array([-15, 450, 50, 30, 100, 50]),
    np.array([-15, 210, 45, 30, 370, 90]),
    np.array([-15, 210, 155, 30, 370, 90]),
    np.array([-150, 15, 100, 300, 30, 200]),
    np.array([-280, -100, 100, 30, 200, 200]),
    np.array([-385, -220, 100, 235, 30, 200]),
    np.array([200, 200, 100, 100, 100, 200]),
    np.array([400, 0, 100, 100, 100, 200]),
    np.array([200, -200, 100, 100, 100, 200]),
    np.array([0, -400, 100, 100, 100, 200]),
]  # cuboids parametrized by [x, y, z, dx, dy, dz]
path1, _ = pps(start, goal, obstacles)
path2, _ = ppo(start, goal, obstacles)
path3, _ = ppm(start, goal, obstacles)
path4, _ = ppf(start, goal, obstacles)

############################################ Plotting ############################################

############################### Helper functions to plot obstacles, start, and goal ################################################################
def plot_obstacles(fig, obstacles):
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
            opacity=0.2,
            alphahull=0
        ))

def plot_start_goal(fig, start, goal):
    fig.add_trace(go.Scatter3d(
        x=[start[0]], y=[start[1]], z=[start[2]],
        mode='markers',
        marker=dict(size=8, color='blue'),
        name='Start',
        showlegend=False
    ))
    fig.add_trace(go.Scatter3d(
        x=[goal[0]], y=[goal[1]], z=[goal[2]],
        mode='markers',
        marker=dict(size=8, color='red'),
        name='Goal',
        showlegend=False
    ))
    # Add annotation for start point
    # fig.add_trace(go.Scatter3d(
    #     x=[start[0]], y=[start[1]], z=[start[2]],
    #     mode='text',
    #     text=["Start"],
    #     textposition="top center",
    #     textfont=dict(size=12, color="black"),
    #     showlegend=False
    # ))
    # # Add annotation for goal point
    # fig.add_trace(go.Scatter3d(
    #     x=[goal[0]], y=[goal[1]], z=[goal[2]],
    #     mode='text',
    #     text=["Goal"],
    #     textposition="top center",
    #     textfont=dict(size=12, color="black"),
    #     showlegend=False
    # ))

###################################### results ##########################################################################
def plotting3D(start, goal, obstacles, path1, path2, path3, path4):
    """Plot the generated RRT* tree and the final path using Plotly"""
    
    #################### convert to meters ############################
    start = start / 100
    goal = goal / 100
    path1, path2, path3, path4 = path1 / 100, path2 / 100, path3 / 100, path4 / 100

    # Define figure size (in pixels)
    fig_width = 700  # Width of the figure
    fig_height = 600  # Height of the figure

    #################### Create the figure 1 ##########################################################################
    fig1 = go.Figure()
    fig1.update_layout(
    scene=dict(
        xaxis=dict(
            title='X Position (m)', range=[-5, 5],
            titlefont=dict(
                size=14  # Change this to your desired font size for axis title
             )
        ),
        yaxis=dict(
            title='Y Position (m)', range=[-5, 5],
            titlefont=dict(
                size=14  # Change this to your desired font size for axis title
             )
        ),
        zaxis=dict(
            title='Z Position (m)', range=[0, 2],
            titlefont=dict(
                size=14  # Change this to your desired font size for axis title
             )
        )
    ),
    legend=dict(
        x=0.75,  # Move the legend closer to the plot
        y=0.65,
        xanchor='left',
        yanchor='top',
        font=dict(
        size=14  # Change this to your desired font size
    )
    ),
    showlegend=False,
    width=fig_width,  # Set figure width
    height=fig_height  # Set figure height
    )

    # Plot obstacles, start, goal and paths in the first figure 
    plot_obstacles(fig1, obstacles)
    plot_start_goal(fig1, start, goal)

    fig1.add_trace(go.Scatter3d(
        x=path1[:, 0], y=path1[:, 1], z=path1[:, 2],
        mode='lines',
        line=dict(color='red', width=4),
        name='Base RRT*'
    ))
    fig1.add_trace(go.Scatter3d(
        x=path2[:, 0], y=path2[:, 1], z=path2[:, 2],
        mode='lines',
        line=dict(color='blue', width=4),
        name='RRT* with<br>Obstacle Avoidance'
    ))
    fig1.show()

    ########################################### Create figure 2 #######################################
    fig2 = go.Figure()
    fig2.update_layout(
    scene=dict(
        xaxis=dict(
            title='X Position (m)', range=[-5, 5],
            titlefont=dict(
                size=14  # Change this to your desired font size for axis title
             )
        ),
        yaxis=dict(
            title='Y Position (m)', range=[-5, 5],
            titlefont=dict(
                size=14  # Change this to your desired font size for axis title
             )
        ),
        zaxis=dict(
            title='Z Position (m)', range=[0, 2],
            titlefont=dict(
                size=14  # Change this to your desired font size for axis title
             )
        )
    ),
    legend=dict(
        x=0.75,  # Move the legend closer to the plot
        y=0.65,
        xanchor='left',
        yanchor='top',
        font=dict(
            size=14  # Change this to your desired font size
        )
    ),
    showlegend=True,
    width=fig_width,  # Set figure width
    height=fig_height  # Set figure height
    )

    # Plot obstacles, start, goal and paths in the second figure
    plot_obstacles(fig2, obstacles)
    plot_start_goal(fig2, start, goal)

    fig2.add_trace(go.Scatter3d(
        x=path3[:, 0], y=path3[:, 1], z=path3[:, 2],
        mode='lines',
        line=dict(color='red', width=4),
        name='RRT* with Turning<br>Constraints'
    ))
    fig2.add_trace(go.Scatter3d(
        x=path4[:, 0], y=path4[:, 1], z=path4[:, 2],
        mode='lines',
        line=dict(color='blue', width=4),
        name='RRT* with Ground<br>Clearance'
    ))
    fig2.show()

plotting3D(start, goal, obstacles, gradient_descent_smoothing(path1), gradient_descent_smoothing(path2), gradient_descent_smoothing(path3), gradient_descent_smoothing(path4))
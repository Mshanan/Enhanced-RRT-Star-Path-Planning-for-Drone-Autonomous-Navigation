import KeyPressModule as kp
from djitellopy import tello
from time import sleep, time
from math import sin, cos, radians
import matplotlib.pyplot as plt
import numpy as np
from RRT3Dstar import pathPlanning
from TrajectorySmoothing import gradient_descent_smoothing

############################ function definitions ###############################

def getKeyboardInput():
    lr, fb, ud, yv = 0, 0, 0, 0
    speed = 50  # Commanded speed in cm/s

    if kp.getKey("a"): lr = -speed 
    elif kp.getKey("d"): lr = speed

    if kp.getKey("w"): fb = speed
    elif kp.getKey("s"): fb = -speed

    if kp.getKey("UP"): ud = speed
    elif kp.getKey("DOWN"): ud = -speed

    if kp.getKey("LEFT"): yv = -speed
    elif kp.getKey("RIGHT"): yv = speed

    if kp.getKey("l"): drone.land()
    if kp.getKey("i"): drone.takeoff()

    return [lr, fb, ud, yv]

def updatePositionModel(x, y, lr, fb, dt):
    
    # Adjust velocities by the error factors
    lr *= 0.95
    fb *= 0.95

    # Calculate displacement in the drone's frame of reference
    dx = fb * dt
    dy = lr * dt
    yaw = drone.get_yaw()

    # Convert to the global frame of reference
    x += dx * cos(radians(yaw)) - dy * sin(radians(yaw))
    y += dx * sin(radians(yaw)) + dy * cos(radians(yaw))
    z = drone.get_height()*1.5
    if z<0:
        z = 0

    # Append current position and yaw to path list
    path_model.append((x, y, z, yaw))

    return x, y, z  # Return the updated position and yaw

def updatePositionIMU(x_imu, y_imu, dt):
    
    # Get the velocity data from the drone and Update positions by integrating velocity
    vx = drone.get_speed_x()*1.2
    vy = drone.get_speed_y()*1.2
    z = drone.get_height()*1.5
    yaw = drone.get_yaw()

    if z<0:
        z = 0

    x_imu += vx * dt
    y_imu += vy * dt

    # Convert to the global frame of reference
    x_imu_global = x_imu * cos(radians(yaw)) - y_imu * sin(radians(yaw))
    y_imu_global = x_imu * sin(radians(yaw)) + y_imu * cos(radians(yaw))

    # Append current position and yaw to path list
    path_imu.append((x_imu_global*10, y_imu_global*10, z, yaw))

    return x_imu, y_imu  # Return the updated position and velocities

def followPathOpenLoop(start, waypoints, prev_time, x_imu, y_imu):
   
    x, y, z = start
    prev_x, prev_y, prev_z = 0, 0, z
    delta_t = 0.75

    for wp in waypoints:
        if kp.getKey("b"):
            while True:
                vals = getKeyboardInput()
                drone.send_rc_control(vals[0], vals[1], vals[2], vals[3]) 
        else:
            x_goal, y_goal, z_goal = wp
            x_dist = x_goal - prev_x
            y_dist = y_goal - prev_y
            z_dist = z_goal - prev_z

            # print(f"x_dist: {x_dist}, y_dist: {y_dist}, z_dist: {z_dist}")

            fb = int((x_dist/delta_t))
            lr = int((y_dist/delta_t))
            ud = int((z_dist/delta_t))
            yv = 0
            drone.send_rc_control(lr, fb, ud, yv) # Send control commands to drone

            # Update position
            current_time = time()
            dt = current_time - prev_time
            prev_time = current_time
            x, y, z = updatePositionModel(x, y, lr, fb, delta_t)
            x_imu, y_imu = updatePositionIMU(x_imu, y_imu, dt)

            prev_x = x_goal
            prev_y = y_goal
            prev_z = z_goal

            # Sleep to simulate time taken to reach waypoint
            sleep(delta_t)
    
    drone.land()    # Land the drone after reaching the goal
    print("Goal Reached")
    return x, y, z, x_imu, y_imu

def plotting(path_model, path_imu):
    # Plotting path in 3D
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111, projection='3d')
    # Set the boundary
    bounds_x = np.array([-250, 250])  # Bounds for x 
    bounds_y = np.array([-250, 250])  # Bounds for y
    bounds_z = np.array([0, 200])  # Bounds for z 
   #  ax1.set_xlim(bounds_x[0], bounds_x[1])
    ax1.set_ylim(bounds_y[1], bounds_y[0])
    ax1.set_zlim(bounds_z[0], bounds_z[1])

    path_x_cmd = [pos[0] for pos in path_model]
    path_y_cmd = [pos[1] for pos in path_model]
    path_z_cmd = [pos[2] for pos in path_model]
    ax1.plot(path_x_cmd, path_y_cmd, path_z_cmd, label='Model Path', linewidth=2)
    ax1.scatter(path_x_cmd[0], path_y_cmd[0], path_z_cmd[0], marker='*', c='b', s=50, label='Start')
    ax1.scatter(path_x_cmd[-1], path_y_cmd[-1], path_z_cmd[-1], marker='x', c='r', s=50, label='Goal')

    ax1.set_xlabel('X (cm)')
    ax1.set_ylabel('Y (cm)')
    ax1.set_zlabel('Z (cm)')
    ax1.set_title('Drone Path Estimation (Model)')

    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111, projection='3d')
    # ax2.set_xlim(bounds_x[0], bounds_x[1])
    ax2.set_ylim(bounds_y[1], bounds_y[0])
    ax2.set_zlim(bounds_z[0], bounds_z[1])

    path_x_imu = [pos[0] for pos in path_imu]
    path_y_imu = [pos[1] for pos in path_imu]
    path_z_imu = [pos[2] for pos in path_imu]
    ax2.plot(path_x_imu, path_y_imu, path_z_imu, label='IMU Path', linewidth=2)
    ax2.scatter(path_x_imu[0], path_y_imu[0], path_z_imu[0], marker='*', c='b', s=50, label='Start')
    ax2.scatter(path_x_imu[-1], path_y_imu[-1], path_z_imu[-1], marker='x', c='r', s=50, label='Goal')

    ax2.set_xlabel('X (cm)')
    ax2.set_ylabel('Y (cm)')
    ax2.set_zlabel('Z (cm)')
    ax2.set_title('Drone Path Estimation (IMU)')
    
    ax1.legend(loc='upper left')
    ax2.legend(loc='upper left')

    plt.show()

def plotting3D(path_model, path_imu, waypoints, start, goal):
    
    # Plotting path in 3D
    # Set bounds
    bounds_xy = np.array([-250, 250])  # Bounds for x and y axes
    bounds_z = np.array([0, 200])  # Bounds for z axis

    # figure 1: model-based 3D position estimation 
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111, projection='3d')

    ax1.set_xlim(bounds_xy[0], bounds_xy[1])
    ax1.set_ylim(bounds_xy[1], bounds_xy[0])
    ax1.set_zlim(bounds_z[0], bounds_z[1])

    ax1.set_xlabel('X (cm)')
    ax1.set_ylabel('Y (cm)')
    ax1.set_zlabel('Z (cm)')
    ax1.set_title('Drone Path Estimation (Model)')

    path_x_cmd = [pos[0] for pos in path_model]
    path_y_cmd = [pos[1] for pos in path_model]
    path_z_cmd = [pos[2] for pos in path_model]
    ax1.plot(path_x_cmd, path_y_cmd, path_z_cmd, label='Model Path', linewidth=2)

    # figure 2: IMU-based 3D position estimation 
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111, projection='3d')

    ax2.set_xlim(bounds_xy[0], bounds_xy[1])
    ax2.set_ylim(bounds_xy[1], bounds_xy[0])
    ax2.set_zlim(bounds_z[0], bounds_z[1])

    ax2.set_xlabel('X (cm)')
    ax2.set_ylabel('Y (cm)')
    ax2.set_zlabel('Z (cm)')
    ax2.set_title('Drone Path Estimation (IMU)')

    path_x_imu = [pos[0] for pos in path_imu]
    path_y_imu = [pos[1] for pos in path_imu]
    path_z_imu = [pos[2] for pos in path_imu]
    ax2.plot(path_x_imu, path_y_imu, path_z_imu, label='IMU Path', linewidth=2)

    # Plotting waypoints on both figure 1 and figre 2
    waypoint_x = [point[0] for point in waypoints]
    waypoint_y = [point[1] for point in waypoints]
    waypoint_z = [point[2] for point in waypoints]
    ax1.plot(waypoint_x, waypoint_y, waypoint_z, linestyle='-', color='r', label='Waypoints')
    ax2.plot(waypoint_x, waypoint_y, waypoint_z, linestyle='-', color='r', label='Waypoints')
    ax1.scatter(start[0], start[1], start[2], c='b', marker='x', s=50, label='Start')
    ax1.scatter(goal[0], goal[1], goal[2], c='r', marker='*', s=50, label='Goal')
    ax2.scatter(start[0], start[1], start[2], c='b', marker='x', s=50, label='Start')
    ax2.scatter(goal[0], goal[1], goal[2], c='r', marker='*', s=50, label='Goal')

    ax1.legend(loc='upper left')
    ax2.legend(loc='upper left')

    plt.show()

####################################### end of function definitions ########################################
####################################### Initialisation #####################################################

# Tello drone initialistion
drone = tello.Tello()
drone.connect()
print(drone.get_battery())
kp.init()

# Initialize position and time variables
x, y, z, yaw = 0, 0, 0, 0
x_imu, y_imu = 0, 0
prev_time = time()

# List to store path data for plotting
path_model = []
path_imu = []
altitude = []
time_stamps = []

# set start, goal, and obstacles for RRT
goal = np.array([200, 0, 0])   # goal location in 3D 
obstacles = [
        np.array([95, -100, 50, 90, 300, 100]),
        np.array([250, 175, 25, 50, 150, 50]),
    ]  # cuboids parametrized by [x, y, z, dx, dy, dz]

####################################### Live Altitude plotting #####################################################

# Initialize plot for live altitude plotting  # <- ADDED THIS INITIALIZATION
# plt.ion()
# fig, ax = plt.subplots()
# line, = ax.plot([], [], label='Altitude over time')
# ax.set_xlabel('Time (s)')
# ax.set_ylabel('Altitude (cm)')
# ax.set_title('Altitude vs Time')
# ax.legend()

# def update_plot():  # <- ADDED THIS FUNCTION
#     line.set_data(time_stamps, altitude)
#     ax.relim()
#     ax.autoscale_view()
#     fig.canvas.draw()
#     fig.canvas.flush_events()

####################################### Main Control Loop #####################################################

while True:
    # calculate dt
    current_time = time()
    dt = current_time - prev_time
    prev_time = current_time
    
    # send keyboard cmd velocities to drone
    vals = getKeyboardInput()
    drone.send_rc_control(vals[0], vals[1], vals[2], vals[3])

    # estimate drone 3D position 
    x, y, z = updatePositionModel(x, y, vals[0], vals[1], dt)
    x_imu, y_imu = updatePositionIMU(x_imu, y_imu, dt)

    # debugging
    # altitude.append(drone.get_height()*1.2)
    # time_stamps.append(current_time)
    # Update live plot
    # update_plot()  # <- ADDED THIS LINE

    # run path planning algorithm 
    if kp.getKey("f"):
        start = np.array([0, 0, z])  # get current position as start
        original, waypoints = pathPlanning(start, goal, obstacles)   # Get path from RRT
        smooth_waypoints_gd = gradient_descent_smoothing(waypoints)
        x, y, z, x_imu, y_imu = followPathOpenLoop(start, smooth_waypoints_gd, prev_time, x_imu, y_imu)    # Follow the planned path with open loop control

    # plot drone trajectory 
    if kp.getKey("p"):
        plotting3D(path_model, path_imu, gradient_descent_smoothing(original), start, goal)
        # plotting(path_model, path_imu)
        break

    # emergency button
    if kp.getKey("b"):
        # drone.land()
        break

####################################### Post-Processing #####################################################

# plt.ioff()  # <- ADDED THIS LINE TO TURN INTERACTIVE MODE OFF
# plt.show()  # <- ADDED THIS LINE TO SHOW THE FINAL PLOT
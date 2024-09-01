import numpy as np
from scipy.interpolate import CubicSpline, BSpline, splprep, splev
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Function to smooth the path using cubic spline interpolation
def cubic_spline_smoothing(waypoints):
    # Extract the x, y, and z coordinates
    x = waypoints[:, 0]
    y = waypoints[:, 1]
    z = waypoints[:, 2]
    
    # Create a parameter t for interpolation
    t = np.linspace(0, 1, len(waypoints))
    
    # Create cubic splines for x, y, and z
    cs_x = CubicSpline(t, x)
    cs_y = CubicSpline(t, y)
    cs_z = CubicSpline(t, z)
    
    # Interpolate more points along the spline
    t_smooth = np.linspace(0, 1, 2000)  # Increase 100 for more points

    x_smooth = cs_x(t_smooth)
    y_smooth = cs_y(t_smooth)
    z_smooth = cs_z(t_smooth)
    
    # Combine the smooth points into a single array
    smooth_path = np.vstack((x_smooth, y_smooth, z_smooth)).T
    
    return smooth_path

# Gradient descent smoothing function
def gradient_descent_smoothing(waypoints, alpha=0.01, lambda_reg=0.05, max_iter=1000, tol=1e-6):
    """
    Smooth the path using gradient descent.
    
    Parameters:
        waypoints (np.ndarray): The original waypoints.
        alpha (float): The learning rate.
        lambda_reg (float): The regularization parameter for waypoint fidelity.
        max_iter (int): Maximum number of iterations.
        tol (float): Tolerance for convergence.
    
    Returns:
        np.ndarray: Smoothed waypoints.
    """
    waypoints_smooth = waypoints.copy()
    n = len(waypoints)
    
    for iter in range(max_iter):
        gradient = np.zeros_like(waypoints_smooth)
        
        # Calculate gradient for smoothness term
        for i in range(1, n - 1):
            gradient[i] += 2 * (waypoints_smooth[i] - waypoints_smooth[i - 1])
            gradient[i] += 2 * (waypoints_smooth[i] - waypoints_smooth[i + 1])
        
        # Calculate gradient for waypoint fidelity term
        gradient += 2 * lambda_reg * (waypoints_smooth - waypoints)
        
        # Update waypoints
        waypoints_smooth -= alpha * gradient
        
        # Check for convergence
        if np.linalg.norm(gradient) < tol:
            print(f'Converged after {iter + 1} iterations')
            break
    
    return waypoints_smooth

# B-spline smoothing function
def smooth_path_bspline(waypoints, degree=3):
    """
    Smooth the path using B-spline.
    
    Parameters:
        waypoints (np.ndarray): The original waypoints.
        degree (int): The degree of the B-spline.
    
    Returns:
        np.ndarray: Smoothed waypoints using B-spline.
    """
    t = np.linspace(0, 1, len(waypoints))
    tck, u = splprep([waypoints[:, 0], waypoints[:, 1], waypoints[:, 2]], k=degree, s=0)
    u_fine = np.linspace(0, 1, 1000)
    x_fine, y_fine, z_fine = splev(u_fine, tck)
    
    smooth_path = np.vstack((x_fine, y_fine, z_fine)).T
    return smooth_path

def minimum_snap_trajectory_1d(waypoints_1d, times, order=7, max_iter=1000):
    """
    Generate a minimum snap trajectory in 1D.
    
    Parameters:
    - waypoints_1d: List of waypoints [x1, x2, ..., xn]
    - times: List of times [t1, t2, ..., tn] at each waypoint
    - order: Order of the polynomial (default is 7 for minimum snap)
    - max_iter: Maximum number of iterations for the optimizer (default is 1000)
    
    Returns:
    - coefficients: List of polynomial coefficients for each segment
    """
    n_segments = len(waypoints_1d) - 1
    n_coeffs = order + 1

    def cost_function(coeffs):
        # Calculate snap (fourth derivative) for each segment
        snap_cost = 0
        for i in range(n_segments):
            segment_coeffs = coeffs[i*n_coeffs:(i+1)*n_coeffs]
            snap = np.polyder(segment_coeffs, 4)
            snap_cost += np.sum(snap**2)
        return snap_cost

    # Initial guess for polynomial coefficients
    coeffs_guess = np.zeros(n_segments * n_coeffs)

    # Constraints: waypoints matching and continuity at joints
    constraints = []
    for i in range(n_segments):
        t0 = times[i]
        t1 = times[i+1]
        constraints.append({
            'type': 'eq',
            'fun': lambda coeffs, i=i, t=t0: np.polyval(coeffs[i*n_coeffs:(i+1)*n_coeffs], t) - waypoints_1d[i]
        })
        constraints.append({
            'type': 'eq',
            'fun': lambda coeffs, i=i, t=t1: np.polyval(coeffs[i*n_coeffs:(i+1)*n_coeffs], t) - waypoints_1d[i+1]
        })

    # Solve the optimization problem
    result = minimize(cost_function, coeffs_guess, constraints=constraints, method='SLSQP', options={'maxiter': max_iter})

    if not result.success:
        raise ValueError("Optimization failed: " + result.message)

    coefficients = result.x.reshape((n_segments, n_coeffs))
    return coefficients

def minimum_snap_trajectory_3d(waypoints, total_time, order=7, max_iter=1000):
    """
    Generate a minimum snap trajectory through the given waypoints in 3D.
    
    Parameters:
    - waypoints: List of waypoints [[x1, y1, z1], [x2, y2, z2], ..., [xn, yn, zn]]
    - total_time: Total time for the entire trajectory
    - order: Order of the polynomial (default is 7 for minimum snap)
    - max_iter: Maximum number of iterations for the optimizer (default is 1000)
    
    Returns:
    - coefficients: List of polynomial coefficients for each segment and dimension
    - times: List of times at each waypoint
    """
    num_waypoints = len(waypoints)
    times = np.linspace(0, total_time, num_waypoints)

    waypoints = np.array(waypoints)
    waypoints_x = waypoints[:, 0]
    waypoints_y = waypoints[:, 1]
    waypoints_z = waypoints[:, 2]

    coefficients_x = minimum_snap_trajectory_1d(waypoints_x, times, order, max_iter)
    coefficients_y = minimum_snap_trajectory_1d(waypoints_y, times, order, max_iter)
    coefficients_z = minimum_snap_trajectory_1d(waypoints_z, times, order, max_iter)

    return coefficients_x, coefficients_y, coefficients_z, times

# Function to plot the raw RRT path, spline smoothed path, and gradient descent smoothed path
def plot_paths(raw_path, smooth_path_spline, smooth_path_gd, smooth_path_bspline, start, goal, obstacles):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot raw RRT path
    ax.plot(raw_path[:, 0], raw_path[:, 1], raw_path[:, 2], '-r', label='Raw RRT Path')
    
    # Plot cubic spline smoothed path
    # ax.plot(smooth_path_spline[:, 0], smooth_path_spline[:, 1], smooth_path_spline[:, 2], '-g', label='Spline Smoothed Path')
    
    # Plot gradient descent smoothed path
    ax.plot(smooth_path_gd[:, 0], smooth_path_gd[:, 1], smooth_path_gd[:, 2], '-b', label='GD Smoothed Path')
    
    # Plot B-spline smoothed path
    # ax.plot(smooth_path_bspline[:, 0], smooth_path_bspline[:, 1], smooth_path_bspline[:, 2], '-g', label='B-Spline Smoothed Path')
    
    # Plot start and goal points
    ax.scatter(start[0], start[1], start[2], c='g', marker='o', s=100, label='Start')
    ax.scatter(goal[0], goal[1], goal[2], c='m', marker='*', s=100, label='Goal')

    # Plot obstacles as cuboids
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

    # Set the boundary
    bounds_xy = np.array([-250, 250])  # Bounds for x and y axes
    bounds_z = np.array([0, 180])  # Bounds for z axis
    ax.set_xlim(bounds_xy[0], bounds_xy[1])
    ax.set_ylim(bounds_xy[1], bounds_xy[0])
    ax.set_zlim(bounds_z[0], bounds_z[1])
    
    # Labels and legend
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.legend()
    # ax.legend(loc='upper right')
    
    # Title
    ax.set_title('RRT* Path Planning and Smoothing')
    
    plt.show()

# Function to plot the raw RRT path, minimum snap
def plot_comparison(raw_path, coefficients_x, coefficients_y, coefficients_z, times, start, goal, obstacles):
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Set the boundary
    bounds_xy = np.array([-250, 250])  # Bounds for x and y axes
    bounds_z = np.array([0, 180])  # Bounds for z axis
    ax.set_xlim(bounds_xy[0], bounds_xy[1])
    ax.set_ylim(bounds_xy[1], bounds_xy[0])
    ax.set_zlim(bounds_z[0], bounds_z[1])
    # Labels and legend
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.legend()
    # ax.legend(loc='upper right')
    # Title
    ax.set_title('RRT* Path Planning and Smoothing')
    
    # Plot raw RRT* path
    ax.plot(raw_path[:, 0], raw_path[:, 1], raw_path[:, 2], '-r', label='Raw RRT Path')

    # plot minimum snap trajectory
    t_vals = np.linspace(times[0], times[-1], 1000)
    x_vals = np.zeros_like(t_vals)
    y_vals = np.zeros_like(t_vals)
    z_vals = np.zeros_like(t_vals)

    for i in range(len(coefficients_x)):
        t_start = times[i]
        t_end = times[i+1]
        t_segment = t_vals[(t_vals >= t_start) & (t_vals <= t_end)]
        x_vals[(t_vals >= t_start) & (t_vals <= t_end)] = np.polyval(coefficients_x[i], t_segment)
        y_vals[(t_vals >= t_start) & (t_vals <= t_end)] = np.polyval(coefficients_y[i], t_segment)
        z_vals[(t_vals >= t_start) & (t_vals <= t_end)] = np.polyval(coefficients_z[i], t_segment)

    ax.plot(x_vals, y_vals, z_vals, label='Minimum Snap Trajectory')

    # Plot gradient descent smoothed path
    # ax.plot(smooth_path[:, 0], smooth_path[:, 1], smooth_path[:, 2], '-b', label='Smoothed Trajectory')
    
    # Plot start and goal points
    ax.scatter(start[0], start[1], start[2], c='g', marker='o', s=100, label='Start')
    ax.scatter(goal[0], goal[1], goal[2], c='m', marker='*', s=100, label='Goal')

    # Plot obstacles as cuboids
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

    plt.show()

################################# End of Functions ###########################################################

# Set start, goal, and obstacles for RRT,  and Run path planning algorithm
# start = np.array([0, 0, 30])  # Start location in 3D
# goal = np.array([200, 0, 0])  # Goal location in 3D
# obstacles = [
#         np.array([95, -100, 50, 90, 300, 100]),
#         np.array([250, 175, 25, 50, 150, 50]),
#     ]  # cuboids parametrized by [x, y, z, dx, dy, dz]
# waypoints, _ = pathPlanning(start, goal, obstacles)

# # Smooth the path using cubic spline
# smooth_waypoints_spline = cubic_spline_smoothing(waypoints)
# # Smooth the path using gradient descent
# smooth_waypoints_gd = gradient_descent_smoothing(waypoints)
# # Smooth the path using B-spline
# smooth_waypoints_bspline = smooth_path_bspline(waypoints)
# # Plot the raw, spline smoothed, gradient descent smoothed, and B-spline smoothed paths
# plot_paths(waypoints, smooth_waypoints_spline, smooth_waypoints_gd, smooth_waypoints_bspline, start, goal, obstacles)

# Minimium snap trajectory 
# total_time = 25  # total duration for the entire trajectory
# coefficients_x, coefficients_y, coefficients_z, times = minimum_snap_trajectory_3d(waypoints, total_time)
# times = np.linspace(0, total_time, len(waypoints))
# plot_comparison(waypoints, coefficients_x, coefficients_y, coefficients_z, times, start, goal, obstacles)




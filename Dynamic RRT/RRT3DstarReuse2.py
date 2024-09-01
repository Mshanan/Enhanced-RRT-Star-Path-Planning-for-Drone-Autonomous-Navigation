import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import plotly.graph_objects as go
from TrajectorySmoothing import gradient_descent_smoothing

def pathPlanning(start, goal, obstacles, prev_path=None):
    
    def get_random_node(goal_sample_rate, local_goal, bounds_x, bounds_y, bounds_z):
        """Sample random node inside bounds or sample goal point"""
        if np.random.rand() > goal_sample_rate:
            # Sample random point inside boundaries
            rnd = np.array([
                np.random.rand() * (bounds_x[1] - bounds_x[0]) + bounds_x[0],
                np.random.rand() * (bounds_y[1] - bounds_y[0]) + bounds_y[0],
                np.random.rand() * (bounds_z[1] - bounds_z[0]) + bounds_z[0]
            ])
        else:
            # Select goal point
            rnd = local_goal
        return rnd

    def get_nearest_node(node_array, node):
        """Find the nearest node in node_array to node"""
        diff = node_array - node
        if diff.ndim == 1:
            d_array = np.sqrt(np.sum(diff**2))
            node_nearest = node_array
        else:
            d_array = np.sqrt(np.sum(diff**2, axis=1))
            minind = np.argmin(d_array)
            node_nearest = node_array[minind, :]
        return node_nearest, minind

    def steer(from_node, to_node, max_extend_length):
        """Connects from_node to a new_node in the direction of to_node with maximum distance max_extend_length"""
        new_node = to_node
        d = to_node - from_node
        dist = np.linalg.norm(d)
        if dist > max_extend_length:
            # Rescale the path to the maximum extend_length
            new_node = from_node + d / dist * max_extend_length
        parent_node = from_node
        return new_node, parent_node

    def collision(nearest_node, new_node, obstacle_list):
        """Check whether the path connecting nearest_node and new_node is in collision with anything from the obstacle_list"""
        p1 = new_node
        p2 = nearest_node

        for o in obstacle_list:
            center_box = o[0:3]  # Get the center coordinate of the obstacle (x, y, z)
            size_box = o[3:6]  # Get the size of the obstacle (dx, dy, dz)

            # Calculate AABB (Axis-Aligned Bounding Box) of obstacle
            min_box = (center_box - size_box / 2) - 15      # 30 is safety margin
            max_box = (center_box + size_box / 2) + 15

            # Check collision between line segment (p1, p2) and AABB of obstacle
            t_min = (min_box - p1) / (p2 - p1 + 1e-10)  # Use a small value for numerical stability
            t_max = (max_box - p1) / (p2 - p1 + 1e-10)

            # Ensure t_min and t_max are ordered correctly
            t1 = np.minimum(t_min, t_max)
            t2 = np.maximum(t_min, t_max)

            t_enter = np.max(t1)
            t_exit = np.min(t2)

            if t_enter <= t_exit:
                # There is an intersection along the line segment
                return True

        return False  # No collision detected
    
    def max_turning_angle(nearest_node, new_node, parent_array):
       
        rows, _ = node_array.shape
        if rows >=2:
            AB = nearest_node - parent_array[-2, :]
            BC = new_node - nearest_node
            dot_product = np.dot(AB, BC)
            magnitude_AB = np.linalg.norm(AB)
            magnitude_BC = np.linalg.norm(BC)
            if magnitude_AB == 0 or magnitude_BC == 0:
                return False
            # Avoid division by zero and numerical issues
            cos_theta = dot_product / (magnitude_AB * magnitude_BC)
            cos_theta = np.clip(cos_theta, -1.0, 1.0)
            theta = np.arccos(cos_theta)
            theta_degrees = np.degrees(theta)
            if 0<=abs(theta_degrees)<=20 or 80<=abs(theta_degrees)<=100 or 170<=abs(theta_degrees)<=190 or 260<=abs(theta_degrees)<=280:
                return False
            else:
                return True
        else:
            return False
        
    def ground_clearance(new_node, node_array, local_goal, max_extend_length):
        rows, _ = node_array.shape
        if rows > 2 and dist_to_goal(new_node, local_goal) >= max_extend_length + 20:
        # if dist_to_goal(node_array[-1], goal) >= 10:
            if new_node[2] <= 20:
                return True
        else:
            return False
        # return False
        
    def dist_to_goal(p, local_goal):
        """Distance from p to goal"""
        return np.linalg.norm(p - local_goal)

    def search_node(node_array, pos):
        """Find index of node in node_array that is very close to pos"""
        for indx_node in range(np.shape(node_array)[0]):
            node = node_array[indx_node, :]
            if np.linalg.norm(node - pos) < 1e-5:
                return indx_node
        return None

    def validate_remaining_path(prev_path, obstacles):
        """Validate previous path from new obstacles"""
        local_goal = None
        local_start = None
        indices = []
        not_valid = []

        # Initialize the validation array with the same number of rows as prev_path
        valid_remaining_path = prev_path

        # if there is a collision, remove invalid node
        for i in range(prev_path.shape[0]-1):
            if collision(prev_path[i, :], prev_path[i+1, :], obstacles):
                not_valid.append(i)
        
        valid_remaining_path = np.delete(valid_remaining_path, not_valid, axis=0)
        valid_remaining_path = np.delete(valid_remaining_path, 0, axis=0)

        for i in range(valid_remaining_path.shape[0]-1):
            if np.linalg.norm(valid_remaining_path[i+1, :] - valid_remaining_path[i, :])>100:
                local_start = valid_remaining_path[i+1,:]
                local_goal = valid_remaining_path[i-2,:]

        for i in range(prev_path.shape[0]):
            if np.array_equal(prev_path[i, :],local_start):
                indices.append(i+1)
            if np.array_equal(prev_path[i, :], local_goal):
                indices.append(i-2)

        if local_start is None or local_goal is None:
            local_start = prev_path[-1,:]
            local_goal = prev_path[0,:]
            
        print(valid_remaining_path)
        print(local_start)
        print(local_goal)

        return local_start, local_goal, prev_path, indices     # Return the index of the last valid node
    
    def find_path(node_array, parent_array, local_start, max_iter, prev_path=None, indices=None):
        """Trace back from the goal node to the start node to get the final path"""
        node = node_array[-1, :]
        node_parent = parent_array[-1, :]
        path_array = node
        
        count = 0
        while (np.linalg.norm(node - local_start) > 1e-5) and (count < max_iter):
            count += 1
            idx_parent = search_node(node_array, node_parent)
            node = node_array[idx_parent, :]
            node_parent = parent_array[idx_parent, :]

            path_array = np.vstack([path_array, node])
        
        # Path merging
        if prev_path is not None:
            prev_path = np.delete(prev_path, np.arange(indices[0],indices[1]+1), axis=0)
            new_path = np.insert(prev_path, indices[0], path_array, axis=0)
            return path_array, new_path
        else:
            return path_array, None

    def get_near_nodes(node_array, new_node, radius):
        """Get indices of nodes within a given radius from new_node"""
        dists = np.linalg.norm(node_array - new_node, axis=1)
        near_indices = np.where(dists <= radius)[0]
        return near_indices

    def calc_new_cost(from_node, to_node, cost_array):
        """Calculate the new cost to reach to_node from from_node"""
        from_node_index = search_node(node_array, from_node)
        cost = cost_array[from_node_index] + np.linalg.norm(to_node - from_node)
        return cost

    def rewire(new_node, node_array, parent_array, cost_array, obstacles):
        """Rewire the tree if a shorter path is found through new_node"""
        near_indices = get_near_nodes(node_array, new_node, search_radius)
        for i in near_indices:
            near_node = node_array[i, :]
            if not collision(new_node, near_node, obstacles) and not max_turning_angle(nearest_node, new_node, parent_array) and not ground_clearance(new_node, node_array, local_goal, max_extend_length):
                new_cost = calc_new_cost(new_node, near_node, cost_array)
                if new_cost < cost_array[i]:
                    parent_array[i, :] = new_node
                    cost_array[i] = new_cost

    ############################## RRT* 3D Path Planning ############################################

    # Set the boundary
    bounds_x = np.array([-500, 500])  # Bounds for x and y axes
    bounds_y = np.array([-500, 500])  # Bounds for x and y axes
    bounds_z = np.array([0, 200])  # Bounds for z axis

    # RRT parameters
    max_extend_length = 25  # The maximal length extended from the nearest node toward the sampled node
    goal_sample_rate = 0.05  # Probability of the sample to be the goal: greediness of the planning toward goal
    max_iter = 3000  # Maximal number of sample points
    search_radius = 90  # Radius for rewiring

    # Get valid parts of previous path, and identify local area to be replanned
    if prev_path is None:
        local_start = start
        local_goal = goal
        indices = None
    elif prev_path is not None:
        local_start, local_goal, prev_path, indices = validate_remaining_path(prev_path, obstacles)

    node_array = local_start.reshape(1, 3)
    parent_array = local_goal.reshape(1, 3)
    cost_array = np.array([0])  # Cost to reach the start node is zero
    
    ID =  10
    np.random.seed(ID)  

    total_nodes_generated = 0  # Counter for total nodes generated

    for i in range(max_iter):
        rnd_node = get_random_node(goal_sample_rate, local_goal, bounds_x, bounds_y, bounds_z)
        nearest_node, nearest_ind = get_nearest_node(node_array, rnd_node)
        new_node, parent_node = steer(nearest_node, rnd_node, max_extend_length)

        if not collision(nearest_node, new_node, obstacles) and not max_turning_angle(nearest_node, new_node, parent_array) and not ground_clearance(new_node, node_array, local_goal, max_extend_length):
            node_array = np.vstack([node_array, new_node])
            parent_array = np.vstack([parent_array, nearest_node])
            cost = cost_array[nearest_ind] + np.linalg.norm(new_node - nearest_node)
            cost_array = np.append(cost_array, cost)
            rewire(new_node, node_array, parent_array, cost_array, obstacles)
            total_nodes_generated += 1

        if dist_to_goal(node_array[-1], local_goal) < max_extend_length:
            if not collision(node_array[-1], local_goal, obstacles) and not max_turning_angle(nearest_node, new_node, parent_array):
                node_array = np.vstack([node_array, local_goal])
                parent_array = np.vstack([parent_array, node_array[-2]])
                cost = cost_array[-1] + np.linalg.norm(node_array[-1] - node_array[-2])
                cost_array = np.append(cost_array, cost)
                print('Path planning finished.')
                break

    if i == max_iter-1:
        print('Cannot find a suitable path.')

    # Trace back from the goal node to the start node to get the final path
    path_array, full = find_path(node_array, parent_array, local_start, max_iter, prev_path, indices)
    total_path_length = np.sum(np.linalg.norm(np.diff(path_array, axis=0), axis=1))
    num_nodes_in_path = path_array.shape[0]
    path_array_new = path_array[:-1]
    path_array_new = np.flip(path_array_new, axis=0)

    print("Improved RRT")
    print(f"Total nodes generated: {total_nodes_generated}")
    print(f"Total path length: {total_path_length}")
    print(f"Number of nodes in path: {num_nodes_in_path}")

    return path_array, path_array_new, full

############################## End of RRT* algorithm ###################################################



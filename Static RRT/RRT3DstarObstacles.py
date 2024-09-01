import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import plotly.graph_objects as go
from TrajectorySmoothing import gradient_descent_smoothing

def pathPlanning(start, goal, obstacles):
    
    def get_random_node(goal_sample_rate, goal, bounds_xy, bounds_z):
        """Sample random node inside bounds or sample goal point"""
        if np.random.rand() > goal_sample_rate:
            # Sample random point inside boundaries
            rnd = np.array([
                np.random.rand() * (bounds_xy[1] - bounds_xy[0]) + bounds_xy[0],
                np.random.rand() * (bounds_xy[1] - bounds_xy[0]) + bounds_xy[0],
                np.random.rand() * (bounds_z[1] - bounds_z[0]) + bounds_z[0]
            ])
        else:
            # Select goal point
            rnd = goal
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
            min_box = (center_box - size_box / 2)       # 30 is safety margin
            max_box = (center_box + size_box / 2) 

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
        
    def dist_to_goal(p, goal):
        """Distance from p to goal"""
        return np.linalg.norm(p - goal)

    def search_node(node_array, pos):
        """Find index of node in node_array that is very close to pos"""
        for indx_node in range(np.shape(node_array)[0]):
            node = node_array[indx_node, :]
            if np.linalg.norm(node - pos) < 1e-5:
                return indx_node
        return None

    def find_path(node_array, parent_array, start, max_iter):
        """Trace back from the goal node to the start node to get the final path"""
        node = node_array[-1, :]
        node_parent = parent_array[-1, :]
        path_array = node

        count = 0
        while (np.linalg.norm(node - start) > 1e-5) and (count < max_iter):
            count += 1
            idx_parent = search_node(node_array, node_parent)
            node = node_array[idx_parent, :]
            node_parent = parent_array[idx_parent, :]

            path_array = np.vstack([path_array, node])

        return path_array

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
            if not collision(new_node, near_node, obstacles):
                new_cost = calc_new_cost(new_node, near_node, cost_array)
                if new_cost < cost_array[i]:
                    parent_array[i, :] = new_node
                    cost_array[i] = new_cost

    def plotting3D(start, goal, obstacles, path_array, node_array, parent_array):
        """Plot the generated RRT* tree and the final path"""
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plot the entire tree
        # for node, parent in zip(node_array, parent_array):
        #     ax.plot([node[0], parent[0]], [node[1], parent[1]], [node[2], parent[2]], "-g", linewidth=1)
        # ax.scatter(node_array[:, 0], node_array[:, 1], node_array[:, 2], c='g', marker='o', s=10, label='Nodes')

        # Plot the final path
        ax.plot(path_array[:, 0], path_array[:, 1], path_array[:, 2], "-r", label="Path", linewidth=2)
        ax.scatter(path_array[:, 0], path_array[:, 1], path_array[:, 2], "o")
        ax.scatter(start[0], start[1], start[2], c='b', marker='x', s=100, label='Start')
        ax.scatter(goal[0], goal[1], goal[2], c='r', marker='*', s=100, label='Goal')

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

        # Set bounds
        ax.set_xlim(bounds_xy[0], bounds_xy[1])
        ax.set_ylim(bounds_xy[1], bounds_xy[0])
        ax.set_zlim(bounds_z[0], bounds_z[1])

        # Label the axes
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')

        # Add a legend
        ax.legend(loc='upper left')
        # Add a title
        ax.set_title('3D RRT* Path Planning')

        plt.tight_layout()
        plt.show()

    ############################## RRT* 3D Path Planning ############################################

    # Set the boundary
    bounds_xy = np.array([-500, 500])  # Bounds for x and y axes
    bounds_z = np.array([0, 200])  # Bounds for z axis

    # RRT parameters
    max_extend_length = 25  # The maximal length extended from the nearest node toward the sampled node
    goal_sample_rate = 0.05  # Probability of the sample to be the goal: greediness of the planning toward goal
    max_iter = 1000  # Maximal number of sample points
    search_radius = 90  # Radius for rewiring

    # Initialize arrays with the start node
    node_array = start.reshape(1, 3)
    parent_array = start.reshape(1, 3)
    cost_array = np.array([0])  # Cost to reach the start node is zero
    ID =  106
    np.random.seed(ID)  

    total_nodes_generated = 0  # Counter for total nodes generated

    for i in range(max_iter):
        rnd_node = get_random_node(goal_sample_rate, goal, bounds_xy, bounds_z)
        nearest_node, nearest_ind = get_nearest_node(node_array, rnd_node)
        new_node, parent_node = steer(nearest_node, rnd_node, max_extend_length)

        if not collision(nearest_node, new_node, obstacles):
            node_array = np.vstack([node_array, new_node])
            parent_array = np.vstack([parent_array, nearest_node])
            cost = cost_array[nearest_ind] + np.linalg.norm(new_node - nearest_node)
            cost_array = np.append(cost_array, cost)
            rewire(new_node, node_array, parent_array, cost_array, obstacles)
            total_nodes_generated += 1

        if dist_to_goal(node_array[-1], goal) < max_extend_length:
            if not collision(node_array[-1], goal, obstacles):
                goal_ind = len(node_array)
                node_array = np.vstack([node_array, goal])
                parent_array = np.vstack([parent_array, node_array[-2]])
                cost = cost_array[-1] + np.linalg.norm(node_array[-1] - node_array[-2])
                cost_array = np.append(cost_array, cost)
                print('Path planning finished.')
                break

    if i == max_iter-1:
        print('Cannot find a suitable path.')

    # Trace back from the goal node to the start node to get the final path
    path_array = find_path(node_array, parent_array, start, max_iter)
    total_path_length = np.sum(np.linalg.norm(np.diff(path_array, axis=0), axis=1))
    num_nodes_in_path = path_array.shape[0]
    path_array_new = path_array[:-1]
    path_array_new = np.flip(path_array_new, axis=0)
    # Plot the entire tree and the final path
    # plotting3D(start, goal, obstacles, path_array, node_array, parent_array)
    print("RRT with Obstacle Avoidance")
    print(f"Total nodes generated: {total_nodes_generated}")
    print(f"Total path length: {total_path_length}")
    print(f"Number of nodes in path: {num_nodes_in_path}")

    return path_array, path_array_new

############################## End of RRT* algorithm ###################################################

# Kitchen counter
# start = np.array([0, 0, 30])  # Start location in 3D
# goal = np.array([200, 0, 0])  # Goal location in 3D
# obstacles = [   
#         np.array([95, -100, 50, 90, 300, 100]),
#         np.array([250, 175, 25, 50, 150, 50]),
#     ]  # cuboids parametrized by [x, y, z, dx, dy, dz]
# _, path = pathPlanning(start, goal, obstacles)

# maze
# start = np.array([-300, -300, 0])  # Start location in 3D
# goal = np.array([-300, 300, 0])  # Goal location in 3D 
# obstacles = [
#     np.array([-50, -200, 100, 500, 50, 200]),
#     np.array([50, 0, 100, 500, 50, 200]),
#     np.array([-50, 200, 100, 500, 50, 200]),
# ]
# _, path = pathPlanning(start, goal, obstacles)

# start = np.array([0, 0, 100])  # Start location in 3D
# goal = np.array([100, 0, 0])  # Goal location in 3D
# obstacles = [
#         # np.array([60, 10, 20, 40, 200, 40])
#     ]  # cuboids parametrized by [x, y, z, dx, dy, dz]
# _, path = pathPlanning(start, goal, obstacles)

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
# path1, _ = pathPlanning(start, goal, obstacles)
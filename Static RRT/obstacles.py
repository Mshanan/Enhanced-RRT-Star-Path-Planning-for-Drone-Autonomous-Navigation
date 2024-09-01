import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


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

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-500, 500])
ax.set_ylim([-500, 500])
ax.set_zlim([0, 200])
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
ax.legend(loc='upper right')
ax.set_title('Improved 3D RRT* Path Planning')


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
            ax.add_collection3d(Poly3DCollection(faces, facecolors='khaki', linewidths=1.0, edgecolors='black', alpha=0.15))

plt.tight_layout()
plt.show()
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Load the PCD file
pcd = o3d.io.read_point_cloud("linear_scan.ply")

# Fit a plane
plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
[a, b, c, d] = plane_model

# Get the array of point coordinates
points = np.asarray(pcd.points)

# Calculate the centroid
centroid = np.mean(points, axis=0)

# Create a Matplotlib 3D figure

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the point cloud with a blue color and label
ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='b', marker='.', label='Point Cloud')

# Plot the fitted plane
xx, yy = np.meshgrid(np.linspace(min(points[:, 0]), max(points[:, 0]), 50),
                     np.linspace(min(points[:, 1]), max(points[:, 1]), 50))
zz = (-a * xx - b * yy - d) / c
ax.plot_surface(xx, yy, zz, color='g', alpha=0.5)

# Plot the centroid as a red point
ax.scatter(centroid[0], centroid[1], centroid[2], c='r', marker='o', s=100, label='Centroid')

# Set labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Create a legend
ax.legend()

# Show the plot
plt.show()

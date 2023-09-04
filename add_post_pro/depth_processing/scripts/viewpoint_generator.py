from stl import mesh
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Load the STL file
your_mesh = mesh.Mesh.from_file('../saved_models/cube.stl')

# Create a new plot
figure = plt.figure()
axes = figure.add_subplot(111, projection='3d')

# Get the vertices and reshape the array
vertices = your_mesh.vectors.reshape(-1, 3)

# Get the individual x, y, z coordinates
x = vertices[:, 0]
y = vertices[:, 1]
z = vertices[:, 2]

# Create an array with shape Nx3
triangles = np.array([[i, i+1, i+2] for i in range(0, len(vertices), 3)])

# Plot the mesh using plot_trisurf
axes.plot_trisurf(x, y, z, triangles=triangles)

# Show the plot
plt.show()

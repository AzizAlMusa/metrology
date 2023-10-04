import trimesh
import numpy as np
# # Load the STL file
# mesh = trimesh.load_mesh('../saved_models/bracket.stl')

# # Find the bounds of the mesh
# min_bound = mesh.bounds[0]
# max_bound = mesh.bounds[1]

# # Calculate the center of the bounding box
# center = (min_bound + max_bound) / 2

# # Translate the mesh to the origin 
# mesh.apply_translation(-center)

# # Find the minimum z-coordinate to translate the bottom to the x-y plane
# min_z = mesh.bounds[0][2]

# # Translate the mesh so that its bottom-most point lies on the x-y plane
# mesh.apply_translation([0, 0, -min_z])

# # Save the modified mesh
# mesh.export('../saved_models/modified_bracket.stl')



# Load the STL file
mesh = trimesh.load_mesh('../saved_models/pump.stl')

# Step 5: Rotate the mesh around the y-axis by 90 degrees
rotation_matrix = trimesh.transformations.rotation_matrix(np.radians(180), [1, 0, 0])
mesh.apply_transform(rotation_matrix)


# Step 1: Translate the mesh so its centroid is at the origin
centroid = mesh.centroid
mesh.apply_translation(-centroid)

# Step 2: The object's centroid-origin is now at the world (0, 0, 0)


# Step 3: Translate the mesh so that its bottom-most point lies on the x-y plane
min_z = mesh.bounds[0][2]
mesh.apply_translation([0, 0, -min_z])



# Step 4: Scale it down
# mesh.apply_scale(0.003)
# Desired box size in meters (change this to your desired size)
box_size = 0.10  # For a 10 cm box, or change to 0.15 for a 15 cm box

# Calculate the maximum dimension of the mesh
max_dimension = max(mesh.extents)

# Calculate the scale factor to fit within the specified box size
scale_factor = box_size / max_dimension

# Apply the scale factor uniformly to the mesh
mesh.apply_scale(scale_factor)





# Save the modified mesh
mesh.export('../saved_models/modified_pump.stl')


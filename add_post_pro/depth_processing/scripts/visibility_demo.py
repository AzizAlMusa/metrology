import numpy as np
import open3d as o3d
import math

# Load the 3D mesh (replace 'mesh_file.obj' with your mesh file)
mesh = o3d.io.read_triangle_mesh('../saved_models/cube.stl')

# Create a 3D visualization window
vis = o3d.visualization.Visualizer()
vis.create_window()

# Add the mesh to the visualization
vis.add_geometry(mesh)

# Define demonstration viewpoints (position and orientation)
viewpoints = [
    {"position": [0, 0, 1], "rotation": [0, 0, 0]},  # Example viewpoint 1
    {"position": [1, 0, 1], "rotation": [0, math.radians(45), 0]},  # Example viewpoint 2
    # Add more viewpoints as needed
]

# Set up camera parameters (e.g., focal length, sensor size)
camera_params = o3d.camera.PinholeCameraParameters()
# Customize camera parameters here

# Define pyramid parameters
pyramid_length = 0.51
pyramid_width = 0.53
pyramid_height = 1.1

# Define pyramid vertices
pyramid_vertices = [
    [0, 0, 0],                                      # Apex of the pyramid (centered at the camera position)
    [-pyramid_width / 2, -pyramid_length / 2, 0],   # Base vertex 1
    [pyramid_width / 2, -pyramid_length / 2, 0],    # Base vertex 2
    [pyramid_width / 2, pyramid_length / 2, 0],     # Base vertex 3
    [-pyramid_width / 2, pyramid_length / 2, 0],    # Base vertex 4
    [0, 0, -pyramid_height]                         # Tip of the pyramid
]

# Define pyramid faces (indices of vertices)
pyramid_faces = [
    [0, 1, 2],
    [0, 2, 3],
    [0, 3, 4],
    [0, 4, 1],
    [1, 2, 5],
    [2, 3, 5],
    [3, 4, 5],
    [4, 1, 5]
]

# Iterate through viewpoints
for viewpoint in viewpoints:
    # Create a transformation matrix combining translation and rotation
    custom_extrinsic = np.eye(4)
    custom_extrinsic[:3, 3] = viewpoint["position"]
    custom_extrinsic[:3, :3] = o3d.geometry.get_rotation_matrix_from_xyz(viewpoint["rotation"])

    # Set the custom extrinsic parameters
    camera_params.extrinsic = custom_extrinsic

    # Set the intrinsic parameters
    camera_params.intrinsic.set_intrinsics(
        1280, 720,  # Example resolution (adjust as needed)
        1000.0,     # Example focal length in pixels (adjust as needed)
        1000.0,     # Example focal length in pixels (adjust as needed)
        640.0,      # Example principal point (adjust as needed)
        360.0       # Example principal point (adjust as needed)
    )

    # Create a visual representation of the camera with FOV as a pyramid
    pyramid = o3d.geometry.TriangleMesh()
    pyramid.vertices = o3d.utility.Vector3dVector(pyramid_vertices)
    pyramid.triangles = o3d.utility.Vector3iVector(pyramid_faces)
    pyramid.compute_vertex_normals()
    pyramid.transform(custom_extrinsic)

    # Render the scene with the camera pyramid
    vis.add_geometry(pyramid)

    # Perform ray casting and intersection tests
    # Implement ray-mesh intersection checks here
    # Highlight intersection points on the mesh

    # Update the visualization window
    vis.update_renderer()

    # Capture the visualization frame (optional)
    vis.capture_screen_image(f"viewpoint_{viewpoint}.png")

# Start the visualization loop (optional)
vis.run()
vis.destroy_window()

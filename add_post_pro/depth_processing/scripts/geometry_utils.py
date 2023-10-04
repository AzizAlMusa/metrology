import open3d as o3d
import numpy as np

def create_grid(levels_x, levels_y, line_width=5.0):
    # Create a list to store LineSet geometries
    lines = []

    # Calculate the number of lines and spacing based on levels for both x and y directions
    num_lines_x = 2 * levels_x + 1
    spacing_x = 1.0

    num_lines_y = 2 * levels_y + 1
    spacing_y = 1.0

    # Create lines along the x-axis
    for i in range(num_lines_x):
        x = (i - levels_x) * spacing_x
        start_x = np.array([x, -20, 0])
        end_x = np.array([x, 20, 0])
        line_x = o3d.geometry.LineSet()
        line_x.points = o3d.utility.Vector3dVector(np.vstack((start_x, end_x)))
        line_x.lines = o3d.utility.Vector2iVector([[0, 1]])
        line_x.colors = o3d.utility.Vector3dVector(np.tile([0.8, 0.8, 0.8], (1, 1)))
        lines.append(line_x)

    # Create lines along the y-axis
    for i in range(num_lines_y):
        y = (i - levels_y) * spacing_y
        start_y = np.array([-20, y, 0])
        end_y = np.array([20, y, 0])
        line_y = o3d.geometry.LineSet()
        line_y.points = o3d.utility.Vector3dVector(np.vstack((start_y, end_y)))
        line_y.lines = o3d.utility.Vector2iVector([[0, 1]])
        line_y.colors = o3d.utility.Vector3dVector(np.tile([0.8, 0.8, 0.8], (1, 1)))
        lines.append(line_y)

    # Combine all line sets into a single LineSet called "grid"
    grid = o3d.geometry.LineSet()
    for line_set in lines:
        grid += line_set

    return grid

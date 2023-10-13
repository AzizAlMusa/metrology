import numpy as np
import trimesh


def sample_points_from_mesh(mesh, num_points):
    face_areas = mesh.area_faces
    face_probs = face_areas / face_areas.sum()
    sampled_face_idxs = np.random.choice(len(mesh.faces), size=num_points, p=face_probs)
    u = np.random.rand(num_points, 1)
    v = np.random.rand(num_points, 1)
    w = 1 - (u + v)
    sampled_faces = mesh.faces[sampled_face_idxs]
    sampled_vertices = mesh.vertices[sampled_faces]
    points = u * sampled_vertices[:, 0, :] + v * sampled_vertices[:, 1, :] + w * sampled_vertices[:, 2, :]
    return points

def save_points_as_pcd(points, filename):
    with open(filename, 'w') as f:
        f.write("# .PCD v.7 - Point Cloud Data file format\n")
        f.write("VERSION .7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write("WIDTH {}\n".format(len(points)))
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write("POINTS {}\n".format(len(points)))
        f.write("DATA ascii\n")
        for point in points:
            f.write(" ".join(map(str, point)) + "\n")

# Load mesh from STL file
mesh = trimesh.load_mesh("./part.ply")

# Sample 5000 points
sampled_points = sample_points_from_mesh(mesh, 50000)

# Save sampled points as a PCD file
save_points_as_pcd(sampled_points, "./part.pcd")


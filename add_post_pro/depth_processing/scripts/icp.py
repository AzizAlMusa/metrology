import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import pdb

def find_nearest_neighbors(source, target):
    distances = np.sum((source[:, :, np.newaxis] - target.T[np.newaxis, :, :]) ** 2, axis=1)
    nearest_indices = np.argmin(distances, axis=1)
    return target[nearest_indices]

def compute_best_fit_transform(A, B):
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    centered_A = A - centroid_A
    centered_B = B - centroid_B

    H = np.dot(centered_A.T, centered_B)

    U, _, Vt = np.linalg.svd(H)

    R = np.dot(Vt.T, U.T)
    translation = centroid_B - np.dot(R, centroid_A)

    return R, translation


def icp_step(source, target, final_R, final_t):
    closest_points = find_nearest_neighbors(source, target)
    R, t = compute_best_fit_transform(source, closest_points)
    source = np.dot(R, source.T).T + t
    final_R = np.dot(final_R, R)
    final_t = final_t + t
    return source, final_R, final_t

def icp(source, target, num_iterations):
    final_R = np.eye(3)
    final_t = np.zeros(3)

    for i in range(num_iterations):
        source, final_R, final_t = icp_step(source, target, final_R, final_t)

    return final_R, final_t


def icp_animation(source, target, max_iterations=100):
    final_R = np.eye(3)
    final_t = np.zeros(3)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    scat_source = ax.scatter(source[:, 0], source[:, 1], source[:, 2], c='r', label='Source', s=2)
    scat_target = ax.scatter(target[:, 0], target[:, 1], target[:, 2], c='b', label='Target', s=2)

    def update(i):
        nonlocal source, final_R, final_t
        source, final_R, final_t = icp_step(source, target, final_R, final_t)
        scat_source._offsets3d = (source[:, 0], source[:, 1], source[:, 2])
        return scat_source,

    ani = FuncAnimation(fig, update, frames=range(max_iterations), blit=False)

    plt.legend()
    ax.set_xlim3d([-2, 2])
    ax.set_ylim3d([-2, 2])
    ax.set_zlim3d([-2, 2])
    plt.title("ICP Animation")
    plt.show()

    return final_R, final_t

def visualize_registration(title, source=None, target=None, transformed_source=None):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    if source is not None:
        ax.scatter(source[:, 0], source[:, 1], source[:, 2], c='r', label='Source', s=6)
    if target is not None:
        ax.scatter(target[:, 0], target[:, 1], target[:, 2], c='b', label='Target', s=5)
    if transformed_source is not None:
        ax.scatter(transformed_source[:, 0], transformed_source[:, 1], transformed_source[:, 2], c='g', label='Transformed Source', s=5)
    
    ax.legend()
    plt.title(title)
    plt.show()


NUM_OF_POINTS = 1000

# Initilize some random source & target (could be anything else)
source = np.random.rand(NUM_OF_POINTS, 3) * 0.5
target = np.random.rand(NUM_OF_POINTS, 3) * 0.5

# Introduce a rotation and a translation to test the ICP
rotation_matrix = np.array([[0.866, -0.5, 0],
                            [0.5, 0.866, 0],
                            [0, 0, 1]])

translation_vector = np.array([0.2, 0.3, 0.4])
source = np.dot(rotation_matrix, target.T).T + translation_vector

# Animate
#R, t = icp_animation(source, target, max_iterations=50)

# ICP
R, t = icp(source, target, num_iterations=1000)


#Calculate difference

# Combine reference components into a transformation matrix
T_ref = np.eye(4)
T_ref[:3, :3] = rotation_matrix
T_ref[:3, 3] = translation_vector

# Combine registration components into a transformation matrix
T_reg = np.eye(4)
T_reg[:3, :3] = R
T_reg[:3, 3] = t

error_frobenius = np.linalg.norm(T_ref - T_reg, 'fro')

print("The frobenius error = ", error_frobenius)

transformed_source = np.dot(R, source.T).T + t


visualize_registration("things and items", source, target, transformed_source)
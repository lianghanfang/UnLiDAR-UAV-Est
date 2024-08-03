import numpy as np
from scipy.spatial.transform import Rotation
import os
import numpy as np
from sklearn.neighbors import NearestNeighbors

def estimate_motion(source_points, target_points):
    if len(target_points) == 0:
        return None, None

    nn = NearestNeighbors(n_neighbors=1, algorithm='auto').fit(target_points)
    distances, indices = nn.kneighbors(source_points)

    translation_vector = np.mean(target_points[indices][:, 0] - source_points, axis=0)

    rotation_matrix = Rotation.from_matrix(np.eye(3))

    return translation_vector, rotation_matrix


def is_moving(translation_vector, rotation_matrix, motion_threshold=0.01):
    if translation_vector is None or rotation_matrix is None:
        return False

    translation_magnitude = np.linalg.norm(translation_vector)
    rotation_condition_number = np.linalg.cond(rotation_matrix.as_matrix())

    if translation_magnitude > motion_threshold or rotation_condition_number > motion_threshold:
        return True
    else:
        return False


def filter_static_objects(frames):
    num_frames = len(frames)
    moving_objects = []

    for i in range(num_frames - 1):
        source_points = frames[i]
        target_points = frames[i + 1]

        translation_vector, rotation_matrix = estimate_motion(source_points, target_points)

        if is_moving(translation_vector, rotation_matrix):
            moving_objects.append(target_points)

    return moving_objects


def read_xyz_file(filepath):
    points = []
    with open(filepath, 'r') as file:
        for line in file:
            if line.strip():
                coordinates = list(map(float, line.strip().split()))
                points.append(coordinates)
    return np.array(points)


def read_point_cloud_frames(folder_path, frame_size=1):
    frame_list = []
    file_list = sorted([f for f in os.listdir(folder_path) if f.endswith('.xyz')])

    for i in range(0, len(file_list), frame_size):
        frame_points = []
        for j in range(frame_size):
            if i + j < len(file_list):
                filepath = os.path.join(folder_path, file_list[i + j])
                points = read_xyz_file(filepath)
                frame_points.extend(points)
        frame_list.append(np.array(frame_points))

    return frame_list

def save_point_cloud(filename, points):
    with open(filename, 'w') as f:
        for point in points:
            f.write(f"{point[0]} {point[1]} {point[2]}\n")

def save_point_clouds(output_folder, moving_objects):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    for i, points in enumerate(moving_objects):
        filename = os.path.join(output_folder, f"moving_object_{i}.xyz")
        save_point_cloud(filename, points)

folder_path = r"E:\project\dataset\Anti-UAV\seq54\avia_split_files"
frames = read_point_cloud_frames(folder_path, frame_size=1)

moving_objects = filter_static_objects(frames)

output_folder = r"E:\project\dataset\Anti-UAV\seq54\moving"

save_point_clouds(output_folder, moving_objects)
print(moving_objects)

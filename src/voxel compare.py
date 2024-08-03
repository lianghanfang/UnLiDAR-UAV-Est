import numpy as np
import open3d as o3d
import os


sequence = 37
file_root = "E:/project/dataset/Anti-UAV/seq" + str(sequence)

# def points_in_same_voxel(pcd1, pcd2, voxel_size):
#     voxel_down_pcd1 = pcd1.voxel_down_sample(voxel_size)
#     voxel_down_pcd2 = pcd2.voxel_down_sample(voxel_size)
#
#     points1 = np.asarray(voxel_down_pcd1.points)
#     points2 = np.asarray(voxel_down_pcd2.points)
#
#     unique_points1_indices = np.setdiff1d(np.arange(len(points1)), np.arange(len(points2)))
#     unique_points2_indices = np.setdiff1d(np.arange(len(points2)), np.arange(len(points1)))
#
#     unique_points1 = points1[unique_points1_indices]
#     unique_points2 = points2[unique_points2_indices]
#
#     return unique_points1, unique_points2
#
# pcd1 = o3d.io.read_point_cloud(os.path.join(file_root, "output_0.xyz"))
# pcd2 = o3d.io.read_point_cloud(os.path.join(file_root, "output_1.xyz"))
#
# voxel_size = 0.3
#
# unique_points1, unique_points2 = points_in_same_voxel(pcd1, pcd2, voxel_size)
#
# unique_pcd1 = o3d.geometry.PointCloud()
# unique_pcd1.points = o3d.utility.Vector3dVector(unique_points1)
# o3d.io.write_point_cloud(os.path.join(file_root, "unique_points1.ply"), unique_pcd1)
#
# unique_pcd2 = o3d.geometry.PointCloud()
# unique_pcd2.points = o3d.utility.Vector3dVector(unique_points2)
# o3d.io.write_point_cloud(os.path.join(file_root, "unique_points2.ply"), unique_pcd2)
#
# print("不相同的点已保存为 unique_points1.ply 和 unique_points2.ply 文件。")


def find_unique_points(pcd1, pcd2, voxel_size):
    voxel_down_pcd1 = pcd1.voxel_down_sample(voxel_size)
    voxel_down_pcd2 = pcd2.voxel_down_sample(voxel_size)

    points1 = np.asarray(voxel_down_pcd1.points)
    points2 = np.asarray(voxel_down_pcd2.points)

    points1_int = np.round(points1 / voxel_size).astype(int)
    points2_int = np.round(points2 / voxel_size).astype(int)

    points1_set = set(tuple(point) for point in points1_int)

    unique_points2_indices = [i for i, point in enumerate(points2_int) if tuple(point) not in points1_set]
    unique_points2 = points2[unique_points2_indices]

    return unique_points2


# pcd1 = o3d.io.read_point_cloud(os.path.join(file_root, "output_1.xyz"))
# pcd2 = o3d.io.read_point_cloud(os.path.join(file_root, "output_3.xyz"))
pcd1 = o3d.io.read_point_cloud(os.path.join(file_root, "unique_points01.ply"))
pcd2 = o3d.io.read_point_cloud(os.path.join(file_root, "unique_points12.ply"))


voxel_size = 2

unique_points2 = find_unique_points(pcd1, pcd2, voxel_size)

if len(unique_points2) > 0:
    unique_pcd2 = o3d.geometry.PointCloud()
    unique_pcd2.points = o3d.utility.Vector3dVector(unique_points2)
    o3d.io.write_point_cloud(os.path.join(file_root, "unique_points01-12.ply"), unique_pcd2)
    print(f"已保存 unique_points2.ply 文件，共有 {len(unique_points2)} 个点。")
else:
    print("两个点云中的点完全相同。")
import os
import math
import torch
import numpy as np
from scipy.spatial.transform import Rotation
from SVO_utils import octree
from SVO_utils.helper import getCenterPos, pos2center



def run(path, Tocc, num_frames, sideLength = 128, minSide = 0.5, device = "cpu", enable_centers = False):
    """
    Construct the background by occupancy ratio

    Args:
        path (string): path of lidar data
        Tocc (_type_): threshold of occupancy ratio
        num_frames (_type_): total number of frames to construct background
        sideLength (int, optional): sidelength of the root voxel. Defaults to 128.
        minSide (float, optional): sidelength of the leave voxel. Defaults to 0.5.
        device (str, optional): which device the data should be put in. Defaults to "cpu".
        enable_centers (bool, optional): whether to record center information while background construction. Defaults to False.

    Returns:
        centers (torch.Tensor): centers of background
    """

    if enable_centers:
        centers_prev = torch.zeros((0, 3), dtype = torch.float, device = device)
    count_prev = torch.zeros((0, ), dtype = torch.int, device = device)
    pos_prev = torch.zeros((0, ), dtype = torch.int, device = device)

    # RAY casting
    for foldername, subfolders, filenames in os.walk(path):
        indices = torch.randperm(len(filenames))[:num_frames].int()
        filenames = [filenames[i] for i in indices.numpy()]
        for i, filename in enumerate(filenames[:num_frames]):

            file_path = os.path.join(foldername, filename)
            # print(f"file_path: {file_path}")

            points_tensor = torch.load(file_path).to(device)
            points_tensor = points_tensor[~torch.any(points_tensor.isnan(),dim=1)]
            if enable_centers:
                centers, pos = getCenterPos(points_tensor, sideLength, minSide)
                # delete repeat centers in octree
                c_unique = torch.unique(centers,  return_inverse=False, dim=0)
                centers = torch.cat((centers_prev, c_unique), dim = 0)
                c_cat, c_counts = torch.unique(centers, return_counts=True, dim=0)
            else:
                _, pos = getCenterPos(points_tensor, sideLength, minSide)

            pos_unique, pos_idx = torch.unique(pos, return_inverse=True, dim=None)
            pos = torch.cat((pos_prev, pos_unique), dim = 0).sort()[0]
            pos_cat, pos_counts = torch.unique(pos, return_counts=True, dim=0)

            counts = torch.ones((pos_cat.shape[0], ), dtype = torch.int, device = device)
            catpos_in_cur = torch.isin(pos_cat, pos_unique, assume_unique=True, invert=False)
            catpos_in_prev = torch.isin(pos_cat, pos_prev, assume_unique=True, invert=False)
            counts[catpos_in_prev] = count_prev
            counts[catpos_in_cur] += 1

            if enable_centers:
                centers_prev = c_cat
            pos_prev = pos_cat
            count_prev = counts

    mask = count_prev > num_frames * Tocc

    if enable_centers:
        _, pos = getCenterPos(centers_prev, sideLength, minSide)
        centers = centers_prev[pos.sort()[1]]
    else:
        centers = pos2center(pos_prev, sideLength, minSide)
    # print(centers)
    return centers[mask]


if __name__ == "__main__":
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu") # which device the data should be put in.
    Tocc = 0.1 # threshold of occupancy ratio
    path = "./raw_lidardata/81_lidar80" # path of lidar data
    num_frames = 190 # total number of frames to construct background
    sideLength = 128 # sidelength of the root voxel
    minSide = 0.5 # sidelength of the leave voxel

    res_points = run(path, Tocc, num_frames, sideLength, minSide, device = device).cpu()
    # save to pytorch file
    torch.save(res_points, "background.pt")
    # save to pcd file
    import open3d as o3d
    pcd_out = o3d.geometry.PointCloud()
    pcd_out.points = o3d.utility.Vector3dVector(res_points.cpu().numpy())
    # save the point cloud to a PCD file
    o3d.io.write_point_cloud("background.pcd", pcd_out)

    # plot 2D figure
    import matplotlib.pyplot as plt
    detect_range = 128
    mask = (abs(res_points[:,0]) <= detect_range) & (abs(res_points[:,1]) <= detect_range)
    res_points = res_points[mask]
    plt.scatter(res_points[:,0], res_points[:,1], s = 1, c = 'red')
    plt.savefig("./background.png")

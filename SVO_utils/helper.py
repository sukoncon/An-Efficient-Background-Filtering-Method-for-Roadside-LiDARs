import torch
import open3d as o3d
import numpy as np

def prepare_points(pcdfile):
    
    # pcdfile = "/home/suzhongling/background/raw_lidardata/81_lidar80/1659.341185000.pcd"
    pcd = o3d.io.read_point_cloud(pcdfile)

    # Convert to numpy array
    points = pcd.points
    points_np = np.asarray(points)

    # Convert to tensor
    points_tensor = torch.from_numpy(points_np).float()
    points_tensor = points_tensor[~torch.isnan(points_tensor).any(dim = 1)]
    
    return points_tensor 
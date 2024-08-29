import torch
import open3d as o3d
import numpy as np
import math

def prepare_points(pcdfile):

    pcd = o3d.io.read_point_cloud(pcdfile)

    # Convert to numpy array
    points = pcd.points
    points_np = np.asarray(points)

    # Convert to tensor
    points_tensor = torch.from_numpy(points_np).float()
    points_tensor = points_tensor[~torch.isnan(points_tensor).any(dim = 1)]

    return points_tensor

def getCenterPos(points, sideLength, minSide):
    centers = torch.zeros_like(points)
    childcenters = torch.zeros_like(points)
    pos_factor =  torch.tensor([1, 2, 4]).to(points.device)
    positions = torch.zeros((points.shape[0], ), device = points.device, dtype = torch.int64)
    factor = 1

    while sideLength >= minSide:
        sideLength /= 2

        point_center_diff = points - centers
        # calculate childcenter
        childcenter_offset = torch.where(point_center_diff < 0, -sideLength, sideLength)
        childcenters = centers + childcenter_offset
        # calculate pos
        positions += ((point_center_diff < 0) * pos_factor).sum(axis = 1) * factor ** 3
        factor *= 2

        centers = childcenters

    return centers, positions

def get_depth(sideLength, minSide):
    depth = 0
    while sideLength >= minSide:
        sideLength /= 2
        depth += 1

    return int(depth)

def pos2center(pos, sideLength, minSide):
    centers = torch.zeros((pos.shape[0], 3), device = pos.device, dtype = torch.float)
    pos_new = torch.clone(pos)
    diff_factor = torch.zeros((pos.shape[0], 3), device = pos.device, dtype = torch.int64)

    depth = get_depth(sideLength, minSide) # depth of the octree
    factor = int(2 ** (depth - 1))

    # print("depth")
    pos_factors = torch.zeros((pos.shape[0], depth), device = pos.device, dtype = torch.int64)
    # restore pos factors
    for l in range(depth-1, -1, -1):

        pos_factors[:, l] = (pos_new / (factor ** 3))
        pos_new = pos_new % (factor ** 3)
        factor = int(factor/2)

    # resotre centers
    for l in range(depth):
        sideLength /= 2
        pos_tmp = pos_factors[:, l]
        diff_factor[:, 2] = torch.where(pos_tmp > 3, -1, 1)
        pos_tmp = pos_tmp % 4
        diff_factor[:, 1] = torch.where(pos_tmp > 1, -1, 1)
        pos_tmp = pos_tmp % 2
        diff_factor[:, 0] = torch.where(pos_tmp > 0, -1, 1)
        centers += diff_factor * sideLength
    return centers


def delete_noise(points, sideLength = 128, minSide = 0.5, neighbor_dist = 4, min_neighbors = 200):
    norm = points.norm(dim = 1)
    points = points[torch.where((norm > 0) & (norm < (sideLength - minSide)/2))]

    dist = torch.cdist(points, points)
    Nneighbors = (dist < neighbor_dist).sum(dim = 1)
    mask = Nneighbors > min_neighbors
    return points[mask]

def diff_intersect(t1, t2):
    combined = torch.cat((t1, t2), dim = 0)
    uniques, counts = combined.unique(return_counts=True, dim = 0)
    difference = uniques[counts == 1]
    intersection = uniques[counts > 1]
    return difference, intersection

def save2pcd(tensor, filename):
    pcd_out = o3d.geometry.PointCloud()
    pcd_out.points = o3d.utility.Vector3dVector(tensor.cpu().numpy())
    # save the point cloud to a PCD file
    o3d.io.write_point_cloud(filename, pcd_out)

# def build_SVO(filepath, sideLength, minSide,, device = "cpu"):
#     """_summary_

#     Args:
#         filepath (str): file of the pytorch checkpoint
#         sideLength (float): sidelength of the root voxel
#         minSide (float): sidelength of the leaf voxel
#     """

#     center_tensor = torch.zeros((1, 3), dtype = torch.float32, device = device)
#     childId_tensor = -1 * torch.ones((1, 8), dtype = torch.int32, device = device)
#     is_end_tensor = torch.ones((1, 1), dtype = bool, device = device)

#     points = torch.load(filepath).to_device()
#     centers = torch.zeros_like(points)
#     positions = torch.zeros((points.shape[0], ), device = points.device, dtype = torch.int64)
#     factor = 1

#     while sideLength >= minSide:
#         sideLength /= 2

#         point_center_diff = points - centers
#         # calculate childcenter
#         childcenter_offset = torch.where(point_center_diff < 0, -sideLength, sideLength)
#         childcenters = centers + childcenter_offset
#         # calculate pos
#         pos_local = ((point_center_diff < 0) * pos_factor).sum(axis = 1)
#         pos_local_unique = torch.unique(pos_local).sort()

#         is_end_tensor_tmp = torch.ones((1, 1), dtype = bool, device = device)


#         factor *= 2



#         centers = childcenters



#     return (center_tensor, childId_tensor, is_end_tensor)

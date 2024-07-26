import numpy as np
import open3d as o3d

############# Build octree from pcd ##################

pcdfile = "./81_lidar80_background.pcd"# Convert to numpy array
sideLength = 128; minside = 0.5

############# Build SVO from octree ##################
from SVO_utils.SVO_tools import BuildSVO

SVO_cuda = BuildSVO(pcdfile, sideLength, minside)
############# Prepare points ###########################
import torch 
from SVO_utils.helper import prepare_points
pcdfile = "./rawdata.pcd"
points_tensor = prepare_points(pcdfile).cuda()
filtered_points = torch.zeros((int(points_tensor.size(0)),3)).cuda()

############# SVO filtering #############################
import SVO_filtering
import time
for i in range(10):
    torch.cuda.synchronize(); start = time.time()
    SVO_filtering.run(points_tensor, filtered_points, SVO_cuda)
    torch.cuda.synchronize(); end = time.time()
print(f"SVO filtering used time: {(end-start)*1000} ms")

########## Save result ###################################
filtered_points = filtered_points[torch.all(filtered_points != 0, dim = 1)]
print(filtered_points.shape)
pcd_out = o3d.geometry.PointCloud()
pcd_out.points = o3d.utility.Vector3dVector(filtered_points.cpu().numpy())

# save the point cloud to a PCD file
o3d.io.write_point_cloud("filtered.pcd", pcd_out)
print(f"filtered pcd is written to filterd.pcd")
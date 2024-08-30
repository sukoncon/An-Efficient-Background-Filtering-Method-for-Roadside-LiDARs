import os
import sys
from argparse import ArgumentParser, Namespace
import warnings
import numpy as np
import open3d as o3d
import torch
import SVO_filtering
import time
warnings.filterwarnings("ignore")

parser = ArgumentParser(description="Filtering parameters")
parser.add_argument("--source_path", "-s", required=True, type=str, help="Path of lidar data to be filtered")
parser.add_argument("--background", "-b", required=True, type=str, help="Path of background lidar data")
parser.add_argument("--sideLength", type=float, default=128, help="Sidelength of the root voxel")
parser.add_argument("--minSide", type=float, default=0.5, help="Sidelength of the leave voxel")
parser.add_argument("--output_path", "-o", required=True, type=str, help="Path of filtered lidar data")

args = parser.parse_args(sys.argv[1:])
os.makedirs(args.output_path, exist_ok=True)
############# Build octree from pcd ##################

pcdfile = args.background
sideLength = args.sideLength; minside = args.minSide

############# Build SVO from octree ##################
from SVO_utils.SVO_tools import BuildSVO

SVO_cuda = BuildSVO(pcdfile, sideLength, minside)

import torch
from SVO_utils.helper import delete_noise
device = torch.device("cuda" if torch.cuda.is_available() else "cpu") # which device the data should be put in.

for foldername, subfolders, filenames in os.walk(args.source_path):
    for i, filename in enumerate(filenames):
        ############# Prepare points ###########################
        pcdfile = os.path.join(foldername, filename)
        points_tensor = torch.load(pcdfile).to(device).to(torch.float)
        filtered_points = torch.zeros_like(points_tensor)

        ############# SVO filtering #############################
        torch.cuda.synchronize(); start = time.time()
        SVO_filtering.run(points_tensor, filtered_points, SVO_cuda)
        torch.cuda.synchronize(); end = time.time()

        ############# Post process #############################
        filtered_points = delete_noise(filtered_points, sideLength, minside, neighbor_dist = 4, min_neighbors = 50)

        ########## Save result ###################################
        filtered_points = filtered_points[torch.all(filtered_points != 0, dim = 1)]
        pcd_out = o3d.geometry.PointCloud()
        pcd_out.points = o3d.utility.Vector3dVector(filtered_points.cpu().numpy())

        # save the point cloud to a PCD file
        o3d.io.write_point_cloud(os.path.join(args.output_path, filename.replace(".pcd", "").replace('.pt', '.pcd')), pcd_out)
        print(f"filtered pcd is written to {os.path.join(args.output_path, filename.replace('.pcd', '').replace('.pt', '.pcd'))}")

print(f"SVO filtering of the last frame used time: {(end-start)*1000} ms")

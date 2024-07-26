import threading
import torch
from SVO_utils import octree 
import numpy as np
import open3d as o3d

def initSVO(num_voxels):
    center_tensor = torch.zeros((num_voxels, 3), dtype = torch.float32)
    childId_tensor = -1 * torch.ones((num_voxels, 8), dtype = torch.int32)
    is_end_tensor = torch.ones((num_voxels, 1), dtype = bool)  
    return (center_tensor, childId_tensor, is_end_tensor)

def Octree2SVO(octree, SVO):
    lock = threading.Lock()
    accId = 0
    def Octree2SVOiter(octree, parentId, childId_tensor, center_tensor, is_end_tensor):
        nonlocal accId, lock
        lock.acquire()

        tmp_childId = [0,0,0,0,0,0,0,0]
        if octree.sideLength <= octree.minSide: # is leaf voxel
            # print(f"minside: {octree.sideLength}")
            is_end_tensor[parentId] = True
            lock.release()
            return
        for i in range(8):
            if octree.children[i] is not None:
                accId += 1; has_child = True
                
                center = octree.children[i].center
                center_tensor[accId, 0] = float(center[0])
                center_tensor[accId, 1] = float(center[1])
                center_tensor[accId, 2] = float(center[2])
                # svo[accId, 3] = octree.children[i].sideLength
                childId_tensor[parentId, i] = accId
                tmp_childId[i] = accId
  
        lock.release()

        is_end_tensor[parentId] = False
        for i in range(8):
            if octree.children[i] is not None:
                Octree2SVOiter(octree.children[i], tmp_childId[i], childId_tensor, center_tensor, is_end_tensor)

    center_tensor = SVO[0]; childId_tensor = SVO[1]; is_end_tensor = SVO[2]
    center = octree.center
    center_tensor[accId, 0] = float(center[0])
    center_tensor[accId, 1] = float(center[1])
    center_tensor[accId, 2] = float(center[2])

    Octree2SVOiter(octree, 0, childId_tensor, center_tensor, is_end_tensor)
    return (center_tensor.cuda(), childId_tensor.cuda(), is_end_tensor.cuda())

def modifySVO(SVO, points):
    center_tensor = SVO[0]; childId_tensor = SVO[1]; is_end_tensor = SVO[2]
    for i,p in enumerate(points):
        node_idx = 0 # Begin from root
        while True:
            pos = 0
            if is_end_tensor[node_idx]:
                break
            center = center_tensor[node_idx]
            if p[2] < center[2]:
                pos += 4
            if p[1] < center[1]:
                pos +=2
            if p[0] < center[0]:
                pos+=1
            childId = childId_tensor[node_idx, pos]
            if childId < 0: # Not in the background
                print("Error")
                break
            
            node_idx = int(childId)


def filtering(SVO, points, filtered_points):
    center_tensor = SVO[0]; childId_tensor = SVO[1]; is_end_tensor = SVO[2]
    for i,p in enumerate(points):
        node_idx = 0 # Begin from root
        while True:
            pos = 0
            if is_end_tensor[node_idx]:
                break
            center = center_tensor[node_idx]
            if p[2] < center[2]:
                pos += 4
            if p[1] < center[1]:
                pos +=2
            if p[0] < center[0]:
                pos+=1
            childId = childId_tensor[node_idx, pos]
            if childId < 0: # Not in the background
                filtered_points[i] = p
                break
            
            node_idx = int(childId)

def BuildSVO(pcdfile, sideLength, minside):
    ############# Build octree from pcd ##################
    points = o3d.io.read_point_cloud(pcdfile).points
    points_np = np.asarray(points)

    my_octree = octree.Node(np.zeros((3,1)), sideLength, minside)
    # build octree from points
    for p in points_np:
        my_octree.insert(p)
        
    # get count of the voxeld
    num_voxels = octree.NodeCount(my_octree)

    ############# Build SVO from octree ##################
    from SVO_utils.SVO_tools import Octree2SVO, initSVO
    import SVO_filtering
    import torch
    SVO = initSVO(num_voxels)
    SVO_cuda = Octree2SVO(my_octree, SVO)
    return SVO_cuda
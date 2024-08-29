import numpy as np
import threading

class Node:
    def __init__(self, center: np.ndarray, sideLength: float, minSide: float):
        self.childcount = 0 # count its children voxel
        self.occupancy_flag = 0 #occupancy_flag()
        self.occupancy_count = 0
        self.id = 0 
        self.center = center # the center of the voxel
        self.sideLength = sideLength # the length of the voxel
        self.childMask = 0
        self.leafMask = 0
        
        self.hit = False # if hit by a laser ray (farest)
        self.minSide = minSide
        
        # Initially, there are no children
        self.children = [None] * 8
        
    

    def update_occupancy_count(self):
        if self.sideLength <= self.minSide:
            for i in range(8):
                if self.children[i] is not None:
                    if self.children[i].occupancy_flag > 0:
                        self.children[i].occupancy_count += 1
                        self.children[i].occupancy_flag = 0
        else:
            for i in range(8):
                if self.children[i] is not None:
                    if self.children[i].occupancy_flag > 0:
                        self.children[i].occupancy_count += 1
                        self.children[i].occupancy_flag = 0
                    self.children[i].update_occupancy_count()

    def insert(self, point):
        if self.sideLength <= self.minSide:
            return

        pos = 0
        childcenter = np.zeros(3)
        if point[2] < self.center[2]:
            pos += 4
            childcenter[2] = self.center[2] - self.sideLength / 2
        else:
            childcenter[2] = self.center[2] + self.sideLength / 2

        if point[1] < self.center[1]:
            pos += 2
            childcenter[1] = self.center[1] - self.sideLength / 2
        else:
            childcenter[1] = self.center[1] + self.sideLength / 2

        if point[0] < self.center[0]:
            pos += 1
            childcenter[0] = self.center[0] - self.sideLength / 2
        else:
            childcenter[0] = self.center[0] + self.sideLength / 2

        self.childMask |= (128 >> pos)

        if self.children[pos] is None:
            self.children[pos] = Node(childcenter, self.sideLength / 2, self.minSide)
        self.children[pos].occupancy_flag += 1
        self.children[pos].insert(point)
        return


def getBG_by_count(octree, counts):
    lock = threading.Lock()
    bg_points = []
    
    def background(octree, counts):
        nonlocal lock, bg_points
        if octree.sideLength <=  octree.minSide  and octree.occupancy_count >= counts:
            bg_points.append(octree.center)
            return
        
        for pos in range(8):
            if octree.children[pos] is not None:
                background(octree.children[pos], counts)
    background(octree, counts)
    return bg_points


def NodeCount(octree):
    lock = threading.Lock()
    accId = 0
    Idlist = []
    def OctreeNodeId(octree):
        nonlocal accId, Idlist, lock
        lock.acquire()
        tmpAcc = 0
        for i in range(8):
            if octree.children[i] is not None:
                tmpAcc += 1
                accId += 1
                Idlist.append(accId)
        lock.release()
        
        if  tmpAcc == 0:
            return
        
        for i in range(8):
            if octree.children[i] is not None:
                OctreeNodeId(octree.children[i])
    

    
    OctreeNodeId(octree)
    count = accId
    return count+1    



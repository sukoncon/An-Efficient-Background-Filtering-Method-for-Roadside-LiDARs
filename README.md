# An Efficient Background Filtering Method for Roadside LiDARs

This repository provides the official implementation of SVO-based background filtering from the following paper.

@ARTICLE{\10318073,\
  author={Su, Zhongling and Cao, Peng and Liu, Xiaobo and Tang, Yandong and Chen, Fei},\
  journal={IEEE Sensors Journal}, \
  title={An Efficient Background Filtering Method for Roadside LiDARs}, \
  year={2024},\
  volume={24},\
  number={14},\
  pages={22056-22069},\
  keywords={Laser radar;Filtering;Octrees;Point cloud compression;Three-dimensional displays;Roads;Real-time systems;Background filtering;light detection and ranging (LiDAR) data;ray casting;sparse voxel octree (SVO);traffic sensing},\
  doi={10.1109/JSEN.2023.3331120}\}

## Prerequisite
1. python package: open3d numpy torch
2. cuda toolkit

## Cloning the Repository
```
git clone git@github.com:sukoncon/An-Efficient-Background-Filtering-Method-for-Roadside-LiDARs.git SVO_BG
cd SVO_BG
```

## Installation
```
python setup.py install --user
```

## A simple case of background construction
We offer multiple frames detected by the 80-beam Lidar. Begin by extracting the files with the following command:
```
unzip -d  ./ raw_lidardata/81_lidar80_small.zip
```
Next, perform background extraction using:
```
python SVO_utils/BgExtract.py
```
You will then find the background pcd file (background.pcd) and png file (background.png).

## A simple case of filtering
We provide a background and a raw lidar data of 80-beam Lidar.
Upon executing the subsequent script, one may produce the filtered Point Cloud Data.

```
python test/filtering.py
```
![Images](images.png)
## Explanation
1. The sideLength origin and minside of the SVO should be of the same with that you use in background extraction

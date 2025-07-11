
import laspy as lp
import open3d as o3d
import numpy as np 

# |%%--%%| <gYS7FWjiHu|r9dPLxO4lZ>

path = r"/home/mahmoudahmed/testbed/arch/pointclouds/AHN/samples/33CN2_16_clip.las" 
point_cloud = lp.read(path)

point_cloud =point_cloud.points[point_cloud.classification == 1]

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.vstack((point_cloud.x, point_cloud.y, point_cloud.z)).transpose())
pcd.colors = o3d.utility.Vector3dVector(np.vstack((point_cloud.red, point_cloud.green, point_cloud.blue)).transpose()/255)

v_size = 2.5

voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size= v_size)

o3d.visualization.draw_geometries([voxel_grid])

# |%%--%%| <r9dPLxO4lZ|BZEwMN84Au>



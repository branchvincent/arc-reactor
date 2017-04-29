import pcl
import numpy as np
np_pc=np.load('pc.npy')

np_pc=np_pc.astype(np.float32)
np_pc=np.reshape(np_pc,(np_pc.size/3,3))

pointcloud=pcl.PointCloud(np_pc)
pointcloud.to_file('pointcloud.pcd')

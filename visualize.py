import argoverse
from argoverse.data_loading.argoverse_tracking_loader import ArgoverseTrackingLoader
import matplotlib
import matplotlib.pyplot as plt
from PIL import Image
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from argoverse.utils.cv2_plotting_utils import plot_bbox_polygon_cv2
import argoverse.visualization.visualization_utils as viz_util

##set root_dir to the correct path to your dataset folder
root_dir =  'argoverse-api/argoverse-tracking/sample/'

argoverse_loader = ArgoverseTrackingLoader(root_dir)
argoverse_loader.print_all()

log_id = 'c6911883-1843-3727-8eaa-41dc8cda8993'#argoverse_loader.log_list[55]
print("Enter idx that you want to visualize")
idx = input()
camera = argoverse_loader.CAMERA_LIST[0]
argoverse_data = argoverse_loader.get(log_id)
city_name = argoverse_data.city_name
print("1: Ca")
idx = input()
print(' ------------------------------------------------------------------------')
print(' ------------------------- Ring Cameras ---------------------------------')
print(' ------------------------------------------------------------------------\n')

f,ax = viz_util.make_grid_ring_camera(argoverse_data,int(idx))
plt.show()

print(' ------------------------------------------------------------------------')
print(' ------------------------- Point cloud ---------------------------------')
print(' ------------------------------------------------------------------------\n')

f2 = plt.figure(figsize=(15, 8))
ax2 = f2.add_subplot(111, projection='3d')

viz_util.draw_point_cloud(ax2, 'Lidar scan', argoverse_data, idx)
ax2.axis('off')

f3 = plt.figure(figsize=(15, 15))
ax3 = f3.add_subplot(111, projection='3d')
idx=30 # current time frame
viz_util.draw_point_cloud_trajectory(
        ax3,
        'Trajectory',
        argoverse_data,idx,axes=[0, 1],xlim3d=(-15,15),ylim3d=(-15,15) # X and Y axes
    )
plt.axis('off')
plt.show()

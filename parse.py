import argoverse
from argoverse.data_loading.argoverse_tracking_loader import ArgoverseTrackingLoader
import matplotlib
import matplotlib.pyplot as plt
from PIL import Image
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from argoverse.utils.cv2_plotting_utils import plot_bbox_polygon_cv2
import argoverse.visualization.visualization_utils as viz_util
import json

##set root_dir to the correct path to your dataset folder
root_dir =  'argoverse-api/argoverse-tracking/sample/'

argoverse_loader = ArgoverseTrackingLoader(root_dir)
# print(argoverse_loader.lidar_count)

log_id = 'c6911883-1843-3727-8eaa-41dc8cda8993'#argoverse_loader.log_list[55]
print("Enter idx that you want to look into: Range( 0 to", str(argoverse_loader.lidar_count - 1), ")")
idx = input()
idx = int(idx)
labels = argoverse_loader.get_label_object(idx)

with open(argoverse_loader._label_list[log_id][idx]) as f:
  data = json.load(f)

print("Enter label id that you want to look into: Range( 0 to", str(len(labels) - 1), ")")
label_id = int(input())

print("TIMESTAMP:", data[label_id]['timestamp'])
print("LABEL CLASS:", data[label_id]['label_class'])
print("CENTER:", data[label_id]['center'])
print("ROTATION:", data[label_id]['rotation'])
print("UUID:", data[label_id]['track_label_uuid'])

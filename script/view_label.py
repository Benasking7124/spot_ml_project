import numpy as np

PATH = '/root/camera_pose_estimation_ws/src/spot_ml_project/dataset/'
LABEL_FILE_NAME = PATH + 'labels.npy'

labels = np.load(LABEL_FILE_NAME)
for l in labels:
    print(l)
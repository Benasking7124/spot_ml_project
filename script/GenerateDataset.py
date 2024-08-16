#! /usr/bin/env python3
import rospy, message_filters, cv2, os
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import numpy as np
from ml_project.srv import GetPose
from tf.transformations import euler_from_quaternion

PATH = '/root/camera_pose_estimation_ws/src/spot_ml_project/dataset/'
LABEL_FILE_NAME = PATH + 'labels'
# DATA_NUMBER = 1000
# TRAIN_NUMBER = int(DATA_NUMBER * 0.8)
# VALID_NUMBER = int(TRAIN_NUMBER + DATA_NUMBER * 0.1)

# Run GenerateDataset.py first, then ChangeLinkPose.py, then pub true to rostopic /change

class GenerateDataset:

    def __init__(self) -> None:
        rospy.init_node("generate_dataset")
        self.pub = rospy.Publisher('/change', Bool, queue_size=10)
        self.index = 0
        self.record = False
        self.bridge = CvBridge()

        rospy.wait_for_service('get_link_pose')
        self.get_pose = rospy.ServiceProxy('get_link_pose', GetPose)
        self.labels = []

        rospy.Subscriber('link/pose', Pose, self.record_callback)
        front_camera_sub = message_filters.Subscriber('front_camera/image_raw', Image)
        back_camera_sub = message_filters.Subscriber('back_camera/image_raw', Image)
        left_camera_sub = message_filters.Subscriber('left_camera/image_raw', Image)
        fright_camera_sub = message_filters.Subscriber('right_camera/image_raw', Image)

        timeSynchronizer = message_filters.ApproximateTimeSynchronizer([front_camera_sub, back_camera_sub, left_camera_sub, fright_camera_sub], 10, 0.1)
        timeSynchronizer.registerCallback(self.callback)

    def callback(self, image1, image2, image3, image4):
        if self.record is False:
            return
        
        self.record = False

        cv2_image1 = self.bridge.imgmsg_to_cv2(image1, desired_encoding='bgr8')
        cv2_image2 = self.bridge.imgmsg_to_cv2(image2, desired_encoding='bgr8')
        cv2_image3 = self.bridge.imgmsg_to_cv2(image3, desired_encoding='bgr8')
        cv2_image4 = self.bridge.imgmsg_to_cv2(image4, desired_encoding='bgr8')

        # if self.index < TRAIN_NUMBER:
        #     folder_name = PATH + 'train/' + format(self.index, '05d')
        
        # elif self.index < VALID_NUMBER:
        #     folder_name = PATH + 'valid/' + format((self.index - TRAIN_NUMBER), '05d')
        
        # else:
        #     folder_name = PATH + 'test/' + format((self.index - VALID_NUMBER), '05d')
        folder_name = PATH + format(self.index, '05d')
        if not os.path.exists(folder_name):
            os.makedirs(folder_name)

        image_name = folder_name + '/0.png'
        cv2.imwrite(image_name, cv2_image1)

        image_name1 = folder_name + '/1.png'
        cv2.imwrite(image_name1, cv2_image2)

        image_name1 = folder_name + '/2.png'
        cv2.imwrite(image_name1, cv2_image3)

        image_name1 = folder_name + '/3.png'
        cv2.imwrite(image_name1, cv2_image4)

        print('Save Image!')

        pose = self.get_pose().pose
        _, _, angle = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        print("Current angle: ", angle)
        self.labels.append(angle)
        np.save(LABEL_FILE_NAME, self.labels)

        self.index += 1
        self.pub.publish(Bool(True))

    def record_callback(self, data):
        print('record!')
        self.record = True

if __name__ == '__main__':
    labels_generator = GenerateDataset()
    rospy.spin()
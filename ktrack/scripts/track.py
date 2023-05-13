#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from kalmanfilter import KalmanFilter
from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt
from drawnow import drawnow

class Track:
    def __init__(self) -> None:
        rospy.Subscriber('midas/points',Float64MultiArray,self.plot)
        rospy.Subscriber('/midas/depth_raw',Image,self.bbox)
        self.pubKimage = rospy.Publisher("/kalman/image", Image, queue_size=1)
        self.kf = KalmanFilter()
        self.bridge = CvBridge()
        self.prediction = [0,0, 0]
        self.x = 0
        self.y = 0
        self.z = 0
        self.X_error = []
        self.file1 = open("/home/soslab/personal_ws/X_eror.txt","w")
        self.file2 = open("/home/soslab/personal_ws/y_eror.txt","w")
        self.file3 = open("/home/soslab/personal_ws/d_eror.txt","w")
        # self.pub = rospy.Publisher("/point", PointStamped, queue_size=1)

    def plot(self,msg):
        self.x = msg.data[4]
        self.y = msg.data[5]
        self.z = msg.data[6]
        self.prediction = self.kf.predict(msg.data[4], msg.data[5], msg.data[6])
        print(self.x-self.prediction[0], self.y-self.prediction[1], self.z-self.prediction[2])
        self.file1.write(str((self.x-self.prediction[0])**2)+"\n")
        self.file2.write(str((self.y-self.prediction[1])**2)+"\n")
        self.file3.write(str((self.z-self.prediction[2])**2)+"\n")
        # self.x = msg.data[4]
        # self.y = msg.data[5]
        # self.z = msg.data[6]


    def bbox(self,msg):
        cv_image =  self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        colorDepth = cv2.circle(cv_image, (self.prediction[0], self.prediction[1]), 1, (0,255,0), 2)
        output_msg = self.bridge.cv2_to_imgmsg(colorDepth)
        self.pubKimage.publish(output_msg)
    

if __name__ == "__main__":
    rospy.init_node("track")
    Track()
    rospy.spin()
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import torch
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from yolov5_ros_msgs.msg import BoundingBox, BoundingBoxes
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
from nav_msgs.msg import Odometry

class pose_converter:    #订阅odom话题（之后并入launch文件）
    def __init__(self):
        amcl_pose = rospy.get_param('~odom_pose', '')#odom话题
        self.pose_sub = rospy.Subscriber(amcl_pose,Odometry,self.posecallback)        #话题订阅/amcl_pose
        self.pose = Odometry()
    def posecallback(self,data):
        self.pose = data
        # rospy.loginfo("x=%f,y=%f,z=%f",self.pose.pose.pose.position.x,self.pose.pose.pose.position.y,self.pose.pose.pose.position.z)

class Yolo_Dect(pose_converter):
    def __init__(self):
        pose_converter.__init__(self)
        # load parameters
        yolov5_path = rospy.get_param('/yolov5_path', '')

        weight_path = rospy.get_param('~weight_path', '')   #权重文件路径
        image_topic = rospy.get_param(
            '~image_topic', '/camera/color/image_raw')              #订阅的话题（在launch文件中修改）
        pub_topic = rospy.get_param('~pub_topic', '/yolov5/BoundingBoxes')      #处理后的话题
        self.camera_frame = rospy.get_param('~camera_frame', '')
        conf = rospy.get_param('~conf', '0.5')

        # load local repository(YoloV5:v6.0)
        self.model = torch.hub.load(yolov5_path, 'custom',
                                    path=weight_path, source='local')

        # which device will be used
        if (rospy.get_param('/use_cpu', 'false')):
            self.model.cpu()
        else:
            self.model.cuda()

        self.model.conf = conf
        self.color_image = Image()
        self.depth_image = Image()
        self.getImageStatus = False

        # Load class color
        self.classes_colors = {}    #在此处定义类别颜色（不定义则随机）

        # image subscribe
        self.color_sub = rospy.Subscriber(image_topic, Image, self.image_callback,
                                          queue_size=1, buff_size=52428800)

        # output publishers
        self.position_pub = rospy.Publisher(
            pub_topic,  BoundingBoxes, queue_size=1)

        self.image_pub = rospy.Publisher(
            '/yolov5/detection_image',  Image, queue_size=1)

        # if no image messages
        while (not self.getImageStatus) :
            rospy.loginfo("waiting for image.")
            rospy.sleep(2)

    def image_callback(self, image):
        self.boundingBoxes = BoundingBoxes()        #目标检测结果
        self.boundingBoxes.header = image.header    #时间戳
        self.boundingBoxes.image_header = image.header  #时间戳
        self.getImageStatus = True               #是否有图像
        self.color_image = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.width, -1)  #转换为opencv格式
        #frombuffer()函数的作用是将数据转换为一维数组，然后reshape()函数将一维数组转换为二维数组
        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)    #转换为opencv格式

        results = self.model(self.color_image)  
        # results.print()  # print results to screen
        # xmin    ymin    xmax   ymax  confidence  class    name

        boxs = results.pandas().xyxy[0].values  # xyxy format
        self.dectshow(self.color_image, boxs, image.height, image.width)    #显示检测结果

        cv2.waitKey(3)  # wait 3ms

    def dectshow(self, org_img, boxs, height, width):   #显示检测结果的函数
        img = org_img.copy()    #复制送来的图像

        count = 0   #目标个数
        for i in boxs:
            count += 1

        for box in boxs:    #遍历每个目标
            boundingBox = BoundingBox()    #目标检测结果
            boundingBox.probability =np.float64(box[4])  #置信度
            boundingBox.xmin = np.int64(box[0]) #目标框的四个坐标
            boundingBox.ymin = np.int64(box[1]) 
            boundingBox.xmax = np.int64(box[2])
            boundingBox.ymax = np.int64(box[3])
            boundingBox.num = np.int16(count)   #目标个数
            boundingBox.Class = box[-1]    #目标类别（名称）

            if box[-1] in self.classes_colors.keys():   #目标类别的颜色
                color = self.classes_colors[box[-1]]    #目标类别的颜色
            else:
                color = np.random.randint(0, 183, 3)    #随机生成颜色
                self.classes_colors[box[-1]] = color    #目标类别的颜色

            cv2.rectangle(img, (int(box[0]), int(box[1])),  #画目标框
                          (int(box[2]), int(box[3])), (int(color[0]),int(color[1]), int(color[2])), 1)  #画目标框

            if box[1] < 20:   #目标类别的位置
                text_pos_y = box[1] + 30    #目标类别的位置
            else:
                text_pos_y = box[1] - 10    #目标类别的位置
            cv2.putText(img, box[-1],
                        (int(box[0]), int(text_pos_y)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)  #画目标类别（文字名称）
            self.boundingBoxes.bounding_boxes.append(boundingBox)   #目标检测结果（添加到列表）
            self.position_pub.publish(self.boundingBoxes)   #目标检测结果（发布）
        self.publish_image(img, height, width)  #显示检测结果
        cv2.putText(img, "x=%f  y=%f  z=%f  w=%f"%(self.pose.pose.pose.position.x,self.pose.pose.pose.position.y,self.pose.pose.pose.position.z,self.pose.pose.pose.orientation.w), (5, 780), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imshow('YOLOv5', img)   #显示检测结果

    def publish_image(self, imgdata, height, width):    #发布检测结果的函数
        image_temp = Image()    #检测结果（图像）
        header = Header(stamp=rospy.Time.now())   #时间戳
        header.frame_id = self.camera_frame  #相机坐标系
        image_temp.height = height  
        image_temp.width = width    
        image_temp.encoding = 'bgr8'    #图像格式
        image_temp.data = np.array(imgdata).tobytes()   #图像数据
        image_temp.header = header  #时间戳
        image_temp.step = width * 3   #图像步长
        self.image_pub.publish(image_temp)  #检测结果（图像）（发布）


def main():
    rospy.init_node('yolov5_ros', anonymous=True)   #初始化节点
    yolo_dect = Yolo_Dect()  #实例化类
    rospy.spin()    #保持程序运行


if __name__ == "__main__":

    main()

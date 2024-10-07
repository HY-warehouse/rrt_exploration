#!/usr/bin/env python
# -*- coding: utf-8 -*-

# import rospy
# import cv2
# from nav_msgs.msg import OccupancyGrid
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import numpy as np

# class MapToImageConverter:

#     def __init__(self):
#         # 订阅地图主题
#         self.map_sub = rospy.Subscriber("/map_merge/map", OccupancyGrid, self.map_callback)
#         # 发布图片主题
#         self.image_pub = rospy.Publisher("/map_image", Image, queue_size=10)
#         # cv_bridge实例化
#         self.bridge = CvBridge()

#     def map_callback(self, data):
#         # 将数据转换为numpy数组
#         map_data = data.data
#         map_width = data.info.width
#         map_height = data.info.height
#         map_resolution = data.info.resolution

#         # 转换为灰度图像
#         map_image = cv2.flip(cv2.rotate(cv2.resize(np.array(map_data).reshape((map_height, map_width)), (0, 0), fx=5, fy=5, interpolation=cv2.INTER_NEAREST), cv2.ROTATE_90_COUNTERCLOCKWISE), 0)

#         # 将数据转换为ROS图像消息
#         image_msg = self.bridge.cv2_to_imgmsg(map_image, "mono8")
#         # 设置消息头
#         image_msg.header.stamp = rospy.Time.now()
#         image_msg.header.frame_id = "map"
#         # 发布消息
#         self.image_pub.publish(image_msg)

# if __name__ == '__main__':
#     # 初始化ROS节点
#     rospy.init_node('map_to_image_converter')
#     # 创建MapToImageConverter实例
#     converter = MapToImageConverter()
#     # 循环等待回调函数
#     rospy.spin()



# import rospy
# from nav_msgs.msg import OccupancyGrid
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2


# class MapToImageConverter:
#     def __init__(self):
#         rospy.init_node('map_to_image_converter', anonymous=True)
#         self.bridge = CvBridge()
#         self.image_pub = rospy.Publisher("map_image", Image, queue_size=1)
#         self.map_sub = rospy.Subscriber("map_merge/map", OccupancyGrid, self.map_callback, queue_size=1)

#     def map_callback(self, map_msg):
#         map_image = self.convert_to_image(map_msg)
#         if map_image is not None:
#             self.image_pub.publish(self.bridge.cv2_to_imgmsg(map_image, encoding="passthrough"))

#     def convert_to_image(self, map_msg):
#         if map_msg.info.width == 0 or map_msg.info.height == 0:
#             rospy.logwarn("Map width or height is zero.")
#             return None

#         resolution = map_msg.info.resolution
#         width = map_msg.info.width
#         height = map_msg.info.height
#         origin_x = map_msg.info.origin.position.x
#         origin_y = map_msg.info.origin.position.y

#         map_data = list(map_msg.data)
#         map_data = [map_data[i:i+width] for i in range(0, len(map_data), width)]

#         map_image = [[255 - int(round((float(map_data[y][x])/100.0) * 255.0)) for x in range(width)] for y in range(height)]

#         cv_map_image = cv2.flip(cv2.rotate(cv2.resize(
#             map_image, (width*10, height*10)), cv2.ROTATE_90_COUNTERCLOCKWISE), 1)

#         return cv_map_image

# if __name__ == '__main__':
#     try:
#         MapToImageConverter()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass



import rospy
import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geopoints_msgs.msg import multi_GeoPoints
from geopoints_msgs.msg import single_GeoPoint
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import ColorRGBA


class Marker_point(object):
    def __init__(self):
        # 创建发布者
        # self.marker_pub = rospy.Publisher('/marker_point', Marker, queue_size=1)
        self.marker_pub = rospy.Publisher('/marker_array', MarkerArray, queue_size=10)

        # 创建Marker_point对象
        # self.marker = Marker()
        # self.marker.header.frame_id = '/robot_1/map'
        # self.marker.type = Marker.POINTS
        # self.marker.scale.x = 0.1
        # self.marker.scale.y = 0.1
        # self.marker.scale.z = 0.1
        # self.marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
        
        #创建MarkerArray对象
        self.marker_array = MarkerArray()
        
        # 创建Marker_text对象
        self.marker_text = Marker()
        self.marker_text.action = Marker.ADD
        self.marker_text.header.frame_id = '/robot_1/map'
        self.marker_text.header.stamp = rospy.Time.now()
        self.marker_text.type = Marker.TEXT_VIEW_FACING
        self.marker_text.scale.x = 1
        self.marker_text.scale.y = 1
        self.marker_text.scale.z = 1
        self.marker_text.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)


class boundingbox_sub(object):  
    def __init__(self):
        rospy.Subscriber("/boundingbox_sub/points", multi_GeoPoints, self.boundingboxcallback)
        self.points = multi_GeoPoints()
        self.pointsdic = {}

    def boundingboxcallback(self, msg):
        self.points = msg
        # 获取目标在/robot_1/map坐标系下的坐标
        for i in range(len(self.points.multi_geopoints)):
            Class = self.points.multi_geopoints[i].Class
            px = self.points.multi_geopoints[i].single_geopoint.point.x
            py = self.points.multi_geopoints[i].single_geopoint.point.y
            self.pointsdic[self.points.multi_geopoints[i].Class] = [Class, px, py]


class MapToPgmNode(boundingbox_sub, Marker_point):
    def __init__(self):
        rospy.loginfo('进入MapToPgmNode')
        # 初始化父类
        boundingbox_sub.__init__(self)
        rospy.loginfo('boundingbox_sub初始化完成')
        Marker_point.__init__(self)
        rospy.loginfo('Marker_point初始化完成')

        # 订阅地图信息，类型为nav_msgs/OccupancyGrid
        rospy.Subscriber('/map_merge/map', OccupancyGrid, self.map_callback)
        
        rospy.loginfo('订阅地图信息完成')
        # 发布pgm格式图片，类型为sensor_msgs/Image
        self.image_pub = rospy.Publisher('/map_image', Image, queue_size=1)
        
        self.bridge = CvBridge()



    def map_callback(self, msg):
        rospy.loginfo('enter map_callback')

        # 从地图消息中提取地图数据
        map_width = msg.info.width
        map_height = msg.info.height
        map_data = msg.data
        
        # 将地图数据转换为numpy数组，并调整大小
        map_array = np.array(map_data).reshape((map_height, map_width))

        rospy.loginfo('(initial)map_height, map_width: {}, {}'.format(map_height, map_width))

        # 将地图数据的值（0-100）映射到0-255的灰度值上
        map_array = (255 - map_array / 100.0 * 255).astype(np.uint8)

        # 将地图数据转换为灰度图像
        map_image = cv2.cvtColor(map_array, cv2.COLOR_GRAY2BGR)

        # 更改分辨率，增加数据流大小
        map_image = cv2.resize(map_image, (map_width*4, map_height*4))
        # map_image = cv2.flip(map_image, 1)   # 左右颠倒
        map_image = cv2.flip(map_image, 0)   # 上下颠倒
        rospy.loginfo('(final)map_height, map_width: {}, {}'.format(map_image.shape[0], map_image.shape[1]))


        # 将pointsdic中的点画到rviz上
        # 设置Marker的位置
        points = []
        texts = []
        rospy.loginfo('添加点')
        for key in self.pointsdic:
            if key != '':
                rospy.loginfo('key: {}'.format(key))
                texts.append(key)
                points.append(Point(int(self.pointsdic[key][1] + 0.5), int(self.pointsdic[key][2] + 0.5), 0))
        
        # self.marker.points = points

        # 发布Marker_point消息
        # for point in points:
        #     rospy.loginfo('发布Marker_point消息')
        #     self.marker.points = [point]
        #     self.marker_pub.publish(self.marker)

        # 发布Marker_text消息
        for i in range(len(texts)):
            rospy.loginfo('i: {}'.format(i))
            rospy.loginfo('texts[i]: {}'.format(texts[i]))
            rospy.loginfo('发布Marker_text消息')
            marker_text = Marker()
            marker_text.action = Marker.ADD
            marker_text.header.frame_id = '/robot_1/map'
            marker_text.header.stamp = rospy.Time.now()
            marker_text.type = Marker.TEXT_VIEW_FACING
            marker_text.scale.x = 1
            marker_text.scale.y = 1
            marker_text.scale.z = 1
            marker_text.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
            marker_text.id = i
            marker_text.pose.position = points[i]
            marker_text.text = texts[i]
            rospy.loginfo('self.marker_text.id:{}'.format(self.marker_text.id))
            # rospy.loginfo(self.marker_array)
            self.marker_array.markers.append(marker_text)
        
        # rospy.loginfo(self.marker_array)
        self.marker_pub.publish(self.marker_array)
        self.marker_array.markers = []

        # 将地图数据保存
        cv2.imwrite('map.jpg', map_image)

        # 将jpg图片格式发布出去
        # pgm_image = cv2.imread('map.pgm', cv2.IMREAD_GRAYSCALE)
        ros_image = self.bridge.cv2_to_imgmsg(map_image, encoding='bgr8')
        self.image_pub.publish(ros_image)

        # # 创建PGM图片格式
        # pgm = cv2.cvtColor(map_image, cv2.COLOR_BGR2GRAY)
        # cv2.imwrite('map.pgm', pgm)

        # # 将PGM图片格式发布出去
        # pgm_image = cv2.imread('map.pgm', cv2.IMREAD_GRAYSCALE)
        # ros_image = self.bridge.cv2_to_imgmsg(pgm_image, encoding='mono8')
        # self.image_pub.publish(ros_image)


if __name__ == '__main__':
    try:
        rospy.init_node('paint_point_to_map_node')
        MapToPgmNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

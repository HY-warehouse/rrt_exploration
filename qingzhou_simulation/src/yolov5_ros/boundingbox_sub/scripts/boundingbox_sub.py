#!  /usr/bin/env python
# coding=utf-8

from __future__ import print_function
from yolov5_ros_msgs.msg import BoundingBox, BoundingBoxes
from geopoints_msgs.msg import single_GeoPoint
from geopoints_msgs.msg import multi_GeoPoints
from geometry_msgs.msg import PointStamped
import std_msgs.msg
import rospy

class BoundingBox_subscriber():
    def __init__(self):
        rgbdtopic1 = rospy.get_param('~rgbdtopic1', '/rgbd_detection2d_3d/points')
        rgbdtopic2 = rospy.get_param('~rgbdtopic2', '/rgbd_detection2d_3d/points')
        rgbdtopic3 = rospy.get_param('~rgbdtopic3', '/rgbd_detection2d_3d/points')
        rgbdtopic4 = rospy.get_param('~rgbdtopic4', '/rgbd_detection2d_3d/points')
        self.multi_Geopoints1_sub = rospy.Subscriber(rgbdtopic1,  multi_GeoPoints, self.points_callback)
        self.multi_Geopoints2_sub = rospy.Subscriber(rgbdtopic2,  multi_GeoPoints, self.points_callback)
        self.multi_Geopoints3_sub = rospy.Subscriber(rgbdtopic3,  multi_GeoPoints, self.points_callback)
        self.multi_Geopoints4_sub = rospy.Subscriber(rgbdtopic4,  multi_GeoPoints, self.points_callback)
        self.Pointdict = {}
        self.Pointdictcache = {}

    def points_callback(self, data):
        self.multi_Geopoints = multi_GeoPoints()
        self.multi_Geopoints = data
        self.result_sinGeoPoint = single_GeoPoint()
        self.result_mulGeoPoints = multi_GeoPoints()
        for i in range(len(self.multi_Geopoints.multi_geopoints)):
            if self.multi_Geopoints.multi_geopoints[i].OriginalClass == 'person' or self.multi_Geopoints.multi_geopoints[i].OriginalClass == 'airplane':
                px = self.multi_Geopoints.multi_geopoints[i].single_geopoint.point.x
                py = self.multi_Geopoints.multi_geopoints[i].single_geopoint.point.y
                pz = self.multi_Geopoints.multi_geopoints[i].single_geopoint.point.z
                count = 1
                pointcache = [px, py, pz]
                if self.multi_Geopoints.multi_geopoints[i].Class in self.Pointdict:             #如果字典中已经存在这个类别的点
                    self.Pointdictcache[self.multi_Geopoints.multi_geopoints[i].Class].append(pointcache)
                    #如果字典中的点数超过100个，则删除字典中的第一个点
                    #使用self.Pointdictcache来保存每个不同class回传的100次以内的坐标，然后取平均值
                    if len(self.Pointdictcache[self.multi_Geopoints.multi_geopoints[i].Class]) > 200:
                        self.Pointdictcache[self.multi_Geopoints.multi_geopoints[i].Class].pop(0)
                    #计算字典中的点的平均值
                    for j in range(len(self.Pointdictcache[self.multi_Geopoints.multi_geopoints[i].Class])):
                        emptylist = []
                        emptylist.append(self.Pointdictcache[self.multi_Geopoints.multi_geopoints[i].Class][j][0])
                        px = sum(emptylist) / len(emptylist)
                        emptylist = []
                        emptylist.append(self.Pointdictcache[self.multi_Geopoints.multi_geopoints[i].Class][j][1])
                        py = sum(emptylist) / len(emptylist)
                        emptylist = []
                        emptylist.append(self.Pointdictcache[self.multi_Geopoints.multi_geopoints[i].Class][j][2])
                        pz = sum(emptylist) / len(emptylist)   
                    print(px, py, pz)
                    #如果这个点的坐标和字典中的点坐标相近，则不更新字典中的点
                    if abs(self.Pointdict[self.multi_Geopoints.multi_geopoints[i].Class][0] - px) < 2 or abs(self.Pointdict[self.multi_Geopoints.multi_geopoints[i].Class][1] - py) < 2:
                        self.Pointdict[self.multi_Geopoints.multi_geopoints[i].Class] = [px, py, pz, count]
                    else:
                        count += 1
                        #如果这个点的坐标和字典中的点坐标不相近，则不改动原来的点，而是在字典中添加一个新的点，对应的Class末尾加上一个数字
                        self.Pointdict[self.multi_Geopoints.multi_geopoints[i].Class + " " + str(self.Pointdict[self.multi_Geopoints.multi_geopoints[i].Class][3])] = [px, py, pz, count]
                else:
                    self.Pointdict[self.multi_Geopoints.multi_geopoints[i].Class] = [px, py, pz, count]
                    self.Pointdictcache[self.multi_Geopoints.multi_geopoints[i].Class] = [[px, py, pz],]
        #将结果写入一个multi_GeoPoints（是single_GeoPoint的列表）类型的变量中
        self.result_mulGeoPoints.multi_geopoints = []
        for key in self.Pointdict:
            # print(key)
            self.result_sinGeoPoint = single_GeoPoint()
            self.result_sinGeoPoint.Class = key
            self.result_sinGeoPoint.single_geopoint.point.x = self.Pointdict[key][0]
            self.result_sinGeoPoint.single_geopoint.point.y = self.Pointdict[key][1]
            self.result_sinGeoPoint.single_geopoint.point.z = self.Pointdict[key][2]
            self.result_mulGeoPoints.multi_geopoints.append(self.result_sinGeoPoint)
        # 发布
        self.multi_Geopoints_pub = rospy.Publisher("/boundingbox_sub/points", multi_GeoPoints, queue_size=1)
        self.multi_Geopoints_pub.publish(self.result_mulGeoPoints)
        print(self.Pointdict)
        print(self.Pointdictcache)
                
def main():
    rospy.init_node('boundingbox_sub', anonymous=True)
    boundingbox_subscriber = BoundingBox_subscriber()
    rospy.spin()

if __name__ == '__main__':
    main()
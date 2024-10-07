#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped, Vector3
import tf2_ros
import tf_conversions

import numpy as np
import math

import functions as fn


robot_maps = {} # dict of OccupancyGrid


def map_callback(data, args):
    global robot_maps
    namespace = args[0]
    rospy.logdebug(f"Received robot map: {namespace}")
    robot_maps[namespace] = data


def node():
    global robot_maps

    rospy.init_node('map_merge', anonymous=False)

    world_map_topic = rospy.get_param('~world_map_topic', 'map')
    world_map_frame = rospy.get_param('~world_map_frame', 'world')
    merge_subprocess_count = int(rospy.get_param('~merge_subprocess_count', 1))
    rateHz = rospy.get_param('~rate', 0.25)
    robot_common_name = rospy.get_param('~robot_common_name', 'robot_')
    robot_count = int(rospy.get_param('~robot_count', 2))


    robot_namespaces = [f'{robot_common_name}{i}' for i in range(1, robot_count + 1)]
    robot_initial_poses = {}


    # ------------------------------------------------

    for robot_namespace in robot_namespaces:
        map_topic = f'{robot_namespace}/map'
        rospy.Subscriber(map_topic, OccupancyGrid, callback=map_callback, callback_args=[robot_namespace])

        params = {
            'x': f'{robot_namespace}/map_merge/init_pose_x',
            'y': f'{robot_namespace}/map_merge/init_pose_y',
            'yaw': f'{robot_namespace}/map_merge/init_pose_yaw',
        }
        poses = {}

        for axis, param in params.items():

            if rospy.has_param(param):
                poses[axis] = rospy.get_param(param) 
            else:
                rospy.logwarn(f"Missing mandatory param: {param}")
                rospy.signal_shutdown(f"Missing mandatory param: {param}")

        robot_initial_poses[robot_namespace] = poses

    world_map_publisher = rospy.Publisher(world_map_topic, OccupancyGrid, queue_size=math.ceil(rateHz * 5))

    rate = rospy.Rate(rateHz)

    world_map = OccupancyGrid()
    world_map.info.origin = Pose(position=Point(x=-5, y=-5, z=0.0), orientation=Quaternion(0, 0, 0, 1))
    world_map.info.resolution = 0.05
    world_map.info.width = 100
    world_map.info.height = 150


    # -----------------------------------------------

    rospy.loginfo("Broadcasting static transforms")

    tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transforms = []
    
    for robot_namespace in robot_namespaces:
        initial_pose = robot_initial_poses[robot_namespace]

        transform = TransformStamped()

        # header
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = world_map_frame

        # child frame
        transform.child_frame_id = f'{robot_namespace}/map'

        # translation and rotation
        transform.transform.translation = Vector3(x=initial_pose['x'], y=initial_pose['y'], z=0)
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, initial_pose['yaw'])
        transform.transform.rotation = Quaternion(q[0], q[1], q[2], q[3])

        static_transforms.append(transform)
        
    tf_broadcaster.sendTransform(static_transforms)

    
    # ------------------------------------------------

    rospy.loginfo("Waiting for robot maps")
    
    # wait for all robot maps
    while len(robot_maps.keys()) != len(robot_namespaces):
        rospy.sleep(0.1)


    # ------------------------------------------------

    rospy.loginfo("Started main loop")


    while not rospy.is_shutdown():
        loop_start_time = rospy.Time.now().to_sec()

        # calculate world map metadata
        world_map = fn.calc_world_map_metadata(world_map, robot_maps, robot_initial_poses)

        # calculate world map data
        world_map = fn.calc_world_map_data(world_map, robot_maps, robot_initial_poses, merge_subprocess_count)

        # setup world map time
        world_map.header = Header(stamp=rospy.Time.now(), frame_id=world_map_frame)
        world_map.info.map_load_time = rospy.Time.now()

        # publish world map
        rospy.logdebug(world_map.info)
        world_map_publisher.publish(world_map)

        # -------------------------------------------

        loop_end_time = rospy.Time.now().to_sec() - loop_start_time
        rospy.loginfo(f"Map merge took {loop_end_time:.3f}s")


        # --------------------------------------------

        rate.sleep()





if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
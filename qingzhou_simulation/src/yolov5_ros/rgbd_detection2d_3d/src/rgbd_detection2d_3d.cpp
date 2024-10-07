#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <yolov5_ros_msgs/BoundingBoxes.h>
#include <vision_msgs/Detection3DArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <geopoints_msgs/single_GeoPoint.h>
#include <geopoints_msgs/multi_GeoPoints.h>

ros::Publisher detection3d_array_pub_;
ros::Publisher marker_array_pub_;
ros::Publisher pub_point_;
float x_thereshold_, y_thereshold_, z_thereshold_;

// namespace rgbd_detection2d_3d
// {
//   class RgbdDetection2d3d
//   {
//   public:
//     RgbdDetection2d3d(ros::NodeHandle& nh, ros::NodeHandle& nhpub);
//     ~RgbdDetection2d3d();
//     void callback(const yolov5_ros_msgs::BoundingBoxes::ConstPtr& detect_2d, const sensor_msgs::PointCloud2::ConstPtr& depth_points, const tf::TransformListener& listener_);
//   };
// }
void callback(const yolov5_ros_msgs::BoundingBoxes::ConstPtr& detect_2d, const sensor_msgs::PointCloud2::ConstPtr& depth_points, const tf::TransformListener& listener_) 
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*depth_points, *pcl_pc);
  // 点云转换为深度图
  listener_.waitForTransform("/robot_1/map", "kinect_frame_optical", ros::Time(0), ros::Duration(3.0));
  vision_msgs::Detection3DArray detection3d_array;
  visualization_msgs::MarkerArray marker_array;
  int num = detect_2d->bounding_boxes.size();
  geometry_msgs::Point geocenter_point[num];
  geometry_msgs::PointStamped geocenter_point_stamped[num];
  geometry_msgs::PointStamped point_stamped_geo[num];
  geopoints_msgs::single_GeoPoint point_stamped_with_class[num];
  geopoints_msgs::multi_GeoPoints point_stamped; 
  for(int i = 0; i < num ; i++) 
  {
    int x_center = (detect_2d->bounding_boxes[i].xmax + detect_2d->bounding_boxes[i].xmin) / 2;
    int y_center = (detect_2d->bounding_boxes[i].ymax + detect_2d->bounding_boxes[i].ymin) / 2;
    //rosinfo打印
    ROS_INFO("x_center: %d, y_center: %d", x_center, y_center);
    int pcl_idx = x_center + y_center * (int)depth_points->width;
    // pcl_idx: 点云的索引
    pcl::PointXYZRGB center_point = pcl_pc->at(pcl_idx);
    // center_point: 点云的中心点
    
    if(std::isnan(center_point.x) || std::isnan(center_point.y) || std::isnan(center_point.z)) 
    {
      continue;
    }
    // 判断是否是nan，如果是nan，就跳过这个点

    float x_min, y_min, z_min, x_max, y_max, z_max;
    x_min = y_min = z_min = std::numeric_limits<float>::max();
    x_max = y_max = z_max = -std::numeric_limits<float>::max();
    // x_min, y_min, z_min, x_max, y_max, z_max: 点云框选区域的最大最小值
    for(int j = detect_2d->bounding_boxes[i].xmin; j < detect_2d->bounding_boxes[i].xmax; j++) 
    {
      for(int k = detect_2d->bounding_boxes[i].ymin; k < detect_2d->bounding_boxes[i].ymax; k++) 
      {
	      pcl_idx = j + k * (int)depth_points->width;
	      pcl::PointXYZRGB point =  pcl_pc->at(pcl_idx);
	// point: 点云的点，嵌套循环遍历点云框选区域的所有点

	      if(std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) 
        {
	        continue;
	      }// 判断是否是nan，如果是nan，就跳过这个点
	
	      if(fabs(point.x - center_point.x) > x_thereshold_) 
        {
	        point.x = point.x > center_point.x ? center_point.x + x_thereshold_ : center_point.x - x_thereshold_;
	      }
  // point.x: 点云的点的x坐标
	      if(fabs(point.y - center_point.y) > y_thereshold_) 
       {
	        point.y = point.y > center_point.y ? center_point.y + y_thereshold_ : center_point.y - y_thereshold_;
	      }
  // point.y: 点云的点的y坐标
	      if(fabs(point.z - center_point.z) > z_thereshold_) 
        {
	        point.z = point.z > center_point.z ? center_point.z + z_thereshold_ : center_point.z - z_thereshold_;
	      }
  // point.z: 点云的点的z坐标
	
	      x_min = std::min(point.x, x_min);
	      y_min = std::min(point.y, y_min);
	      z_min = std::min(point.z, z_min);
	// x_min, y_min, z_min: 点云框选区域的最小值

	      x_max = std::max(point.x, x_max);
	      y_max = std::max(point.y, y_max);
	      z_max = std::max(point.z, z_max);
  // x_max, y_max, z_max: 点云框选区域的最大值
      }
    }
  // 遍历结束，得到点云框选区域的最大最小值
    
    vision_msgs::Detection3D detection3d;       // vision_msgs::Detection3D 里面有header, results, bbox, source_cloud
    detection3d.header.seq = i;       // header.seq: 序列号
    detection3d.header.frame_id = detect_2d->bounding_boxes[i].Class;       // header.frame_id: 帧id
    vision_msgs::ObjectHypothesisWithPose ohwp;       // vision_msgs::ObjectHypothesisWithPose 里面有id, score, pose
    ohwp.score = detect_2d->bounding_boxes[i].probability;        // ohwp.score: 置信度
    detection3d.results.push_back(ohwp);        // detection3d.results: 置信度
    detection3d.bbox.center.position.x = center_point.x;        // detection3d.bbox.center.position.x: 点云框选区域的中心点的x坐标
    detection3d.bbox.center.position.y = center_point.y;        // detection3d.bbox.center.position.y: 点云框选区域的中心点的y坐标
    detection3d.bbox.center.position.z = center_point.z;        // detection3d.bbox.center.position.z: 点云框选区域的中心点的z坐标
    detection3d.bbox.size.x = fabs(x_max - x_min);        // detection3d.bbox.size.x: 点云框选区域的x轴长度
    detection3d.bbox.size.y = fabs(y_max - y_min);        // detection3d.bbox.size.y: 点云框选区域的y轴长度
    detection3d.bbox.size.z = fabs(z_max - z_min);        // detection3d.bbox.size.z: 点云框选区域的z轴长度
    // TODO: detection3d.source_cloud here if needed.
    detection3d_array.detections.push_back(detection3d);    // detection3d_array.detections: 检测到的物体
    
    //定义一个中心坐标，用于计算点云框选区域的中心点
    geocenter_point[i].x = (x_max + x_min) / 2;
    geocenter_point[i].y = (y_max + y_min) / 2;
    geocenter_point[i].z = (z_max + z_min) / 2;
    //使用tf变换，将点云框选区域的中心点转换到 /robot_1/map 坐标系下
    geocenter_point_stamped[i].header = depth_points->header;
    geocenter_point_stamped[i].point = geocenter_point[i];
    geocenter_point_stamped[i].header.stamp = ros::Time(0);
    if(!listener_.waitForTransform("/robot_1/map", geocenter_point_stamped[i].header.frame_id, geocenter_point_stamped[i].header.stamp, ros::Duration(1.0)))
    {
      ROS_ERROR("Failed to transform from %s to %s", geocenter_point_stamped[i].header.frame_id.c_str(), "map");
      return;
    }
    //获取当前时刻的时间戳进行tf变换
    try
    {
      listener_.transformPoint("/robot_1/map", geocenter_point_stamped[i], point_stamped_geo[i]);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      return;
    }
    point_stamped_with_class[i].Class = detect_2d->bounding_boxes[i].Class;
    point_stamped_with_class[i].OriginalClass = detect_2d->bounding_boxes[i].Class;
    point_stamped_with_class[i].single_geopoint = point_stamped_geo[i];
    visualization_msgs::Marker marker;        // visualization_msgs::Marker 里面有header, ns, id, type, action, pose, scale, color, lifetime, frame_locked, points, colors, text, mesh_resource, mesh_use_embedded_materials
    marker.header = depth_points->header;       // marker.header: 帧头是点云的帧头
    marker.ns = "rgbd_detection2d_3d";        // marker.ns: 命名空间是rgbd_detection2d_3d
    marker.id = i;        // marker.id: id
    marker.type = visualization_msgs::Marker::LINE_LIST;        // marker.type: 类型是线列表
    geometry_msgs::Point p[24];       // geometry_msgs::Point 里面有x, y, z
    // 此处坐标为x轴向右，y轴向下，z轴向前
    p[0].x = x_max; p[0].y = y_max; p[0].z = z_max;       // p[0]: 点云框选区域的最大值
    p[1].x = x_min; p[1].y = y_max; p[1].z = z_max;       // p[1]: 点云框选区域的最大值
    p[2].x = x_max; p[2].y = y_max; p[2].z = z_max;
    p[3].x = x_max; p[3].y = y_min; p[3].z = z_max;
    p[4].x = x_max; p[4].y = y_max; p[4].z = z_max;
    p[5].x = x_max; p[5].y = y_max; p[5].z = z_min;
    p[6].x = x_min; p[6].y = y_min; p[6].z = z_min;
    p[7].x = x_max; p[7].y = y_min; p[7].z = z_min;
    p[8].x = x_min; p[8].y = y_min; p[8].z = z_min;
    p[9].x = x_min; p[9].y = y_max; p[9].z = z_min;
    p[10].x = x_min; p[10].y = y_min; p[10].z = z_min;      // p[10]: 点云框选区域的最小值
    p[11].x = x_min; p[11].y = y_min; p[11].z = z_max;
    p[12].x = x_min; p[12].y = y_max; p[12].z = z_max;
    p[13].x = x_min; p[13].y = y_max; p[13].z = z_min;
    p[14].x = x_min; p[14].y = y_max; p[14].z = z_max;
    p[15].x = x_min; p[15].y = y_min; p[15].z = z_max;
    p[16].x = x_max; p[16].y = y_min; p[16].z = z_max;
    p[17].x = x_max; p[17].y = y_min; p[17].z = z_min;
    p[18].x = x_max; p[18].y = y_min; p[18].z = z_max;
    p[19].x = x_min; p[19].y = y_min; p[19].z = z_max;
    p[20].x = x_max; p[20].y = y_max; p[20].z = z_min;
    p[21].x = x_min; p[21].y = y_max; p[21].z = z_min;
    p[22].x = x_max; p[22].y = y_max; p[22].z = z_min;
    p[23].x = x_max; p[23].y = y_min; p[23].z = z_min;
    // 画出点云框选区域的六个面
    for(int i = 0; i < 24; i++) 
    {
      marker.points.push_back(p[i]);
    }
    // marker.points: 点云框选区域的六个面的点
    marker.pose.orientation.w = 1.0;        // marker.pose.orientation.w: 旋转四元数的w
    marker.scale.x = 0.02;        // marker.scale.x: 线的宽度
    marker.color.a = 1.0;       // marker.color.a: 透明度
    marker.color.r = 0.0;       //  marker.color.r: 红色
    marker.color.g = 0.5;       // marker.color.g: 绿色
    marker.color.b = 1.0;       // marker.color.b: 蓝色
    
    marker.lifetime = ros::Duration(0.1);       // marker.lifetime: 生命周期
    marker_array.markers.push_back(marker);       // marker_array.markers: marker数组
  }

  if(detection3d_array.detections.size()) 
  {
    detection3d_array.header = depth_points->header;
    detection3d_array_pub_.publish(detection3d_array);
  }
  //  如果检测到物体，就发布检测到的物体的信息  
  if(marker_array.markers.size()) 
  {
    marker_array_pub_.publish(marker_array);
  }
  for(int i = 0; i < num; i++)
  {
      if(point_stamped_with_class[i].single_geopoint.point.x != 0 || point_stamped_with_class[i].single_geopoint.point.y != 0 || point_stamped_with_class[i].single_geopoint.point.z != 0)
      {
        point_stamped.multi_geopoints.push_back(point_stamped_with_class[i]);
      }
  }
  pub_point_.publish(point_stamped);         //待修改，希望能够直接发布整个数组
  // 如果检测到物体，就发布检测到的物体的点云框选区域的六个面
}
//获取目标中心的坐标，进行坐标转换，得到目标在地图上的坐标

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "rgbd_detection2d_3d");

  /*** Subscribers ***/
  ros::NodeHandle nh("~");
  //从launch文件中获取yolo话题和点云话题
  std::string detection_2d_topic = nh.param<std::string>("detection_2d_topic", "/yolov5/BoundingBoxes");
  std::string depth_registered_points_topic = nh.param<std::string>("depth_registered_points_topic", "/kinect/depth/points");
  tf::TransformListener listener_;
  // 订阅检测到的物体的信息
  message_filters::Subscriber<yolov5_ros_msgs::BoundingBoxes> detection_2d(nh, detection_2d_topic, 1);
  // 订阅点云信息
  message_filters::Subscriber<sensor_msgs::PointCloud2> depth_registered_points(nh, depth_registered_points_topic, 1);
  typedef message_filters::sync_policies::ApproximateTime<yolov5_ros_msgs::BoundingBoxes, sensor_msgs::PointCloud2> ApproximateTimePolicy;        // 同步策略
  message_filters::Synchronizer<ApproximateTimePolicy> sync(ApproximateTimePolicy(10), detection_2d, depth_registered_points);        // 同步检测到的物体的信息和点云信息
  sync.registerCallback(boost::bind(&callback, _1, _2,  boost::ref(listener_)));        // 回调函数
  
  /*** Publishers ***/
  ros::NodeHandle pub_nh("~");        // 创建私有节点句柄
  detection3d_array_pub_ =pub_nh.advertise<vision_msgs::Detection3DArray>("rgbd_detection/detection3d_array", 1);       // 发布检测到的物体的信息
  marker_array_pub_ =pub_nh.advertise<visualization_msgs::MarkerArray>("rgbd_detection/marker_array", 1);       // 发布检测到的物体的点云框选区域的六个面
  pub_point_ =pub_nh.advertise<geopoints_msgs::multi_GeoPoints>("rgbd_detection/points", 10);       // 发布目标在地图上的坐标
  
  /*** Parameters ***/
pub_nh.param<float>("x_thereshold", x_thereshold_, 0.5);        // 获取参数
pub_nh.param<float>("y_thereshold", y_thereshold_, 1.0);        // 获取参数
pub_nh.param<float>("z_thereshold", z_thereshold_, 0.5);        // 获取参数
  ros::spin();
  return 0;
}


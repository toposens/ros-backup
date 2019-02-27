#include "toposens_pointcloud/mapping.h"

#include <toposens_driver/sensor.h>


namespace toposens_pointcloud
{

GridMapping::GridMapping(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
	private_nh.param<float>("octree_size", octree_size, 20.0);
	private_nh.param<float>("octree_divisions", octree_divisions, 11.0);
	private_nh.param<std::string>("fixed_frame", fixed_frame, "odom");

	_sub = nh.subscribe(toposens_driver::kScanTopic, 100, &GridMapping::_convert, this);
  _pub = nh.advertise<TsCloud>("ts_cloud", 100);

	octree = new Octree(octree_size, octree_divisions);

//  float resolution = 12.0f;
  // do this using boost shared ptr
   ot = new pcl::octree::OctreePointCloud<pcl::PointXYZI>(12);
  //pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

 // octree.setInputCloud(cloud);
 // octree.addPointsFromInputCloud ();

}


void GridMapping::_convert(const toposens_msgs::TsScan::ConstPtr& msg)
{
  //ROS_WARN_STREAM(*msg);

  TsCloud::Ptr tc(new TsCloud);
  tc->header.frame_id = fixed_frame;
  tc->height = 1;
  pcl_conversions::toPCL(msg->header.stamp, tc->header.stamp);

  std_msgs::Header header = msg->header;
  std::vector<toposens_msgs::TsPoint> points = msg->points;

  try {
    tfListener.waitForTransform(fixed_frame, header.frame_id, header.stamp, ros::Duration(0.5));
    for (auto it2 = points.begin(); it2 != points.end(); ++it2) {
      tc->points.push_back(_transform(header, *it2));

     // ot.addPointToCloud(_transform(header, *it2), tc);
    }
  } catch (tf::TransformException ex){
    ROS_INFO_STREAM(ex.what());
  }

 // ROS_WARN_STREAM(tc->points.size());
  tc->width = tc->points.size();
  _pub.publish (tc);
}


pcl::PointXYZI GridMapping::_transform(std_msgs::Header h, toposens_msgs::TsPoint p)
{
  geometry_msgs::PointStamped ps;
  ps.header = h;
  ps.point = p.location;
  tfListener.transformPoint(fixed_frame, ps, ps);

  pcl::PointXYZI point(p.intensity);
  point.x = ps.point.x;
  point.y = ps.point.y;
  point.z = ps.point.z;
  return point;
}

} // namespace toposens_pointcloud



/*


  GridMapping grid_mapping;

  ros::Rate loop_rate(200);

  int div = 100;
  srand(time(0));


  while (ros::ok()) {

    // move this chunk out of while statement if you want persistent points
   PointCloud::Ptr msg (new PointCloud);
    msg->header.frame_id = "odom";
    msg->height = 1;

    for (int i = -10; i < 10; i++) {
      for (int j = -10; j < 10; j++) {
        for (int k = 0; k < 10; k++) {
          if (rand() % 2 == 0) {
            float x = ((float)i)/div;
            float y = ((float)j)/div;
            float z = ((float)k)/div;
            float v = (x + y + z);
            pcl::PointXYZI point = pcl::PointXYZI(v);
            point.x = x;
            point.y = y;
            point.z = z;
            msg->points.push_back (point);
          }
        }
      }
    }

    msg->width = msg->points.size();
    ROS_WARN_STREAM(msg->points.size());
    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    grid_mapping.pc_pub.publish (msg);


    ros::spinOnce();
    loop_rate.sleep();

  }
*/
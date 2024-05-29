#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

void callback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  static ros::NodeHandle nh;
  static ros::Publisher pub = nh.advertise<sensor_msgs::LaserScan>("/scan/filtered", 10);
  auto copy = *scan;
  for (auto& v : copy.ranges) {
    if (std::isnan(v) || !std::isfinite(v) || v < copy.range_min || v > copy.range_max) {
      v = -1;
    }
  }
  pub.publish(copy);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "filter_lidar_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/scan/raw", 10, callback);
  ros::spin();
}

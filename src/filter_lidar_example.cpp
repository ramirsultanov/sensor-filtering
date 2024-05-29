#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <optional>

void callback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  static std::optional<float> lastValue = {};
  static std::size_t counterScans = 1;
  static std::size_t counterAvg = 0;
  static float sum = 0;
  auto& data = *scan;
  constexpr auto range = 10; //! degrees
  const std::size_t size = (data.angle_max - data.angle_min) / data.angle_increment;
  const auto mid = size / 2;
  const auto angleIncrementDegrees = 180 * data.angle_increment / float(M_PI);
  const std::size_t min = mid - range / 2 / angleIncrementDegrees;
  const std::size_t max = mid + range / 2 / angleIncrementDegrees;
  for (auto i = min; i <= max; i++) {
    if (data.ranges[i] == -1) {
      continue;
    }
    sum += data.ranges[i];
    counterAvg++;
  }
  if (counterScans % 10 == 0) {
    if (counterAvg != 0) {
      const auto avg = sum / counterAvg;
      counterAvg = 0;
      sum = 0;
      if (static_cast<bool>(lastValue)) {
        auto dif = avg - *lastValue;
        if (std::abs(dif) < 0.025) {
          dif = 0;
        }
        std::cout << "Difference: " << dif << '\n';
      }
      lastValue = avg;
    }
  }
  counterScans++;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "filter_lidar_example_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/scan/filtered", 10, callback);
  ros::spin();
}

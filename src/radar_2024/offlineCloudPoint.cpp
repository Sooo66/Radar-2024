#include <fstream>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <regex>
#include <string>

int main(int argc, char **argv) {
  ros::init(argc, argv, "offline_node");
  ros::NodeHandle nh;
  ros::Publisher publisher = nh.advertise<sensor_msgs::PointCloud2>("lidar", 1000);
  ros::Rate rate(10);

  std::ifstream ifs;
  ifs.open("/Radar-2024/src/radar_2024/assets/tup/pcds.txt");
  if (!ifs.is_open()) {
    ROS_ERROR("Failed to open txt.");
    return -1;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::PointCloud2 msg;

  std::string line;

  auto start = ros::Time::now();

  while (ros::ok()) {
    std::getline(ifs, line);
    if (line.compare("") == 0) {
      ifs.clear();
      ifs.seekg(0, std::ios::beg);
      auto end = ros::Time::now();
      std::cout << "total time: " << end - start << "\n";
      break;
    }

    if (line != "-e-") {
      line = std::regex_replace(line, std::regex("\\["), "");
      line = std::regex_replace(line, std::regex("\\]"), "");
      pcl::PointXYZ pt;
      std::stringstream ss(line);
      ss >> pt.x;
      ss >> pt.y;
      ss >> pt.z;
      cloud.points.push_back(pt);
    }
    else {
      msg.header.stamp = ros::Time::now();
      pcl::toROSMsg(cloud, msg);
      msg.header.frame_id = "map";
      std::cout << ros::Time::now() << "\n";
      publisher.publish(msg);
      rate.sleep();
      cloud.clear();
    }
  }

  ros::shutdown();

  return 0;
}
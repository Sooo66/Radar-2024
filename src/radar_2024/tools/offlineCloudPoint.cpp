#include <iostream>
#include <fstream>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <regex>

using namespace std;

int main(int argc, char **argv)
{
    string pcd_path = "/home/sooo/Desktop/Radar-2024/src/radar_2024/assets/tup/pcds.txt";
    ros::init(argc, argv, "lidar_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("lidar_node", 1000);
    bool if_exit_program = false;
    nh.setParam("/radar2023/ExitProgram", if_exit_program);
    ros::Rate rate(10);
    ifstream ifs;
    ifs.open(pcd_path, ios::in);
    if (!ifs.is_open())
    {
        ROS_ERROR("txt文件打开失败");
    }
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 output_msg;
    string line;
    while (ros::ok())
    {
        while (true)
        {
            ros::spinOnce();
            getline(ifs, line);
            if (line.compare("") == 0)
            {
                ifs.clear();
                ifs.seekg(0, ios::beg);
            }
            else
            {
                if (line == "-e-")
                {
                    output_msg.header.stamp = ros::Time::now();
                    pcl::toROSMsg(cloud, output_msg);
                    output_msg.header.frame_id = "map";
                    pub.publish(output_msg);
                    rate.sleep();
                    cloud.clear();
                }
                else
                {
                    line = regex_replace(line, regex("\\["), "");
                    line = regex_replace(line, regex("\\]"), "");
                    pcl::PointXYZ point;
                    std::stringstream ss(line);
                    ss >> point.x;
                    ss >> point.y;
                    ss >> point.z;
                    cloud.points.push_back(point);
                }
            }
            std::string param_name;
            if (nh.searchParam("/radar2023/ExitProgram", param_name))
            {
                nh.getParam(param_name, if_exit_program);
            }
            else
            {
                ROS_WARN("Parameter ExitProgram not defined");
            }
            if (if_exit_program)
            {
                break;
            }
        }
        break;
    }
    ros::shutdown();
    return 0;
}
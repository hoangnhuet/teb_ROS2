#ifndef CMD_VEL_SCAN_PROCESSOR_HPP_
#define CMD_VEL_SCAN_PROCESSOR_HPP_
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include"nav_msgs/msg/odometry.hpp"
#include <Eigen/Dense> 
class testObs: public rclcpp::Node
{
    public : 
        testObs();
    private:
        void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        double gain_x = 5.0;
        double gain_y  =5.0;
        double map_height = 400.0;
        double value;
        double angle;
        double x;
        double y;
        double obs_x;
        double obs_y; 
        // std::vector<Eigen::Vector2d> obs;
};

struct obstacle_point
{
    Eigen::Vector2d xy;
    double angle;
};

#endif 
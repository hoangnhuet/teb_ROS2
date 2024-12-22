#include"teb_ROS2/test_obs.h"
#include"teb_planner/include/obstacle/obstacle.h"
#include<iostream>
using namespace teb_local_planner;
int z = 0;
testObs::testObs(): Node("test_obs")
{
    scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&testObs::scanCallback, this, std::placeholders::_1));
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&testObs::odomCallback, this, std::placeholders::_1));

}
void testObs::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    std::vector<obstacle_point> vec_obs;
    std::vector<PointObstacle> global_obs;
    double local_angle;
    for (int i = 0; i < scan->ranges.size(); i++)
    {
        if (scan->ranges[i] != NAN)
        {
            value = scan->ranges[i];
            local_angle = scan->angle_min + i * scan->angle_increment;
            obs_x = x + value * cos(local_angle);
            obs_y = y + value * sin(local_angle);
            obstacle_point obs_point;
            obs_point.xy = Eigen::Vector2d(obs_x, obs_y);
            obs_point.angle = local_angle;
            vec_obs.push_back(obs_point);
        }
    }
    // use x,y to trans vec_obs to global frame
    for (int i = 0; i < vec_obs.size(); i++)
    {
        Eigen::Vector2d obs_vec = vec_obs[i].xy;
        Eigen::Vector2d obs_global = Eigen::Vector2d(x, y) + Eigen::Rotation2Dd(angle) * obs_vec;
        PointObstacle obs(obs_global);
        global_obs.push_back(obs);
    }
    // visualize all obstacle
    cv::Mat map = cv::Mat::zeros(600, 800, CV_8UC3);
    // if the obs ignored by NaN value so we cann not visualise it
    for (int i = 0; i< global_obs.size(); i++)
    {
        global_obs[i].visualize(map, cv::Scalar(0, 0, 255), gain_x, gain_y, map_height);
    }
    cv::imshow("Obstacle", map);
    cv::waitKey(1);
}

void testObs::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
    x = odom->pose.pose.position.x;
    y = odom->pose.pose.position.y;
    angle =  odom->pose.pose.orientation.z;
}
int main(int argc,char **argv)
{
    //if ctrlc, exit and stop node
    
    rclcpp::init(argc,argv);
    while (rclcpp::ok())
    {
        rclcpp::spin(std::make_shared<testObs>());
    }
    rclcpp::shutdown();
    return 0;
}
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>


class SimpleDWA {
public:
    SimpleDWA() {
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("ypspur_ros/cmd_vel", 10);
        global_path_sub_ = nh_.subscribe("global_path", 10, &SimpleDWA::globalPathCallback, this);
        laser_sub_ = nh_.subscribe("scan", 10, &SimpleDWA::laserCallback, this);
        amcl_sub_ = nh_.subscribe("/amcl_pose", 10, &SimpleDWA::amclPoseCallback, this);
        path_pub_ = nh_.advertise<nav_msgs::Path>("smooth_path", 10);
    }

    void globalPathCallback(const nav_msgs::Path::ConstPtr& msg) {
        waypoints_.clear();
        for (const auto& pose : msg->poses) {
            waypoints_.emplace_back(pose.pose.position.x, pose.pose.position.y);
        }
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        laser_ranges_ = msg->ranges;
        laser_min_angle_ = msg->angle_min;
        laser_max_angle_ = msg->angle_max;
        laser_increment_ = msg->angle_increment;
    }

    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        robot_r_ = tf::getYaw(msg->pose.pose.orientation);
        // robot_r_ = msg->pose.pose.orientation;

        // ROS_INFO("Current estimated pose: [robot_x: %f, robot_y: %f, theta: %f]", robot_x_, robot_y_, tf::getYaw(msg->pose.pose.orientation));
    }

    void computeControl() {
        if (waypoints_.empty()) return;

        double best_v = 0.0, best_w = 0.0;
        double max_score = -1e9;

        for (double v = 0.0; v <= max_vel_; v += vel_res_) {
            for (double w = -max_omega_; w <= max_omega_; w += omega_res_) {
                goal_cost = computeGoalCost(v, w);
                obstacle_cost = computeObstacleCost(v, w);
                velocity_cost = computeVelocityCost(v);

                // もともとは下のようになっていた
                // double score = goal_weight_ * goal_cost 
                //              - obstacle_weight_ * obstacle_cost 
                //              + velocity_weight_ * velocity_cost;

                double score = goal_weight_ * goal_cost 
                             + obstacle_weight_ * obstacle_cost 
                             + velocity_weight_ * velocity_cost;

                if (score > max_score) {
                
                    max_score = score;
                    best_v = v;
                    best_w = w;
                }
            }
        }
        std::cout << "goal_cost: " << goal_weight_*goal_cost << ", obstacle_cost: " << obstacle_weight_*obstacle_cost << ", velocity_cost: " << velocity_weight_*velocity_cost << std::endl;
        std::cout << std::endl;

        geometry_msgs::Twist cmd;
        cmd.linear.x = best_v;
        cmd.angular.z = best_w;
        cmd_pub_.publish(cmd);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_;
    ros::Subscriber global_path_sub_;
    ros::Subscriber laser_sub_;
    ros::Subscriber amcl_sub_;
    ros::Publisher path_pub_;
    std::vector<std::pair<double, double>> waypoints_;
    std::vector<float> laser_ranges_;
    double laser_min_angle_, laser_max_angle_, laser_increment_;
    double robot_x_, robot_y_, robot_r_;

    double goal_cost, obstacle_cost, velocity_cost;

    // パラメータ
    double max_vel_ = 0.3, max_omega_ = 1.0, vel_res_ = 0.01, omega_res_ = 0.01;
    double obstacle_weight_ = 0.1;   // 障害物回避の重み
    double goal_weight_ = 1.0;       // 目標到達の重み
    double velocity_weight_ = 0.5;   // 速度の重み
    double safe_distance_ = 2.0;     // 衝突回避のための距離閾値

    double computeGoalCost(double v, double w) {
        if (waypoints_.empty()) return 0.0;


        double goal_x = waypoints_.back().first;
        double goal_y = waypoints_.back().second;

        double goal_direction = std::atan2(goal_y - robot_y_, goal_x - robot_x_);
        // double predicted_direction = std::atan2(v * std::sin(w), v * std::cos(w));
        double predicted_direction = robot_r_ + w;
        // std::cout << "robot_r: " << robot_r_ << std::endl;
        // std::cout << "goal_direction: " << goal_direction << std::endl;
        // std::cout << "predicted_direction: " << predicted_direction << std::endl;
        double heading_error = 1.0 *std::abs(goal_direction - predicted_direction);
        return std::max(0.0, 1.0 - heading_error);
    }

    double computeObstacleCost(double v, double w) {
        if (laser_ranges_.empty()) return 0.0;

        double cost = 0.0;
        int center_index = laser_ranges_.size() / 2;
        int range_size = laser_ranges_.size() * 0.1;

        for (int i = center_index - range_size; i <= center_index + range_size; ++i) {
            if (i < 0 || i >= laser_ranges_.size()) continue;
            double point_x = laser_ranges_[i] * std::cos(laser_min_angle_ + i * laser_increment_);
            double point_y = laser_ranges_[i] * std::sin(laser_min_angle_ + i * laser_increment_);
            double distance_x = point_x - robot_x_ - v * std::cos(robot_r_ + w);
            double distance_y = point_y - robot_y_ - v * std::sin(robot_r_ + w);
            double distance = std::sqrt(distance_x * distance_x + distance_y * distance_y);
            if (distance < safe_distance_) {  
                cost += 1.0 / (distance + 0.1);
            }
        }
        return cost;
    }

    double computeVelocityCost(double v) {
        return v / max_vel_;  // 最大速度に近いほど高評価
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_dwa");
    SimpleDWA dwa;
    ros::Rate rate(10);
    while (ros::ok()) {
        dwa.computeControl();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

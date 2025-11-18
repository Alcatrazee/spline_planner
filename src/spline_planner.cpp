#include <algorithm>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <nav2_util/node_utils.hpp>
#include <cmath>
#include <memory>
#include <string>

#include "nav2_core/exceptions.hpp"
#include <spline_planner/spline_planner.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


namespace spline_planner
{
    using namespace std;
    void SplinePlanner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        tf_ = tf;
        node_ = parent.lock();
        name_ = name;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();

        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".spline_type", rclcpp::ParameterValue(QUINTIC_SPLINE));
        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".planner_param",rclcpp::ParameterValue(0.3));
        node_->get_parameter(name_ + ".interpolation_resolution",
                             interpolation_resolution_);
        node_->get_parameter(name_ + ".spline_type",
                            spline_switch_);
        node_->get_parameter(name_ + ".planner_param",
                             planner_param_);

        RCLCPP_INFO(node_->get_logger(), "SplinePlanner '%s' parameters:", name_.c_str());
        RCLCPP_INFO(node_->get_logger(), "  interpolation_resolution = %.6f", interpolation_resolution_);
        RCLCPP_INFO(node_->get_logger(), "  spline_type = %d", spline_switch_);
        RCLCPP_INFO(node_->get_logger(), "  planner_param = %.6f", planner_param_);
    }

    void SplinePlanner::cleanup()
    {
        RCLCPP_INFO(node_->get_logger(), "正在清理类型为 spline planner 的插件 %s",
                    name_.c_str());
    }

    void SplinePlanner::activate()
    {
        RCLCPP_INFO(node_->get_logger(), "正在激活类型为 spline planner 的插件 %s",
                    name_.c_str());
    }

    void SplinePlanner::deactivate()
    {
        RCLCPP_INFO(node_->get_logger(), "正在停用类型为 spline planner 的插件 %s",
                    name_.c_str());
    }

    nav_msgs::msg::Path
    SplinePlanner::createPlan(const geometry_msgs::msg::PoseStamped &start,
                              const geometry_msgs::msg::PoseStamped &goal)
    {
        // 1.声明并初始化 global_path
        nav_msgs::msg::Path global_path;
        global_path.poses.clear();
        global_path.header.stamp = node_->now();
        global_path.header.frame_id = global_frame_;

        // 2.检查目标和起始状态是否在全局坐标系中
        if (start.header.frame_id != global_frame_)
        {
            RCLCPP_ERROR(node_->get_logger(), "规划器仅接受来自 %s 坐标系的起始位置",
                         global_frame_.c_str());
            return global_path;
        }
        // 3. get path
        cout << "generating path" << endl;

        double start_yaw = tf2::getYaw(start.pose.orientation);
        double end_yaw = tf2::getYaw(goal.pose.orientation);

        const vector<double> start_pose{start.pose.position.x, start.pose.position.y,start_yaw};
        const vector<double> end_pose{goal.pose.position.x, goal.pose.position.y,end_yaw};
        vector<vector<double>> vec_path;

        bool dir = true;
        double yaw_start_to_goal = atan2(end_pose[1]-start_pose[1],end_pose[0]-start_pose[0]);
        double yaw_diff = std::remainder(yaw_start_to_goal-start_pose[2], 2.0 * M_PI);
        if(abs(yaw_diff) > (M_PI/2))
        {
            dir = false;
        }
        cout << "yaw diff" << yaw_diff << endl;

        bool succ;
        switch (spline_switch_) {
            case BEZIER: {
                bezier_curve_generator bplanner(bezier_curve_generator::RATIO,interpolation_resolution_);
                succ = bplanner.generate_bezier_curve(start_pose, end_pose, vec_path,dir); break;
            }
            case CUBIC_SPLINE: {
                CubicSplineGenerator cSpline(interpolation_resolution_,planner_param_); 
                succ = cSpline.generate(start_pose, end_pose, vec_path,dir);
                break;
            }
            case QUINTIC_SPLINE: {
                QuinticSpline2D qSpline(interpolation_resolution_,planner_param_);
                succ = qSpline.generate(start_pose, end_pose, vec_path,dir);
                break;
            }
            default: {
                bezier_curve_generator bplanner(bezier_curve_generator::RATIO,interpolation_resolution_);
                succ = bplanner.generate_bezier_curve(start_pose, end_pose, vec_path,dir);
            }
        }

        if(succ == true){
            int counter = 0;
            for(auto point : vec_path){
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = global_frame_;
                pose.header.stamp = node_->now();
                pose.pose.position.x = point[0];
                pose.pose.position.y = point[1];
                if(counter == 0){
                    pose.pose.orientation = start.pose.orientation;
                }else if(counter == vec_path.size()-1){
                    pose.pose.orientation = goal.pose.orientation;
                }else{
                    double yaw  = 0;
                    if(dir == true)
                       yaw = atan2(vec_path[counter][1]-vec_path[counter-1][1],vec_path[counter][0]-vec_path[counter-1][0]);
                    else
                       yaw = atan2(vec_path[counter-1][1]-vec_path[counter][1],vec_path[counter-1][0]-vec_path[counter][0]);
                    pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0,0,1),yaw));
                }
                counter++;
                global_path.poses.push_back(pose);
            }
        }else{
            cout << "generate path failed" << endl;
            throw nav2_core::PlannerException(
           "unable to plan" + std::to_string(goal.pose.position.x) + "," +
            std::to_string(goal.pose.position.y));
        }
        cout << "generate done." << endl;

        // 5.使用 costmap 检查该条路径是否经过障碍物
        for(auto pose : global_path.poses){
            uint32_t mx,my;     // grid cell index
            if(costmap_ ->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)){
                uint8_t cost = costmap_->getCost(mx, my);
                if (cost == nav2_costmap_2d::LETHAL_OBSTACLE){
                    RCLCPP_ERROR(node_->get_logger(), "规划器在(%lf,%lf)发现障碍物",
                        pose.pose.position.x, pose.pose.position.y);
                    //throw nav2_core::PlannerException(
                    //    "unable to plan" + std::to_string(goal.pose.position.x) + "," +
                    //    std::to_string(goal.pose.position.y));
                }
            }
        }

        // 6.收尾，将目标点作为路径的最后一个点并返回路径
        // geometry_msgs::msg::PoseStamped goal_pose = goal;
        // goal_pose.header.frame_id = global_frame_;
        // goal_pose.header.stamp = node_->now();
        // global_path.poses.push_back(goal_pose);
        return global_path;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(spline_planner::SplinePlanner, nav2_core::GlobalPlanner)

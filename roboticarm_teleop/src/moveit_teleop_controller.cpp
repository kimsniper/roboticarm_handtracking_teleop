#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <mutex>

class MoveItTeleopController : public rclcpp::Node
{
public:
    MoveItTeleopController()
        : Node("moveit_teleop_controller")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing MoveIt Teleop Controller...");

        // MoveIt setup (Arm group)
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(rclcpp::Node::SharedPtr(this, [](auto*){}),"arm");

        move_group_->setPlanningTime(1.0);
        move_group_->setNumPlanningAttempts(5);
        move_group_->setMaxVelocityScalingFactor(0.5);
        move_group_->setMaxAccelerationScalingFactor(0.5);

        move_group_->setPoseReferenceFrame("base_link");
        move_group_->setEndEffectorLink("clawbase");

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target_pose",
            10,
            std::bind(&MoveItTeleopController::poseCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10 Hz safe teleop
            std::bind(&MoveItTeleopController::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "MoveIt Teleop Controller Ready.");
    }

private:

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::PoseStamped latest_pose_;
    std::mutex pose_mutex_;
    bool has_new_pose_ = false;

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        latest_pose_ = *msg;
        has_new_pose_ = true;
    }

    // Safety filter
    bool isPoseSafe(const geometry_msgs::msg::Pose& pose)
    {
        if (pose.position.x < -0.6 || pose.position.x > 0.6) return false;
        if (pose.position.y < -0.6 || pose.position.y > 0.6) return false;
        if (pose.position.z < 0.05 || pose.position.z > 0.8) return false;

        return true;
    }

    void controlLoop()
    {
        geometry_msgs::msg::PoseStamped target;

        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            if (!has_new_pose_)
                return;

            target = latest_pose_;
            has_new_pose_ = false;
        }

        if (!isPoseSafe(target.pose))
        {
            RCLCPP_WARN(this->get_logger(), "Unsafe pose rejected");
            return;
        }

        move_group_->setPoseTarget(target);

        moveit::planning_interface::MoveGroupInterface::Plan plan;

        bool success =
            (move_group_->plan(plan) ==
             moveit::core::MoveItErrorCode::SUCCESS);

        if (!success)
        {
            RCLCPP_WARN(this->get_logger(), "Planning failed");
            return;
        }

        auto exec_result = move_group_->execute(plan);

        if (exec_result != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Execution failed");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Executed pose successfully");
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveItTeleopController>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
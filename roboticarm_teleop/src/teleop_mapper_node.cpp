#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>

#include <mutex>
#include <algorithm>

class TeleopMapperNode : public rclcpp::Node
{
public:
    TeleopMapperNode()
    : Node("teleop_mapper_node")
    {
        RCLCPP_INFO(this->get_logger(), "Teleop Mapper Node Started");

        hand_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/hand_tracking/hand_pose", 10,
            std::bind(&TeleopMapperNode::handCallback, this, std::placeholders::_1));

        pinch_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/hand_tracking/pinch_distance", 10,
            std::bind(&TeleopMapperNode::pinchCallback, this, std::placeholders::_1));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/target_pose", 10);

        gripper_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "/gripper_command", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 20 Hz control
            std::bind(&TeleopMapperNode::updateLoop, this));

        filtered_x_ = 0.0;
        filtered_y_ = 0.0;
        filtered_z_ = 0.3;

        RCLCPP_INFO(this->get_logger(), "Mapper Ready.");
    }

private:

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr hand_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pinch_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gripper_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex mutex_;

    double raw_x_ = 0.0;
    double raw_y_ = 0.0;
    double raw_z_ = 0.3;

    float pinch_ = 1.0;

    double filtered_x_;
    double filtered_y_;
    double filtered_z_;

    void handCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        raw_x_ = msg->pose.position.x;
        raw_y_ = msg->pose.position.y;
        raw_z_ = msg->pose.position.z;
    }

    void pinchCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pinch_ = msg->data;
    }

    double mapRange(double v, double in_min, double in_max,
                    double out_min, double out_max)
    {
        return (v - in_min) * (out_max - out_min) /
               (in_max - in_min) + out_min;
    }

    double lowPass(double prev, double current, double alpha)
    {
        return alpha * prev + (1.0 - alpha) * current;
    }

    bool deadzone(double v, double threshold)
    {
        return std::abs(v) > threshold;
    }

    void updateLoop()
    {
        double x, y, z;
        float pinch;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            x = raw_x_;
            y = raw_y_;
            z = raw_z_;
            pinch = pinch_;
        }

        // Scale camera
        double target_x = mapRange(x, 0.0, 1.0, -0.35, 0.35);
        double target_y = mapRange(y, 0.0, 1.0, -0.35, 0.35);
        double target_z = mapRange(z, 0.0, 1.0, 0.15, 0.6);

        filtered_x_ = lowPass(filtered_x_, target_x, 0.8);
        filtered_y_ = lowPass(filtered_y_, target_y, 0.8);
        filtered_z_ = lowPass(filtered_z_, target_z, 0.8);

        if (!deadzone(filtered_x_, 0.005)) filtered_x_ = 0.0;
        if (!deadzone(filtered_y_, 0.005)) filtered_y_ = 0.0;

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->now();
        pose.header.frame_id = "base_link";

        pose.pose.position.x = filtered_x_;
        pose.pose.position.y = filtered_y_;
        pose.pose.position.z = filtered_z_;

        // fixed orientation for now (wrist controlled later)
        pose.pose.orientation.w = 1.0;

        pose_pub_->publish(pose);

        std_msgs::msg::Float32 grip;

        grip.data = (pinch < 0.05) ? 1.0f : 0.0f;

        gripper_pub_->publish(grip);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopMapperNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
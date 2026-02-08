#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <stdio.h>
#include <std_msgs/msg/string.hpp>
#include "geometry_msgs/msg/pose.hpp"
#include <chrono>
#include <iostream>
#include <memory>
// #include <libusbp.hpp>
#include <streambuf>
#include <thread>

#include "std_msgs/msg/int8.hpp"

enum Axes
{
    XL,
    YL,
    XR,
    L2_FLOAT,
    R2_FLOAT,
    YR,
    X_BTN_FLOAT,
    Y_BTN_FLOAT
};

enum Buttons
{
    SQUARE,
    CROSS,
    CIRCLE,
    TRIANGLE,
    L1,
    R1,
    L2,
    R2,
    SHARE,
    OPTIONS,
    X_BTN,
    Y_BTN,
    PS
};

struct Joystick
{
    float axes[8];
    int buttons[13];
};

float JoySpeed[3];

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode() : Node("controller_node")
    {
        RCLCPP_INFO(this->get_logger(), "controller_node has started");

        sub_joy = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", rclcpp::QoS(10), std::bind(&ControllerNode::joy_callback, this, std::placeholders::_1));
        pub_joy = this->create_publisher<geometry_msgs::msg::Pose>("pose", 10);
        feedback = this->create_subscription<std_msgs::msg::Int8>("feedback", 10, std::bind(&ControllerNode::feedback_callback, this, std::placeholders::_1));
        current_pose.position.x = 0.0;
        current_pose.position.y = 0.0;
        current_pose.position.z = 0.7;
        before.position.x = 0.0;
        before.position.y = 0.0;   
        before.position.z = 0.7;
    }
private:
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr feedback;
    Joystick jb;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_joy;
    geometry_msgs::msg::Pose current_pose;
    geometry_msgs::msg::Pose before;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {

        // if (msg->axes.size() < 8 || msg->buttons.size() < 13)
        // {
        //     RCLCPP_WARN(this->get_logger(), "Received Joy message with insufficient axes or buttons");
        //     return;
        // }

        jb.axes[XL] = msg->axes[0]; // Left stick X
        jb.axes[YL] = msg->axes[1]; // Left stick Y
        jb.axes[XR] = msg->axes[3]; // Right stick X (rotation)
        jb.axes[L2_FLOAT] = msg->axes[2];
        jb.axes[R2_FLOAT] = msg->axes[5];
        jb.axes[YR] = msg->axes[4];
        jb.axes[X_BTN_FLOAT] = msg->axes[6];
        jb.axes[Y_BTN_FLOAT] = msg->axes[7];

        jb.buttons[SQUARE] = msg->buttons[0];
        jb.buttons[CROSS] = msg->buttons[1];
        jb.buttons[CIRCLE] = msg->buttons[2];
        jb.buttons[TRIANGLE] = msg->buttons[3];
        jb.buttons[L1] = msg->buttons[4];
        jb.buttons[R1] = msg->buttons[5];
        jb.buttons[L2] = msg->buttons[6];
        jb.buttons[R2] = msg->buttons[7];
        jb.buttons[SHARE] = msg->buttons[8];
        jb.buttons[OPTIONS] = msg->buttons[9];
        jb.buttons[X_BTN] = msg->buttons[10];
        jb.buttons[Y_BTN] = msg->buttons[11];
        jb.buttons[PS] = msg->buttons[12];

        if (jb.axes[XL] > 0.1 || jb.axes[XL] < -0.1)
        {
            current_pose.position.y -= jb.axes[XL] * 0.01; // Adjust sensitivity as needed
        }
        if (jb.axes[YL] > 0.1 || jb.axes[YL] < -0.1)
        {
            current_pose.position.x += jb.axes[YL] * 0.01; // Adjust sensitivity as needed
        }
        if (jb.axes[YR] > 0.1 || jb.axes[YR] < -0.1)
        {
            current_pose.position.z += jb.axes[YR] * 0.01; // Adjust sensitivity as needed
        }
        RCLCPP_INFO(this->get_logger(), "Current Pose - x: %.2f, y: %.2f, z: %.2f",
                    current_pose.position.x,
                    current_pose.position.y,
                    current_pose.position.z);
        pub_joy->publish(current_pose);
    }

    void feedback_callback(const std_msgs::msg::Int8::SharedPtr msg)
    {
        if (msg->data == 0)
        {
            current_pose = before;
        }
        else
        {
            before = current_pose;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}


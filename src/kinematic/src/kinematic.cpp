#include <iostream>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

class kinematic : public rclcpp::Node
{
public:
    kinematic() : Node("kinematic_node")
    {
    pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", 10
    );
    sub_ = this->create_subscription<geometry_msgs::msg::Pose>("pose", 10,
        std::bind(&kinematic::PoseCallback, this, std::placeholders::_1)
    );
    controller_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/joint_trajectory_controller/joint_trajectory", 10
    );
    feedback = this->create_publisher<std_msgs::msg::Int8>("feedback", 10);
    }
private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr controller_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr feedback;
    sensor_msgs::msg::JointState before;
    std_msgs::msg::Int8 feedback_msg;
    double buffer[5];

    const double L1 = 0.05;
    const double L2 = 0.3;
    const double L3 = 0.3;   // wrist pitch link
    const double L4 = 0.10;   // gripper
    const double BASE = 0.05;

    // joint limits
    const double J1_MIN = -3.14;
    const double J1_MAX =  3.14;

    const double J2_MIN = -1.57;
    const double J2_MAX =  1.57;

    const double J3_MIN = -2.0;
    const double J3_MAX =  2.0;

    const double J4_MIN = -1.57;
    const double J4_MAX =  1.57;

    const double J5_MIN = -3.14;
    const double J5_MAX =  3.14;

    bool within_limits(double t1, double t2, double t3,
                   double t4, double t5)
    {
        return
            (t1 >= J1_MIN && t1 <= J1_MAX) &&
            (t2 >= J2_MIN && t2 <= J2_MAX) &&
            (t3 >= J3_MIN && t3 <= J3_MAX) &&
            (t4 >= J4_MIN && t4 <= J4_MAX) &&
            (t5 >= J5_MIN && t5 <= J5_MAX);
    }
    
    void PoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        double x = msg->position.x;
        double y = msg->position.y;
        double z = msg->position.z;

        double roll, pitch, yaw, w;
        roll = msg->orientation.x;
        pitch = msg->orientation.y;
        yaw = msg->orientation.z;
        w = msg->orientation.w;

        // =============================
        // Joint 1
        // =============================

        double theta1 = atan2(y, x);

        // =============================
        // Wrist Center
        // =============================

        double wx = x - (L4) * cos(theta1) * sin(pitch);
        double wy = y - (L4) * sin(theta1) * sin(pitch);
        double wz = z - (L4) * cos(pitch);

        // =============================
        // Planar reduction
        // =============================
        // After wrist center calculation:
        double r = sqrt(wx*wx + wy*wy);
        double s = wz - BASE - L1;  // L1 is vertical offset

        // Solve for L2 and L3 (not L1 and L2!)
        double D = (r*r + s*s - L2*L2 - L3*L3) / (2 * L2 * L3);

        if (fabs(D) > 1.0)
        {
            RCLCPP_WARN(get_logger(), "Target unreachable");
            feedback_msg.data = 0;
            feedback->publish(feedback_msg);
            return;
        }

        // =============================
        // Elbow DOWN solution
        // =============================
        
        double theta3 = M_PI - atan2(-sqrt(1 - D*D), -D);
        double theta2 = atan2(r, s) - atan2(L3 * sin(theta3), L2 + L3*cos(theta3));

        // =============================
        // Wrist pitch
        // =============================
        double theta4 = pitch - (theta2 + theta3);

        // Wrist roll
        double theta5 = roll;

        // if(!within_limits(theta1, theta2, theta3, theta4, theta5))
        // {
        //     RCLCPP_WARN(get_logger(), "Joint limit exceeded — rejecting pose");
        //     feedback_msg.data = 0;
        //     feedback->publish(feedback_msg);
        //     return;
        // }

        // =============================
        // Publish JointState
        // =============================
        sensor_msgs::msg::JointState js;
        trajectory_msgs::msg::JointTrajectory traj;

        js.header.stamp = now();

        js.name = {
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5"
        };

        js.position = {
            theta1,
            theta2,
            theta3,
            theta4,
            theta5
        };

        traj.joint_names = js.name;
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = js.position;
        point.time_from_start = rclcpp::Duration::from_seconds(0.1);
        traj.points.push_back(point);

        controller_->publish(traj);
        feedback_msg.data = 1;
        feedback->publish(feedback_msg);

    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<kinematic>());
    rclcpp::shutdown();
    return 0;
}

/*
x = 0.30
y = 0.00
z = 0.35

x = 0.35
y = 0
z = 0.15

x = 0.20
y = 0
z = 0.55

x = 0.25
y = 0.25
z = 0.30

*/
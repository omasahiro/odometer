#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>
#include <cmath>
#include "odometer.hpp"
#include <pigpiod_if2.h>

using namespace std::chrono_literals;

struct EncoderPins {
    int pinA;
    int pinB;
};

EncoderPins pins[2] = {
    {17, 27},
    {16, 26}
};

int main(int argc, char *argv[]) {
    int pi = pigpio_start(NULL, NULL);
    if (pi < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("odom_publisher"), "Failed to initialize pigpio.");
        return -1;
    }

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("odom_publisher");
    auto odom_publisher = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    Odometer encoders[2] = {
        Odometer(pi, pins[0].pinA, pins[0].pinB),
        Odometer(pi, pins[1].pinA, pins[1].pinB)
    };

    auto publish_odometry = [&]() {
        auto message = nav_msgs::msg::Odometry();
        
        int left_ticks = encoders[0].getRotation();
        int right_ticks = encoders[1].getRotation();
        
        double wheel_radius = 0.385;  //[m]
        double wheel_base = 0.5;    //[m]
        double ticks_per_revolution = 2048.0;

        double left_distance = 2 * M_PI * wheel_radius * (left_ticks / ticks_per_revolution);
        double right_distance = 2 * M_PI * wheel_radius * (right_ticks / ticks_per_revolution);
        double distance = (left_distance + right_distance) / 2.0;

        double delta_time = 0.1;
        double linear_velocity = distance / delta_time;
        double angular_velocity = (right_distance - left_distance) / wheel_base / delta_time;

        message.header.stamp = node->get_clock()->now();
        message.header.frame_id = "odom";

        message.pose.pose.position.x += distance * cos(message.pose.pose.orientation.z);
        message.pose.pose.position.y += distance * sin(message.pose.pose.orientation.z);

        message.twist.twist.linear.x = linear_velocity;
        message.twist.twist.angular.z = angular_velocity;

        odom_publisher->publish(message);
    };

    auto timer = node->create_wall_timer(100ms, publish_odometry);

    rclcpp::spin(node);
    rclcpp::shutdown();
    pigpio_stop(pi);
    return 0;
}

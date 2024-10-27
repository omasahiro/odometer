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
    {17, 27}, // 左クローラー
    {16, 26}  // 右クローラー
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
        Odometer(pi, pins[0].pinA, pins[0].pinB), // 左クローラーエンコーダ
        Odometer(pi, pins[1].pinA, pins[1].pinB)  // 右クローラーエンコーダ
    };

    double crawler_radius = 0.0391; // クローラーの半径（メートル）
    double wheel_base = 0.389;      // クローラー間の距離（メートル）
    double ticks_per_revolution = 2048.0; // エンコーダの1回転あたりのティック数
    double x = 0.0; // ロボットのX位置
    double y = 0.0; // ロボットのY位置
    double theta = 0.0; // ロボットの向き

    auto publish_odometry = [&]() {
        auto message = nav_msgs::msg::Odometry();

        int left_ticks = encoders[0].getRotation();
        int right_ticks = encoders[1].getRotation();

        // クローラーの半径を使用して移動距離を計算
        double left_distance = 2 * M_PI * crawler_radius * (left_ticks / ticks_per_revolution);
        double right_distance = 2 * M_PI * crawler_radius * (right_ticks / ticks_per_revolution);
        double distance = (left_distance + right_distance) / 2.0; // 平均距離

        double delta_time = 0.1; // 0.1秒間隔で更新
        double linear_velocity = distance / delta_time; // 線速度の計算
        double angular_velocity = (right_distance - left_distance) / wheel_base / delta_time; // 角速度の計算

        // 新しい位置の計算
        theta += angular_velocity * delta_time; // 新しい角度の計算
        x += distance * cos(theta); // X座標の更新
        y += distance * sin(theta); // Y座標の更新

        message.header.stamp = node->get_clock()->now();
        message.header.frame_id = "odom";

        message.pose.pose.position.x = x;
        message.pose.pose.position.y = y;

        // オリエンテーションの設定（クォータニオン）
        message.pose.pose.orientation.z = sin(theta / 2);
        message.pose.pose.orientation.w = cos(theta / 2);

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

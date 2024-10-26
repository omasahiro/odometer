#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>
#include <cmath>
#include "odometer/odometer.hpp"

using namespace std::chrono_literals;

// ピン番号の定義
struct EncoderPins {
    int pinA;
    int pinB;
};

EncoderPins pins[2] = {
    {17, 27},
    {16, 26}
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("odom_publisher");
    auto odom_publisher = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    // エンコーダオブジェクトの配列を初期化
    Odometer encoders[2] = {
        Odometer(pins[0].pinA, pins[0].pinB),
        Odometer(pins[1].pinA, pins[1].pinB)
    };

    auto publish_odometry = [&]() {
        auto message = nav_msgs::msg::Odometry();
        
        // エンコーダの回転数を取得
        int left_ticks = encoders[0].getRotation();
        int right_ticks = encoders[1].getRotation();
        
        // 定数
        double wheel_radius = 0.1; // 車輪の半径
        double wheel_base = 0.5;   // 車輪間の距離
        double ticks_per_revolution = 2048.0; // 1回転あたりのパルス数

        // 左右の車輪の移動距離を計算
        double left_distance = 2 * M_PI * wheel_radius * (left_ticks / ticks_per_revolution);
        double right_distance = 2 * M_PI * wheel_radius * (right_ticks / ticks_per_revolution);
        double distance = (left_distance + right_distance) / 2.0;

        // 速度を計算
        double delta_time = 0.1; // サンプリング周期（秒）
        double linear_velocity = distance / delta_time;
        double angular_velocity = (right_distance - left_distance) / wheel_base / delta_time;

        // オドメトリのメッセージを構築
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
    return 0;
}



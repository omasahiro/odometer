cmake_minimum_required(VERSION 3.5)
project(odometer)

# 依存関係の宣言
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# ヘッダーファイルのディレクトリを含める
include_directories(include)

# ライブラリの追加
add_library(odometer_lib src/odometer.cpp)
ament_target_dependencies(odometer_lib rclcpp std_msgs)

# 実行ファイルの追加
add_executable(odometer_node src/main.cpp)
target_link_libraries(odometer_node odometer_lib)
ament_target_dependencies(odometer_node rclcpp std_msgs wiringPi)

# インストール指示
install(TARGETS
  odometer_lib
  odometer_node
  DESTINATION lib/${PROJECT_NAME})

# パッケージの定義
ament_package()


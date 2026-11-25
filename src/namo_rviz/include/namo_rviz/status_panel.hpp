#pragma once
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <QSize>
#include <QLabel>
#include <QTimer>
#include <chrono>
namespace namo_rviz {
class StatusPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit StatusPanel(QWidget * parent=nullptr);
  void onInitialize() override;
private:
  QLabel *label_;
  QLabel *freshness_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  double map_width_=0., map_height_=0., center_x_=0., center_y_=0.;
  QSize fitted_size_;
  void fitMap();
  std::chrono::steady_clock::time_point last_{};
};
}

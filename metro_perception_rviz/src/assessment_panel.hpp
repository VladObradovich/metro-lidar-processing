#pragma once

#include <QLineEdit>
#include <QPlainTextEdit>
#include <QString>

#include "metro_perception_interfaces/msg/path_assessment.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"

namespace metro_perception_rviz {

// RViz panel with the latest PathAssessment as plain text, like `ros2 topic
// echo`.
class AssessmentPanel : public rviz_common::Panel {
  Q_OBJECT

 public:
  explicit AssessmentPanel(QWidget* parent = nullptr);
  void onInitialize() override;
  void load(const rviz_common::Config& config) override;
  void save(rviz_common::Config config) const override;

 private Q_SLOTS:
  void subscribe();
  void show_text(const QString& text);

 private:
  QLineEdit* topic_;
  QPlainTextEdit* text_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<metro_perception_interfaces::msg::PathAssessment>::SharedPtr subscription_;
};

}  // namespace metro_perception_rviz

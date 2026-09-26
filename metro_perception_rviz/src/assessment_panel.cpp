#include "assessment_panel.hpp"

#include <QFontDatabase>
#include <QVBoxLayout>

#include "metro_perception_rviz/assessment_text.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"

namespace metro_perception_rviz {

AssessmentPanel::AssessmentPanel(QWidget* parent)
    : rviz_common::Panel(parent),
      topic_(new QLineEdit("/metro/assessment")),
      text_(new QPlainTextEdit) {
  text_->setReadOnly(true);
  text_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
  text_->setPlainText("waiting for assessment");
  auto* layout = new QVBoxLayout;
  layout->addWidget(topic_);
  layout->addWidget(text_);
  setLayout(layout);
  connect(topic_, &QLineEdit::editingFinished, this, &AssessmentPanel::subscribe);
}

void AssessmentPanel::onInitialize() {
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  subscribe();
}

void AssessmentPanel::subscribe() {
  if (!node_) return;
  subscription_.reset();
  const auto topic = topic_->text().trimmed().toStdString();
  if (topic.empty()) return;
  // The executor may run apart from the Qt thread: hand the text over as a
  // queued call.
  subscription_ = node_->create_subscription<metro_perception_interfaces::msg::PathAssessment>(
      topic, rclcpp::QoS(1),
      [this](metro_perception_interfaces::msg::PathAssessment::ConstSharedPtr assessment) {
        const auto text = QString::fromStdString(assessment_text(*assessment));
        QMetaObject::invokeMethod(this, "show_text", Qt::QueuedConnection, Q_ARG(QString, text));
      });
}

void AssessmentPanel::show_text(const QString& text) {
  if (text_->toPlainText() != text) text_->setPlainText(text);
}

void AssessmentPanel::load(const rviz_common::Config& config) {
  rviz_common::Panel::load(config);
  QString topic;
  if (config.mapGetString("Topic", &topic)) {
    topic_->setText(topic);
    subscribe();
  }
}

void AssessmentPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  config.mapSetValue("Topic", topic_->text());
}

}  // namespace metro_perception_rviz

PLUGINLIB_EXPORT_CLASS(metro_perception_rviz::AssessmentPanel, rviz_common::Panel)

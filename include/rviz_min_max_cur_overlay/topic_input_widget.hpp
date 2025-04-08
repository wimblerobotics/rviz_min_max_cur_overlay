#ifndef RVIZ_MIN_MAX_CUR_OVERLAY__TOPIC_INPUT_WIDGET_HPP_
#define RVIZ_MIN_MAX_CUR_OVERLAY__TOPIC_INPUT_WIDGET_HPP_

#include <QWidget>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSignalMapper>

#include <rclcpp/rclcpp.hpp>

namespace rviz_min_max_cur_overlay
{

class TopicInputWidget : public QWidget
{
  Q_OBJECT

public:
  TopicInputWidget(QWidget *parent = nullptr);
  ~TopicInputWidget() override;

  std::string getTopic() const;
  void setTopic(const std::string &topic);

signals:
  void topicChanged(const std::string &topic);

private slots:
  void onTopicEdited();

private:
  QLineEdit *topic_line_edit_{nullptr};
  QHBoxLayout *main_layout_{nullptr};
};

}  // namespace rviz_min_max_cur_overlay

#endif  // RVIZ_MIN_MAX_CUR_OVERLAY__TOPIC_INPUT_WIDGET_HPP_

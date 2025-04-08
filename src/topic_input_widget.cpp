#include "rviz_min_max_cur_overlay/topic_input_widget.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>

namespace rviz_min_max_cur_overlay
{

TopicInputWidget::TopicInputWidget(QWidget *parent)
: QWidget(parent)
{
  main_layout_ = new QHBoxLayout(this);

  QLabel *topic_label = new QLabel("Topic:", this);
  main_layout_->addWidget(topic_label);

  topic_line_edit_ = new QLineEdit(this);
  main_layout_->addWidget(topic_line_edit_);

  connect(topic_line_edit_, &QLineEdit::editingFinished, this, &TopicInputWidget::onTopicEdited);

  setLayout(main_layout_);
}

TopicInputWidget::~TopicInputWidget()
{
}

std::string TopicInputWidget::getTopic() const
{
  return topic_line_edit_->text().toStdString();
}

void TopicInputWidget::setTopic(const std::string &topic)
{
  topic_line_edit_->setText(QString::fromStdString(topic));
}

void TopicInputWidget::onTopicEdited()
{
  emit topicChanged(getTopic());
}

}  // namespace rviz_min_max_cur_overlay

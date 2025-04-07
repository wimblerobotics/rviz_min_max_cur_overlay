#include "rviz_min_max_cur_overlay/overlay_display.hpp"

// Export the plugin class first to ensure proper symbol linkage
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_min_max_cur_overlay::OverlayDisplay, rviz_common::Display)

// Standard includes
#include <memory>
#include <string>

// Qt includes
#include <QPainter>
#include <QFontMetrics>
#include <QTextOption>
#include <QDir>
#include <QBitmap>
#include <QPalette>

// ROS includes
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/validate_floats.hpp>

// Create a new function to validate floats in our custom message type
bool validateMinMaxCurrMsg(const rviz_min_max_cur_overlay::msg::MinMaxCurr & msg)
{
  // Check each float field
  return rviz_common::validateFloats(msg.min) &&
         rviz_common::validateFloats(msg.max) &&
         rviz_common::validateFloats(msg.current) &&
         rviz_common::validateFloats(msg.current_color);
}

namespace rviz_min_max_cur_overlay
{

OverlayDisplay::OverlayDisplay()
: Display()
{
  // Initialize topic property - this is critical for correct functionality
  // Note the exact message type name must match what's registered with ROS
  topic_property_ = new rviz_common::properties::RosTopicProperty(
    "Topic", "",
    "rviz_min_max_cur_overlay/msg/MinMaxCurr",
    "Topic to subscribe to for min/max/current values",
    this,
    SLOT(updateTopic()));

  // Initialize other properties
  width_property_ = new rviz_common::properties::IntProperty(
    "Width", 200, "Width of the overlay in pixels", this, SLOT(updateVisualization()));
  width_property_->setMin(10);

  height_property_ = new rviz_common::properties::IntProperty(
    "Height", 20, "Height of the overlay in pixels", this, SLOT(updateVisualization()));
  height_property_->setMin(10);

  orientation_property_ = new rviz_common::properties::EnumProperty(
    "Orientation", "Horizontal", "Orientation of the overlay", this, SLOT(updateVisualization()));
  orientation_property_->addOption("Horizontal", 0);
  orientation_property_->addOption("Vertical", 1);

  display_mode_property_ = new rviz_common::properties::EnumProperty(
    "Display Mode", "Line", "How to display the current value", this, SLOT(updateVisualization()));
  display_mode_property_->addOption("Line", 0);
  display_mode_property_->addOption("Rectangle", 1);

  line_width_property_ = new rviz_common::properties::FloatProperty(
    "Line Width", 2.0, "Width of the current value line", this, SLOT(updateVisualization()));
  line_width_property_->setMin(0.1);

  border_width_property_ = new rviz_common::properties::FloatProperty(
    "Border Width", 1.0, "Width of the plot border", this, SLOT(updateVisualization()));
  border_width_property_->setMin(0.1);

  font_size_property_ = new rviz_common::properties::IntProperty(
    "Font Size", 12, "Size of the font for values", this, SLOT(updateVisualization()));
  font_size_property_->setMin(6);

  background_color_property_ = new rviz_common::properties::ColorProperty(
    "Background Color", QColor(0, 0, 0, 50), "Background color of the plot", this,
    SLOT(updateVisualization()));

  text_color_property_ = new rviz_common::properties::ColorProperty(
    "Text Color", QColor(255, 255, 255), "Color of the text labels", this,
    SLOT(updateVisualization()));
}

// Explicitly define the destructor
OverlayDisplay::~OverlayDisplay()
{
  unsubscribe();
  if (scene_node_ && scene_manager_) {
    scene_manager_->destroySceneNode(scene_node_);
    scene_node_ = nullptr;
  }
}

void OverlayDisplay::onInitialize()
{
  Display::onInitialize();

  if (!scene_manager_) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Scene Manager", "No scene manager found");
    return;
  }

  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  // Create the overlay widget for display
  overlay_widget_ = std::make_unique<QWidget>();
  overlay_widget_->setAttribute(Qt::WA_TranslucentBackground);
  overlay_widget_->setWindowFlags(Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);
  overlay_widget_->setAutoFillBackground(true);

  // Create an initial subscription
  updateTopic();
}

void OverlayDisplay::onEnable()
{
  Display::onEnable();
  if (overlay_widget_) {
    overlay_widget_->show();
  }
  updateVisualization();
}

void OverlayDisplay::onDisable()
{
  Display::onDisable();
  if (overlay_widget_) {
    overlay_widget_->hide();
  }
}

void OverlayDisplay::reset()
{
  Display::reset();
  updateVisualization();
}

void OverlayDisplay::unsubscribe()
{
  subscriber_.reset();
}

void OverlayDisplay::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;
  (void)ros_dt;
  updateVisualization();
}

bool OverlayDisplay::validateMessage(const rviz_min_max_cur_overlay::msg::MinMaxCurr::ConstSharedPtr & msg)
{
  if (msg->min > msg->max) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Message",
      "Invalid message: min value is greater than max value");
    return false;
  }

  if (msg->current_color.a < 0.0 || msg->current_color.a > 1.0 ||
    msg->current_color.r < 0.0 || msg->current_color.r > 1.0 ||
    msg->current_color.g < 0.0 || msg->current_color.g > 1.0 ||
    msg->current_color.b < 0.0 || msg->current_color.b > 1.0)
  {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Message",
      "Invalid message: color values must be between 0.0 and 1.0");
    return false;
  }

  return true;
}

void OverlayDisplay::processMessage(const rviz_min_max_cur_overlay::msg::MinMaxCurr::ConstSharedPtr & msg)
{
  if (!validateMessage(msg)) {
    return;
  }

  last_msg_ = msg;
  updateVisualization();
}

QColor OverlayDisplay::toQColor(const std_msgs::msg::ColorRGBA & color)
{
  return QColor(
    static_cast<int>(color.r * 255),
    static_cast<int>(color.g * 255),
    static_cast<int>(color.b * 255),
    static_cast<int>(color.a * 255));
}

void OverlayDisplay::drawHorizontal(
  QPainter & painter,
  const rviz_min_max_cur_overlay::msg::MinMaxCurr::ConstSharedPtr & msg)
{
  const int width = width_property_->getInt();
  const int height = height_property_->getInt();
  const float line_width = line_width_property_->getFloat();
  const int font_size = font_size_property_->getInt();
  const QColor text_color = text_color_property_->getColor();
  const bool is_line_mode = display_mode_property_->getOptionInt() == 0;

  // Set up font
  QFont font = painter.font();
  font.setPixelSize(font_size);
  painter.setFont(font);
  QFontMetrics fm(font);

  // Draw min/max values
  QString min_str = QString::number(msg->min, 'f', 1);
  QString max_str = QString::number(msg->max, 'f', 1);
  QString cur_str = QString::number(msg->current, 'f', 1);

  int min_width = fm.horizontalAdvance(min_str);
  int max_width = fm.horizontalAdvance(max_str);
  int cur_width = fm.horizontalAdvance(cur_str);

  painter.setPen(text_color);
  painter.drawText(
    QRect(-min_width - 5, 0, min_width, height),
    Qt::AlignRight | Qt::AlignVCenter, min_str);
  painter.drawText(
    QRect(width + 5, 0, max_width, height),
    Qt::AlignLeft | Qt::AlignVCenter, max_str);

  // Calculate current value position
  float range = msg->max - msg->min;
  float normalized_pos = range != 0 ? (msg->current - msg->min) / range : 0.5f;
  int cur_x = static_cast<int>(normalized_pos * width);

  QColor current_color = toQColor(msg->current_color);

  if (is_line_mode) {
    // Draw line mode
    painter.setPen(QPen(current_color, line_width));
    painter.drawLine(cur_x, 0, cur_x, height);
    painter.setPen(text_color);
    painter.drawText(
      QRect(cur_x - cur_width/2, -font_size - 5, cur_width, font_size),
      Qt::AlignCenter, cur_str);
  } else {
    // Draw rectangle mode
    painter.fillRect(0, 0, cur_x, height, current_color);
    painter.setPen(text_color);
    painter.drawText(
      QRect(cur_x/2 - cur_width/2, 0, cur_width, height),
      Qt::AlignCenter, cur_str);
  }
}

void OverlayDisplay::drawVertical(
  QPainter & painter,
  const rviz_min_max_cur_overlay::msg::MinMaxCurr::ConstSharedPtr & msg)
{
  const int width = width_property_->getInt();
  const int height = height_property_->getInt();
  const float line_width = line_width_property_->getFloat();
  const int font_size = font_size_property_->getInt();
  const QColor text_color = text_color_property_->getColor();
  const bool is_line_mode = display_mode_property_->getOptionInt() == 0;

  // Set up font
  QFont font = painter.font();
  font.setPixelSize(font_size);
  painter.setFont(font);
  QFontMetrics fm(font);

  // Draw min/max values
  QString min_str = QString::number(msg->min, 'f', 1);
  QString max_str = QString::number(msg->max, 'f', 1);
  QString cur_str = QString::number(msg->current, 'f', 1);

  int min_width = fm.horizontalAdvance(min_str);
  int max_width = fm.horizontalAdvance(max_str);
  int cur_width = fm.horizontalAdvance(cur_str);

  painter.setPen(text_color);
  painter.drawText(
    QRect(0, height + 5, min_width, font_size),
    Qt::AlignLeft | Qt::AlignTop, min_str);
  painter.drawText(
    QRect(0, -font_size - 5, max_width, font_size),
    Qt::AlignLeft | Qt::AlignBottom, max_str);

  // Calculate current value position
  float range = msg->max - msg->min;
  float normalized_pos = range != 0 ? (msg->current - msg->min) / range : 0.5f;
  int cur_y = height - static_cast<int>(normalized_pos * height);

  QColor current_color = toQColor(msg->current_color);

  if (is_line_mode) {
    // Draw line mode
    painter.setPen(QPen(current_color, line_width));
    painter.drawLine(0, cur_y, width, cur_y);
    painter.setPen(text_color);
    painter.drawText(
      QRect(width + 5, cur_y - font_size/2, cur_width, font_size),
      Qt::AlignLeft | Qt::AlignVCenter, cur_str);
  } else {
    // Draw rectangle mode
    painter.fillRect(0, cur_y, width, height - cur_y, current_color);
    painter.setPen(text_color);
    painter.drawText(
      QRect(0, cur_y + (height - cur_y)/2 - font_size/2, width, font_size),
      Qt::AlignCenter, cur_str);
  }
}

void OverlayDisplay::updateVisualization()
{
  if (!overlay_widget_) {
    return;
  }

  const int width = width_property_->getInt();
  const int height = height_property_->getInt();
  const float border_width = border_width_property_->getFloat();
  const QColor bg_color = background_color_property_->getColor();
  const QColor text_color = text_color_property_->getColor();
  const bool is_horizontal = orientation_property_->getOptionInt() == 0;

  overlay_widget_->setFixedSize(width, height);

  // Create a new QPixmap for drawing
  QPixmap pixmap(width, height);
  pixmap.fill(Qt::transparent);

  QPainter painter(&pixmap);
  painter.setRenderHint(QPainter::Antialiasing);

  // Draw background
  painter.fillRect(0, 0, width, height, bg_color);

  // Draw border
  painter.setPen(QPen(text_color, border_width));
  painter.drawRect(0, 0, width-1, height-1);

  if (last_msg_) {
    if (is_horizontal) {
      drawHorizontal(painter, last_msg_);
    } else {
      drawVertical(painter, last_msg_);
    }
  }

  // Set the pixmap as the widget's background
  QPalette palette = overlay_widget_->palette();
  palette.setBrush(QPalette::Window, QBrush(pixmap));
  overlay_widget_->setPalette(palette);
  overlay_widget_->update();
}

void OverlayDisplay::updateTopic()
{
  // First unsubscribe from any existing topic
  unsubscribe();

  // Only proceed if we're enabled
  if (!isEnabled()) {
    return;
  }

  // Validate the topic_property
  if (!topic_property_) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic", "Invalid topic property");
    return;
  }

  // Get the topic string
  const std::string topic_str = topic_property_->getTopicStd();
  if (topic_str.empty()) {
    setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "No topic set");
    return;
  }

  // Get the ROS node from the context
  if (!context_) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Context", "No valid display context");
    return;
  }

  auto node_abstraction = context_->getRosNodeAbstraction().lock();
  if (!node_abstraction) {
    setStatus(rviz_common::properties::StatusProperty::Error, "ROS", "No ROS node abstraction available");
    return;
  }

  auto node = node_abstraction->get_raw_node();
  if (!node) {
    setStatus(rviz_common::properties::StatusProperty::Error, "ROS", "No ROS node available");
    return;
  }

  // Create the subscription with proper error handling
  try {
    rclcpp::QoS qos(10);
    subscriber_ = node->create_subscription<rviz_min_max_cur_overlay::msg::MinMaxCurr>(
      topic_str,
      qos,
      [this](const rviz_min_max_cur_overlay::msg::MinMaxCurr::ConstSharedPtr msg) {
        if (msg && validateMinMaxCurrMsg(*msg)) {  // Use our custom validation function
          this->processMessage(msg);
        }
      });

    setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", QString::fromStdString(topic_str));
  } catch (const std::exception & e) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic", QString("Error: ") + e.what());
  }
}

void OverlayDisplay::fixedFrameChanged()
{
  // Nothing to do here since we don't use transforms
}

}  // namespace rviz_min_max_cur_overlay
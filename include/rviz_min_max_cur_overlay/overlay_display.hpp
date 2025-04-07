#ifndef RVIZ_MIN_MAX_CUR_OVERLAY__OVERLAY_DISPLAY_HPP_
#define RVIZ_MIN_MAX_CUR_OVERLAY__OVERLAY_DISPLAY_HPP_

#include <memory>
#include <string>

// RViz includes
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>

// ROS includes
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include "rviz_min_max_cur_overlay/msg/min_max_curr.hpp"

// Ogre includes
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

// Qt includes
#include <QWidget>
#include <QPainter>
#include <QColor>
#include <QImage>
#include <QPixmap>
#include <QPalette>
#include <QApplication>
#include <QThread>

// Plugin export macros
#include <pluginlib/class_list_macros.hpp>

// Visibility control
#include "rviz_min_max_cur_overlay/visibility_control.hpp"

namespace rviz_min_max_cur_overlay
{

class RVIZ_MIN_MAX_CUR_OVERLAY_PUBLIC OverlayDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  OverlayDisplay();
  virtual ~OverlayDisplay() override;

protected:
  void onInitialize() override;
  void reset() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;
  void fixedFrameChanged() override;

private Q_SLOTS:
  void updateTopic();
  void updateVisualization();

private:
  void unsubscribe();
  bool validateMessage(const rviz_min_max_cur_overlay::msg::MinMaxCurr::ConstSharedPtr & msg);
  void drawHorizontal(QPainter & painter, const rviz_min_max_cur_overlay::msg::MinMaxCurr::ConstSharedPtr & msg);
  void drawVertical(QPainter & painter, const rviz_min_max_cur_overlay::msg::MinMaxCurr::ConstSharedPtr & msg);
  void processMessage(const rviz_min_max_cur_overlay::msg::MinMaxCurr::ConstSharedPtr & msg);
  QColor toQColor(const std_msgs::msg::ColorRGBA & color);

  // ROS Subscriber
  rclcpp::Subscription<rviz_min_max_cur_overlay::msg::MinMaxCurr>::SharedPtr subscriber_;
  
  // Properties
  rviz_common::properties::RosTopicProperty* topic_property_{nullptr};
  rviz_common::properties::IntProperty* width_property_{nullptr};
  rviz_common::properties::IntProperty* height_property_{nullptr};
  rviz_common::properties::EnumProperty* orientation_property_{nullptr};
  rviz_common::properties::EnumProperty* display_mode_property_{nullptr};
  rviz_common::properties::FloatProperty* line_width_property_{nullptr};
  rviz_common::properties::FloatProperty* border_width_property_{nullptr};
  rviz_common::properties::IntProperty* font_size_property_{nullptr};
  rviz_common::properties::ColorProperty* background_color_property_{nullptr};
  rviz_common::properties::ColorProperty* text_color_property_{nullptr};

  // Message storage
  rviz_min_max_cur_overlay::msg::MinMaxCurr::ConstSharedPtr last_msg_;

  // Rendering objects
  Ogre::SceneNode* scene_node_{nullptr};
  std::unique_ptr<QWidget> overlay_widget_;
};

}  // namespace rviz_min_max_cur_overlay

#endif  // RVIZ_MIN_MAX_CUR_OVERLAY__OVERLAY_DISPLAY_HPP_
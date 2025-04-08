#ifndef RVIZ_MIN_MAX_CUR_OVERLAY__OVERLAY_DISPLAY_HPP_
#define RVIZ_MIN_MAX_CUR_OVERLAY__OVERLAY_DISPLAY_HPP_

#include <memory>
#include <string>

// RViz includes
#include <rviz_common/display.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/string_property.hpp>

// ROS message includes
#include <std_msgs/msg/color_rgba.hpp>

// OGRE includes
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

// Qt includes
class QWidget;
class QPainter;

// Our message type
#include "rviz_min_max_cur_overlay/msg/min_max_curr.hpp"

// Custom widget includes
#include "rviz_min_max_cur_overlay/topic_input_widget.hpp"

namespace rviz_min_max_cur_overlay
{

/**
 * @class OverlayDisplay
 * @brief Display plugin for RViz that shows min/max/current values as an overlay
 */
class OverlayDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  /**
   * @brief Constructor
   */
  OverlayDisplay();
  
  /**
   * @brief Destructor
   */
  ~OverlayDisplay() override;

protected:
  /**
   * @brief Initialize the display
   */
  void onInitialize() override;
  
  /**
   * @brief Reset the display
   */
  void reset() override;
  
  /**
   * @brief Called when the display is enabled
   */
  void onEnable() override;
  
  /**
   * @brief Called when the display is disabled
   */
  void onDisable() override;
  
  /**
   * @brief Update the display
   */
  void update(float wall_dt, float ros_dt) override;
  
  /**
   * @brief Called when the fixed frame changes
   */
  void fixedFrameChanged() override;

private Q_SLOTS:
  /**
   * @brief Update the subscribed topic
   */
  void updateTopic();
  
  /**
   * @brief Update the visualization
   */
  void updateVisualization();

private:
  void unsubscribe();
  bool validateMessage(const rviz_min_max_cur_overlay::msg::MinMaxCurr::ConstSharedPtr & msg);
  void drawHorizontal(QPainter & painter, 
                     const rviz_min_max_cur_overlay::msg::MinMaxCurr::ConstSharedPtr & msg);
  void drawVertical(QPainter & painter, 
                   const rviz_min_max_cur_overlay::msg::MinMaxCurr::ConstSharedPtr & msg);
  void processMessage(const rviz_min_max_cur_overlay::msg::MinMaxCurr::ConstSharedPtr & msg);
  QColor toQColor(const std_msgs::msg::ColorRGBA & color);

  // ROS Subscriber
  rclcpp::Subscription<rviz_min_max_cur_overlay::msg::MinMaxCurr>::SharedPtr subscriber_;
  
  // Properties
  rviz_common::properties::RosTopicProperty* topic_property_{nullptr}; // Use RosTopicProperty
  rviz_common::properties::IntProperty* width_property_{nullptr};
  rviz_common::properties::IntProperty* height_property_{nullptr};
  rviz_common::properties::EnumProperty* orientation_property_{nullptr};
  rviz_common::properties::EnumProperty* display_mode_property_{nullptr};
  rviz_common::properties::FloatProperty* line_width_property_{nullptr};
  rviz_common::properties::FloatProperty* border_width_property_{nullptr};
  rviz_common::properties::IntProperty* font_size_property_{nullptr};
  rviz_common::properties::ColorProperty* background_color_property_{nullptr};
  rviz_common::properties::ColorProperty* text_color_property_{nullptr};

  // Custom Topic Input Widget
  TopicInputWidget* topic_input_widget_{nullptr};
  QWidget* property_panel_{nullptr};

  // Message storage
  rviz_min_max_cur_overlay::msg::MinMaxCurr::ConstSharedPtr last_msg_;

  // Rendering objects
  Ogre::SceneNode* scene_node_{nullptr};
  std::unique_ptr<QWidget> overlay_widget_;
};

}  // namespace rviz_min_max_cur_overlay

#endif  // RVIZ_MIN_MAX_CUR_OVERLAY__OVERLAY_DISPLAY_HPP_
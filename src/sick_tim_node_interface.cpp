#include <sick_tim/sick_tim_node_interface.h>

namespace sick_tim
{

sickTimNode::sickTimNode() : Node("sick_driver")
{
  params_callback_ = this->add_on_set_parameters_callback(
    std::bind(&sickTimNode::onParameterEvent, this, std::placeholders::_1));

  this->declare_parameter("timelimit", 5);
  this->declare_parameter("port", "2112");
  this->declare_parameter("hostname", "");

  this->declare_parameter("subscribe_datagram", false);
  this->declare_parameter("device_number", 0);
      // datagram publisher (only for debug)
  this->declare_parameter("publish_datagram", false);
  // Declare Sick Tim Parameters
  this->declare_parameter("min_ang", -0.75 * M_PI);
  this->declare_parameter("max_ang", 0.75 * M_PI);
  this->declare_parameter("intensity", true);
  this->declare_parameter("skip", 0);
  this->declare_parameter("frame_id", "laser");
  this->declare_parameter("time_offset", -0.001);
  this->declare_parameter("auto_reboot", true);
}

rcl_interfaces::msg::SetParametersResult sickTimNode::onParameterEvent(
  const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  for (const auto &parameter : parameters)
  {
    if (parameter.get_name() == "min_ang")
    {
      double value = parameter.as_double();
      if (value < -0.75 * M_PI || value > 0.75 * M_PI){
        result.successful = false;
        result.reason = "Minimum angle outside limits [-0.75*pi,0.75*pi]. Parameter left unchanged";
        return result;
      }
      RCLCPP_INFO(this->get_logger(), "Parameter 'min_ang' set to %f", value);
    }
    if (parameter.get_name() == "max_ang")
    {
      double value = parameter.as_double();
      if (value < -0.75 * M_PI || value > 0.75 * M_PI){
        result.successful = false;
        result.reason = "Maximum angle outside limits [-0.75*pi,0.75*pi]. Parameter left unchanged";
        return result;
      }
      RCLCPP_INFO(this->get_logger(), "Parameter 'max_ang' set to %f", value);
    }
    if (parameter.get_name() == "intensity")
    {
      bool value = parameter.as_bool();
      RCLCPP_INFO(this->get_logger(), "Parameter 'intensity' set to %s ", value ? "True" : "False");
    }
    if (parameter.get_name() == "skip")
    {
      int value = parameter.as_int();
      if (value < 0 || value > 9){
        result.successful = false;
        result.reason = "Skip outside limits [0,9]. Parameter left unchanged";
        return result;
      }
      RCLCPP_INFO(this->get_logger(), "Parameter 'skip' set to %i", value);
    }
    if (parameter.get_name() == "frame_id")
    {
      std::string value = parameter.as_string();
      RCLCPP_INFO(this->get_logger(), "Parameter 'frame_id' set to %s ", value.c_str());
    }
    if (parameter.get_name() == "time_offset")
    {
      double value = parameter.as_double();
      if (value < -0.25 || value > 0.25){
        result.successful = false;
        result.reason = "Time offset outside limits [-0.25,0.25]. Parameter left unchanged";
        return result;
      }
      RCLCPP_INFO(this->get_logger(), "Parameter 'time_offset' set to %f", value);
    }
    if (parameter.get_name() == "auto_reboot")
    {
      bool value = parameter.as_bool();
      RCLCPP_INFO(this->get_logger(), "Parameter 'auto_reboot' set to %s ", value ? "True" : "False");
    }
  }

  return result;
}

}
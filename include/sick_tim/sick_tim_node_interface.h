#ifndef SICK_NODE_INTERFACE_H_
#define SICK_NODE_INTERFACE_H_

#include <example_interfaces/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sick_tim/sick_tim_config.h>

namespace sick_tim
{

class sickTimNode : public rclcpp::Node
{
  public:
    sickTimNode();
    ~sickTimNode() {}

    // Sick Tim Config

  private:
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_;
    rcl_interfaces::msg::SetParametersResult onParameterEvent(const std::vector<rclcpp::Parameter> &parameters);
};
} /* namespace sick_tim */
#endif /* SICK_NODE_INTERFACE_H_ */
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
    SickTimConfig config_;
    bool paramChanged;

  private:
    // Parameter Client pointer
    std::shared_ptr<rclcpp::SyncParametersClient> parameter_client_;

    // Parameter Event subscription
    std::shared_ptr<rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent,
      std::allocator<void>>> parameter_subscription_;

    // Datagram Publisher
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr datagram_pub_;

    // Parameter Callback
  void onParameterEvent(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);
};
} /* namespace sick_tim */
#endif /* SICK_NODE_INTERFACE_H_ */
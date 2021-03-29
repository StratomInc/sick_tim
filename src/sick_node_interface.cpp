#include <sick_tim/sick_node_interface.h>

namespace sick_tim
{

sickTimNode::sickTimNode() : Node("sick_driver")
{
    paramChanged = true;
    while (!parameter_client_->wait_for_service(std::chrono::seconds(5))) 
    {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
        }
        RCLCPP_INFO(
        this->get_logger(),
        "service not available, waiting again...");
    }

    parameter_subscription_ = parameter_client_->on_parameter_event(
        std::bind(&sickTimNode::onParameterEvent, this, std::placeholders::_1));
}

// Parameter Callback
void sickTimNode::onParameterEvent(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
    paramChanged = true;
    for (auto & changed_parameter : event->changed_parameters) {
        if(changed_parameter.name == "min_ang")
        {
        if(changed_parameter.value.double_value < -0.75 * M_PI || changed_parameter.value.double_value > 0.75 * M_PI)
        {
            RCLCPP_WARN(this->get_logger(), "Minimum angle outside limits: [-0.75*pi,0.75*pi] | Leaving parameter unchanged");
            paramChanged = false;
            return;
        }
        if (changed_parameter.value.double_value > config_.max_ang)
        {
            RCLCPP_WARN(this->get_logger(), "Minimum angle must be less than minimum angle. Adjusting min_ang.");
            config_.min_ang = config_.max_ang;
        }
        else
        {
            config_.min_ang = changed_parameter.value.double_value;
        }
        } else if(changed_parameter.name == "max_ang")
        {
        if(changed_parameter.value.double_value < -0.75 * M_PI || changed_parameter.value.double_value > 0.75 * M_PI)
        {
            RCLCPP_WARN(this->get_logger(), "Maximum angle outside limits: [-0.75*pi,0.75*pi] | Leaving parameter unchanged");
            paramChanged = false;
            return;
        }
        if (changed_parameter.value.double_value < config_.min_ang)
        {
            RCLCPP_WARN(this->get_logger(), "Maximum angle must be greater than minimum angle. Adjusting max_ang.");
            config_.max_ang = config_.min_ang;
        }
        else
        {
            config_.max_ang = changed_parameter.value.double_value;
        }
        } else if(changed_parameter.name == "intensity")
        {
        config_.intensity = changed_parameter.value.bool_value;
        } else if(changed_parameter.name == "skip")
        {
        if(changed_parameter.value.integer_value < 0 || changed_parameter.value.integer_value > 9)
        {
            RCLCPP_WARN(this->get_logger(), "Skip outside limits = [0,9] | Leaving parameter unchanged");
            paramChanged = false;
            return;
        }
        config_.skip = changed_parameter.value.integer_value;
        } else if(changed_parameter.name == "frame_id")
        {
        config_.frame_id = changed_parameter.value.string_value;
        } else if(changed_parameter.name == "time_offset")
        {
        if(changed_parameter.value.double_value < -0.25 || changed_parameter.value.double_value > 0.25)
        {
            RCLCPP_WARN(this->get_logger(), "Time offset outside limits = [-0.25,0.25] | Leaving parameter unchanged");
            paramChanged = false;
            return;
        }
        config_.time_offset = changed_parameter.value.double_value;
        } else if(changed_parameter.name == "auto_reboot")
        {
        config_.auto_reboot = changed_parameter.value.bool_value;
        } else if(changed_parameter.name == "publish_datagram")
        {
        config_.publish_datagram = changed_parameter.value.bool_value;
        if(config_.publish_datagram)
        {
            datagram_pub_ = this->create_publisher<example_interfaces::msg::String>("datagram", 1000);
        }
        else
        {
            datagram_pub_.reset();
        }
        }
        else {
        RCLCPP_WARN(this->get_logger(), "Unknown Param: %s", changed_parameter.name);
        paramChanged = false;
        }
    }
}
}
#include <memory>
#include <sstream>

#include "interactive_markers/interactive_marker_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/marker.hpp"

class SimpleMarker : public rclcpp::Node
{
public:
    SimpleMarker() : Node("simple_marker")
    {
        server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("simple_marker", this);

        visualization_msgs::msg::InteractiveMarker interactive_marker;
        interactive_marker.header.frame_id = "world";
        interactive_marker.header.stamp = this->get_clock()->now();
        interactive_marker.name = "my_marker";
        interactive_marker.description = "Simple 1-DOF Control";

        visualization_msgs::msg::Marker box_marker;
        box_marker.type = visualization_msgs::msg::Marker::CUBE;
        box_marker.scale.x = 0.2;
        box_marker.scale.y = 0.2;
        box_marker.scale.z = 0.2;
        box_marker.color.r = 0.5;
        box_marker.color.g = 0.5;
        box_marker.color.b = 0.5;
        box_marker.color.a = 1.0;

        visualization_msgs::msg::InteractiveMarkerControl box_control;
        box_control.always_visible = true;
        box_control.markers.push_back(box_marker);

        visualization_msgs::msg::InteractiveMarkerControl rotate_control;
        rotate_control.name = "move_x";
        rotate_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;

        interactive_marker.controls.push_back(box_control);
        interactive_marker.controls.push_back(rotate_control);

        auto processFeedback = [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback,
                                      rclcpp::Logger logger) -> void
        {
            std::ostringstream oss;
            oss << feedback->marker_name << " is now at " << feedback->pose.position.x << ", " << feedback->pose.position.y << ", " << feedback->pose.position.z;
            RCLCPP_INFO(logger, "%s", oss.str().c_str());
        };

        server_->insert(interactive_marker, std::bind(processFeedback, std::placeholders::_1, this->get_logger()));

        server_->applyChanges();

        RCLCPP_INFO(this->get_logger(), "Ready");
    }

private:
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleMarker>());
    rclcpp::shutdown();

    return 0;
}
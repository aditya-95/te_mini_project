#include "rclcpp/rclcpp.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"

class SimpleMarker: public rclcpp::Node
{
public:
    SimpleMarker() : Node("simple_marker")
    {
        server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("simple_marker", this);

        visualization_msgs::msg::InteractiveMarker interactive_marker;
        interactive_marker.header.frame_id = "base_link";
        interactive_marker.header.stamp = this->get_clock()->now();
        interactive_marker.name = "my_marker";
        interactive_marker.description = "Simple 1-dof control";

        // box
        visualization_msgs::msg::Marker cube_marker;
        cube_marker.type = visualization_msgs::msg::Marker::CUBE;
        cube_marker.scale.x = 0.45;
        cube_marker.scale.y = 0.45;
        cube_marker.scale.z = 0.45;
        cube_marker.color.r = 0.0;
        cube_marker.color.g = 1.0;
        cube_marker.color.b = 0.0;
        cube_marker.color.a = 0.0;

        visualization_msgs::msg::InteractiveMarkerControl control;
        control.always_visible = true;
        control.markers.push_back(cube_marker);
        interactive_marker.controls.push_back(control);

        visualization_msgs::msg::InteractiveMarkerControl rotate_control;
        rotate_control.name = "move_x";
        rotate_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        interactive_marker.controls.push_back(rotate_control);
        
        server_->insert(interactive_marker);

        server_->applyChanges();
    }

private:
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleMarker>());
    rclcpp::shutdown();

    return 0;
}
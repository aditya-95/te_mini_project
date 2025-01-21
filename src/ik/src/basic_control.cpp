#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"

visualization_msgs::msg::Marker makeBox(const visualization_msgs::msg::InteractiveMarker & msg)
{
    visualization_msgs::msg::Marker marker;

    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.scale.x = msg.scale * 0.75;
    marker.scale.y = msg.scale * 0.75;
    marker.scale.z = msg.scale * 0.75;
    marker.color.r = 0.1;
    marker.color.g = 0.1;
    marker.color.b = 0.1;
    marker.color.a = 0.2;

    return marker;
}

visualization_msgs::msg::InteractiveMarkerControl makeBoxControl(visualization_msgs::msg::InteractiveMarker & msg)
{
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(makeBox(msg));
    msg.controls.push_back(control);

    return msg.controls.back();
}

class BasicControl : public rclcpp::Node
{
public:
    BasicControl() : Node("basic_control")
    {
        // server
        server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("basic_control", this);

        tf2::Vector3 position(0.3, 0, 0.4);

        visualization_msgs::msg::InteractiveMarker int_marker;
        int_marker.header.frame_id = "world";
        int_marker.pose.position.x = position.getX();
        int_marker.pose.position.y = position.getY();
        int_marker.pose.position.z = position.getZ();
        int_marker.scale = 0.2;
        int_marker.name = "simple_6dof";
        int_marker.description = "Simple 6dof control";

        // insert a box
        makeBoxControl(int_marker);
        int_marker.controls[0].interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;

        visualization_msgs::msg::InteractiveMarkerControl control;

        tf2::Quaternion orien(1.0, 0.0, 0.0, 1.0);
        orien.normalize();
        control.orientation = tf2::toMsg(orien);
        control.name = "rotate_x";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        orien = tf2::Quaternion(0.0, 1.0, 0.0, 1.0);
        orien.normalize();
        control.orientation = tf2::toMsg(orien);
        control.name = "rotate_z";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        orien = tf2::Quaternion(0.0, 0.0, 1.0, 1.0);
        orien.normalize();
        control.orientation = tf2::toMsg(orien);
        control.name = "rotate_y";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        auto processFeedback = [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback,
                                      rclcpp::Logger logger) -> void
        {
            target_pose_ = feedback->pose;
        };

        server_->insert(int_marker);
        server_->setCallback(int_marker.name, std::bind(processFeedback, std::placeholders::_1, this->get_logger()));
        server_->applyChanges();

        update_timer_ = this->create_wall_timer(std::chrono::milliseconds(16),
                                                std::bind(&BasicControl::updateTargetTransform, this));

    }

private:
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::Pose target_pose_;

    void updateTargetTransform()
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = "world";
        transform_stamped.child_frame_id = "target";
        transform_stamped.transform.translation.x = target_pose_.position.x;
        transform_stamped.transform.translation.y = target_pose_.position.y;
        transform_stamped.transform.translation.z = target_pose_.position.z;
        transform_stamped.transform.rotation.x = target_pose_.orientation.x;
        transform_stamped.transform.rotation.y = target_pose_.orientation.y;
        transform_stamped.transform.rotation.z = target_pose_.orientation.z;
        transform_stamped.transform.rotation.w = target_pose_.orientation.w;

        tf_broadcaster_->sendTransform(transform_stamped);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BasicControl>());
    rclcpp::shutdown();

    return 0;
}
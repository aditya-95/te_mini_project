#include <iostream>
#include <chrono>
#include <memory>
#include <string>
#include <math.h>
#include <iomanip>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "kdl/chain.hpp"
#include "kdl/frameacc.hpp"
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "urdf/model.h"

using namespace std::chrono_literals;

class ToJointStatePublisher : public rclcpp::Node
{
public:
    ToJointStatePublisher() : Node("ik_publisher")
    {
        t1 = -0.0008;
        t2 = -0.3480;
        t3 = 0.6945;
        t4 = t5 = t6 = 0.0;
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        std::string urdf_string = loadURDF("../urdf/manipulator2.urdf");

        KDL::Tree kdl_tree;
        if (!kdl_parser::treeFromString(urdf_string, kdl_tree))
        {
            std::cerr << "Failed to parse urdf into KDL tree" << std::endl;
        }

        KDL::Chain chain;
        if (!kdl_tree.getChain("world", "l6", chain))
        {
            std::cerr << "failed to extract chain from KDL tree" << std::endl;
        }

        KDL::JntArray q_init(chain.getNrOfJoints());
        q_init(0) = 0.0;
        q_init(1) = 0.0;
        q_init(2) = 0.0;
        q_init(3) = 0.0;
        q_init(4) = 0.0;
        q_init(5) = 0.0;

        auto timer_callback = [this, q_init, chain]() -> void
        {
            // get target transform
            std::string fromFrameRel = target_frame_.c_str();
            std::string toFrameRel = "world";

            try
            {
                geometry_msgs::msg::TransformStamped t;
                t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
                // RCLCPP_INFO(this->get_logger(), "%f %f %f", t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);

                KDL::Frame desired_pose(KDL::Rotation::Quaternion(t.transform.rotation.x,
                                                                  t.transform.rotation.y,
                                                                  t.transform.rotation.z,
                                                                  t.transform.rotation.w),
                                        KDL::Vector(t.transform.translation.x,
                                                    t.transform.translation.y,
                                                    t.transform.translation.z));
                KDL::JntArray q_sol(chain.getNrOfJoints());
                KDL::ChainIkSolverPos_LMA ik_solver(chain);
                int result = ik_solver.CartToJnt(q_init, desired_pose, q_sol);
                if (result >= 0)
                {
                    t1 = q_sol(0);
                    t2 = q_sol(1);
                    t3 = q_sol(2);
                    t4 = q_sol(3);
                    t5 = q_sol(4);
                    t6 = q_sol(5);
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "failed to find a solution");
                }

                auto joint_state = sensor_msgs::msg::JointState();
                joint_state.header.stamp = this->get_clock()->now();
                joint_state.name = {"world_to_l1", "l1_to_l2", "l2_to_l3", "l3_to_l4", "l4_to_l5", "l5_to_l6"};
                joint_state.position = {t1, t2, t3, t4, t5, t6};
                joint_state.velocity = {};
                joint_state.effort = {};

                this->publisher_->publish(joint_state);
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
                return;
            }
        };

        target_frame_ = this->declare_parameter<std::string>("target_frame", "target");
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


        timer_ = this->create_wall_timer(16ms, timer_callback);
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string target_frame_;

    float t1, t2, t3, t4, t5, t6;

    std::string loadURDF(const std::string& urdf_file_path)
    {
        std::ifstream urdf_file(urdf_file_path);
        if(!urdf_file.is_open())
        {
            throw std::runtime_error("could not open URDF file: " + urdf_file_path);
        }
        std::string urdf_content((std::istreambuf_iterator<char>(urdf_file)), std::istreambuf_iterator<char>());
        urdf_file.close();
        return urdf_content;
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ToJointStatePublisher>());
    rclcpp::shutdown();
    return 0;
}

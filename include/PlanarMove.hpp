#ifndef PLANAR_MOVE_HPP
#define PLANAR_MOVE_HPP

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/transport/Node.hh>
#include <gz/math/Vector3.hh>
#include <geometry_msgs/msg/twist.hpp>
#include <gz/msgs/double.pb.h>
#include <rclcpp/rclcpp.hpp>

#define DEFAULT_NS "robot"
#define DEFAULT_VELOCITY_CMD_TOPIC "/cmd_vel"
#define DEFAULT_BACK_JOINT "rim_back_joint"
#define DEFAULT_RIGHT_JOINT "rim_right_joint"
#define DEFAULT_LEFT_JOINT "rim_left_joint"
#define DEFAULT_WHEEL_RADIUS 0.06
#define DEFAULT_WHEEL_DISTANCE 0.14

namespace gz_planar_move
{
    class PlanarMove : public gz::sim::System,
                       public gz::sim::ISystemConfigure,
                       public gz::sim::ISystemPreUpdate
    {
    public:
        PlanarMove();
        void Configure(const gz::sim::Entity &_entity,
                       const std::shared_ptr<const sdf::Element> &,
                       gz::sim::EntityComponentManager &_ecm,
                       gz::sim::EventManager &) override;

        void PreUpdate(const gz::sim::UpdateInfo &,
                       gz::sim::EntityComponentManager &_ecm) override;

    private:
        void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg);

        gz::sim::Model model{gz::sim::kNullEntity};
        geometry_msgs::msg::Twist cmd_vel;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub;

        gz::sim::Entity joint_back;
        gz::sim::Entity joint_right;
        gz::sim::Entity joint_left;

        // Parameters
        double wheel_radius;
        double wheel_distance;
        std::string velocity_cmd_topic;
        std::string back_joint;
        std::string left_joint;
        std::string right_joint;
        std::string ns;

        std::shared_ptr<rclcpp::Node> ros_node;
        rclcpp::Time last_cmd_time_;
    };
} // namespace gz_planar_move

#endif // PLANAR_MOVE_HPP

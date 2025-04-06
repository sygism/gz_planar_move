#include "PlanarMove.hpp"
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointVelocityCmd.hh>

namespace gz_planar_move
{
    PlanarMove::PlanarMove()
    {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        ros_node = std::make_shared<rclcpp::Node>("gz_planar_move");
        last_cmd_time_ = ros_node->now();
    }

    void PlanarMove::Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &
    ) {
        model = gz::sim::Model(_entity);
        if (!model.Valid(_ecm)) {
            RCLCPP_ERROR(ros_node->get_logger(), "Invalid Model Entity!");
            return;
        }

        wheel_radius = (sdf->HasElement("wheel_radius")) ? sdf->Get<double>("wheel_radius") : DEFAULT_WHEEL_RADIUS;
        wheel_distance = (sdf->HasElement("wheel_distance")) ? sdf->Get<double>("wheel_distance") : DEFAULT_WHEEL_DISTANCE;
        velocity_cmd_topic = (sdf->HasElement("velocity_cmd_topic")) ? sdf->Get<std::string>("velocity_cmd_topic") : DEFAULT_VELOCITY_CMD_TOPIC;
        back_joint = (sdf->HasElement("back_joint")) ? sdf->Get<std::string>("back_joint") : DEFAULT_BACK_JOINT;
        left_joint = (sdf->HasElement("left_joint")) ? sdf->Get<std::string>("left_joint") : DEFAULT_LEFT_JOINT;
        right_joint = (sdf->HasElement("right_joint")) ? sdf->Get<std::string>("right_joint") : DEFAULT_RIGHT_JOINT;
        ns = (sdf->HasElement("ns")) ? sdf->Get<std::string>("ns") : DEFAULT_NS;

        RCLCPP_INFO(ros_node->get_logger(), "PlanarMove started with Namespace: [%s]", ns.c_str());

        sub = ros_node->create_subscription<geometry_msgs::msg::Twist>(
            velocity_cmd_topic,
            10,
            std::bind(&PlanarMove::OnCmdVel, this, std::placeholders::_1)
        );

        joint_back = model.JointByName(_ecm, back_joint);
        joint_right = model.JointByName(_ecm, right_joint);
        joint_left = model.JointByName(_ecm, left_joint);

        if (!joint_back || !joint_right || !joint_left) {
            RCLCPP_ERROR(ros_node->get_logger(), "Could not find one or more joints.");
            return;
        }

        RCLCPP_INFO(ros_node->get_logger(), "Joints found: Back [%lu], Right [%lu], Left [%lu]",
                    joint_back, joint_right, joint_left);
    }

    void PlanarMove::PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &_ecm) {
        if (!model.Valid(_ecm))
            return;

        rclcpp::Time current_time = ros_node->now();
        double time_since_last_cmd = (current_time - last_cmd_time_).seconds();

        if (time_since_last_cmd > 0.5) {
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;
        }

        const double vx = cmd_vel.linear.x;
        const double vy = cmd_vel.linear.y;
        const double wz = cmd_vel.angular.z;

        double w_back  = (vy + wheel_distance * wz) / wheel_radius;
        const double w_right = (-0.866 * vx - 0.5 * vy + wheel_distance * wz) / wheel_radius;
        const double w_left  = ( 0.866 * vx - 0.5 * vy + wheel_distance * wz) / wheel_radius;


        _ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(joint_back, {w_back});
        _ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(joint_right, {w_right});
        _ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(joint_left, {w_left});

        rclcpp::spin_some(ros_node);
    }

    void PlanarMove::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg) {
        cmd_vel = *_msg;
        last_cmd_time_ = ros_node->now();
    }

} // namespace gz_planar_move

GZ_ADD_PLUGIN(
    gz_planar_move::PlanarMove,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate
)

#include "PlanarMove.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/components/Pose.hh>

namespace gz_planar_move
{
    PlanarMove::PlanarMove()
    {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        ros_node = std::make_shared<rclcpp::Node>("gz_planar_move_pose_updater");
    }

    void PlanarMove::Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &)
    {
        ros_node->set_parameter(rclcpp::Parameter("use_sim_time", true));
        model = gz::sim::Model(_entity);
        if (!model.Valid(_ecm)) {
            RCLCPP_ERROR(ros_node->get_logger(), "Invalid Model Entity!");
            return;
        }

        std::string odom_topic = "/odom";
        if (sdf->HasElement("odom_topic")) {
            odom_topic = sdf->Get<std::string>("odom_topic");
        }

        RCLCPP_INFO(ros_node->get_logger(), "Subscribing to odometry on topic: %s", odom_topic.c_str());

        odom_sub = ros_node->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic,
            10,
            std::bind(&PlanarMove::OnOdom, this, std::placeholders::_1)
        );
    }

    void PlanarMove::PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &_ecm)
    {
        if (!model.Valid(_ecm)) {
            return;
        }

        auto poseComp = _ecm.Component<gz::sim::components::Pose>(model.Entity());
        if (!poseComp) {
            return;
        }

        gz::math::Pose3d current_pose = poseComp->Data();
        double current_z = current_pose.Pos().Z();
        double roll = current_pose.Rot().Roll();
        double pitch = current_pose.Rot().Pitch();

        double x = latest_pose.Pos().X();
        double y = latest_pose.Pos().Y();
        double yaw = latest_pose.Rot().Yaw();

        gz::math::Vector3d position(x, y, current_z);
        gz::math::Quaterniond rotation(roll, pitch, yaw);
        gz::math::Pose3d adjusted_pose(position, rotation);

        const double pos_epsilon = 1e-4;
        const double rot_epsilon = 1e-3;

        if (adjusted_pose.Pos().Distance(last_applied_pose.Pos()) > pos_epsilon ||
            adjusted_pose.Rot().Euler().Distance(last_applied_pose.Rot().Euler()) > rot_epsilon)
        {
            _ecm.SetComponentData<gz::sim::components::Pose>(model.Entity(), adjusted_pose);
            _ecm.SetChanged(model.Entity(), gz::sim::components::Pose::typeId, gz::sim::ComponentState::OneTimeChange);
            last_applied_pose = adjusted_pose;
        }

        rclcpp::spin_some(ros_node);
    }

    void PlanarMove::OnOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_pose = gz::math::Pose3d(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z,
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z
        );
    }

} // namespace gz_planar_move

GZ_ADD_PLUGIN(
    gz_planar_move::PlanarMove,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate
)

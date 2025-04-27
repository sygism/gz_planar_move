#ifndef GZ_PLANAR_MOVE_HPP
#define GZ_PLANAR_MOVE_HPP

#include <mutex>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/math/Pose3.hh>

namespace gz_planar_move
{
    class PlanarMove : public gz::sim::System,
                       public gz::sim::ISystemConfigure,
                       public gz::sim::ISystemPreUpdate
    {
    public:
        PlanarMove();
        ~PlanarMove() = default;

        void Configure(
            const gz::sim::Entity &_entity,
            const std::shared_ptr<const sdf::Element> &sdf,
            gz::sim::EntityComponentManager &_ecm,
            gz::sim::EventManager &_eventMgr) override;

        void PreUpdate(
            const gz::sim::UpdateInfo &_info,
            gz::sim::EntityComponentManager &_ecm) override;

    private:
        gz::sim::Model model{gz::sim::kNullEntity};
        rclcpp::Node::SharedPtr ros_node;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

        gz::math::Pose3d latest_pose{gz::math::Pose3d::Zero};
        gz::math::Pose3d last_applied_pose{gz::math::Pose3d::Zero};
        std::mutex pose_mutex;

        void OnOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
    };
} // namespace gz_planar_move

#endif // GZ_PLANAR_MOVE_HPP

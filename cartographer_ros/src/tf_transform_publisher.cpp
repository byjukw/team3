#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

class DynamicTfPublisher : public rclcpp::Node {
public:
    DynamicTfPublisher() : Node("tf_transform_publisher") {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DynamicTfPublisher::broadcast_tf, this));
    }

private:
    void broadcast_tf() {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->now();
        t.header.frame_id = "base_footprint";
        t.child_frame_id = "base_link";
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.205;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);  // 회전 없음
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamicTfPublisher>());
    rclcpp::shutdown();
    return 0;
}


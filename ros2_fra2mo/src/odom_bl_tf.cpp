#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath> 

class DynamicTfPublisher : public rclcpp::Node {
public:
    DynamicTfPublisher() : Node("dynamic_tf_publisher") {
        // Inizializzazione del TF Broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Sottoscrizione al topic di odometria
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/fra2mo/odometry", 10,
            std::bind(&DynamicTfPublisher::odomCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Dynamic TF publisher initialized.");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Legge la posizione e l'orientamento dal messaggio
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        double yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

        // Creazione e pubblicazione della trasformazione dinamica
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = "fra2mo/odom";
        transform_stamped.child_frame_id = "base_footprint";

        transform_stamped.transform.translation.x = x;
        transform_stamped.transform.translation.y = y;
        transform_stamped.transform.translation.z = 0.0;
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        // Invia la trasformazione
        tf_broadcaster_->sendTransform(transform_stamped);

        // Log per debug (facoltativo)
        RCLCPP_DEBUG(this->get_logger(),
                     "Published transform: [frame: fra2mo/odom -> base_footprint] [x: %.2f, y: %.2f, yaw: %.2f]",
                     x, y, yaw);
    }

    // TF Broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Sottoscrizione al topic di odometria
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicTfPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

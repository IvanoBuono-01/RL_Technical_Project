#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <memory>

class ArucoDetector : public rclcpp::Node {
public:
    ArucoDetector() : Node("aruco_detector") {
        // Subscriber alla camera IIWA
        camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/iiwa/camera/image_raw", 10,
            std::bind(&ArucoDetector::image_callback, this, std::placeholders::_1));

        // Publishers per i marker
        marker_0_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/aruco_marker_0/pose", 10);
        marker_999_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/aruco_marker_999/pose", 10);

        // Initializzare il dizionario ArUco (API OpenCV 4.1.x - compatibile con Docker)
        // getPredefinedDictionary ritorna Dictionary, lo wrappamo in Ptr
        cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        aruco_dict_ = cv::makePtr<cv::aruco::Dictionary>(dict);
        
        RCLCPP_INFO(this->get_logger(), "ArUco detector initialized with OpenCV 4.1.x API");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Convertire ROS image a OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat image = cv_ptr->image;

            if (image.empty()) {
                RCLCPP_WARN(this->get_logger(), "Received empty image");
                return;
            }

            // Rilevare i marker ArUco usando l'API di OpenCV 4.1.x
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners, rejected;
            
            // Usa detectMarkers con l'API vecchia
            cv::aruco::detectMarkers(image, aruco_dict_, corners, ids, cv::aruco::DetectorParameters::create(), rejected);

            if (!ids.empty()) {
                RCLCPP_DEBUG(this->get_logger(), "Detected %zu markers", ids.size());
                
                for (size_t i = 0; i < ids.size(); ++i) {
                    int marker_id = ids[i];
                    
                    // Pubblicare solo i marker di interesse (0 e 999)
                    if (marker_id == 0 || marker_id == 999) {
                        std::vector<cv::Point2f> marker_corners = corners[i];
                        
                        // Calcolare il centroide del marker
                        cv::Point2f center(0, 0);
                        for (const auto& corner : marker_corners) {
                            center += corner;
                        }
                        center /= 4.0;

                        // Creare il messaggio PoseStamped
                        geometry_msgs::msg::PoseStamped pose_msg;
                        pose_msg.header.frame_id = "iiwa_camera_link";
                        pose_msg.header.stamp = msg->header.stamp;

                        // Posizione (approssimazione 2D a 3D)
                        float assumed_depth = 0.5; // 50 cm
                        float focal_length = 500.0; // Focal length approssimativa
                        
                        pose_msg.pose.position.x = (center.x - image.cols / 2.0) * assumed_depth / focal_length;
                        pose_msg.pose.position.y = (center.y - image.rows / 2.0) * assumed_depth / focal_length;
                        pose_msg.pose.position.z = assumed_depth;

                        // Orientazione (identità per semplicità)
                        pose_msg.pose.orientation.x = 0.0;
                        pose_msg.pose.orientation.y = 0.0;
                        pose_msg.pose.orientation.z = 0.0;
                        pose_msg.pose.orientation.w = 1.0;

                        if (marker_id == 0) {
                            marker_0_pub_->publish(pose_msg);
                            RCLCPP_INFO(this->get_logger(), "Detected and published ArUco marker 0");
                        } else if (marker_id == 999) {
                            marker_999_pub_->publish(pose_msg);
                            RCLCPP_INFO(this->get_logger(), "Detected and published ArUco marker 999");
                        }
                    }
                }
            }
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr marker_0_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr marker_999_pub_;
    
    cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetector>());
    rclcpp::shutdown();
    return 0;
}

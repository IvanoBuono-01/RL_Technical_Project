#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <iostream>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_kdl_package/action/trajectory.hpp"

using namespace std::chrono_literals;
using Trajectory = ros2_kdl_package::action::Trajectory;
using GoalHandleTrajectory = rclcpp_action::ClientGoalHandle<Trajectory>;

class TrajectoryActionClient : public rclcpp::Node
{
public:
  TrajectoryActionClient()
  : Node("trajectory_action_client")
  {
    // Crea il client per l'action "trajectory"
    client_ptr_ = rclcpp_action::create_client<Trajectory>(this, "trajectory");
    declare_parameter("traj_duration", 5.0);
    get_parameter("traj_duration", traj_duration_);
    declare_parameter("acc_duration", 5.0);
    get_parameter("acc_duration", acc_duration_);
    declare_parameter("total_time", 5.0);
    get_parameter("total_time", total_time_);
    declare_parameter("trajectory_len", 200);
    get_parameter("trajectory_len", trajectory_len_);
    declare_parameter("Kp", 1.0);
    get_parameter("Kp", Kp_);

    declare_parameter("end_position_x", 0.5);
    get_parameter("end_position_x", end_pos_x_);
    declare_parameter("end_position_y", 0.3);
    get_parameter("end_position_y", end_pos_y_);
    declare_parameter("end_position_z", 0.5);
    get_parameter("end_position_z", end_pos_z_);
  }

  void send_goal()
  {
    // Attendi che il server sia disponibile
    if (!client_ptr_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    auto goal_msg = Trajectory::Goal();

    goal_msg.traj_duration   = traj_duration_;
    goal_msg.acc_duration    = acc_duration_;
    goal_msg.trajectory_len   = trajectory_len_;
    goal_msg.total_time      = total_time_;
    goal_msg.kp             = Kp_;
    goal_msg.end_position_x  = end_pos_x_;
    goal_msg.end_position_y  = end_pos_y_;
    goal_msg.end_position_z  = end_pos_z_;

    RCLCPP_INFO(this->get_logger(),
            "Sending goal with params: traj_duration=%.2f acc_duration=%.2f total_time=%.2f end_pos=(%.3f, %.3f, %.3f)",
            traj_duration_, acc_duration_, total_time_,
            end_pos_x_, end_pos_y_, end_pos_z_);

    // Invia il goal con callback per il feedback e il risultato
    auto send_goal_options = rclcpp_action::Client<Trajectory>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&TrajectoryActionClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&TrajectoryActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&TrajectoryActionClient::result_callback, this, std::placeholders::_1);

    client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Trajectory>::SharedPtr client_ptr_;
    double traj_duration_;
    double acc_duration_;
    int trajectory_len_;
    double Kp_;
    double total_time_;
    double end_pos_x_;
    double end_pos_y_;
    double end_pos_z_;

  void goal_response_callback(GoalHandleTrajectory::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result...");
    }
  }

  void feedback_callback(
      GoalHandleTrajectory::SharedPtr,
      const std::shared_ptr<const Trajectory::Feedback> feedback)
  {
    // Il tuo .action definisce 'float64 position_error' come feedback
    RCLCPP_INFO(this->get_logger(), "Feedback: position_error = %.6f", feedback->position_error);
  }

  void result_callback(const GoalHandleTrajectory::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Result: success = %s", result.result->success ? "true" : "false");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }

    // Dopo il risultato, puoi chiudere il nodo
    rclcpp::shutdown();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryActionClient>();
  node->send_goal();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

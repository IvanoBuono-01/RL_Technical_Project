#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <functional>
#include <Eigen/Dense>                          // per Eigen::Vector3d, Matrix, ecc.
#include <Eigen/Geometry>                       // per Eigen::Quaterniond

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"

#include "ros2_kdl_package/action/trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"   // per il subscriber ArUco

using FloatArray = std_msgs::msg::Float64MultiArray;
using TrajectoryAction = ros2_kdl_package::action::Trajectory;              // alias chiaro
using GoalHandleTrajectory = rclcpp_action::ServerGoalHandle<TrajectoryAction>;
using namespace std::chrono_literals;

class Iiwa_pub_sub : public rclcpp::Node
{
public:
    Iiwa_pub_sub()
    : Node("ros2_kdl_node"),
      node_handle_(std::shared_ptr<Iiwa_pub_sub>(this, [](Iiwa_pub_sub*){}))
    {
        // Parametri esistenti
        declare_parameter("cmd_interface", "position");
        get_parameter("cmd_interface", cmd_interface_);
        declare_parameter("ctrl", "velocity_ctrl");
        get_parameter("ctrl", ctrl_);

        /*declare_parameter("traj_duration", 1.5);
        get_parameter("traj_duration", traj_duration_);
        declare_parameter("acc_duration", 0.5);
        get_parameter("acc_duration", acc_duration_);
        declare_parameter("total_time", 1.5);
        get_parameter("total_time", total_time_);
        declare_parameter("trajectory_len", 150);
        get_parameter("trajectory_len", trajectory_len_);
        declare_parameter("Kp", 5.0);
        get_parameter("Kp", Kp_);

        declare_parameter("end_position_x", 0.4);
        get_parameter("end_position_x", end_pos_x_);
        declare_parameter("end_position_y", 0.015);
        get_parameter("end_position_y", end_pos_y_);
        declare_parameter("end_position_z", 0.4);
        get_parameter("end_position_z", end_pos_z_);

        RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());
        RCLCPP_INFO(get_logger(),
                    "Loaded parameters:\n"
                    "  traj_duration: %.2f\n"
                    "  acc_duration: %.2f\n"
                    "  total_time: %.2f\n"
                    "  trajectory_len: %d\n"
                    "  Kp: %.2f\n"
                    "  end_position: [%.3f, %.3f, %.3f]",
                        traj_duration_,
                        acc_duration_,
                        total_time_,
                        trajectory_len_,
                        Kp_,
                        end_pos_x_,
                        end_pos_y_,
                        end_pos_z_);*/

        if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort" ))
        {
            RCLCPP_ERROR(get_logger(),"Selected cmd interface is not valid! Use 'position', 'velocity' or 'effort' instead...");
            throw std::runtime_error("Invalid cmd_interface");
        }
        // Inizializza la posizione del marker
        marker_pos_ = KDL::Frame::Identity();

        declare_parameter("traj_type", "linear");
        get_parameter("traj_type", traj_type_);
        RCLCPP_INFO(get_logger(),"Current trajectory type is: '%s'", traj_type_.c_str());
        if (!(traj_type_ == "linear" || traj_type_ == "circular"))
        {
            RCLCPP_ERROR(get_logger(),"Selected traj type is not valid!");
            throw std::runtime_error("Invalid traj_type");
        }

        declare_parameter("s_type", "trapezoidal");
        get_parameter("s_type", s_type_);
        RCLCPP_INFO(get_logger(),"Current s type is: '%s'", s_type_.c_str());
        if (!(s_type_ == "trapezoidal" || s_type_ == "cubic"))
        {
            RCLCPP_ERROR(get_logger(),"Selected s type is not valid!");
            throw std::runtime_error("Invalid s_type");
        }

        iteration_ = 0; t_ = 0.0;
        joint_state_available_ = false;

        // robot_description 
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
        while (!parameters_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                rclcpp::shutdown();
            }
            RCLCPP_INFO(this->get_logger(), "robot_state_publisher parameters service not available, waiting again...");
        }
        auto parameter = parameters_client->get_parameters({"robot_description"});

        KDL::Tree robot_tree;
        if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
            throw std::runtime_error("Failed to retrieve robot_description param!");
        }
        robot_ = std::make_shared<KDLRobot>(robot_tree);

        // Limiti giunti (TODO: leggi da URDF se disponibile)
        unsigned int nj = robot_->getNrJnts();
        KDL::JntArray q_min(nj), q_max(nj);
        q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96;
        q_max.data <<  2.96, 2.09, 2.96, 2.09, 2.96, 2.09, 2.96;
        robot_->setJntLimits(q_min,q_max);

        joint_positions_.resize(nj);
        joint_velocities_.resize(nj);
        joint_positions_cmd_.resize(nj);
        joint_velocities_cmd_.resize(nj);
        joint_efforts_cmd_.resize(nj);
        joint_efforts_cmd_.data.setZero();

        // Subscriber agli stati giunto
        jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

        // Attendi il primo JointState
        while(!joint_state_available_){
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for /joint_states ...");
            rclcpp::spin_some(node_handle_);
        }
        
        // Subscriber alla posa dell'ArUco marker
        arucoSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_single/pose",    // nome topic pubblicato da aruco_ros
            300,                      // queue size
            std::bind(&Iiwa_pub_sub::aruco_pose_callback, this, std::placeholders::_1)
        );

        // Inizializza robot, EE, controller
        robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
        KDL::Frame f_T_ee = KDL::Frame::Identity();
        robot_->addEE(f_T_ee);
        robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

        init_cart_pose_ = robot_->getEEFrame();

        controller_ = std::make_shared<KDLController>(*robot_);

        // Publisher del comando (resta uguale, ma sarà usato all'interno dell'esecuzione dell'action)
        if (cmd_interface_ == "position") {
            cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
        } else if (cmd_interface_ == "velocity") {
            cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
            if(ctrl_ == "vision"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                std::bind(&Iiwa_pub_sub::cmd_publisher_vision, this));
            }
        } else { // effort
            cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
        }

        // ACTION SERVER
        action_server_ = rclcpp_action::create_server<TrajectoryAction>(
            this,
            "trajectory", // nome action server
            std::bind(&Iiwa_pub_sub::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&Iiwa_pub_sub::handle_cancel, this, std::placeholders::_1),
            std::bind(&Iiwa_pub_sub::handle_accepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Action server 'trajectory' ready to receive goals.");
    }

private:
    // ACTION HANDLERS
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const TrajectoryAction::Goal> /*goal*/)
    {
        // Per semplicità accettiamo sempre.
        RCLCPP_INFO(this->get_logger(), "Received new goal request!");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleTrajectory> goal_handle)
    {
        RCLCPP_WARN(this->get_logger(), "Received request to cancel trajectory");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleTrajectory> goal_handle)
    {
        // Esegui in thread separato per non bloccare l'esecutore
        std::thread{std::bind(&Iiwa_pub_sub::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleTrajectory> goal_handle)
    {
    RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");

    auto goal = goal_handle->get_goal();

    // Aggiorna i parametri dal goal
    traj_duration_ = goal->traj_duration;
    acc_duration_  = goal->acc_duration;
    total_time_    = goal->total_time;
    trajectory_len_= goal->trajectory_len;
    Kp_            = goal->kp;
    end_pos_x_     = goal->end_position_x;
    end_pos_y_     = goal->end_position_y;
    end_pos_z_     = goal->end_position_z;

    auto feedback = std::make_shared<TrajectoryAction::Feedback>();
    auto result   = std::make_shared<TrajectoryAction::Result>();

    iteration_ = 0;
    t_ = 0.0;

    //Pianifica traiettoria come nel nodo originale
    Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0,0,0.1));
    //Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data));
    Eigen::Vector3d end_position; end_position << end_pos_x_, -end_pos_y_, end_pos_z_;
    double traj_radius = 0.15;

    if(traj_type_ == "linear"){
        planner_ = KDLPlanner(traj_duration_, acc_duration_, init_position, end_position);
        p_ = (s_type_ == "trapezoidal") ? planner_.linear_traj_trapezoidal(t_) : planner_.linear_traj_cubic(t_);
    } else {
        planner_ = KDLPlanner(traj_duration_, init_position, traj_radius, acc_duration_);
        p_ = (s_type_ == "trapezoidal") ? planner_.circular_traj_trapezoidal(t_) : planner_.circular_traj_cubic(t_);
    }

    
    int loop_rate = trajectory_len_ / total_time_;
    double dt = 1.0 / loop_rate;
    
    rclcpp::Rate rate(loop_rate);

    bool canceled = false;

    while (rclcpp::ok() && t_ < total_time_)
    {
        if (goal_handle->is_canceling()) { canceled = true; break; }

        iteration_++;
        t_ += dt;

        // Aggiorna il punto della traiettoria
        if(traj_type_ == "linear"){
            p_ = (s_type_ == "trapezoidal") ? planner_.linear_traj_trapezoidal(t_) : planner_.linear_traj_cubic(t_);
        } else {
            p_ = (s_type_ == "trapezoidal") ? planner_.circular_traj_trapezoidal(t_) : planner_.circular_traj_cubic(t_);
        }

        // Calcola errore e stato attuale
        KDL::Frame cartpos = robot_->getEEFrame();
        
        
        Eigen::Vector3d error   = computeLinearError(p_.pos, Eigen::Vector3d(cartpos.p.data));
        Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));

        // QUI IL BLOCCO DI CONTROLLO 
        
        if(cmd_interface_ == "position"){
                    // Next Frame
                    KDL::Frame nextFrame; nextFrame.M = cartpos.M; nextFrame.p = cartpos.p + (toKDL(p_.vel) + toKDL(Kp_*error))*dt; 

                    // Compute IK
                    joint_positions_cmd_ = joint_positions_;
                    robot_->getInverseKinematics(nextFrame, joint_positions_cmd_);
                }
                else if(cmd_interface_ == "velocity")
                {
                    // VELOCITY CONTROL
                    if(ctrl_ == "velocity_ctrl")
                    {
                        // Classical velocity controller
                        Vector6d cartvel;
                        cartvel << p_.vel + Kp_ * error, o_error;
                        joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data) * cartvel;
                        RCLCPP_INFO_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        1000, // ogni 1 secondo
                        "[TRAJ DEBUG] t=%.2f | pos=(%.3f, %.3f, %.3f) | vel=(%.3f, %.3f, %.3f) | acc=(%.3f, %.3f, %.3f)",
                        t_,
                        p_.pos[0], p_.pos[1], p_.pos[2],
                        p_.vel[0], p_.vel[1], p_.vel[2],
                        p_.acc[0], p_.acc[1], p_.acc[2]
                    );
                    }
                    else if(ctrl_ == "velocity_ctrl_null")
                    {
                        // Null-space velocity control
                        joint_velocities_cmd_.data = controller_->velocity_ctrl_null(
                            p_.pos,
                            Eigen::Vector3d(cartpos.p.data),
                            Kp_);
                    }
                }

                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                if(cmd_interface_ == "position"){
                    // Set joint position commands
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "velocity"){
                    // Set joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_velocities_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "effort"){
                    // Set joint effort commands
                    for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                        desired_commands_[i] = joint_efforts_cmd_(i);
                    }
                } 

        std_msgs::msg::Float64MultiArray cmd_msg;
        cmd_msg.data = desired_commands_;
        cmdPublisher_->publish(cmd_msg);

        // FEEDBACK
        feedback->position_error = error.norm();
        goal_handle->publish_feedback(feedback);

        rate.sleep();
    }

    // FASE DI STOP 
    if (canceled) {
        RCLCPP_WARN(this->get_logger(), "Trajectory canceled.");
        result->success = false;
        goal_handle->canceled(result);
        return;
    }

    // Alla fine, ferma il robot
    if (cmd_interface_ == "velocity" || cmd_interface_ == "effort") {
    for (auto &val : desired_commands_) val = 0.0;
    std_msgs::msg::Float64MultiArray stop_msg;
    stop_msg.data = desired_commands_;
    cmdPublisher_->publish(stop_msg);
    }

    RCLCPP_INFO(this->get_logger(), "Trajectory executed successfully.");
    result->success = true;
    goal_handle->succeed(result);

    }

    void cmd_publisher_vision(){
            // Verifica che il messaggio ArUco sia valido
            if (!aruco_msg) {
                RCLCPP_WARN(this->get_logger(), "No ArUco message available!");
                return;
            }
            std::cout << "Frame id = " << aruco_msg->header.frame_id << std::endl;
 
            iteration_ = iteration_ + 1;
            KDLController controller_(*robot_);
 
            // RCLCPP_INFO(this->get_logger(), "Executing %s", ctrl_.c_str());
 
            // Calcola matrice di trasformazione da aruco a image Plane
            KDL::Frame T_imgP_aruco = toKDL(aruco_msg->pose);
 
            // // matrice di trasformazione da image Plane a camera di tipo Eigen
            // double focal_length = 1.047;
            // double width = 320;
            // double height = 240;
            // Eigen::Matrix4d T_cam_imgP_eigen = T_camera_image_from_fov(focal_length, width, height);
 
            // // Conversione esplicita a KDL
            // Eigen::Matrix3d R_eigen = T_cam_imgP_eigen.block<3,3>(0,0);
            // KDL::Rotation R_c_i(
            //     R_eigen(0,0), R_eigen(0,1), R_eigen(0,2),
            //     R_eigen(1,0), R_eigen(1,1), R_eigen(1,2),
            //     R_eigen(2,0), R_eigen(2,1), R_eigen(2,2)
            // );
            // // Vector position
            // Eigen::Vector3d p_eigen = T_cam_imgP_eigen.block<3,1>(0,3);
            // KDL::Vector p_c_i(p_eigen(0), p_eigen(1), p_eigen(2));
            // // Matrice KDL
            // KDL::Frame T_cam_imgP = KDL::Frame(
            //     R_c_i,
            //     p_c_i
            // );
 
            // Matrix of the camera wrt ee
            KDL::Frame T_ee_cam = KDL::Frame(
                KDL::Rotation::RPY(3.14, -1.57, 0.0),
                KDL::Vector(0.0, 0.0, 0.17)
            );
            // pose and orientation of the aruco wrt ee
            current_aruco_frame_ee = T_ee_cam*T_imgP_aruco; // <-- Transformation in ee base
 
            // Compute EE frame
            KDL::Frame cartpos = robot_->getEEFrame();
 
            // Compute desired Frame (aruco wrt base)
            KDL::Frame desFrame = cartpos*current_aruco_frame_ee;
 
            // EE's trajectory end position: arucotag
            Eigen::Vector3d end_position_(desFrame.p.data); // -0.88, -0.450, 0.40
            // end_position_ << desFrame.p.data[0], desFrame.p.data[1], desFrame.p.data[2];
            
            std::cout << "end_position = " << end_position_ << std::endl;
 
            // compute errors
            // Eigen::Vector3d error = computeLinearError(p_.pos, Eigen::Vector3d(cartpos.p.data));
            Eigen::Vector3d error = computeLinearError(
                Eigen::Vector3d(desFrame.p.data),
                Eigen::Vector3d(cartpos.p.data));
            // Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));
            double p_err_norm = (double)error.norm();
            std::cout << "||e_p("<< t_ <<")|| = " << p_err_norm << std::endl;
            t_ ++;
 
            // error threshold
            double epsilon = 1e-3;
            double e_max = 10000;
 
            if(p_err_norm < epsilon || p_err_norm > e_max){
                // Set joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = 0.0; // Stop it
                }
                RCLCPP_INFO_ONCE(this->get_logger(), "Aruco tag followed successfully!");
            }
            else{
                // std::cout << "Running vision control." << std::endl;
                KDL::Chain chain_;
                robot_tree.getChain(
                                    robot_tree.getRootSegment()->first,
                                    std::prev(
                                                std::prev(
                                                            robot_tree.getSegments().end())
                                    )->first,
                                    chain_);
 
                unsigned int n = robot_->getNrJnts();
 
                double K_gain = 1;
                Eigen::MatrixXd K = Eigen::MatrixXd::Identity(n,n)*K_gain;
 
                joint_velocities_cmd_.data = controller_.vision_control(this->aruco_msg,
                                                                        chain_,
                                                                        K);
 
                // double dt = 0.1; // dipende dal timer
                // for (int i = 0; i < joint_positions_.data.size(); ++i) {
                //     joint_positions_.data[i] += joint_velocities_cmd_.data[i] * dt;
                //     // joint_velocities_.data[i] = joint_velocities_cmd_.data[i];
                // }
 
                // Update KDLrobot structure
                // robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
 
                // Set joint velocity commands
                // double vel_max = 0.3;
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    // if(joint_velocities_cmd_(i) < vel_max && joint_velocities_cmd_(i)>-vel_max)
                        desired_commands_[i] = joint_velocities_cmd_(i);
                    // else if(joint_velocities_cmd_(i) > vel_max){
                    //     desired_commands_[i] = vel_max;
                    // }
                    // else if(joint_velocities_cmd_(i) < -vel_max){
                    //     desired_commands_[i] = -vel_max;
                    // }
                }
 
            }// end else
            
            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);
 
        } // end function

    // JointState callback 
    void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){
        joint_state_available_ = true;
        for (unsigned int i  = 0; i < sensor_msg.position.size() && i < joint_positions_.rows(); i++){
            joint_positions_.data[i] = sensor_msg.position[i];
            if (i < sensor_msg.velocity.size())
                joint_velocities_.data[i] = sensor_msg.velocity[i];
        }
    }

    void aruco_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg){
            aruco_state_avaliable_ = true;
            RCLCPP_INFO(this->get_logger(),
                "ArucoTag pose: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
            
                aruco_msg = msg;
    }

    //  Membri 
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
    rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
    rclcpp::Node::SharedPtr node_handle_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr subTimer_;

    std::shared_ptr<KDLController> controller_;
    std::shared_ptr<KDLRobot> robot_;
    rclcpp_action::Server<TrajectoryAction>::SharedPtr action_server_;

    // Subscriber alla posa dell'ArUco marker
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arucoSubscriber_;

    // Flag per indicare se il marker è visibile
    bool marker_visible_ = false;
    KDL::Frame marker_pos_;  // posizione del marker nel frame camera

    std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    KDL::JntArray joint_positions_;
    KDL::JntArray joint_velocities_;
    KDL::JntArray joint_positions_cmd_;
    KDL::JntArray joint_velocities_cmd_;
    KDL::JntArray joint_efforts_cmd_;

    KDLPlanner planner_;
    trajectory_point p_;

    int iteration_;
    bool joint_state_available_;
    double t_;
    double traj_duration_;
    double acc_duration_;
    double total_time_;
    int trajectory_len_;
    double end_pos_x_;
    double end_pos_y_;
    double end_pos_z_;
    double Kp_;

    std::string cmd_interface_;
    std::string traj_type_;
    std::string s_type_;
    std::string ctrl_;

    KDL::Frame init_cart_pose_;
    geometry_msgs::msg::PoseStamped::ConstSharedPtr aruco_msg;
    KDL::Frame current_aruco_frame_ee;
    bool aruco_state_avaliable_;
    KDL::Tree robot_tree;
    
};

int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 0;
}

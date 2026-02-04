#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <cmath>
#include <string>
#include <vector>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" 
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "ros_gz_interfaces/srv/set_entity_pose.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class BiblioNode : public rclcpp::Node
{
    public:
        BiblioNode()
        : Node("biblio_node"), 
        node_handle_(std::shared_ptr<BiblioNode>(this))
        {
            // --- Inizializzazione variabili ---
            step_ = 0; 
            t_ = 0;
            joint_state_available_ = false; 
            item_grasped_ = false; 
            sent_completion_ = false;
            finish_timer_ = 0.0;
            traj_duration_ = 5.0;

            basket_detected_ = false;
            book_detected_ = false;
            observation_timer_ = 0.0;

            // --- Recupero parametri ---
            declare_parameter("robot_description", "");
            std::string robot_desc_string = get_parameter("robot_description").as_string();
            
            if(robot_desc_string.empty()) {
                 RCLCPP_ERROR(get_logger(), "Parametro robot_description vuoto!");
                 return;
            }

            // --- Setup KDL Robot 
            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(robot_desc_string, robot_tree)){
                RCLCPP_ERROR(get_logger(), "Failed to construct kdl tree");
                return;
            }
            
            robot_ = std::make_shared<KDLRobot>(robot_tree);  
            unsigned int nj = robot_->getNrJnts();
            
            // Impostazione limiti giunti 
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96;
            q_max.data <<  2.96, 2.09, 2.96, 2.09, 2.96, 2.09, 2.96;
            robot_->setJntLimits(q_min,q_max);            
            
            joint_positions_.resize(nj); 
            joint_velocities_.resize(nj); 
            joint_positions_cmd_.resize(nj); 
            desired_commands_.resize(nj);

            // --- Pub/Sub ---
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&BiblioNode::joint_state_subscriber, this, std::placeholders::_1));

            arucoBasketSub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/aruco_basket/pose", 10, std::bind(&BiblioNode::basket_callback, this, std::placeholders::_1));
            
            arucoBookSub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/aruco_book/pose", 10, std::bind(&BiblioNode::book_callback, this, std::placeholders::_1));

            cmdPublisher_ = this->create_publisher<FloatArray>("/biblio_joint_position_controller/commands", 10);
            taskDonePublisher_ = this->create_publisher<std_msgs::msg::Bool>("/iiwa_task_complete", 10);
            
            set_entity_pose_client_ = this->create_client<ros_gz_interfaces::srv::SetEntityPose>(
                "/world/libreria_world/set_pose"
            );

            ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                "/fra2mo/ready_for_load", 10, 
                [this](const std_msgs::msg::Bool::SharedPtr msg) {
                    if (msg->data) {
                        // Se il robot aveva finito (step 8), resettiamo tutto per una nuova ricerca
                        if (this->step_ == 8) {
                           this->step_ = 0;             
                            this->side_count_ = 0; 
                            this->success_flag_ = false;
                            this->sent_completion_ = false;
                            this->finish_timer_ = 0.0;
                            this->item_grasped_ = false;
                            this->book_detected_ = false;
                            this->observation_timer_ = 0.0;
                            RCLCPP_INFO(this->get_logger(), "Reset: IIWA pronto a cercare di nuovo il Tag Aruco sulla cesta.");
                        }
                        
                        this->fra2mo_ready_ = true; // Sblocca l' execution_loop
                    }
                });
           
            RCLCPP_INFO(this->get_logger(), "Waiting for joint states...");
            while(!joint_state_available_){
               rclcpp::spin_some(node_handle_);
            }

            // Inizializzazione cinematica End Effector
            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            robot_->addEE(f_T_ee);
            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

            // Setup Target
            init_cart_pose_ = robot_->getEEFrame();
           
            target_rotation_ = KDL::Rotation::RotY(M_PI); 
            target_pos_lower_ = Eigen::Vector3d(0.0, 0.0, 0.0);
            current_hover_ = Eigen::Vector3d(0.25, 0.6, 1.30);
            current_grasp_ = Eigen::Vector3d(0.25, 0.6, 1.05);

            // Avvio Timer
            timer_ = this->create_wall_timer(20ms, std::bind(&BiblioNode::execution_loop, this));
            RCLCPP_INFO(this->get_logger(), "Node Ready. Counter Step Mode.");
        }

    private:

        void basket_callback(const geometry_msgs::msg::PoseStamped::SharedPtr)
        {   
            if (!fra2mo_ready_) return;

            if (step_ == 0) {
                RCLCPP_INFO(get_logger(), "Step 1 : Rilevamento tag Aruco sulla cesta");
                robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
                init_cart_pose_ = robot_->getEEFrame();
                mobile_pos_ = Eigen::Vector3d(init_cart_pose_.p.data); 

                
                planner_ = KDLPlanner(5.0, 0.5, mobile_pos_, current_hover_);
                step_ = 1; 
                t_ = 0; 
            }
        }

        void book_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
                            
                if (step_ != 1) return;

                if (!book_detected_) {
                    RCLCPP_INFO(this->get_logger(), "Tag del libro rilevato (ID 15) ");
                }
                book_detected_ = true;

            
        }

        void execution_loop()
        {   if (!joint_state_available_) return;
        
            double dt = 0.02; 
            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
            KDL::Frame current_pose = robot_->getEEFrame();
            Eigen::Vector3d current_pos_vec(current_pose.p.data);

                    // Se Fra2mo non è pronto, inviamo la posa attuale come comando (per stare fermi) ed usciamo
            if (!fra2mo_ready_ && step_ == 0) {
                std_msgs::msg::Float64MultiArray cmd_msg;
                for (unsigned int i = 0; i < 7; ++i) cmd_msg.data.push_back(joint_positions_cmd_(i));
                cmdPublisher_->publish(cmd_msg);
                return;
            }

            // Se siamo allo Step 0 (in attesa del Tag Aruco), non calcoliamo traiettorie nuove
            if (step_ == 0) return;
            
            if (item_grasped_) teleport();

            // Calcolo Errore Dinamico
            double error_norm = 100.0; 
            if (step_ == 1 || step_ == 4) error_norm = (current_pos_vec - current_hover_).norm();
            else if (step_ == 2) error_norm = (current_pos_vec - current_grasp_).norm();
            else if (step_ == 5 || step_ == 7) error_norm = (current_pos_vec - mobile_pos_).norm();
            else if (step_ == 6) error_norm = (current_pos_vec - target_pos_lower_).norm();
            if (t_ <= traj_duration_) t_ += dt;

            
            if (t_ >= traj_duration_) {
                if (step_ == 1 && error_norm < 0.05) { 
        
                    
                    observation_timer_ += dt; 

                    if (book_detected_) {
                        
                        RCLCPP_INFO(get_logger(), "Step 2 : Libro rilevato. Procedo al prelievo.");
                        step_ = 2; t_ = 0; traj_duration_ = 1.5; 
                        planner_ = KDLPlanner(traj_duration_, 0.5, current_hover_, current_grasp_);
                        
                    
                    } 
                    else if (observation_timer_ >= MAX_OBSERVATION_TIME) {
                        
                        RCLCPP_WARN(get_logger(), "Step 2 : Libro NON trovato. Ritorno alla posizione iniziale.");
                        

                        // In entrambi i casi (SX o DX), torna alla posizione mobile
                        step_ = 7; 
                        t_ = 0; 
                        traj_duration_ = 3.0; 
                        planner_ = KDLPlanner(traj_duration_, 0.5, current_hover_, mobile_pos_);
                        observation_timer_ = 0; // Reset per sicurezza
                        
                    }
                    
                    if (observation_timer_ < MAX_OBSERVATION_TIME) {
                        book_detected_ = false; 
                    }
                  
                }
                else if (step_ == 2 && error_norm < 0.02) { 
                    RCLCPP_INFO(get_logger(), "Step 3 : Prelevamento libro");
                    step_ = 3; t_ = 0; traj_duration_ = 2.0; item_grasped_ = true;
                    planner_ = KDLPlanner(traj_duration_, 0.5, current_grasp_, current_grasp_);
                }
                else if (step_ == 3) { 
                    
                    step_ = 4; t_ = 0; traj_duration_ = 3.0; 
                    planner_ = KDLPlanner(traj_duration_, 0.5, current_grasp_, current_hover_);
                }
                else if (step_ == 4 && error_norm < 0.05) { 
                    RCLCPP_INFO(get_logger(), "Step 4 : Trasporto libro");
                    step_ = 5; t_ = 0; traj_duration_ = 3.0; 
                    planner_ = KDLPlanner(traj_duration_, 0.5, current_hover_, mobile_pos_);
                }
                else if (step_ == 5 && error_norm < 0.05) { 
                    RCLCPP_INFO(get_logger(), "Step 5 : Posizionamento del libro nella cesta");
                    step_ = 6; t_ = 0; traj_duration_ = 3.0;
                    target_pos_lower_ = mobile_pos_; target_pos_lower_.z() -= 0.40; 
                    planner_ = KDLPlanner(traj_duration_, 0.5, mobile_pos_, target_pos_lower_);
                }
                else if (step_ == 6 && error_norm < 0.02) { 
                    // 1. Fermiamo il teletrasporto: il libro resta nella cesta
                    item_grasped_ = false; 
                    
                    // 2. Salviamo il successo: così il robot sa che non deve andare a destra
                    success_flag_ = true;

                    step_ = 7; t_ = 0; traj_duration_ = 3.0;
                    planner_ = KDLPlanner(traj_duration_, 0.5, target_pos_lower_, mobile_pos_);
                }
                else if (step_ == 7 && error_norm < 0.02) { 
                    if (success_flag_) {
                        RCLCPP_INFO(get_logger(), "Step 6 : Deposito completato. Missione riuscita!");
                        
                        // Reset per la prossima eventuale missione
                        success_flag_ = false; 
                        
                        step_ = 8; // Fine: manderà Bool(true) a Fra2mo
                    }
                    
                    else if (side_count_ == 0) {
                        // --- TRANSIZIONE DA SINISTRA A DESTRA ---
                        RCLCPP_INFO(get_logger(), "Lato Sinistro completato. Spostamento a DESTRA.");
                        side_count_ = 1;
                        step_ = 1; // RESET DEGLI STEP
                        t_ = 0;
                        traj_duration_ = 5.0;
                        book_detected_ = false;
                        observation_timer_ = 0;
                        
                        // Aggiorna i target per la destra (Y negativa)
                        current_hover_ = Eigen::Vector3d(0.2, -0.4, 1.30);
                        current_grasp_ = Eigen::Vector3d(0.2, -0.4, 1.05);
                        
                        planner_ = KDLPlanner(traj_duration_, 0.5, mobile_pos_, current_hover_);
                    } else {
                        // --- CASO FINALE: Libro non trovato né a sinistra né a destra ---
                        RCLCPP_ERROR(get_logger(), "Libro non trovato in nessuna posizione. Invio FALLIMENTO a Fra2mo.");
                        std_msgs::msg::Bool fail_msg;
                        fail_msg.data = false;
                        taskDonePublisher_->publish(fail_msg);
                        sent_completion_ = true; 
                        step_ = 8;
                    }
                }
            }

            // Esecuzione o Fine
            if (step_ < 8) {
                trajectory_point p = planner_.linear_traj_trapezoidal(t_);
                
                
                double s = (traj_duration_ > 0) ? (t_ / traj_duration_) : 1.0;
                if (s > 1.0) s = 1.0;
                KDL::Rotation R_diff = init_cart_pose_.M.Inverse() * target_rotation_;
                KDL::Vector rot_axis; double rot_angle = R_diff.GetRotAngle(rot_axis); 
                KDL::Rotation current_des_rot = init_cart_pose_.M * KDL::Rotation::Rot(rot_axis, rot_angle * s);

                KDL::Frame desFrame(current_des_rot, toKDL(p.pos));
                robot_->getInverseKinematics(desFrame, joint_positions_cmd_); 

                std_msgs::msg::Float64MultiArray cmd_msg;
                for (unsigned int i = 0; i < 7; ++i) cmd_msg.data.push_back(joint_positions_cmd_(i));
                cmdPublisher_->publish(cmd_msg);
            } 
            else {
                
                finish_timer_ += 0.02;
                if (finish_timer_ >= 4.0 && !sent_completion_) {
                    std_msgs::msg::Bool msg; msg.data = true;
                    taskDonePublisher_->publish(msg);
                    RCLCPP_INFO(this->get_logger(), "IIWA ha terminato, Fra2mo può partire!");
                    sent_completion_ = true;

                    fra2mo_ready_ = false; 
                    step_ = 0;

                }
                std_msgs::msg::Float64MultiArray cmd_msg;
                for (unsigned int i = 0; i < 7; ++i) cmd_msg.data.push_back(joint_positions_cmd_(i));
                cmdPublisher_->publish(cmd_msg);
            }
        }

        void teleport() {
            if (!set_entity_pose_client_->service_is_ready()) return; 
            KDL::Frame ee = robot_->getEEFrame();
            auto request = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
            request->entity.name = "tomo"; 
            request->pose.position.x = ee.p.x();
            request->pose.position.y = ee.p.y();
            request->pose.position.z = ee.p.z() - 0.07;
            ee.M.GetQuaternion(request->pose.orientation.x, request->pose.orientation.y, 
                               request->pose.orientation.z, request->pose.orientation.w);
            set_entity_pose_client_->async_send_request(request);
        }

        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){
            joint_state_available_ = true;
            for (unsigned int i = 0; i < 7; i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
            }
        }

        // Variabili membro
        int step_; 
        double t_, traj_duration_, finish_timer_;
        bool joint_state_available_, item_grasped_, sent_completion_;
        bool success_flag_ = false;
        bool basket_detected_;
        bool book_detected_;
        bool fra2mo_ready_ = false; // Inizialmente false per bloccare il robot
        double observation_timer_;
        const double MAX_OBSERVATION_TIME = 3.0;
        int side_count_ = 0; // 0 per sinistra, 1 per destra
        Eigen::Vector3d current_hover_, current_grasp_;
        
        KDL::JntArray joint_positions_{7}, joint_velocities_{7}, joint_positions_cmd_{7};
        std::vector<double> desired_commands_;
        std::shared_ptr<KDLRobot> robot_;
        KDLPlanner planner_;

        KDL::Frame init_cart_pose_;
        
        Eigen::Vector3d mobile_pos_;
        Eigen::Vector3d target_pos_lower_;
        KDL::Rotation target_rotation_;

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arucoBasketSub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arucoBookSub_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr taskDonePublisher_;
        rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr set_entity_pose_client_;
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::Node::SharedPtr node_handle_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BiblioNode>());
    rclcpp::shutdown();
    return 0;


    rclcpp::shutdown();
    return 0;
}
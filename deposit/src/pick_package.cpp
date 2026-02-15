#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/empty.hpp"
#include "ros_gz_interfaces/srv/set_entity_pose.hpp"

// TF2 for obtaining end-effector transform (world -> ee link)
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::chrono_literals;

struct ArucoTag {
    int id;
    double x, y, z;
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
    bool picked = false;
    bool frozen = false;
    double distance_to_camera = 1e6;
};

class PickAndPlaceNode : public rclcpp::Node {
public:
    PickAndPlaceNode() : Node("pick_package_node") {
        
        // Subscriber ai joint state
        joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/iiwa/joint_states", 10, 
            std::bind(&PickAndPlaceNode::joint_callback, this, std::placeholders::_1));
        
        // Subscriber ai tag ArUco
        aruco_0_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_single_0/pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { 
                RCLCPP_DEBUG(get_logger(), "[SUB] Ricevuto messaggio da /aruco_single_0/pose");
                this->aruco_callback(msg, 0); 
            });
        
        aruco_999_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_single_999/pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { 
                RCLCPP_DEBUG(get_logger(), "[SUB] Ricevuto messaggio da /aruco_single_999/pose");
                this->aruco_callback(msg, 999); 
            });

        RCLCPP_INFO(get_logger(), "[INIT] Subscribed to /aruco_single_0/pose and /aruco_single_999/pose");

        // Publisher per comandi ai giunti
        cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/iiwa/iiwa_arm_controller/commands", 10);

        // Publisher per vacuum gripper (attach/detach)
        vacuum_attach_pub_ = create_publisher<std_msgs::msg::Empty>(
            "/iiwa/gripper/attach", 10);
        vacuum_detach_pub_ = create_publisher<std_msgs::msg::Empty>(
            "/iiwa/gripper/detach", 10);

        // Publisher per pose dell'end-effector
        ee_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/ee_pose", 10);

        // ===== CLIENT PER SET_ENTITY_POSE - MUOVE IL PACKAGE AL GRIPPER =====
        set_entity_pose_client_ = this->create_client<ros_gz_interfaces::srv::SetEntityPose>(
            "/world/personal_project_world/set_pose");

        // Initialize TF2 buffer and listener so we can query the end-effector pose (in world frame)
        // We will lookup the transform from "world" -> "iiwa_camera_link" (camera attached to EE)
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Timer per control loop
        timer_ = create_wall_timer(20ms, 
            std::bind(&PickAndPlaceNode::control_loop, this));

        state_ = STATE_MOVE_TO_BIN;
        tags_placed_ = 0;
        joint_state_ready_ = false;
        item_grasped_ = false;
        current_target_id_ = -1;
        last_state_change_time_ = std::chrono::steady_clock::now();
        max_velocity_ = 1.2;
    }

private:
    enum State { 
        STATE_DEBUG_MOVE,       // DEBUG: Sposta verso coordinate fisse
        STATE_MOVE_TO_BIN,      // Sposta il gripper sopra il cassone
        STATE_SEARCH_TAGS,      // Cerca i tag più vicini
        STATE_MOVE_TO_TAG,      // Sposta il gripper verso il tag
        STATE_GRASP_OBJECT,     // Attacca l'oggetto con vacuum
        STATE_MOVE_TO_SHELF,    // Sposta il gripper verso la shelf
        STATE_RELEASE_OBJECT,   // Rilascia l'oggetto
        STATE_RETURN_TO_BIN,    // Torna al cassone
        STATE_FINISHED          // Missione completata
    };

    State state_;
    int tags_placed_;
    bool joint_state_ready_;
    bool item_grasped_;
    int current_target_id_;
    std::vector<ArucoTag> detected_tags_;
    std::vector<double> current_joint_positions_;
    std::chrono::steady_clock::time_point last_state_change_time_;
    int state_iteration_counter_ = 0;
    
    bool movement_phase_active_ = false;
    double saved_tag_x_ = 0.0, saved_tag_y_ = 0.0, saved_tag_z_ = 0.0;
    bool tag_pose_saved_ = false;
    
    // ===== VARIABILI STATICHE PER OGNI STATO (COME MEMBRO DI CLASSE) =====
    bool bin_movement_started_ = false;
    bool shelf_movement_started_ = false;
    bool return_movement_started_ = false;
    bool pose_saved_for_current_target_ = false;
    double saved_x_for_tag_ = 0.0, saved_y_for_tag_ = 0.0, saved_z_for_tag_ = 0.0;
    bool grasp_started_ = false;
    
    std::vector<double> target_joint_positions_;
    std::vector<double> interpolated_joint_positions_;
    double max_velocity_ = 0.3;
    bool interpolating_ = false;
    
    std::chrono::steady_clock::time_point last_teleport_time_;
    int teleport_interval_ms_ = 750;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_0_sub_, aruco_999_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr vacuum_attach_pub_, vacuum_detach_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_pub_;
    rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr set_entity_pose_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->position.size() < 7) return;
        
        current_joint_positions_.clear();
        current_joint_positions_.resize(7);
        
        for (size_t i = 0; i < 7 && i < msg->position.size(); ++i) {
            current_joint_positions_[i] = msg->position[i];
        }
        
        joint_state_ready_ = true;
    }
         

    void aruco_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int tag_id) {
        RCLCPP_DEBUG(get_logger(), "[ARUCO_CB] Ricevuto messaggio per tag ID=%d", tag_id);
        
        // Aggiorna o crea il tag
        bool found = false;
        for (auto &tag : detected_tags_) {
            if (tag.id == tag_id) {
                if (tag.frozen) {
                    return;
                }
                
                // Primo aggiornamento: salva la posa
                tag.x = msg->pose.position.x;
                tag.y = msg->pose.position.y;
                tag.z = msg->pose.position.z;
                tag.qx = msg->pose.orientation.x;
                tag.qy = msg->pose.orientation.y;
                tag.qz = msg->pose.orientation.z;
                tag.qw = msg->pose.orientation.w;
                
                // Calcola distanza dalla camera (assumendo camera in origine frame)
                tag.distance_to_camera = std::sqrt(msg->pose.position.x * msg->pose.position.x + 
                                                   msg->pose.position.y * msg->pose.position.y + 
                                                   msg->pose.position.z * msg->pose.position.z);
                
                tag.frozen = true;
                RCLCPP_INFO(get_logger(), "Tag ID=%d rilevato. Pose: x=%.4f y=%.4f z=%.4f, Distanza=%.3f", 
                           tag_id, tag.x, tag.y, tag.z, tag.distance_to_camera);
                found = true;
                break;
            }
        }
        if (!found) {
            ArucoTag new_tag;
            new_tag.id = tag_id;
            new_tag.x = msg->pose.position.x;
            new_tag.y = msg->pose.position.y;
            new_tag.z = msg->pose.position.z;
            new_tag.qx = msg->pose.orientation.x;
            new_tag.qy = msg->pose.orientation.y;
            new_tag.qz = msg->pose.orientation.z;
            new_tag.qw = msg->pose.orientation.w;
            
            new_tag.distance_to_camera = std::sqrt(msg->pose.position.x * msg->pose.position.x + 
                                                   msg->pose.position.y * msg->pose.position.y + 
                                                   msg->pose.position.z * msg->pose.position.z);
            
            // ===== FREEZING: Marca come freezato =====
            new_tag.frozen = true;
            detected_tags_.push_back(new_tag);
            RCLCPP_INFO(get_logger(), "Tag ArUco rilevato: ID=%d, Pose: x=%.4f y=%.4f z=%.4f", 
                       tag_id, new_tag.x, new_tag.y, new_tag.z);
        }
    }

    ArucoTag* get_closest_unpicked_tag() {
        ArucoTag* closest = nullptr;
        double min_distance = 1e6;
        for (auto &tag : detected_tags_) {
            if (!tag.picked && tag.distance_to_camera < min_distance) {
                min_distance = tag.distance_to_camera;
                closest = &tag;
            }
        }
        return closest;
    }

    ArucoTag* get_tag_by_id(int tag_id) {
        for (auto &tag : detected_tags_) {
            if (tag.id == tag_id) return &tag;
        }
        return nullptr;
    }

    void send_joint_command(const std::vector<double>& joint_angles) {
        if (joint_angles.size() != 7) {
            RCLCPP_WARN(get_logger(), "Joint command con dimensione errata: %zu (attesi 7)", 
                       joint_angles.size());
            return;
        }
        
        target_joint_positions_ = joint_angles;
        interpolated_joint_positions_ = current_joint_positions_;
        if (interpolated_joint_positions_.size() != 7) {
            interpolated_joint_positions_.resize(7, 0.0);
        }
        interpolating_ = true;
        
        // Log SOLO una volta quando il target cambia (non ad ogni ciclo)
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
            "[SEND_CMD] INIZIO INTERPOLAZIONE verso: J0=%.4f J1=%.4f J2=%.4f J3=%.4f J4=%.4f J5=%.4f J6=%.4f",
            joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3],
            joint_angles[4], joint_angles[5], joint_angles[6]);
    }
    
    void update_interpolation() {
        if (!interpolating_ || target_joint_positions_.size() != 7) return;
        
        // Intervallo di tempo per ogni pubblicazione (50ms = 0.05s dal timer)
        double dt = 0.04;
        bool reached_target = true;
        
        for (size_t i = 0; i < 7; ++i) {
            double diff = target_joint_positions_[i] - interpolated_joint_positions_[i];
            double max_change = max_velocity_ * dt;  // Massimo cambio permesso in questo step
            
            if (std::abs(diff) > max_change) {
                // Non abbiamo raggiunto il target, continua a muoversi
                interpolated_joint_positions_[i] += (diff > 0 ? max_change : -max_change);
                reached_target = false;
            } else {
                // Abbiamo raggiunto il target per questo giunto
                interpolated_joint_positions_[i] = target_joint_positions_[i];
            }
        }
        
        // Pubblica la posizione interpolata
        std_msgs::msg::Float64MultiArray cmd;
        cmd.data = interpolated_joint_positions_;
        cmd_pub_->publish(cmd);
        
        // Se abbiamo raggiunto il target, interrompi l'interpolazione
        if (reached_target) {
            interpolating_ = false;
        }
    }
    
    // Funzione originale (mantiene compatibilità)
    void send_joint_command_old(const std::vector<double>& joint_angles) {
        if (joint_angles.size() != 7) {
            RCLCPP_WARN(get_logger(), "Joint command con dimensione errata: %zu (attesi 7)", 
                       joint_angles.size());
            return;
        }
        std_msgs::msg::Float64MultiArray cmd;
        cmd.data = joint_angles;
        
        // ===== LOG DETTAGLIATO DEI COMANDI MANDATI =====
        RCLCPP_INFO(get_logger(), 
            "[SEND_CMD] COMANDO MANDATO: J0=%.4f J1=%.4f J2=%.4f J3=%.4f J4=%.4f J5=%.4f J6=%.4f",
            joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3],
            joint_angles[4], joint_angles[5], joint_angles[6]);
        
        cmd_pub_->publish(cmd);
    }

    void vacuum_attach() {
        if (!vacuum_attach_pub_) {
            RCLCPP_ERROR(get_logger(), "ERROR: vacuum_attach_pub_ è NULL!");
            return;
        }
        auto msg = std_msgs::msg::Empty();
        vacuum_attach_pub_->publish(msg);
        item_grasped_ = true;
        RCLCPP_INFO(get_logger(), "[VACUUM] ✓ ATTACH pubblicato per tag ID %d", current_target_id_);
    }

    void vacuum_detach() {
        if (!vacuum_detach_pub_) {
            RCLCPP_ERROR(get_logger(), "ERROR: vacuum_detach_pub_ è NULL!");
            return;
        }
        auto msg = std_msgs::msg::Empty();
        vacuum_detach_pub_->publish(msg);
        item_grasped_ = false;
        RCLCPP_INFO(get_logger(), "[VACUUM] ✓ DETACH pubblicato per tag ID %d", current_target_id_);
    }

    void attach_tag_to_gripper(int tag_id) {
        // ===== ATTACCA IL TAG: SETTA IL FLAG item_grasped_ =====
        RCLCPP_INFO(get_logger(), "[ATTACH] ✓ Tag ID %d marcato come afferrato. Inizierò il teletrasporto continuo.", tag_id);
        item_grasped_ = true;
    }

    void teleport_package() {
        if (!item_grasped_) {
            return;
        }
        
        // Determina il nome del package basandosi sul current_target_id_
        std::string package_name = (current_target_id_ == 0) ? "package1" : "package2";
        
        // Leggi la posa attuale dell'EE dal TF2
        geometry_msgs::msg::TransformStamped ee_tf;
        try {
            ee_tf = tf_buffer_->lookupTransform("world", "camera_link", tf2::TimePointZero);
        } catch (const std::exception &ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, 
                "TF lookup fallito: %s", ex.what());
            return;
        }
        
        // Pubblica la posa EE su /ee_pose per debug
        geometry_msgs::msg::PoseStamped ee_pose_msg;
        ee_pose_msg.header.stamp = get_clock()->now();
        ee_pose_msg.header.frame_id = "world";
        ee_pose_msg.pose.position.x = ee_tf.transform.translation.x;
        ee_pose_msg.pose.position.y = ee_tf.transform.translation.y;
        ee_pose_msg.pose.position.z = ee_tf.transform.translation.z;
        ee_pose_msg.pose.orientation.x = ee_tf.transform.rotation.x;
        ee_pose_msg.pose.orientation.y = ee_tf.transform.rotation.y;
        ee_pose_msg.pose.orientation.z = ee_tf.transform.rotation.z;
        ee_pose_msg.pose.orientation.w = ee_tf.transform.rotation.w;
        ee_pose_pub_->publish(ee_pose_msg);
        
        // Crea la richiesta SetEntityPose con la posa attuale dell'EE
        auto request = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
        request->entity.name = package_name;
        
        // ===== OFFSET DOVUTO ALLO SPAWN DELL'IIWA =====
        // L'IIWA è spawnato a (8.40, -4, 0.1) nel launch file
        double iiwa_spawn_x = 8.40;
        double iiwa_spawn_y = -4.0;
        double iiwa_spawn_z = 0.0;
        
        request->pose.position.x = ee_tf.transform.translation.x + iiwa_spawn_x + 0.08;
        request->pose.position.y = ee_tf.transform.translation.y + iiwa_spawn_y;
        request->pose.position.z = ee_tf.transform.translation.z + iiwa_spawn_z;
        
        request->pose.orientation.x = ee_tf.transform.rotation.x;
        request->pose.orientation.y = ee_tf.transform.rotation.y;
        request->pose.orientation.z = ee_tf.transform.rotation.z;
        request->pose.orientation.w = ee_tf.transform.rotation.w;
        
        auto future_result = set_entity_pose_client_->async_send_request(request);
        
        // Log DEBUG per il teleport - molto verbose, solo per debug
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
            "[TELEPORT] 🎯 %s -> (x=%.4f y=%.4f z=%.4f)",
            package_name.c_str(), 
            request->pose.position.x, request->pose.position.y, request->pose.position.z);
    }

    std::vector<double> get_home_position() {
        // Posizione di "home" (predefinita per IIWA)
        return {0.0, 0.0, 0.0, -1.57, 0.0, 1.0, 0.0};
    }

    std::vector<double> get_bin_hover_position() {
        return {0.0, 0.0, 0.0, -1.0, 0.0, +1.9, 0.0};
    }

    std::vector<double> get_low_shelf_position() {
        return {-1.57, 0.6, 0.4, -0.8, 0.0, 1.2, 0.0};
    }

    std::vector<double> get_high_shelf_position() {
        return {1.57, 0.6, 0.4, -0.8, 0.0, 1.2, 0.0};
    }

    std::vector<double> get_debug_position() {
        return {0.0, 0.1, 0.0, -0.7, 0.0, 0.5, 0.0};
    }

    std::vector<double> cartesian_to_joint_angles(double x, double y, double z) {
        // ===== CONVERSIONE COORDINATE CARTESIANE -> JOINT ANGLES =====
        // Questa è una conversione SEMPLIFICATA per il pick & place
        
        std::vector<double> joint_angles(7);
        
        joint_angles[0] = 0.0;
        joint_angles[1] = 0.3;
        joint_angles[2] = 0.0;
        joint_angles[3] = -1.7;
        joint_angles[4] = 0.0;
        joint_angles[5] = 1.1;
        joint_angles[6] = 0.0;
        
        return joint_angles;
    }

    void control_loop() {
        update_interpolation();
        
        if (item_grasped_) {
            teleport_package();
        }
        
        if (!joint_state_ready_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, 
                                "In attesa di joint states...");
            return;
        }

        auto now = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_state_change_time_).count();

        switch (state_) {
            case STATE_DEBUG_MOVE: {
                send_joint_command(get_debug_position());
                break;
            }

            case STATE_MOVE_TO_BIN: {
                if (!bin_movement_started_) {
                    RCLCPP_INFO(get_logger(), "[STATE] MOVE_TO_BIN - Sposta gripper sopra cassone");
                    bin_movement_started_ = true;
                }
                
                if (elapsed_ms < 7000) {
                    send_joint_command(get_bin_hover_position());
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, 
                        "[MOVE_TO_BIN] 🔴 MOVIMENTO IN CORSO (%.1f sec / 5 sec)", elapsed_ms / 1000.0);
                } else {
                    // ===== FASE 2: HOLD (restanti secondi - robot FERMO) =====
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                        "[MOVE_TO_BIN] 🟢 HOLD POSITION - ROBOT FERMO (%.1f sec)", (elapsed_ms - 7000) / 1000.0);
                    // ← NESSUN send_joint_command() qui!
                }
                
                if (elapsed_ms > 7000) {
                    bin_movement_started_ = false;
                    state_ = STATE_SEARCH_TAGS;
                    last_state_change_time_ = now;
                }
                break;
            }

            case STATE_SEARCH_TAGS: {
                ArucoTag* closest = get_closest_unpicked_tag();
                if (closest) {
                    current_target_id_ = closest->id;
                    state_ = STATE_MOVE_TO_TAG;
                    last_state_change_time_ = now;
                    RCLCPP_INFO(get_logger(), 
                               "[TRANSITION] STATE_SEARCH_TAGS -> STATE_MOVE_TO_TAG (tag_id=%d, dist=%.3f)",
                               closest->id, closest->distance_to_camera);
                } else {
                    if (tags_placed_ >= 2) {
                        state_ = STATE_FINISHED;
                        RCLCPP_INFO(get_logger(), "[TRANSITION] Tutti i tag sono stati piazzati! FINISHED");
                    } else {
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, 
                                           "Nessun tag non-picked trovato");
                    }
                }
                break;
            }

            case STATE_MOVE_TO_TAG: {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, 
                                    "[STATE] MOVE_TO_TAG - Sposta verso tag ID %d", 
                                    current_target_id_);
                
                ArucoTag* target = get_tag_by_id(current_target_id_);
                if (target && target->frozen) {
                    if (!pose_saved_for_current_target_) {
                        saved_x_for_tag_ = target->x;
                        saved_y_for_tag_ = target->y;
                        saved_z_for_tag_ = target->z;
                        pose_saved_for_current_target_ = true;
                        
                        RCLCPP_INFO(get_logger(), 
                            "[MOVE_TO_TAG] ✓ POSA DEL TAG SALVATA! ID=%d, Pose fissa: x=%.4f y=%.4f z=%.4f",
                            target->id, saved_x_for_tag_, saved_y_for_tag_, saved_z_for_tag_);
                    }
                    
                    // ===== FASE 1: MOVIMENTO (primi 3 secondi) =====
                    if (elapsed_ms < 5000) {
                        // Converte coordinate cartesiane SALVATE in joint angles
                        std::vector<double> target_angles = cartesian_to_joint_angles(saved_x_for_tag_, saved_y_for_tag_, saved_z_for_tag_);
                        send_joint_command(target_angles);
                        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                            "[MOVE_TO_TAG] Movimento in corso... (%.1f sec)", elapsed_ms / 1000.0);
                    } else {
                        // ===== FASE 2: HOLD (restanti 7 secondi - robot FERMO) =====
                        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                            "[MOVE_TO_TAG] Hold position completato (%.1f sec / 10 sec)", elapsed_ms / 1000.0);
                    }
                    
                    if (elapsed_ms > 10000) {  // ===== 10 SECONDI PER OSSERVARE =====
                        pose_saved_for_current_target_ = false;  // Reset per il prossimo tag
                        state_ = STATE_GRASP_OBJECT;
                        last_state_change_time_ = now;
                        RCLCPP_INFO(get_logger(), "[TRANSITION] STATE_MOVE_TO_TAG -> STATE_GRASP_OBJECT (OSSERVAZIONE COMPLETATA)");
                    }
                } else {
                    state_ = STATE_SEARCH_TAGS;
                    current_target_id_ = -1;
                    last_state_change_time_ = now;
                }
                break;
            }

            case STATE_GRASP_OBJECT: {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "[STATE] GRASP_OBJECT - Attacca il tag all'end-effector");
                
                ArucoTag* target = get_tag_by_id(current_target_id_);
                if (target && target->frozen) {
                    // Al primo ingresso, attiva il grasp
                    if (!grasp_started_) {
                        auto empty_msg = std_msgs::msg::Empty();
                        vacuum_attach_pub_->publish(empty_msg);
                        
                        item_grasped_ = true;
                        target->picked = true;
                        grasp_started_ = true;
                        
                        RCLCPP_INFO(get_logger(), "[GRASP] Pausa di 500ms per stabilizzare l'attach...");
                        std::this_thread::sleep_for(std::chrono::milliseconds(500));
                        RCLCPP_INFO(get_logger(), "[GRASP] Robot FERMO - Teletrasporto sincronizzando il pacco al gripper...");
                    }
                    
                    if (elapsed_ms > 3000) {
                        grasp_started_ = false;
                        state_ = STATE_MOVE_TO_SHELF;
                        last_state_change_time_ = now;
                    }
                } else {
                    grasp_started_ = false;
                    state_ = STATE_SEARCH_TAGS;
                    current_target_id_ = -1;
                    last_state_change_time_ = now;
                }
                break;
            }

            case STATE_MOVE_TO_SHELF: {
                // ===== RESET AL PRIMO INGRESSO NELLO STATO =====
                if (!shelf_movement_started_) {
                    RCLCPP_INFO(get_logger(), "[STATE] MOVE_TO_SHELF - Trasporta verso la shelf (PATTERN KUKA_BRAIN)");
                    shelf_movement_started_ = true;
                }
                
                ArucoTag* target = get_tag_by_id(current_target_id_);
                if (!target) {
                    state_ = STATE_SEARCH_TAGS;
                    break;
                }

                // ===== FASE 1: MOVIMENTO (primi 5 secondi) =====
                if (elapsed_ms < 5000) {
                    if (target->id == 0) {
                        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, 
                                            "[MOVE_TO_SHELF] MOVIMENTO verso shelf BASSA (%.1f sec / 5 sec)", elapsed_ms / 1000.0);
                        send_joint_command(get_low_shelf_position());
                    } else {
                        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, 
                                            "[MOVE_TO_SHELF] MOVIMENTO verso shelf ALTA (%.1f sec / 5 sec)", elapsed_ms / 1000.0);
                        send_joint_command(get_high_shelf_position());
                    }
                } else {
                    // ===== FASE 2: HOLD (restanti secondi - robot FERMO) =====
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                        "[MOVE_TO_SHELF] HOLD POSITION - ROBOT FERMO (%.1f sec)", (elapsed_ms - 5000) / 1000.0);
                    // ← NESSUN send_joint_command() qui!
                }

                if (elapsed_ms > 5000) {
                    shelf_movement_started_ = false;
                    state_ = STATE_RELEASE_OBJECT;
                    last_state_change_time_ = now;
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                }
                break;
            }

            case STATE_RELEASE_OBJECT: {
                RCLCPP_INFO(get_logger(), "[STATE] RELEASE_OBJECT - Rilascia oggetto (vacuum detach)");
                
                // ===== PATTERN DA KUKA_BRAIN: PAUSA PRIMA DEL DETACH =====
                RCLCPP_INFO(get_logger(), "[RELEASE]   Pausa di 500ms prima del detach per stabilizzare...");
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                
                // Ferma il teletrasporto: il package rimane dove è
                item_grasped_ = false;
                RCLCPP_INFO(get_logger(), "[RELEASE] ✓ STOP TELETRASPORTO - Il package rimane in posizione");
                
                ArucoTag* target = get_tag_by_id(current_target_id_);
                if (target) {
                    target->picked = true;
                    tags_placed_++;
                    RCLCPP_INFO(get_logger(), "Tag ID %d piazzato! Totale: %d/2", 
                               target->id, tags_placed_);
                }
                
                if (!vacuum_detach_pub_) {
                    RCLCPP_ERROR(get_logger(), "ERROR: vacuum_detach_pub_ è NULL!");
                } else {
                    auto empty_msg = std_msgs::msg::Empty();
                    vacuum_detach_pub_->publish(empty_msg);
                    RCLCPP_INFO(get_logger(), "[VACUUM] ✓ DETACH pubblicato per tag ID %d", current_target_id_);
                }
                
                if (tags_placed_ < 2) {
                    state_ = STATE_RETURN_TO_BIN;
                } else {
                    state_ = STATE_FINISHED;
                }
                last_state_change_time_ = now;
                current_target_id_ = -1;
                RCLCPP_INFO(get_logger(), "[TRANSITION] STATE_RELEASE_OBJECT -> %s", 
                           (tags_placed_ < 2) ? "STATE_RETURN_TO_BIN" : "STATE_FINISHED");
                RCLCPP_INFO(get_logger(), "[SLEEP] Pausa di 1 secondo per stabilizzare la simulazione...");
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                break;
            }

            case STATE_RETURN_TO_BIN: {
                // ===== RESET AL PRIMO INGRESSO NELLO STATO =====
                if (!return_movement_started_) {
                    RCLCPP_INFO(get_logger(), "[STATE] RETURN_TO_BIN - Torna al cassone");
                    return_movement_started_ = true;
                }
                
                // ===== FASE 1: MOVIMENTO (primi 5 secondi) =====
                if (elapsed_ms < 8000) {
                    send_joint_command(get_bin_hover_position());
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, 
                                        "[RETURN_TO_BIN] MOVIMENTO in corso (%.1f sec / 8 sec)", elapsed_ms / 1000.0);
                } else {
                    // ===== FASE 2: HOLD (restanti secondi - robot FERMO) =====
                    // Log solo una volta all'inizio della hold phase (quando elapsed_ms è appena >= 5000)
                    static bool hold_phase_logged = false;
                    if (!hold_phase_logged) {
                        auto target_bin = get_bin_hover_position();
                        RCLCPP_INFO(get_logger(),
                            "[RETURN_TO_BIN_HOLD] TARGET che doveva raggiungere:");
                        RCLCPP_INFO(get_logger(),
                            "  J0=%.4f  J1=%.4f  J2=%.4f  J3=%.4f  J4=%.4f  J5=%.4f  J6=%.4f",
                            target_bin[0], target_bin[1], target_bin[2], target_bin[3],
                            target_bin[4], target_bin[5], target_bin[6]);
                        RCLCPP_INFO(get_logger(),
                            "[RETURN_TO_BIN_HOLD] INIZIO HOLD POSITION - Posizione ATTUALE dei giunti:");
                        RCLCPP_INFO(get_logger(),
                            "  J0=%.4f  J1=%.4f  J2=%.4f  J3=%.4f  J4=%.4f  J5=%.4f  J6=%.4f",
                            current_joint_positions_[0], current_joint_positions_[1], current_joint_positions_[2],
                            current_joint_positions_[3], current_joint_positions_[4], current_joint_positions_[5],
                            current_joint_positions_[6]);
                        hold_phase_logged = true;
                    }
                    
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                        "[RETURN_TO_BIN] 🟢 HOLD POSITION - ROBOT FERMO (%.1f sec)", (elapsed_ms - 5000) / 1000.0);
                    // ← NESSUN send_joint_command() qui!
                }
                
                if (elapsed_ms > 5000) {
                    return_movement_started_ = false;
                    state_ = STATE_SEARCH_TAGS;
                    last_state_change_time_ = now;
                    RCLCPP_INFO(get_logger(), "[TRANSITION] STATE_RETURN_TO_BIN -> STATE_SEARCH_TAGS (TRANSIZIONE VELOCE)");
                    RCLCPP_INFO(get_logger(), "[SLEEP] Pausa di 1 secondo per stabilizzare la simulazione...");
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                }
                break;
            }

            case STATE_FINISHED: {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, 
                                    "=== MISSIONE COMPLETATA ===");
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, 
                                    "Entrambi i tag sono stati piazzati sulle loro shelf!");
                break;
            }
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PickAndPlaceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
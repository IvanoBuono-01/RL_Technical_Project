#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/empty.hpp"

using namespace std::chrono_literals;

struct ArucoTag {
    int id;
    double x, y, z;
    bool picked = false;
    double distance_to_camera = 1e6;
};

class PickAndPlaceNode : public rclcpp::Node {
public:
    PickAndPlaceNode() : Node("pick_package_node") {
        RCLCPP_INFO(get_logger(), "=== Nodo Pick & Place Inizializzato ===");
        
        // Subscriber ai joint state
        joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/iiwa/joint_states", 10, 
            std::bind(&PickAndPlaceNode::joint_callback, this, std::placeholders::_1));
        
        // Subscriber ai tag ArUco
        aruco_0_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_marker_0/pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { 
                this->aruco_callback(msg, 0); 
            });
        
        aruco_999_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_marker_999/pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { 
                this->aruco_callback(msg, 999); 
            });

        // Publisher per comandi ai giunti
        cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/iiwa/iiwa_arm_controller/commands", 10);

        // Publisher per vacuum gripper (attach/detach)
        vacuum_attach_pub_ = create_publisher<std_msgs::msg::Empty>(
            "/iiwa/gripper/attach", 10);
        vacuum_detach_pub_ = create_publisher<std_msgs::msg::Empty>(
            "/iiwa/gripper/detach", 10);

        // Timer per control loop
        timer_ = create_wall_timer(50ms, 
            std::bind(&PickAndPlaceNode::control_loop, this));

        // Variabili di stato
        state_ = STATE_MOVE_TO_BIN;
        tags_placed_ = 0;
        joint_state_ready_ = false;
        item_grasped_ = false;
        current_target_id_ = -1;
        last_state_change_time_ = std::chrono::steady_clock::now();

        RCLCPP_INFO(get_logger(), "Sottoscritti topic ArUco: /aruco_marker_0/pose e /aruco_marker_999/pose");
        RCLCPP_INFO(get_logger(), "Inizio missione di pick & place...");
    }

private:
    enum State { 
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

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_0_sub_, aruco_999_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr vacuum_attach_pub_, vacuum_detach_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->position.size() < 7) return;
        joint_state_ready_ = true;
        current_joint_positions_ = msg->position;
    }

    void aruco_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int tag_id) {
        // Aggiorna o crea il tag
        bool found = false;
        for (auto &tag : detected_tags_) {
            if (tag.id == tag_id) {
                tag.x = msg->pose.position.x;
                tag.y = msg->pose.position.y;
                tag.z = msg->pose.position.z;
                // Calcola distanza dalla camera (assumendo camera in origine frame)
                tag.distance_to_camera = std::sqrt(msg->pose.position.x * msg->pose.position.x + 
                                                   msg->pose.position.y * msg->pose.position.y + 
                                                   msg->pose.position.z * msg->pose.position.z);
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
            new_tag.distance_to_camera = std::sqrt(msg->pose.position.x * msg->pose.position.x + 
                                                   msg->pose.position.y * msg->pose.position.y + 
                                                   msg->pose.position.z * msg->pose.position.z);
            detected_tags_.push_back(new_tag);
            RCLCPP_INFO(get_logger(), "Rilevato nuovo tag ArUco: ID=%d, Distanza camera=%.3f", 
                       tag_id, new_tag.distance_to_camera);
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
        std_msgs::msg::Float64MultiArray cmd;
        cmd.data = joint_angles;
        cmd_pub_->publish(cmd);
    }

    void vacuum_attach() {
        auto msg = std_msgs::msg::Empty();
        vacuum_attach_pub_->publish(msg);
        item_grasped_ = true;
        RCLCPP_INFO(get_logger(), "==> VACUUM ATTACH per tag ID %d", current_target_id_);
    }

    void vacuum_detach() {
        auto msg = std_msgs::msg::Empty();
        vacuum_detach_pub_->publish(msg);
        item_grasped_ = false;
        RCLCPP_INFO(get_logger(), "==> VACUUM DETACH tag ID %d", current_target_id_);
    }

    std::vector<double> get_home_position() {
        // Posizione di "home" (predefinita per IIWA)
        return {0.0, 0.0, 0.0, -1.57, 0.0, 1.0, 0.0};
    }

    std::vector<double> get_bin_hover_position() {
        // Posizione sopra il cassone (modificare in base alla geometria)
        // Questi sono angles per IIWA, non posizione cartesiana
        return {0.0, 0.2, 0.0, -1.2, 0.0, 0.8, 0.0};
    }

    std::vector<double> get_low_shelf_position() {
        // Posizione per shelf bassa (tag 0)
        return {1.57, -0.7, 1.57, -1.5, -1.57, 0.5, 0.0};
    }

    std::vector<double> get_high_shelf_position() {
        // Posizione per shelf alta (tag 999)
        return {1.57, -0.3, 1.57, -1.2, -1.57, 0.5, 0.0};
    }

    void control_loop() {
        if (!joint_state_ready_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, 
                                "In attesa di joint states...");
            return;
        }

        auto now = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_state_change_time_).count();

        RCLCPP_DEBUG(get_logger(), "State: %d, Tags picked: %d/%d, Current target: %d, Elapsed: %ld ms",
                    state_, tags_placed_, 2, current_target_id_, elapsed_ms);

        switch (state_) {
            case STATE_MOVE_TO_BIN:
                RCLCPP_INFO(get_logger(), "[STATE] MOVE_TO_BIN - Sposta gripper sopra cassone");
                send_joint_command(get_bin_hover_position());
                if (elapsed_ms > 5000) {  // Attendi 5 secondi per il movimento
                    state_ = STATE_SEARCH_TAGS;
                    last_state_change_time_ = now;
                    RCLCPP_INFO(get_logger(), "[TRANSITION] STATE_MOVE_TO_BIN -> STATE_SEARCH_TAGS");
                }
                break;

            case STATE_SEARCH_TAGS: {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, 
                                    "[STATE] SEARCH_TAGS - Ricerca tag ArUco");
                
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
                if (target) {
                    // Qui potresti inviare comando per muoverti verso target->x, target->y, target->z
                    // Per ora usiamo una posizione approssimativa
                    send_joint_command(get_bin_hover_position());
                    
                    if (elapsed_ms > 4000) {  // Attendi 4 secondi
                        state_ = STATE_GRASP_OBJECT;
                        last_state_change_time_ = now;
                        RCLCPP_INFO(get_logger(), "[TRANSITION] STATE_MOVE_TO_TAG -> STATE_GRASP_OBJECT");
                    }
                } else {
                    state_ = STATE_SEARCH_TAGS;
                    current_target_id_ = -1;
                    last_state_change_time_ = now;
                }
                break;
            }

            case STATE_GRASP_OBJECT: {
                RCLCPP_INFO(get_logger(), "[STATE] GRASP_OBJECT - Attacca oggetto (vacuum)");
                vacuum_attach();
                
                state_ = STATE_MOVE_TO_SHELF;
                last_state_change_time_ = now;
                RCLCPP_INFO(get_logger(), "[TRANSITION] STATE_GRASP_OBJECT -> STATE_MOVE_TO_SHELF");
                break;
            }

            case STATE_MOVE_TO_SHELF: {
                ArucoTag* target = get_tag_by_id(current_target_id_);
                if (!target) {
                    state_ = STATE_SEARCH_TAGS;
                    break;
                }

                if (target->id == 0) {
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, 
                                        "[STATE] MOVE_TO_SHELF - Verso shelf BASSA (tag 0)");
                    send_joint_command(get_low_shelf_position());
                } else {
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, 
                                        "[STATE] MOVE_TO_SHELF - Verso shelf ALTA (tag 999)");
                    send_joint_command(get_high_shelf_position());
                }

                if (elapsed_ms > 5000) {  // Attendi 5 secondi per il movimento
                    state_ = STATE_RELEASE_OBJECT;
                    last_state_change_time_ = now;
                    RCLCPP_INFO(get_logger(), "[TRANSITION] STATE_MOVE_TO_SHELF -> STATE_RELEASE_OBJECT");
                }
                break;
            }

            case STATE_RELEASE_OBJECT: {
                RCLCPP_INFO(get_logger(), "[STATE] RELEASE_OBJECT - Rilascia oggetto (vacuum detach)");
                
                ArucoTag* target = get_tag_by_id(current_target_id_);
                if (target) {
                    target->picked = true;
                    tags_placed_++;
                    RCLCPP_INFO(get_logger(), "Tag ID %d piazzato! Totale: %d/2", 
                               target->id, tags_placed_);
                }
                
                vacuum_detach();
                
                if (tags_placed_ < 2) {
                    state_ = STATE_RETURN_TO_BIN;
                } else {
                    state_ = STATE_FINISHED;
                }
                last_state_change_time_ = now;
                current_target_id_ = -1;
                RCLCPP_INFO(get_logger(), "[TRANSITION] STATE_RELEASE_OBJECT -> %s", 
                           (tags_placed_ < 2) ? "STATE_RETURN_TO_BIN" : "STATE_FINISHED");
                break;
            }

            case STATE_RETURN_TO_BIN: {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, 
                                    "[STATE] RETURN_TO_BIN - Torna al cassone");
                send_joint_command(get_bin_hover_position());
                
                if (elapsed_ms > 5000) {  // Attendi 5 secondi
                    state_ = STATE_SEARCH_TAGS;
                    last_state_change_time_ = now;
                    RCLCPP_INFO(get_logger(), "[TRANSITION] STATE_RETURN_TO_BIN -> STATE_SEARCH_TAGS");
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
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
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;  // Quaternione orientamento
    bool picked = false;
    bool frozen = false;  // ===== FREEZING: Flag per congelare la posa =====
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

        // Timer per control loop
        timer_ = create_wall_timer(50ms, 
            std::bind(&PickAndPlaceNode::control_loop, this));

        // Variabili di stato
        state_ = STATE_MOVE_TO_BIN;  // Ripristinato: NORMAL MODE
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
    
    // ===== VARIABILI PER CONTROLLO FERMO ROBOT =====
    bool movement_phase_active_ = false;  // true durante movimento, false durante hold
    double saved_tag_x_ = 0.0, saved_tag_y_ = 0.0, saved_tag_z_ = 0.0;
    bool tag_pose_saved_ = false;
    
    // ===== VARIABILI STATICHE PER OGNI STATO (COME MEMBRO DI CLASSE) =====
    bool bin_movement_started_ = false;
    bool shelf_movement_started_ = false;
    bool return_movement_started_ = false;
    bool pose_saved_for_current_target_ = false;
    double saved_x_for_tag_ = 0.0, saved_y_for_tag_ = 0.0, saved_z_for_tag_ = 0.0;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_0_sub_, aruco_999_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr vacuum_attach_pub_, vacuum_detach_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->position.size() < 7) return;
        
        // Salva le posizioni dei giunti in ordine crescente (0-6)
        current_joint_positions_.clear();
        current_joint_positions_.resize(7);
        
        for (size_t i = 0; i < 7 && i < msg->position.size(); ++i) {
            current_joint_positions_[i] = msg->position[i];
            RCLCPP_DEBUG(get_logger(), "[JOINT_CB] Giunto %zu: posizione = %.4f rad", i, msg->position[i]);
        }
        
        joint_state_ready_ = true;
        RCLCPP_DEBUG(get_logger(), "[JOINT_CB] Posizioni giunti aggiornate");
    }
         

    void aruco_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int tag_id) {
        RCLCPP_DEBUG(get_logger(), "[ARUCO_CB] Ricevuto messaggio per tag ID=%d", tag_id);
        
        // Aggiorna o crea il tag
        bool found = false;
        for (auto &tag : detected_tags_) {
            if (tag.id == tag_id) {
                // ===== FREEZING: Se già rilevato, non aggiornare più =====
                if (tag.frozen) {
                    RCLCPP_DEBUG(get_logger(), "[ARUCO_CB] Tag ID=%d già freezato, ignoro aggiornamento", tag_id);
                    return;  // Non aggiornare, la posa è freezata
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
                
                // ===== FREEZING: Marca come freezato =====
                tag.frozen = true;
                RCLCPP_INFO(get_logger(), "[ARUCO_CB] Tag ID=%d rilevato e FREEZATO! Pose: x=%.4f y=%.4f z=%.4f, Distanza=%.3f", 
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
            RCLCPP_INFO(get_logger(), "[ARUCO_CB] Rilevato nuovo tag ArUco: ID=%d, Pose: x=%.4f y=%.4f z=%.4f, Distanza camera=%.3f, FREEZATO!", 
                       tag_id, new_tag.x, new_tag.y, new_tag.z, new_tag.distance_to_camera);
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
        
        // ===== LOG DETTAGLIATO DEI COMANDI MANDATI =====
        RCLCPP_INFO(get_logger(), 
            "[SEND_CMD] COMANDO MANDATO: J0=%.4f J1=%.4f J2=%.4f J3=%.4f J4=%.4f J5=%.4f J6=%.4f",
            joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3],
            joint_angles[4], joint_angles[5], joint_angles[6]);
        
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
        return {0.0, 0.1, 0.0, -0.7, 0.0, 0.5, 0.0};
    }

    std::vector<double> get_low_shelf_position() {
        // Posizione per shelf bassa (tag 0)
        return {1.57, -0.7, 1.57, -1.5, -1.57, 0.5, 0.0};
    }

    std::vector<double> get_high_shelf_position() {
        // Posizione per shelf alta (tag 999)
        return {1.57, -0.3, 1.57, -1.2, -1.57, 0.5, 0.0};
    }

    std::vector<double> get_debug_position() {
        // ============ COORDINATE DI DEBUG FISSE PER TEST ============
        // Puoi modificare questi valori per testare il movimento
        // MODIFICA QUESTI VALORI PER TESTARE DIVERSI MOVIMENTI:
        return {0.0, 0.1, 0.0, -0.7, 0.0, 0.5, 0.0};
        // ===== ESEMPI DI COORDINATE CARTESIANE (JOINT ANGLES) =====
        // HOME:       {0.0, 0.0, 0.0, -1.57, 0.0, 1.0, 0.0}
        // BIN HOVER:  {0.0, 0.8, 0.0, -1.2, 0.0, 0.8, 0.0}
        // LOW SHELF:  {1.57, -0.7, 1.57, -1.5, -1.57, 0.5, 0.0}
        // HIGH SHELF: {1.57, -0.3, 1.57, -1.2, -1.57, 0.5, 0.0}
    }

    std::vector<double> cartesian_to_joint_angles(double x, double y, double z) {
        // ===== CONVERSIONE COORDINATE CARTESIANE -> JOINT ANGLES =====
        // Questa è una conversione SEMPLIFICATA per il pick & place
        // In un sistema reale useremmo IK (Inverse Kinematics) con KDLPlanner
        // Per ora: mappiamo le coordinate cartesiane in angoli approssimativi
        
        RCLCPP_INFO(get_logger(), 
            "[CARTESIAN_TO_IK] Convertendo coordinate cartesiane: x=%.4f y=%.4f z=%.4f", x, y, z);
        
        std::vector<double> joint_angles(7);
        
        // ===== STRATEGY SEMPLIFICATA =====
        // Usa la posizione bin_hover come base e modifica solo J1 e J5
        // in funzione di x e y per muoversi verso il tag
        
        joint_angles[0] = 0.0;      // J0: fisso
        joint_angles[1] = 0.8 + (y * 0.5);  // J1: aumenta con y (movimento base)
        joint_angles[2] = 0.0;      // J2: fisso
        joint_angles[3] = -1.2;     // J3: fisso (posizione braccio)
        joint_angles[4] = x * 0.5;  // J4: varia con x
        joint_angles[5] = 0.5 + (z * 0.2);  // J5: aumenta con z (altezza)
        joint_angles[6] = 0.0;      // J6: fisso
        
        RCLCPP_INFO(get_logger(),
            "[CARTESIAN_TO_IK] Joint angles calcolati: J0=%.4f J1=%.4f J2=%.4f J3=%.4f J4=%.4f J5=%.4f J6=%.4f",
            joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3],
            joint_angles[4], joint_angles[5], joint_angles[6]);
        
        return joint_angles;
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
            case STATE_DEBUG_MOVE: {
                // ===== DEBUG MODE: MOVIMENTO CONTINUO VERSO COORDINATE FISSE =====
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, 
                    "[DEBUG] ===== STATO DEBUG: MOVIMENTO CONTINUO VERSO COORDINATE FISSE =====");
                
                // Manda il comando ad ogni iterazione (come prima)
                send_joint_command(get_debug_position());
                
                // Stampa le posizioni CORRENTI vs TARGET per debuggare il movimento
                auto target = get_debug_position();
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                    "[DEBUG] TARGET:    J0=%.4f J1=%.4f J2=%.4f J3=%.4f J4=%.4f J5=%.4f J6=%.4f",
                    target[0], target[1], target[2], target[3], target[4], target[5], target[6]);
                    
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                    "[DEBUG] CORRENTI:  J0=%.4f J1=%.4f J2=%.4f J3=%.4f J4=%.4f J5=%.4f J6=%.4f",
                    current_joint_positions_[0], current_joint_positions_[1], current_joint_positions_[2],
                    current_joint_positions_[3], current_joint_positions_[4], current_joint_positions_[5],
                    current_joint_positions_[6]);
                
                // RIMANE IN QUESTO STATO INDEFINITAMENTE - Il braccio si muove continuamente verso il target
                // Per uscire dal debug e passare a MOVE_TO_BIN, decommentare:
                // if (elapsed_ms > 60000) {
                //     state_ = STATE_MOVE_TO_BIN;
                //     last_state_change_time_ = now;
                //     RCLCPP_INFO(get_logger(), "[DEBUG] ===== TIMEOUT DEBUG - Passando a MOVE_TO_BIN =====");
                // }
                break;
            }

            case STATE_MOVE_TO_BIN: {
                // ===== RESET AL PRIMO INGRESSO NELLO STATO =====
                if (!bin_movement_started_) {
                    RCLCPP_INFO(get_logger(), "[STATE] MOVE_TO_BIN - Sposta gripper sopra cassone");
                    bin_movement_started_ = true;
                }
                
                // ===== FASE 1: MOVIMENTO (primi 3 secondi) =====
                if (elapsed_ms < 3000) {
                    send_joint_command(get_bin_hover_position());
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, 
                        "[MOVE_TO_BIN] 🔴 MOVIMENTO IN CORSO (%.1f sec / 3 sec)", elapsed_ms / 1000.0);
                } else {
                    // ===== FASE 2: HOLD (restanti 9 secondi - robot FERMO) =====
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                        "[MOVE_TO_BIN] 🟢 HOLD POSITION - ROBOT FERMO (%.1f sec / 9 sec)", (elapsed_ms - 3000) / 1000.0);
                    // ← NESSUN send_joint_command() qui!
                }
                
                if (elapsed_ms > 12000) {
                    bin_movement_started_ = false;  // Reset per il prossimo volta
                    state_ = STATE_SEARCH_TAGS;
                    last_state_change_time_ = now;
                    RCLCPP_INFO(get_logger(), "[TRANSITION] STATE_MOVE_TO_BIN -> STATE_SEARCH_TAGS (OSSERVAZIONE COMPLETATA)");
                }
                break;
            }

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
                if (target && target->frozen) {
                    // ===== SALVA LA POSA DEL TAG AL PRIMO INGRESSO =====
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
                    if (elapsed_ms < 3000) {
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
                    RCLCPP_WARN(get_logger(), "[MOVE_TO_TAG] ❌ Tag ID %d non trovato o non freezato! Torno a SEARCH_TAGS", current_target_id_);
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
                // ===== RESET AL PRIMO INGRESSO NELLO STATO =====
                if (!shelf_movement_started_) {
                    RCLCPP_INFO(get_logger(), "[STATE] MOVE_TO_SHELF");
                    shelf_movement_started_ = true;
                }
                
                ArucoTag* target = get_tag_by_id(current_target_id_);
                if (!target) {
                    state_ = STATE_SEARCH_TAGS;
                    break;
                }

                // ===== FASE 1: MOVIMENTO (primi 3 secondi) =====
                if (elapsed_ms < 3000) {
                    if (target->id == 0) {
                        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, 
                                            "[MOVE_TO_SHELF] 🔴 MOVIMENTO verso shelf BASSA (%.1f sec / 3 sec)", elapsed_ms / 1000.0);
                        send_joint_command(get_low_shelf_position());
                    } else {
                        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, 
                                            "[MOVE_TO_SHELF] 🔴 MOVIMENTO verso shelf ALTA (%.1f sec / 3 sec)", elapsed_ms / 1000.0);
                        send_joint_command(get_high_shelf_position());
                    }
                } else {
                    // ===== FASE 2: HOLD (restanti 6 secondi - robot FERMO) =====
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                        "[MOVE_TO_SHELF] 🟢 HOLD POSITION - ROBOT FERMO (%.1f sec / 6 sec)", (elapsed_ms - 3000) / 1000.0);
                    // ← NESSUN send_joint_command() qui!
                }

                if (elapsed_ms > 9000) {  // ===== 9 SECONDI PER OSSERVARE =====
                    shelf_movement_started_ = false;  // Reset per il prossimo volta
                    state_ = STATE_RELEASE_OBJECT;
                    last_state_change_time_ = now;
                    RCLCPP_INFO(get_logger(), "[TRANSITION] STATE_MOVE_TO_SHELF -> STATE_RELEASE_OBJECT (OSSERVAZIONE COMPLETATA)");
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
                // ===== RESET AL PRIMO INGRESSO NELLO STATO =====
                if (!return_movement_started_) {
                    RCLCPP_INFO(get_logger(), "[STATE] RETURN_TO_BIN - Torna al cassone");
                    return_movement_started_ = true;
                }
                
                // ===== FASE 1: MOVIMENTO (primi 3 secondi) =====
                if (elapsed_ms < 3000) {
                    send_joint_command(get_bin_hover_position());
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, 
                                        "[RETURN_TO_BIN] 🔴 MOVIMENTO in corso (%.1f sec / 3 sec)", elapsed_ms / 1000.0);
                } else {
                    // ===== FASE 2: HOLD (restanti 6 secondi - robot FERMO) =====
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                        "[RETURN_TO_BIN] 🟢 HOLD POSITION - ROBOT FERMO (%.1f sec / 6 sec)", (elapsed_ms - 3000) / 1000.0);
                    // ← NESSUN send_joint_command() qui!
                }
                
                if (elapsed_ms > 9000) {  // ===== 9 SECONDI PER OSSERVARE =====
                    return_movement_started_ = false;  // Reset per il prossimo volta
                    state_ = STATE_SEARCH_TAGS;
                    last_state_change_time_ = now;
                    RCLCPP_INFO(get_logger(), "[TRANSITION] STATE_RETURN_TO_BIN -> STATE_SEARCH_TAGS (OSSERVAZIONE COMPLETATA)");
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
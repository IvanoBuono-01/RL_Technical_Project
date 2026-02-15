#ifndef KDLControl
#define KDLControl

#include "Eigen/Dense"
#include "kdl_robot.h"
#include "utils.h"
#include "geometry_msgs/msg/pose_stamped.hpp" 

class KDLController
{

public:

    KDLController(KDLRobot &_robot);

    Eigen::VectorXd idCntr(KDL::JntArray &_qd,
                           KDL::JntArray &_dqd,
                           KDL::JntArray &_ddqd,
                           double _Kp,
                           double _Kd);

    Eigen::VectorXd idCntr(KDL::Frame &_desPos,
                           KDL::Twist &_desVel,
                           KDL::Twist &_desAcc,
                           double _Kpp,
                           double _Kpo,
                           double _Kdp,
                           double _Kdo);



// funzione per il nullo
    Eigen::VectorXd velocity_ctrl_null(const Eigen::Vector3d &_p_des,
                                       const Eigen::Vector3d &_p_cur,
                                       double _Kp
                                       );
// funzione per il vision
    Eigen::VectorXd vision_control (geometry_msgs::msg::PoseStamped::ConstSharedPtr aruco_msg,
                                    KDL::Chain chain_,
                                    Eigen::MatrixXd K);
private:

    Eigen::VectorXd compute_q0_dot(KDL::Frame &_desPos);
                                                    
    Eigen::MatrixXd compute_J_cam(KDL::Chain chain_);
    
    KDLRobot* robot_;

};

#endif

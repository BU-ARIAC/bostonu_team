#include <map>
#include <vector>
#include <ros/ros.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <nist_gear/VacuumGripperControl.h>
#include <nist_gear/VacuumGripperState.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ExecuteTrajectoryActionResult.h>
#include <moveit_msgs/MoveGroupActionResult.h>
#include <sensor_msgs/JointState.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#pragma once

static const std::string PLANNING_GROUP_KITTING = "kitting_arm";
static const std::string PLANNING_DESC_KITTING = "/ariac/kitting/robot_description";
static const std::string PLANNING_SCENE_NS_KITTING = "/ariac/kitting";

static const std::string PLANNING_GROUP_GANTRY = "gantry_arm";
static const std::string PLANNING_DESC_GANTRY = "/ariac/gantry/robot_description";
static const std::string PLANNING_SCENE_NS_GANTRY = "/ariac/gantry";

static const std::string PLANNING_GROUP_GANTRY_TORSO = "gantry_torso";

static const std::string PLANNING_GROUP_GANTRY_FULL = "gantry_full";

// Locations for gantry torso
// --------------------------
static const int LOC_BINS = 1;
static const int LOC_AS = 2;
static const int LOC_NEUTRAL_BIN = 3;
static const int LOC_NEUTRAL_AS = 4;

static const int LOC_AS1_AGV1 = 11;
static const int LOC_AS1_AGV2 = 12;
static const int LOC_AS1_PCASE = 13;
static const int LOC_AS1_CASE = 14;

static const int LOC_AS2_AGV1 = 21;
static const int LOC_AS2_AGV2 = 22;
static const int LOC_AS2_PCASE = 23;
static const int LOC_AS2_CASE = 24;

static const int LOC_AS3_AGV3 = 33; 
static const int LOC_AS3_AGV4 = 34; 
static const int LOC_AS3_PCASE = 35;
static const int LOC_AS3_CASE = 36; 
static const int GANTRY_ARM_CASE = 3;

static const int LOC_AS4_AGV3 = 43;
static const int LOC_AS4_AGV4 = 44;
static const int LOC_AS4_PCASE = 45;
static const int LOC_AS4_CASE = 46;
// --------------------------


// KITTING Joint Group Positions (jgp) 
// -----------------------------------
static const std::vector<double> jgp_kit_neutral = { -0.01, 3.14, -1.38, 1.88, -2.08, -1.58, 0.00 };
static const std::vector<double> jgp_kit_agv1 = { 4.80, 3.14, -1.38, 1.88, -2.08, -1.58, 0.00 };
static const std::vector<double> jgp_kit_agv2 = { 1.50, 3.14, -1.38, 1.88, -2.08, -1.58, 0.00 };
static const std::vector<double> jgp_kit_agv3 = { -1.20, 3.14, -1.38, 1.88, -2.08, -1.58, 0.00 };
static const std::vector<double> jgp_kit_agv4 = { -4.60, 3.14, -1.38, 1.88, -2.08, -1.58, 0.00 };
static const std::vector<double> jgp_kit_bins14 = { 3.10, 3.14, -1.38, 1.88, -2.08, -1.58, 0.00 };
static const std::vector<double> jgp_kit_bins58 = { -2.80, 3.14, -1.38, 1.88, -2.08, -1.58, 0.00 };
static const std::vector<double> jgp_kit_badparts = { 0.10, 3.14, -1.38, 1.88, -2.08, -1.58, 0.00 };
static const std::vector<double> jgp_kit_conveyor = { -0.01, 0.04, -1.08, 2.05, -2.5476, -1.58, 0.00 };  // wrist1 was 3.74
static const std::vector<double> jgp_kit_handoff = { 0.10, 3.14, -1.38, 1.25, 3.26, -1.57, 3.14 };  // wrist_3 rotates part by 180 degrees so orientation is correct when grabbed by gantry
// Order above: linear_arm_actuator_joint, shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint
// -----------------------------------


// GANTRY Joint Group Positions (jgp) - "gt" = gantry torso; "ga" = gantry arm
// ---------------------------------------------------------------------------
// Torso order: small_long_joint (x-axis), torso_rail_joint (y-axis), torso_base_main_joint (yaw)
// Arm order: shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint
static const std::vector<double> jgp_gt_as1_agv1 = { -3.40, -3.60, 0.0 };
static const std::vector<double> jgp_gt_as1_agv2 = { -3.40, -2.35, M_PI }; //same x as 11, same y as 22
static const std::vector<double> jgp_gt_as1_pcase = { -2.80, -2.975, M_PI/2 };
static const std::vector<double> jgp_gt_as1_case = { -3.50, -2.975, M_PI/2 };
static const std::vector<double> jgp_gt_as2_agv1 = { -8.40, -3.6, 0.0 };
static const std::vector<double> jgp_gt_as2_agv2 = { -8.40, -2.35, M_PI };
static const std::vector<double> jgp_gt_as2_pcase = { -7.80, -2.975, M_PI/2 };
static const std::vector<double> jgp_gt_as2_case = { -8.50, -2.975, M_PI/2 };

static const std::vector<double> jgp_gt_as3_agv3 = { -3.40, 2.40, 0.0 };
static const std::vector<double> jgp_gt_as3_agv4 = { -3.40, 3.65, M_PI };
static const std::vector<double> jgp_gt_as3_pcase = { -2.80, 3.00, M_PI/2 };
static const std::vector<double> jgp_gt_as3_case = { -3.50, 3.00, M_PI/2 };
static const std::vector<double> jgp_gt_as4_agv3 = { -8.40, 2.40, 0.0 };
static const std::vector<double> jgp_gt_as4_agv4 = { -8.40, 3.65, M_PI };
static const std::vector<double> jgp_gt_as4_pcase = { -7.80, 3.00, M_PI/2 };
static const std::vector<double> jgp_gt_as4_case = { -8.50, 3.00, M_PI/2 };

static const std::vector<double> jgp_ga_case = { 3.21, -1.51, -0.58, -1.0, 1.58, 0.0 }; 
// Should be usable for ALL cases! all joint names below start "gantry_arm_"
// elbow_joint = -0.58, shoulder_lift_joint = -1.51, shoulder_pan_joint = 3.21, wrist_1_joint = -1.00, wrist_2_joint = 1.58, wrist_3_joint = 0.00
static const std::vector<double> jgp_ga_as_agv = { 3.15, -2.14, -1.13, 0.13, 1.58, 0.00 }; 
// OLDOLD: elbow_joint = 1.87617, shoulder_lift_joint = -1.127, shoulder_pan_joint = 0.00, wrist_1_joint = 2.39, wrist_2_joint = -1.58, wrist_3_joint = 0.829883
// New: elbow_joint = -1.13, shoulder_lift_joint = -2.14, shoulder_pan_joint = 3.15, wrist_1_joint = 0.13, wrist_2_joint = 1.58, wrist_3_joint = 0.00

static const std::vector<double> jgp_gt_prehandoff = { -3.20, -0.10, -1.57 };
static const std::vector<double> jgp_gt_handoff = { -2.05, -0.10, -1.57 };
static const std::vector<double> jgp_ga_handoff = { 3.15, -2.14, 0.80, -0.23, 1.57, 0.00 };
static const std::vector<double> jgp_gt_kit_agv1 = { -0.30, -3.60, 0.0 };  
static const std::vector<double> jgp_gt_kit_agv2 = { -0.30, -0.30, 0.0 };  
static const std::vector<double> jgp_gt_kit_agv3 = { -0.10, 0.20, 3.14 };  
static const std::vector<double> jgp_gt_kit_agv4 = { -0.10, 3.56, 3.14 };  
static const std::vector<double> jgp_gt_kit_bin1 = { 0.0, -2.40, 0.0 };
static const std::vector<double> jgp_gt_kit_bin2 = { 0.20, -3.65, 3.14 };
static const std::vector<double> jgp_gt_kit_bin3 = { -0.50, -3.65, 3.14 };  
static const std::vector<double> jgp_gt_kit_bin4 = { -0.70, -2.40, 0.0 };  
static const std::vector<double> jgp_gt_kit_bin5 = { 0.20, 2.70, 3.14 };  
static const std::vector<double> jgp_gt_kit_bin6 = { 0.00, 3.65, 0.0 };  
static const std::vector<double> jgp_gt_kit_bin7 = { -0.70, 3.65, 0.0 };  
static const std::vector<double> jgp_gt_kit_bin8 = { -0.50, 2.70, 3.14 };  
// ---------------------------------------------------------------------------

void setAvgCartesianSpeed(moveit::planning_interface::MoveGroupInterface::Plan &, const std::string &, const double &, const std::string &);

class Robot
{
    public:
        Robot(const std::string &, const bool &);  // Input string should be "kitting" or "gantry"; bool is for gantry: true=arm, false=torso
        std::string robot_name_;   // Name used in ARIAC topics and services
        geometry_msgs::PoseStamped sp_;  // Initial stamped pose of the robot (includes a header w/ 3 fields: frame_id , stamp , and seq)
        geometry_msgs::Pose p_;  // Current pose of the robot (no header)
        int move_group_success;
        int trajectory_success;
        moveit::planning_interface::MoveGroupInterface::Plan my_plan_;
        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        void gripper_callback(const nist_gear::VacuumGripperState::ConstPtr &);
        void trajectory_callback(const moveit_msgs::ExecuteTrajectoryActionResult::ConstPtr &);
        void move_group_callback(const moveit_msgs::MoveGroupActionResult::ConstPtr &);
        void add_collision_objects();
        // int move_near(const geometry_msgs::TransformStamped &);  // Move near a position defined by the msg
        // int move_near(const geometry_msgs::Pose &);  // Move near a position defined by the msg
        // int pickup_part(const geometry_msgs::TransformStamped &);  // Pick up the part at the location defined by the msg
        // int drop_part(const geometry_msgs::TransformStamped &);  // Drop the part at the location defined by the msg
        int enable_gripper();
        int disable_gripper();
        int move_to(const std::vector<double> &);  // Function to move to a specific location by joint group position
        // Robot-specific functions:
        // int move_bin_side();  // Move the kitting arm to the bin side of the linear actuator
        // int move_conveyor_side();  // Move the kitting arm to the conveyro side of the linear actuator
        // int move_torso(const geometry_msgs::Pose &);  // Move the gantry torso to a position defined by the msg


    // private:
        ros::NodeHandle robot_node;
        ros::NodeHandle planning_node;
        ros::Subscriber gripper_subscriber, trajectory_subscriber, move_group_subscriber;
        ros::ServiceClient submit_client;
        nist_gear::VacuumGripperControl gripper_service_;
        nist_gear::VacuumGripperState gripper_status_;
        ros::ServiceClient gripper_client_;
        bool gripper_attached = false;
        moveit::planning_interface::MoveGroupInterface::Options loadOptions;
        moveit::planning_interface::MoveGroupInterface move_group;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        const robot_state::JointModelGroup* joint_model_group;
        moveit::core::RobotStatePtr current_state;

};

class Kitting : public Robot
{
    public:
        Kitting(const std::string & robot_type="kitting", const bool & gantry_arm=false)
            : Robot{ std::move(robot_type), std::move(gantry_arm) } {
                kitting_joint_trajectory_publisher_ = robot_node.advertise<trajectory_msgs::JointTrajectory>(
                    "/ariac/kitting/kitting_arm_controller/command", 10);
                kitting_joint_state_subscriber = robot_node.subscribe("/ariac/kitting/joint_states", 10, &Kitting::kitting_joint_state_callback, this);
            };
        bool check_joints = false;  // Used to determine when to process checks in joint_state_callback
        bool check_joints2 = false;  // Used to determine when to process checks in joint_state_callback
        bool attached_ready = false;
        bool watch_linac = false;
        int testing = 0;
        double linac_destination;
        ros::Publisher kitting_joint_trajectory_publisher_;
        ros::Subscriber kitting_joint_state_subscriber;
        // Robot-specific functions:
        int move_bin_side();  // Move the kitting arm to the bin side of the linear actuator
        int move_conveyor_side();  // Move the kitting arm to the conveyor side of the linear actuator
        int move_near(const geometry_msgs::TransformStamped &);  // Move near a position defined by the msg
        int move_near(const geometry_msgs::Pose &);  // Move near a position defined by the msg
        int pickup_part(const geometry_msgs::TransformStamped &);  // Pick up the part at the location defined by the msg
        int drop_part(const geometry_msgs::TransformStamped &);  // Drop the part at the location defined by the msg
        int ZeroArm();  // Move the arm, by joint values, to a known good pose for picking parts
        void kitting_joint_state_callback(const sensor_msgs::JointState::ConstPtr &);
        void move_joints(const std::vector<std::vector<double>> &, const std::vector<double> &, const std::vector<double> &);
        void clear_arm_goal();
};

class Gantry_Arm : public Robot
{
    public:
        Gantry_Arm(const std::string & robot_type="gantry", const bool & gantry_arm=true)
            : Robot{ std::move(robot_type), std::move(gantry_arm) } {
                gantry_arm_joint_trajectory_publisher_ = robot_node.advertise<trajectory_msgs::JointTrajectory>(
                    "/ariac/gantry/gantry_arm_controller/command", 10);
                gantry_arm_joint_state_subscriber = robot_node.subscribe("/ariac/gantry/joint_states", 10, &Gantry_Arm::gantry_arm_joint_state_callback, this);
            };
        bool check_joints = false;  // Used to determine when to process checks in joint_state_callback
        ros::Publisher gantry_arm_joint_trajectory_publisher_;
        ros::Subscriber gantry_arm_joint_state_subscriber;
        // stuff
        int move_near(const geometry_msgs::TransformStamped &);  // Move near a position defined by the msg
        int move_near(const geometry_msgs::Pose &);  // Move near a position defined by the msg
        int pickup_part(const geometry_msgs::TransformStamped &);  // Pick up the part at the location defined by the msg
        int pickup_part_bins(const geometry_msgs::TransformStamped &);  // Pick up the part at the location defined by the msg from one of the bins
        int drop_part(const geometry_msgs::TransformStamped &);  // Drop the part at the location defined by the msg
        int drop_part_agv(const geometry_msgs::TransformStamped &);  // Drop the part at the location defined by the msg onto an agv
        int ZeroArm();  // Move the arm, by joint values, to a known good pose for picking parts
        int CaseArm();  // Move the arm, by joint values, to a known good pose over the case for assembly
        void gantry_arm_joint_state_callback(const sensor_msgs::JointState::ConstPtr &);
        void clear_arm_goal();
};

class Gantry_Torso : public Robot
{
    public:
        Gantry_Torso(const std::string & robot_type="gantry", const bool & gantry_arm=false)
            : Robot{ std::move(robot_type), std::move(gantry_arm) } { current_location=0; };
        // Robot-specific parameters, functions:
        int current_location;  // Uses the locations statically defined above to keep track of general current location
        int move_neutral();  // Move torso to a neutral nearby location based on current location
        int move_to_bin(const geometry_msgs::TransformStamped &);  // Move torso to a bin to pick a part
        int move_torso(const geometry_msgs::Pose &);  // Move the gantry torso to a position defined by the msg
        int move_torso(const int &);  // Move the gantry torso to a position defined by text (e.g., "as1")
};

int flip_part_handoff(Kitting &, Gantry_Arm &, Gantry_Torso &);

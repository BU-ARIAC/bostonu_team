#include <map>
#include <vector>
#include "cmath"
#include <ros/ros.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/LinearMath/Quaternion.h>
#include <nist_gear/Order.h>
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
#include "robot_mgmt.h"


// Called when a gripper message is received.
void Robot::gripper_callback(const nist_gear::VacuumGripperState::ConstPtr & gripper_msg) {
    gripper_attached = gripper_msg->attached;
}

// Called when a gripper message is received.
void Robot::trajectory_callback(const moveit_msgs::ExecuteTrajectoryActionResult::ConstPtr & trajectory_msg) {
    if (trajectory_msg->status.status == 3) {
        std::cout << "In trajectory_callback, status.text: " << trajectory_msg->status.text << "\n";
        this->trajectory_success++;
    }
}

// Called when a gripper message is received.
void Robot::move_group_callback(const moveit_msgs::MoveGroupActionResult::ConstPtr & move_group_msg) {
    if (move_group_msg->status.status == 3) {
        std::cout << "In move_group_msg, status.text: " << move_group_msg->status.text << "\n";
        this->move_group_success++;
    }
}

Robot::Robot(const std::string & robot_type, const bool & gantry_arm): // robot_type should be "kitting" or "gantry"; bool is for gantry: true=arm, false=torso
        planning_node("/ariac/"+robot_type),
        loadOptions((robot_type=="kitting" ? PLANNING_GROUP_KITTING : (gantry_arm ? PLANNING_GROUP_GANTRY : PLANNING_GROUP_GANTRY_TORSO)), "/ariac/"+robot_type+"/robot_description", planning_node),
        planning_scene_interface("/ariac/"+robot_type),
        move_group(loadOptions)  { 
    
    move_group.setPlanningTime(20);
    move_group.setNumPlanningAttempts(10);
    move_group.setMaxVelocityScalingFactor(0.9);
    move_group.setMaxAccelerationScalingFactor(0.9);
    move_group.allowReplanning(true);

    robot_name_ = robot_type + (gantry_arm ? "_arm" : "");  // sets the name to kitting, gantry, or gantry_arm 
    
    // Subscribe to the necessary topics.
    gripper_subscriber = robot_node.subscribe("/ariac/"+robot_type+"/arm/gripper/state", 10, &Robot::gripper_callback, this);
    trajectory_subscriber = robot_node.subscribe("/ariac/"+robot_type+"/execute_trajectory/result", 10, &Robot::trajectory_callback, this);
    move_group_subscriber = robot_node.subscribe("/ariac/"+robot_type+"/move_group/result", 10, &Robot::move_group_callback, this);

    // Setup the gripper client
    gripper_client_ = robot_node.serviceClient<nist_gear::VacuumGripperControl>("/ariac/"+robot_type+"/arm/gripper/control");

    // Get the initial pose
    joint_model_group = move_group.getCurrentState()->getJointModelGroup((robot_type=="kitting" ? PLANNING_GROUP_KITTING : (gantry_arm ? PLANNING_GROUP_GANTRY : PLANNING_GROUP_GANTRY_TORSO)));
    sp_ = move_group.getCurrentPose();
    p_ = sp_.pose;
    printf("CLASS End effector pose->position (x,y,z): (%f,%f,%f)\n", sp_.pose.position.x, sp_.pose.position.y, sp_.pose.position.z);
    printf("CLASS End effector pose->orientation (x,y,z,w): (%f,%f,%f,%f)\n", sp_.pose.orientation.x, sp_.pose.orientation.y, sp_.pose.orientation.z, sp_.pose.orientation.w);    
    move_group_success = 0;
};

int Robot::move_to(const std::vector<double> & joint_group_positions_) {
    current_state = this->move_group.getCurrentState();
    
    this->move_group.setJointValueTarget(joint_group_positions_);

    bool success = (this->move_group.plan(this->my_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    this->move_group.move();
    
    this->sp_ = this->move_group.getCurrentPose();
    this->p_ = this->sp_.pose;

    return 1;
}

int flip_part_handoff(Kitting & kit_, Gantry_Arm & gantry_arm_, Gantry_Torso & gantry_torso_) {
    // Track part handoff for part flipping
    // 0=kitting arm has part
    // 1=gantry arm attached
    // 2=kitting arm detatched
    int flip_part_status = 0;
    int test = 0;
    int mg_counter = 0;
    std::vector<geometry_msgs::Pose> waypoints;
    printf("in flip_part\n");

    // First move both robots into position
    test = gantry_torso_.move_to(jgp_gt_prehandoff);  // Move the torso into position far enough away to also swing the arm into position
    test = gantry_arm_.move_to(jgp_ga_handoff);
    test = kit_.move_to(jgp_kit_handoff);
    test = gantry_torso_.move_to(jgp_gt_handoff);  // Now that evereyone else is in position, move the gantry closer

    // Next, move the gantry in until it's gripper is attached
    gantry_arm_.enable_gripper();
    gantry_torso_.sp_ = gantry_torso_.move_group.getCurrentPose();
    gantry_torso_.p_ = gantry_torso_.sp_.pose;
    geometry_msgs::Pose target_pose = gantry_torso_.p_;
    int move_counter =  0;
    while (!gantry_arm_.gripper_attached && move_counter < 15) {
        target_pose.position.x += 0.005;  // Move closer to part in kitting gripper
        waypoints.clear();
        waypoints.push_back(target_pose);
        double fraction = gantry_torso_.move_group.computeCartesianPath(waypoints, gantry_torso_.eef_step, gantry_torso_.jump_threshold, gantry_torso_.trajectory);
        gantry_torso_.my_plan_.trajectory_ = gantry_torso_.trajectory;
        gantry_torso_.move_group.execute(gantry_torso_.my_plan_);
        printf("done moving nearer part...\n");
        // ros::Duration(0.25).sleep();
        move_counter++;
    }
    printf("hopefully part attached now\n");
    gantry_torso_.trajectory_success = 0;
    mg_counter = 0; 

    // Then, detacth the kitting gripper
    kit_.disable_gripper();

    // Finally, move the part (on the gantry arm) to a position to be placed on an AGV
    test = gantry_torso_.move_to(jgp_gt_prehandoff);
    test = gantry_arm_.ZeroArm();
    test = kit_.move_bin_side();

    return 1;
}

void Robot::add_collision_objects() {
    
    // Create the initial collision objects
    moveit_msgs::CollisionObject collision_object_conveyor;
    collision_object_conveyor.header.frame_id = move_group.getPlanningFrame();
    collision_object_conveyor.id = "conveyor";  // The id of the object is used to identify it
    moveit_msgs::CollisionObject collision_object_kitting_rail;
    collision_object_kitting_rail.header.frame_id = move_group.getPlanningFrame();
    collision_object_kitting_rail.id = "kitting_rail";
    moveit_msgs::CollisionObject collision_object_bins_1to4;
    collision_object_bins_1to4.header.frame_id = move_group.getPlanningFrame();
    collision_object_bins_1to4.id = "bins_1to4";
    moveit_msgs::CollisionObject collision_object_bins_5to8;
    collision_object_bins_5to8.header.frame_id = move_group.getPlanningFrame();
    collision_object_bins_5to8.id = "bins_5to8";
    moveit_msgs::CollisionObject collision_object_faulty_bin;
    collision_object_faulty_bin.header.frame_id = move_group.getPlanningFrame();
    collision_object_faulty_bin.id = "faulty_bin";

    moveit_msgs::CollisionObject collision_object_agv1_ks1;
    collision_object_agv1_ks1.header.frame_id = move_group.getPlanningFrame();
    collision_object_agv1_ks1.id = "agv1_ks1";
    moveit_msgs::CollisionObject collision_object_agv1_ks1_mast;
    collision_object_agv1_ks1_mast.header.frame_id = move_group.getPlanningFrame();
    collision_object_agv1_ks1_mast.id = "agv1_ks1_mast";
    moveit_msgs::CollisionObject collision_object_agv1_as1_mast;
    collision_object_agv1_as1_mast.header.frame_id = move_group.getPlanningFrame();
    collision_object_agv1_as1_mast.id = "agv1_as1_mast";
    moveit_msgs::CollisionObject collision_object_agv1_as2_mast;
    collision_object_agv1_as2_mast.header.frame_id = move_group.getPlanningFrame();
    collision_object_agv1_as2_mast.id = "agv1_as2_mast";

    moveit_msgs::CollisionObject collision_object_agv2_ks2;
    collision_object_agv2_ks2.header.frame_id = move_group.getPlanningFrame();
    collision_object_agv2_ks2.id = "agv2_ks2";
    moveit_msgs::CollisionObject collision_object_agv2_ks2_mast;
    collision_object_agv2_ks2_mast.header.frame_id = move_group.getPlanningFrame();
    collision_object_agv2_ks2_mast.id = "agv2_ks2_mast";
    moveit_msgs::CollisionObject collision_object_agv2_as1_mast;
    collision_object_agv2_as1_mast.header.frame_id = move_group.getPlanningFrame();
    collision_object_agv2_as1_mast.id = "agv2_as1_mast";
    moveit_msgs::CollisionObject collision_object_agv2_as2_mast;
    collision_object_agv2_as2_mast.header.frame_id = move_group.getPlanningFrame();
    collision_object_agv2_as2_mast.id = "agv2_as2_mast";

    moveit_msgs::CollisionObject collision_object_agv3_ks3;
    collision_object_agv3_ks3.header.frame_id = move_group.getPlanningFrame();
    collision_object_agv3_ks3.id = "agv3_ks3";
    moveit_msgs::CollisionObject collision_object_agv3_ks3_mast;
    collision_object_agv3_ks3_mast.header.frame_id = move_group.getPlanningFrame();
    collision_object_agv3_ks3_mast.id = "agv3_ks3_mast";
    moveit_msgs::CollisionObject collision_object_agv3_as3_mast;
    collision_object_agv3_as3_mast.header.frame_id = move_group.getPlanningFrame();
    collision_object_agv3_as3_mast.id = "agv3_as3_mast";
    moveit_msgs::CollisionObject collision_object_agv3_as4_mast;
    collision_object_agv3_as4_mast.header.frame_id = move_group.getPlanningFrame();
    collision_object_agv3_as4_mast.id = "agv3_as4_mast";

    moveit_msgs::CollisionObject collision_object_agv4_ks4;
    collision_object_agv4_ks4.header.frame_id = move_group.getPlanningFrame();
    collision_object_agv4_ks4.id = "agv4_ks4";
    moveit_msgs::CollisionObject collision_object_agv4_ks4_mast;
    collision_object_agv4_ks4_mast.header.frame_id = move_group.getPlanningFrame();
    collision_object_agv4_ks4_mast.id = "agv4_ks4_mast";
    moveit_msgs::CollisionObject collision_object_agv4_as3_mast;
    collision_object_agv4_as3_mast.header.frame_id = move_group.getPlanningFrame();
    collision_object_agv4_as3_mast.id = "agv4_as3_mast";
    moveit_msgs::CollisionObject collision_object_agv4_as4_mast;
    collision_object_agv4_as4_mast.header.frame_id = move_group.getPlanningFrame();
    collision_object_agv4_as4_mast.id = "agv4_as4_mast";

    moveit_msgs::CollisionObject collision_object_as1_base;
    collision_object_as1_base.header.frame_id = move_group.getPlanningFrame();
    collision_object_as1_base.id = "as1_base";
    moveit_msgs::CollisionObject collision_object_as1_back;
    collision_object_as1_back.header.frame_id = move_group.getPlanningFrame();
    collision_object_as1_back.id = "as1_back";
    moveit_msgs::CollisionObject collision_object_as1_left;
    collision_object_as1_left.header.frame_id = move_group.getPlanningFrame();
    collision_object_as1_left.id = "as1_left";
    moveit_msgs::CollisionObject collision_object_as1_right;
    collision_object_as1_right.header.frame_id = move_group.getPlanningFrame();
    collision_object_as1_right.id = "as1_right";
    moveit_msgs::CollisionObject collision_object_as2_base;
    collision_object_as2_base.header.frame_id = move_group.getPlanningFrame();
    collision_object_as2_base.id = "as2_base";
    moveit_msgs::CollisionObject collision_object_as2_back;
    collision_object_as2_back.header.frame_id = move_group.getPlanningFrame();
    collision_object_as2_back.id = "as2_back";
    moveit_msgs::CollisionObject collision_object_as2_left;
    collision_object_as2_left.header.frame_id = move_group.getPlanningFrame();
    collision_object_as2_left.id = "as2_left";
    moveit_msgs::CollisionObject collision_object_as2_right;
    collision_object_as2_right.header.frame_id = move_group.getPlanningFrame();
    collision_object_as2_right.id = "as2_right";
    moveit_msgs::CollisionObject collision_object_as3_base;
    collision_object_as3_base.header.frame_id = move_group.getPlanningFrame();
    collision_object_as3_base.id = "as3_base";
    moveit_msgs::CollisionObject collision_object_as3_back;
    collision_object_as3_back.header.frame_id = move_group.getPlanningFrame();
    collision_object_as3_back.id = "as3_back";
    moveit_msgs::CollisionObject collision_object_as3_left;
    collision_object_as3_left.header.frame_id = move_group.getPlanningFrame();
    collision_object_as3_left.id = "as3_left";
    moveit_msgs::CollisionObject collision_object_as3_right;
    collision_object_as3_right.header.frame_id = move_group.getPlanningFrame();
    collision_object_as3_right.id = "as3_right";
    moveit_msgs::CollisionObject collision_object_as4_base;
    collision_object_as4_base.header.frame_id = move_group.getPlanningFrame();
    collision_object_as4_base.id = "as4_base";
    moveit_msgs::CollisionObject collision_object_as4_back;
    collision_object_as4_back.header.frame_id = move_group.getPlanningFrame();
    collision_object_as4_back.id = "as4_back";
    moveit_msgs::CollisionObject collision_object_as4_left;
    collision_object_as4_left.header.frame_id = move_group.getPlanningFrame();
    collision_object_as4_left.id = "as4_left";
    moveit_msgs::CollisionObject collision_object_as4_right;
    collision_object_as4_right.header.frame_id = move_group.getPlanningFrame();
    collision_object_as4_right.id = "as4_right";
    // Define a box for the objects
    shape_msgs::SolidPrimitive primitive_conveyor;
    primitive_conveyor.type = primitive_conveyor.BOX;
    primitive_conveyor.dimensions.resize(3);
    primitive_conveyor.dimensions[0] = 0.68;
    primitive_conveyor.dimensions[1] = 9.0;
    primitive_conveyor.dimensions[2] = 0.84;
    
        shape_msgs::SolidPrimitive primitive_kitting_rail;
    primitive_kitting_rail.type = primitive_kitting_rail.BOX;
    primitive_kitting_rail.dimensions.resize(3);
    primitive_kitting_rail.dimensions[0] = 0.2;
    primitive_kitting_rail.dimensions[1] = 10.0;
    primitive_kitting_rail.dimensions[2] = 0.1;
        shape_msgs::SolidPrimitive primitive_bins_1to4;
    primitive_bins_1to4.type = primitive_bins_1to4.BOX;
    primitive_bins_1to4.dimensions.resize(3);
    primitive_bins_1to4.dimensions[0] = 1.4;
    primitive_bins_1to4.dimensions[1] = 1.46;
    primitive_bins_1to4.dimensions[2] = 0.85;   
        shape_msgs::SolidPrimitive primitive_bins_5to8;
    primitive_bins_5to8.type = primitive_bins_5to8.BOX;
    primitive_bins_5to8.dimensions.resize(3);
    primitive_bins_5to8.dimensions[0] = 1.4;
    primitive_bins_5to8.dimensions[1] = 1.46;
    primitive_bins_5to8.dimensions[2] = 0.85; 
        shape_msgs::SolidPrimitive primitive_faulty_bin;
    primitive_faulty_bin.type = primitive_faulty_bin.BOX;
    primitive_faulty_bin.dimensions.resize(3);
    primitive_faulty_bin.dimensions[0] = 0.75;
    primitive_faulty_bin.dimensions[1] = 0.9;
    primitive_faulty_bin.dimensions[2] = 0.9;

        shape_msgs::SolidPrimitive primitive_agv1_ks1;
    primitive_agv1_ks1.type = primitive_agv1_ks1.BOX;
    primitive_agv1_ks1.dimensions.resize(3);
    primitive_agv1_ks1.dimensions[0] = 1.18;
    primitive_agv1_ks1.dimensions[1] = 0.68;
    primitive_agv1_ks1.dimensions[2] = 0.82;
    shape_msgs::SolidPrimitive primitive_agv1_ks1_mast;
    primitive_agv1_ks1_mast.type = primitive_agv1_ks1_mast.BOX;
    primitive_agv1_ks1_mast.dimensions.resize(3);
    primitive_agv1_ks1_mast.dimensions[0] = 0.18;
    primitive_agv1_ks1_mast.dimensions[1] = 0.18;
    primitive_agv1_ks1_mast.dimensions[2] = 1.42;
    shape_msgs::SolidPrimitive primitive_agv1_as1_mast;
    primitive_agv1_as1_mast.type = primitive_agv1_as1_mast.BOX;
    primitive_agv1_as1_mast.dimensions.resize(3);
    primitive_agv1_as1_mast.dimensions[0] = 0.18;
    primitive_agv1_as1_mast.dimensions[1] = 0.18;
    primitive_agv1_as1_mast.dimensions[2] = 1.42;
    shape_msgs::SolidPrimitive primitive_agv1_as2_mast;
    primitive_agv1_as2_mast.type = primitive_agv1_as2_mast.BOX;
    primitive_agv1_as2_mast.dimensions.resize(3);
    primitive_agv1_as2_mast.dimensions[0] = 0.18;
    primitive_agv1_as2_mast.dimensions[1] = 0.18;
    primitive_agv1_as2_mast.dimensions[2] = 1.42;

        shape_msgs::SolidPrimitive primitive_agv2_ks2;
    primitive_agv2_ks2.type = primitive_agv2_ks2.BOX;
    primitive_agv2_ks2.dimensions.resize(3);
    primitive_agv2_ks2.dimensions[0] = 1.18;
    primitive_agv2_ks2.dimensions[1] = 0.68;
    primitive_agv2_ks2.dimensions[2] = 0.82;
    shape_msgs::SolidPrimitive primitive_agv2_ks2_mast;
    primitive_agv2_ks2_mast.type = primitive_agv2_ks2_mast.BOX;
    primitive_agv2_ks2_mast.dimensions.resize(3);
    primitive_agv2_ks2_mast.dimensions[0] = 0.18;
    primitive_agv2_ks2_mast.dimensions[1] = 0.18;
    primitive_agv2_ks2_mast.dimensions[2] = 1.42;
    shape_msgs::SolidPrimitive primitive_agv2_as1_mast;
    primitive_agv2_as1_mast.type = primitive_agv2_as1_mast.BOX;
    primitive_agv2_as1_mast.dimensions.resize(3);
    primitive_agv2_as1_mast.dimensions[0] = 0.18;
    primitive_agv2_as1_mast.dimensions[1] = 0.18;
    primitive_agv2_as1_mast.dimensions[2] = 1.42;
    shape_msgs::SolidPrimitive primitive_agv2_as2_mast;
    primitive_agv2_as2_mast.type = primitive_agv2_as2_mast.BOX;
    primitive_agv2_as2_mast.dimensions.resize(3);
    primitive_agv2_as2_mast.dimensions[0] = 0.18;
    primitive_agv2_as2_mast.dimensions[1] = 0.18;
    primitive_agv2_as2_mast.dimensions[2] = 1.42;

        shape_msgs::SolidPrimitive primitive_agv3_ks3;
    primitive_agv3_ks3.type = primitive_agv3_ks3.BOX;
    primitive_agv3_ks3.dimensions.resize(3);
    primitive_agv3_ks3.dimensions[0] = 1.18;
    primitive_agv3_ks3.dimensions[1] = 0.68;
    primitive_agv3_ks3.dimensions[2] = 0.82;
    shape_msgs::SolidPrimitive primitive_agv3_ks3_mast;
    primitive_agv3_ks3_mast.type = primitive_agv3_ks3_mast.BOX;
    primitive_agv3_ks3_mast.dimensions.resize(3);
    primitive_agv3_ks3_mast.dimensions[0] = 0.18;
    primitive_agv3_ks3_mast.dimensions[1] = 0.18;
    primitive_agv3_ks3_mast.dimensions[2] = 1.42;
    shape_msgs::SolidPrimitive primitive_agv3_as3_mast;
    primitive_agv3_as3_mast.type = primitive_agv3_as3_mast.BOX;
    primitive_agv3_as3_mast.dimensions.resize(3);
    primitive_agv3_as3_mast.dimensions[0] = 0.18;
    primitive_agv3_as3_mast.dimensions[1] = 0.18;
    primitive_agv3_as3_mast.dimensions[2] = 1.42;
    shape_msgs::SolidPrimitive primitive_agv3_as4_mast;
    primitive_agv3_as4_mast.type = primitive_agv3_as4_mast.BOX;
    primitive_agv3_as4_mast.dimensions.resize(3);
    primitive_agv3_as4_mast.dimensions[0] = 0.18;
    primitive_agv3_as4_mast.dimensions[1] = 0.18;
    primitive_agv3_as4_mast.dimensions[2] = 1.42;

        shape_msgs::SolidPrimitive primitive_agv4_ks4;
    primitive_agv4_ks4.type = primitive_agv4_ks4.BOX;
    primitive_agv4_ks4.dimensions.resize(3);
    primitive_agv4_ks4.dimensions[0] = 1.18;
    primitive_agv4_ks4.dimensions[1] = 0.68;
    primitive_agv4_ks4.dimensions[2] = 0.82;
    shape_msgs::SolidPrimitive primitive_agv4_ks4_mast;
    primitive_agv4_ks4_mast.type = primitive_agv4_ks4_mast.BOX;
    primitive_agv4_ks4_mast.dimensions.resize(3);
    primitive_agv4_ks4_mast.dimensions[0] = 0.18;
    primitive_agv4_ks4_mast.dimensions[1] = 0.18;
    primitive_agv4_ks4_mast.dimensions[2] = 1.42;
    shape_msgs::SolidPrimitive primitive_agv4_as3_mast;
    primitive_agv4_as3_mast.type = primitive_agv4_as3_mast.BOX;
    primitive_agv4_as3_mast.dimensions.resize(3);
    primitive_agv4_as3_mast.dimensions[0] = 0.18;
    primitive_agv4_as3_mast.dimensions[1] = 0.18;
    primitive_agv4_as3_mast.dimensions[2] = 1.42;
    shape_msgs::SolidPrimitive primitive_agv4_as4_mast;
    primitive_agv4_as4_mast.type = primitive_agv4_as4_mast.BOX;
    primitive_agv4_as4_mast.dimensions.resize(3);
    primitive_agv4_as4_mast.dimensions[0] = 0.18;
    primitive_agv4_as4_mast.dimensions[1] = 0.18;
    primitive_agv4_as4_mast.dimensions[2] = 1.42;

        shape_msgs::SolidPrimitive primitive_as1_base;
    primitive_as1_base.type = primitive_as1_base.BOX;
    primitive_as1_base.dimensions.resize(3);
    primitive_as1_base.dimensions[0] = 1.20;
    primitive_as1_base.dimensions[1] = 1.28;
    primitive_as1_base.dimensions[2] = 1.17;
        shape_msgs::SolidPrimitive primitive_as1_back;
    primitive_as1_back.type = primitive_as1_back.BOX;
    primitive_as1_back.dimensions.resize(3);
    primitive_as1_back.dimensions[0] = 0.26;
    primitive_as1_back.dimensions[1] = 1.28;
    primitive_as1_back.dimensions[2] = 0.72;
        shape_msgs::SolidPrimitive primitive_as1_left;
    primitive_as1_left.type = primitive_as1_left.BOX;
    primitive_as1_left.dimensions.resize(3);
    primitive_as1_left.dimensions[0] = 0.74;
    primitive_as1_left.dimensions[1] = 0.10;
    primitive_as1_left.dimensions[2] = 0.63;
        shape_msgs::SolidPrimitive primitive_as1_right;
    primitive_as1_right.type = primitive_as1_right.BOX;
    primitive_as1_right.dimensions.resize(3);
    primitive_as1_right.dimensions[0] = 0.74;
    primitive_as1_right.dimensions[1] = 0.10;
    primitive_as1_right.dimensions[2] = 0.63;

        shape_msgs::SolidPrimitive primitive_as2_base;
    primitive_as2_base.type = primitive_as2_base.BOX;
    primitive_as2_base.dimensions.resize(3);
    primitive_as2_base.dimensions[0] = 1.20;
    primitive_as2_base.dimensions[1] = 1.28;
    primitive_as2_base.dimensions[2] = 1.17;
        shape_msgs::SolidPrimitive primitive_as2_back;
    primitive_as2_back.type = primitive_as2_back.BOX;
    primitive_as2_back.dimensions.resize(3);
    primitive_as2_back.dimensions[0] = 0.26;
    primitive_as2_back.dimensions[1] = 1.28;
    primitive_as2_back.dimensions[2] = 0.72;
        shape_msgs::SolidPrimitive primitive_as2_left;
    primitive_as2_left.type = primitive_as2_left.BOX;
    primitive_as2_left.dimensions.resize(3);
    primitive_as2_left.dimensions[0] = 0.74;
    primitive_as2_left.dimensions[1] = 0.10;
    primitive_as2_left.dimensions[2] = 0.63;
        shape_msgs::SolidPrimitive primitive_as2_right;
    primitive_as2_right.type = primitive_as2_right.BOX;
    primitive_as2_right.dimensions.resize(3);
    primitive_as2_right.dimensions[0] = 0.74;
    primitive_as2_right.dimensions[1] = 0.10;
    primitive_as2_right.dimensions[2] = 0.63;

        shape_msgs::SolidPrimitive primitive_as3_base; 
    primitive_as3_base.type = primitive_as3_base.BOX;
    primitive_as3_base.dimensions.resize(3);
    primitive_as3_base.dimensions[0] = 1.20;
    primitive_as3_base.dimensions[1] = 1.28;
    primitive_as3_base.dimensions[2] = 1.17;
        shape_msgs::SolidPrimitive primitive_as3_back;
    primitive_as3_back.type = primitive_as3_back.BOX;
    primitive_as3_back.dimensions.resize(3);
    primitive_as3_back.dimensions[0] = 0.26;
    primitive_as3_back.dimensions[1] = 1.28;
    primitive_as3_back.dimensions[2] = 0.72;
        shape_msgs::SolidPrimitive primitive_as3_left;
    primitive_as3_left.type = primitive_as3_left.BOX;
    primitive_as3_left.dimensions.resize(3);
    primitive_as3_left.dimensions[0] = 0.74;
    primitive_as3_left.dimensions[1] = 0.10;
    primitive_as3_left.dimensions[2] = 0.63;
        shape_msgs::SolidPrimitive primitive_as3_right;
    primitive_as3_right.type = primitive_as3_right.BOX;
    primitive_as3_right.dimensions.resize(3);
    primitive_as3_right.dimensions[0] = 0.74;
    primitive_as3_right.dimensions[1] = 0.10;
    primitive_as3_right.dimensions[2] = 0.63;

        shape_msgs::SolidPrimitive primitive_as4_base;
    primitive_as4_base.type = primitive_as4_base.BOX;
    primitive_as4_base.dimensions.resize(3);
    primitive_as4_base.dimensions[0] = 1.20;
    primitive_as4_base.dimensions[1] = 1.28;
    primitive_as4_base.dimensions[2] = 1.17;
        shape_msgs::SolidPrimitive primitive_as4_back;
    primitive_as4_back.type = primitive_as4_back.BOX;
    primitive_as4_back.dimensions.resize(3);
    primitive_as4_back.dimensions[0] = 0.26;
    primitive_as4_back.dimensions[1] = 1.28;
    primitive_as4_back.dimensions[2] = 0.72;
        shape_msgs::SolidPrimitive primitive_as4_left;
    primitive_as4_left.type = primitive_as4_left.BOX;
    primitive_as4_left.dimensions.resize(3);
    primitive_as4_left.dimensions[0] = 0.74;
    primitive_as4_left.dimensions[1] = 0.10;
    primitive_as4_left.dimensions[2] = 0.63;
        shape_msgs::SolidPrimitive primitive_as4_right;
    primitive_as4_right.type = primitive_as4_right.BOX;
    primitive_as4_right.dimensions.resize(3);
    primitive_as4_right.dimensions[0] = 0.74;
    primitive_as4_right.dimensions[1] = 0.10;
    primitive_as4_right.dimensions[2] = 0.63;
    

    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose box_pose_conveyor;
    box_pose_conveyor.orientation.w = 1.0;
    box_pose_conveyor.position.x = -0.573076;
    box_pose_conveyor.position.y = 0.0;
    box_pose_conveyor.position.z = 0.5;
        geometry_msgs::Pose box_pose_kitting_rail;
    box_pose_kitting_rail.orientation.w = 1.0;
    box_pose_kitting_rail.position.x = -1.3;
    box_pose_kitting_rail.position.y = 0.0;
    box_pose_kitting_rail.position.z = 0.93;
        geometry_msgs::Pose box_pose_bins_1to4;
    box_pose_bins_1to4.orientation.w = 1.0;
    box_pose_bins_1to4.position.x = -2.275340;
    box_pose_bins_1to4.position.y = 2.972460;
    box_pose_bins_1to4.position.z = 0.425;
        geometry_msgs::Pose box_pose_bins_5to8;
    box_pose_bins_5to8.orientation.w = 1.0;
    box_pose_bins_5to8.position.x = -2.275340;
    box_pose_bins_5to8.position.y = -2.972460;
    box_pose_bins_5to8.position.z = 0.425;
        geometry_msgs::Pose box_pose_faulty_bin;
    box_pose_faulty_bin.orientation.w = 1.0;
    box_pose_faulty_bin.position.x = -2.186829;
    box_pose_faulty_bin.position.y = 0.0;
    box_pose_faulty_bin.position.z = 0;
        
        geometry_msgs::Pose box_pose_agv1_ks1;
    box_pose_agv1_ks1.orientation.w = 1.0;
    box_pose_agv1_ks1.position.x = -2.26568;
    box_pose_agv1_ks1.position.y = 4.675400;
    box_pose_agv1_ks1.position.z = 0.41;
    geometry_msgs::Pose box_pose_agv1_ks1_mast;
    box_pose_agv1_ks1_mast.orientation.w = 1.0;
    box_pose_agv1_ks1_mast.position.x = -2.542;
    box_pose_agv1_ks1_mast.position.y = 4.675400;
    box_pose_agv1_ks1_mast.position.z = 0.7;
    geometry_msgs::Pose box_pose_agv1_as1_mast;
    box_pose_agv1_as1_mast.orientation.w = 1.0;
    box_pose_agv1_as1_mast.position.x = -5.87632;
    box_pose_agv1_as1_mast.position.y = 4.675400;
    box_pose_agv1_as1_mast.position.z = 0.7;
    geometry_msgs::Pose box_pose_agv1_as2_mast;
    box_pose_agv1_as2_mast.orientation.w = 1.0;
    box_pose_agv1_as2_mast.position.x = -10.87632;
    box_pose_agv1_as2_mast.position.y = 4.675400;
    box_pose_agv1_as2_mast.position.z = 0.7;

        geometry_msgs::Pose box_pose_agv2_ks2;
    box_pose_agv2_ks2.orientation.w = 1.0;
    box_pose_agv2_ks2.position.x = -2.26568;
    box_pose_agv2_ks2.position.y = 1.367643;
    box_pose_agv2_ks2.position.z = 0.41;
    geometry_msgs::Pose box_pose_agv2_ks2_mast;
    box_pose_agv2_ks2_mast.orientation.w = 1.0;
    box_pose_agv2_ks2_mast.position.x = -2.542;
    box_pose_agv2_ks2_mast.position.y = 1.367643;
    box_pose_agv2_ks2_mast.position.z = 0.7;
    geometry_msgs::Pose box_pose_agv2_as1_mast;
    box_pose_agv2_as1_mast.orientation.w = 1.0;
    box_pose_agv2_as1_mast.position.x = -5.87632;
    box_pose_agv2_as1_mast.position.y = 1.367643;
    box_pose_agv2_as1_mast.position.z = 0.7;
    geometry_msgs::Pose box_pose_agv2_as2_mast;
    box_pose_agv2_as2_mast.orientation.w = 1.0;
    box_pose_agv2_as2_mast.position.x = -10.87632;
    box_pose_agv2_as2_mast.position.y = 1.367643;
    box_pose_agv2_as2_mast.position.z = 0.7;

        geometry_msgs::Pose box_pose_agv3_ks3;
    box_pose_agv3_ks3.orientation.w = 1.0;
    box_pose_agv3_ks3.position.x = -2.26568;
    box_pose_agv3_ks3.position.y = -1.333917;
    box_pose_agv3_ks3.position.z = 0.41;
    geometry_msgs::Pose box_pose_agv3_ks3_mast;
    box_pose_agv3_ks3_mast.orientation.w = 1.0;
    box_pose_agv3_ks3_mast.position.x = -2.542;
    box_pose_agv3_ks3_mast.position.y = -1.333917;
    box_pose_agv3_ks3_mast.position.z = 0.7;
    geometry_msgs::Pose box_pose_agv3_as3_mast;
    box_pose_agv3_as3_mast.orientation.w = 1.0;
    box_pose_agv3_as3_mast.position.x = -5.87632;
    box_pose_agv3_as3_mast.position.y = -1.333917;
    box_pose_agv3_as3_mast.position.z = 0.7;
    geometry_msgs::Pose box_pose_agv3_as4_mast;
    box_pose_agv3_as4_mast.orientation.w = 1.0;
    box_pose_agv3_as4_mast.position.x = -10.87632;
    box_pose_agv3_as4_mast.position.y = -1.333917;
    box_pose_agv3_as4_mast.position.z = 0.7;
    
        geometry_msgs::Pose box_pose_agv4_ks4;
    box_pose_agv4_ks4.orientation.w = 1.0;
    box_pose_agv4_ks4.position.x = -2.26568;
    box_pose_agv4_ks4.position.y = -4.696062;
    box_pose_agv4_ks4.position.z = 0.41;
    geometry_msgs::Pose box_pose_agv4_ks4_mast;
    box_pose_agv4_ks4_mast.orientation.w = 1.0;
    box_pose_agv4_ks4_mast.position.x = -2.542;
    box_pose_agv4_ks4_mast.position.y = -4.696062;
    box_pose_agv4_ks4_mast.position.z = 0.7;
    geometry_msgs::Pose box_pose_agv4_as3_mast;
    box_pose_agv4_as3_mast.orientation.w = 1.0;
    box_pose_agv4_as3_mast.position.x = -5.87632;
    box_pose_agv4_as3_mast.position.y = -4.696062;
    box_pose_agv4_as3_mast.position.z = 0.7;
    geometry_msgs::Pose box_pose_agv4_as4_mast;
    box_pose_agv4_as4_mast.orientation.w = 1.0;
    box_pose_agv4_as4_mast.position.x = -10.87632;
    box_pose_agv4_as4_mast.position.y = -4.696062;
    box_pose_agv4_as4_mast.position.z = 0.7;

        geometry_msgs::Pose box_pose_as1_base;
    box_pose_as1_base.orientation.w = 1.0;
    box_pose_as1_base.position.x = -7.6;
    box_pose_as1_base.position.y = 3.0;
    box_pose_as1_base.position.z = 0.6;
    geometry_msgs::Pose box_pose_as1_back;
    box_pose_as1_back.orientation.w = 1.0;
    box_pose_as1_back.position.x = -8.07;
    box_pose_as1_back.position.y = 3.0;
    box_pose_as1_back.position.z = 1.56;
    geometry_msgs::Pose box_pose_as1_left;
    box_pose_as1_left.orientation.w = 1.0;
    box_pose_as1_left.position.x = -7.8;
    box_pose_as1_left.position.y = 2.4;
    box_pose_as1_left.position.z = 1.5;
    geometry_msgs::Pose box_pose_as1_right;
    box_pose_as1_right.orientation.w = 1.0;
    box_pose_as1_right.position.x = -7.8;
    box_pose_as1_right.position.y = 3.6;
    box_pose_as1_right.position.z = 1.5;

        geometry_msgs::Pose box_pose_as2_base;
    box_pose_as2_base.orientation.w = 1.0;
    box_pose_as2_base.position.x = -12.6;
    box_pose_as2_base.position.y = 3.0;
    box_pose_as2_base.position.z = 0.6;
    geometry_msgs::Pose box_pose_as2_back;
    box_pose_as2_back.orientation.w = 1.0;
    box_pose_as2_back.position.x = -13.07;
    box_pose_as2_back.position.y = 3.0;
    box_pose_as2_back.position.z = 1.56;
    geometry_msgs::Pose box_pose_as2_left;
    box_pose_as2_left.orientation.w = 1.0;
    box_pose_as2_left.position.x = -12.8;
    box_pose_as2_left.position.y = 2.4;
    box_pose_as2_left.position.z = 1.5;
    geometry_msgs::Pose box_pose_as2_right;
    box_pose_as2_right.orientation.w = 1.0;
    box_pose_as2_right.position.x = -12.8;
    box_pose_as2_right.position.y = 3.6;
    box_pose_as2_right.position.z = 1.5;

        geometry_msgs::Pose box_pose_as3_base;
    box_pose_as3_base.orientation.w = 1.0;
    box_pose_as3_base.position.x = -7.6;
    box_pose_as3_base.position.y = -3.0;
    box_pose_as3_base.position.z = 0.6;
    geometry_msgs::Pose box_pose_as3_back;
    box_pose_as3_back.orientation.w = 1.0;
    box_pose_as3_back.position.x = -8.07;
    box_pose_as3_back.position.y = -3.0;
    box_pose_as3_back.position.z = 1.56;
    geometry_msgs::Pose box_pose_as3_left;
    box_pose_as3_left.orientation.w = 1.0;
    box_pose_as3_left.position.x = -7.8;
    box_pose_as3_left.position.y = -2.4;
    box_pose_as3_left.position.z = 1.5;
    geometry_msgs::Pose box_pose_as3_right;
    box_pose_as3_right.orientation.w = 1.0;
    box_pose_as3_right.position.x = -7.8;
    box_pose_as3_right.position.y = -3.6;
    box_pose_as3_right.position.z = 1.5;

        geometry_msgs::Pose box_pose_as4_base;
    box_pose_as4_base.orientation.w = 1.0;
    box_pose_as4_base.position.x = -12.6;
    box_pose_as4_base.position.y = -3.0;
    box_pose_as4_base.position.z = 0.6;
    geometry_msgs::Pose box_pose_as4_back;
    box_pose_as4_back.orientation.w = 1.0;
    box_pose_as4_back.position.x = -13.07;
    box_pose_as4_back.position.y = -3.0;
    box_pose_as4_back.position.z = 1.56;
    geometry_msgs::Pose box_pose_as4_left;
    box_pose_as4_left.orientation.w = 1.0;
    box_pose_as4_left.position.x = -12.8;
    box_pose_as4_left.position.y = -2.4;
    box_pose_as4_left.position.z = 1.5;
    geometry_msgs::Pose box_pose_as4_right;
    box_pose_as4_right.orientation.w = 1.0;
    box_pose_as4_right.position.x = -12.8;
    box_pose_as4_right.position.y = -3.6;
    box_pose_as4_right.position.z = 1.5;

    // And add it to the collision object
    collision_object_conveyor.primitives.push_back(primitive_conveyor);
    collision_object_conveyor.primitive_poses.push_back(box_pose_conveyor);
    collision_object_conveyor.operation = collision_object_conveyor.ADD;
        collision_object_kitting_rail.primitives.push_back(primitive_kitting_rail);
    collision_object_kitting_rail.primitive_poses.push_back(box_pose_kitting_rail);
    collision_object_kitting_rail.operation = collision_object_kitting_rail.ADD;
        collision_object_bins_1to4.primitives.push_back(primitive_bins_1to4);
    collision_object_bins_1to4.primitive_poses.push_back(box_pose_bins_1to4);
    collision_object_bins_1to4.operation = collision_object_bins_1to4.ADD;
        collision_object_bins_5to8.primitives.push_back(primitive_bins_5to8);
    collision_object_bins_5to8.primitive_poses.push_back(box_pose_bins_5to8);
    collision_object_bins_5to8.operation = collision_object_bins_5to8.ADD;
        collision_object_faulty_bin.primitives.push_back(primitive_faulty_bin);
    collision_object_faulty_bin.primitive_poses.push_back(box_pose_faulty_bin);
    collision_object_faulty_bin.operation = collision_object_faulty_bin.ADD;

        collision_object_agv1_ks1.primitives.push_back(primitive_agv1_ks1);
    collision_object_agv1_ks1.primitive_poses.push_back(box_pose_agv1_ks1);
    collision_object_agv1_ks1.operation = collision_object_agv1_ks1.ADD;
    collision_object_agv1_ks1_mast.primitives.push_back(primitive_agv1_ks1_mast);
    collision_object_agv1_ks1_mast.primitive_poses.push_back(box_pose_agv1_ks1_mast);
    collision_object_agv1_ks1_mast.operation = collision_object_agv1_ks1_mast.ADD;
    collision_object_agv1_as1_mast.primitives.push_back(primitive_agv1_as1_mast);
    collision_object_agv1_as1_mast.primitive_poses.push_back(box_pose_agv1_as1_mast);
    collision_object_agv1_as1_mast.operation = collision_object_agv1_as1_mast.ADD;
    collision_object_agv1_as2_mast.primitives.push_back(primitive_agv1_as2_mast);
    collision_object_agv1_as2_mast.primitive_poses.push_back(box_pose_agv1_as2_mast);
    collision_object_agv1_as2_mast.operation = collision_object_agv1_as2_mast.ADD;

        collision_object_agv2_ks2.primitives.push_back(primitive_agv2_ks2);
    collision_object_agv2_ks2.primitive_poses.push_back(box_pose_agv2_ks2);
    collision_object_agv2_ks2.operation = collision_object_agv2_ks2.ADD;
    collision_object_agv2_ks2_mast.primitives.push_back(primitive_agv2_ks2_mast);
    collision_object_agv2_ks2_mast.primitive_poses.push_back(box_pose_agv2_ks2_mast);
    collision_object_agv2_ks2_mast.operation = collision_object_agv2_ks2_mast.ADD;
    collision_object_agv2_as1_mast.primitives.push_back(primitive_agv2_as1_mast);
    collision_object_agv2_as1_mast.primitive_poses.push_back(box_pose_agv2_as1_mast);
    collision_object_agv2_as1_mast.operation = collision_object_agv2_as1_mast.ADD;
    collision_object_agv2_as2_mast.primitives.push_back(primitive_agv2_as2_mast);
    collision_object_agv2_as2_mast.primitive_poses.push_back(box_pose_agv2_as2_mast);
    collision_object_agv2_as2_mast.operation = collision_object_agv2_as2_mast.ADD;

        collision_object_agv3_ks3.primitives.push_back(primitive_agv3_ks3);
    collision_object_agv3_ks3.primitive_poses.push_back(box_pose_agv3_ks3);
    collision_object_agv3_ks3.operation = collision_object_agv3_ks3.ADD;
    collision_object_agv3_ks3_mast.primitives.push_back(primitive_agv3_ks3_mast);
    collision_object_agv3_ks3_mast.primitive_poses.push_back(box_pose_agv3_ks3_mast);
    collision_object_agv3_ks3_mast.operation = collision_object_agv3_ks3_mast.ADD;
    collision_object_agv3_as3_mast.primitives.push_back(primitive_agv3_as3_mast);
    collision_object_agv3_as3_mast.primitive_poses.push_back(box_pose_agv3_as3_mast);
    collision_object_agv3_as3_mast.operation = collision_object_agv3_as3_mast.ADD;
    collision_object_agv3_as4_mast.primitives.push_back(primitive_agv3_as4_mast);
    collision_object_agv3_as4_mast.primitive_poses.push_back(box_pose_agv3_as4_mast);
    collision_object_agv3_as4_mast.operation = collision_object_agv3_as4_mast.ADD;

        collision_object_agv4_ks4.primitives.push_back(primitive_agv4_ks4);
    collision_object_agv4_ks4.primitive_poses.push_back(box_pose_agv4_ks4);
    collision_object_agv4_ks4.operation = collision_object_agv4_ks4.ADD;
    collision_object_agv4_ks4_mast.primitives.push_back(primitive_agv4_ks4_mast);
    collision_object_agv4_ks4_mast.primitive_poses.push_back(box_pose_agv4_ks4_mast);
    collision_object_agv4_ks4_mast.operation = collision_object_agv4_ks4_mast.ADD;
    collision_object_agv4_as3_mast.primitives.push_back(primitive_agv4_as3_mast);
    collision_object_agv4_as3_mast.primitive_poses.push_back(box_pose_agv4_as3_mast);
    collision_object_agv4_as3_mast.operation = collision_object_agv4_as3_mast.ADD;
    collision_object_agv4_as4_mast.primitives.push_back(primitive_agv4_as4_mast);
    collision_object_agv4_as4_mast.primitive_poses.push_back(box_pose_agv4_as4_mast);
    collision_object_agv4_as4_mast.operation = collision_object_agv4_as4_mast.ADD;
    
        collision_object_as1_base.primitives.push_back(primitive_as1_base);
    collision_object_as1_base.primitive_poses.push_back(box_pose_as1_base);
    collision_object_as1_base.operation = collision_object_as1_base.ADD;
    collision_object_as1_back.primitives.push_back(primitive_as1_back);
    collision_object_as1_back.primitive_poses.push_back(box_pose_as1_back);
    collision_object_as1_back.operation = collision_object_as1_back.ADD;
    collision_object_as1_left.primitives.push_back(primitive_as1_left);
    collision_object_as1_left.primitive_poses.push_back(box_pose_as1_left);
    collision_object_as1_left.operation = collision_object_as1_left.ADD;
    collision_object_as1_right.primitives.push_back(primitive_as1_right);
    collision_object_as1_right.primitive_poses.push_back(box_pose_as1_right);
    collision_object_as1_right.operation = collision_object_as1_right.ADD;
        collision_object_as2_base.primitives.push_back(primitive_as2_base);
    collision_object_as2_base.primitive_poses.push_back(box_pose_as2_base);
    collision_object_as2_base.operation = collision_object_as2_base.ADD;
    collision_object_as2_back.primitives.push_back(primitive_as2_back);
    collision_object_as2_back.primitive_poses.push_back(box_pose_as2_back);
    collision_object_as2_back.operation = collision_object_as2_back.ADD;
    collision_object_as2_left.primitives.push_back(primitive_as2_left);
    collision_object_as2_left.primitive_poses.push_back(box_pose_as2_left);
    collision_object_as2_left.operation = collision_object_as2_left.ADD;
    collision_object_as2_right.primitives.push_back(primitive_as2_right);
    collision_object_as2_right.primitive_poses.push_back(box_pose_as2_right);
    collision_object_as2_right.operation = collision_object_as2_right.ADD;
        collision_object_as3_base.primitives.push_back(primitive_as3_base);
    collision_object_as3_base.primitive_poses.push_back(box_pose_as3_base);
    collision_object_as3_base.operation = collision_object_as3_base.ADD;
    collision_object_as3_back.primitives.push_back(primitive_as3_back);
    collision_object_as3_back.primitive_poses.push_back(box_pose_as3_back);
    collision_object_as3_back.operation = collision_object_as3_back.ADD;
    collision_object_as3_left.primitives.push_back(primitive_as3_left);
    collision_object_as3_left.primitive_poses.push_back(box_pose_as3_left);
    collision_object_as3_left.operation = collision_object_as3_left.ADD;
    collision_object_as3_right.primitives.push_back(primitive_as3_right);
    collision_object_as3_right.primitive_poses.push_back(box_pose_as3_right);
    collision_object_as3_right.operation = collision_object_as3_right.ADD;
        collision_object_as4_base.primitives.push_back(primitive_as4_base);
    collision_object_as4_base.primitive_poses.push_back(box_pose_as4_base);
    collision_object_as4_base.operation = collision_object_as4_base.ADD;
    collision_object_as4_back.primitives.push_back(primitive_as4_back);
    collision_object_as4_back.primitive_poses.push_back(box_pose_as4_back);
    collision_object_as4_back.operation = collision_object_as4_back.ADD;
    collision_object_as4_left.primitives.push_back(primitive_as4_left);
    collision_object_as4_left.primitive_poses.push_back(box_pose_as4_left);
    collision_object_as4_left.operation = collision_object_as4_left.ADD;
    collision_object_as4_right.primitives.push_back(primitive_as4_right);
    collision_object_as4_right.primitive_poses.push_back(box_pose_as4_right);
    collision_object_as4_right.operation = collision_object_as4_right.ADD;
    

    // Create the collision objects container and add the collision object(s)
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object_conveyor);
    collision_objects.push_back(collision_object_kitting_rail);
    collision_objects.push_back(collision_object_bins_1to4);
    collision_objects.push_back(collision_object_bins_5to8);
    collision_objects.push_back(collision_object_faulty_bin);
    collision_objects.push_back(collision_object_agv1_ks1);
    collision_objects.push_back(collision_object_agv1_ks1_mast);
    collision_objects.push_back(collision_object_agv2_ks2);
    collision_objects.push_back(collision_object_agv2_ks2_mast);
    collision_objects.push_back(collision_object_agv3_ks3);
    collision_objects.push_back(collision_object_agv3_ks3_mast);
    collision_objects.push_back(collision_object_agv4_ks4);
    collision_objects.push_back(collision_object_agv4_ks4_mast);

    if (robot_name_ != "kitting") {
        // Only the gantry cares about the assembly station objects
        collision_objects.push_back(collision_object_agv1_as1_mast);
        collision_objects.push_back(collision_object_agv1_as2_mast);
        collision_objects.push_back(collision_object_agv2_as1_mast);
        collision_objects.push_back(collision_object_agv2_as2_mast);
        collision_objects.push_back(collision_object_agv3_as3_mast);
        collision_objects.push_back(collision_object_agv3_as4_mast);
        collision_objects.push_back(collision_object_agv4_as3_mast);
        collision_objects.push_back(collision_object_agv4_as4_mast);
        collision_objects.push_back(collision_object_as1_base);
        collision_objects.push_back(collision_object_as1_back);
        collision_objects.push_back(collision_object_as1_left);
        collision_objects.push_back(collision_object_as1_right);
        collision_objects.push_back(collision_object_as2_base);
        collision_objects.push_back(collision_object_as2_back);
        collision_objects.push_back(collision_object_as2_left);
        collision_objects.push_back(collision_object_as2_right);
        collision_objects.push_back(collision_object_as3_base);
        collision_objects.push_back(collision_object_as3_back);
        collision_objects.push_back(collision_object_as3_left);
        collision_objects.push_back(collision_object_as3_right);
        collision_objects.push_back(collision_object_as4_base);
        collision_objects.push_back(collision_object_as4_back);
        collision_objects.push_back(collision_object_as4_left);
        collision_objects.push_back(collision_object_as4_right);
    }
    
    // Now, let’s add the collision object into the world
    printf("Add the collision objects into the world\n");
    this->planning_scene_interface.addCollisionObjects(collision_objects);
    
}

int Robot::enable_gripper() {
    this->gripper_service_.request.enable = true;
    this->gripper_client_.call(gripper_service_);
    if (this->gripper_service_.response.success) {
      ROS_INFO_STREAM("Gripper activated!");
      printf("Gripper activated\n");
    } else {
      ROS_WARN_STREAM("Gripper activation failed!");
      printf("Gripper activation failed\n");
    }
    return 1;
}

int Robot::disable_gripper() {
    this->gripper_service_.request.enable = false;
    this->gripper_client_.call(gripper_service_);
    if (this->gripper_service_.response.success) {
      ROS_INFO_STREAM("Gripper deactivated!");
      printf("Gripper deactivated\n");
    } else {
      ROS_WARN_STREAM("Gripper deactivation failed!");
      printf("Gripper deactivation failed\n");
    }
    return 1;
}
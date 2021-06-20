#include <map>
#include <vector>
#include <cmath>
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


int Gantry_Arm::move_near(const geometry_msgs::TransformStamped & dest_pose_) {
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose target_pose = this->p_;
    waypoints.push_back(target_pose);  // Always start with the current pose before adding additional poses

    geometry_msgs::Pose target_pose0 = target_pose;
    target_pose0.position.y = dest_pose_.transform.translation.y;  // Just move to it on the y-axis
    waypoints.push_back(target_pose0); 

    geometry_msgs::Pose target_pose1 = target_pose0;
    target_pose1.position.x = dest_pose_.transform.translation.x;
    target_pose1.position.z = dest_pose_.transform.translation.z+.1;  // Move over part
    waypoints.push_back(target_pose1);

    double fraction = this->move_group.computeCartesianPath(waypoints, this->eef_step, this->jump_threshold, this->trajectory);
    this->my_plan_.trajectory_ = this->trajectory;
    this->move_group.execute(this->my_plan_);
    printf("done moving near location\n");

    this->p_ = target_pose1;

    return 1;
}

int Gantry_Arm::move_near(const geometry_msgs::Pose & dest_pose_) {
    std::vector<geometry_msgs::Pose> waypoints;

    waypoints.push_back(dest_pose_);

    double fraction = this->move_group.computeCartesianPath(waypoints, this->eef_step, this->jump_threshold, this->trajectory);
    this->my_plan_.trajectory_ = this->trajectory;
    this->move_group.execute(this->my_plan_);
    printf("done moving near location\n");

    // this->p_ = target_pose1;

    return 1;
}

int Gantry_Torso::move_torso(const geometry_msgs::Pose & dest_pose_) {  // Move the gantry torso to a position defined by the msg
    std::vector<geometry_msgs::Pose> waypoints;

    geometry_msgs::Pose target_pose2 = dest_pose_;

    waypoints.push_back(dest_pose_);

    double fraction = this->move_group.computeCartesianPath(waypoints, this->eef_step, this->jump_threshold, this->trajectory);
    this->my_plan_.trajectory_ = this->trajectory;
    this->move_group.execute(this->my_plan_);
    printf("done moving torso near location\n");

    this->p_ = target_pose2;
    printf("move_torso End effector pose->position (x,y,z): (%f,%f,%f)\n", target_pose2.position.x, target_pose2.position.y, target_pose2.position.z);
    printf("move_torso End effector pose->orientation (x,y,z,w): (%f,%f,%f,%f)\n", target_pose2.orientation.x, target_pose2.orientation.y, target_pose2.orientation.z, target_pose2.orientation.w);    

    return 1;
}

int Gantry_Torso::move_torso(const int & dest_) {
    std::vector<double> joint_group_positions;
    printf("In move_torso, dest_: %d\n", dest_);
    switch (dest_)
    {
        case LOC_AS1_AGV1:
            joint_group_positions = jgp_gt_as1_agv1;
            break;
        case LOC_AS1_AGV2:
            joint_group_positions = jgp_gt_as1_agv2;
            break;
        case LOC_AS1_PCASE:
            joint_group_positions = jgp_gt_as1_pcase;
            break;
        case LOC_AS1_CASE:
            joint_group_positions = jgp_gt_as1_case;
            break;
        case LOC_AS2_AGV1:
            joint_group_positions = jgp_gt_as2_agv1;
            break;
        case LOC_AS2_AGV2:
            joint_group_positions = jgp_gt_as2_agv2;
            break;
        case LOC_AS2_PCASE:
            joint_group_positions = jgp_gt_as2_pcase;
            break;
        case LOC_AS2_CASE:
            joint_group_positions = jgp_gt_as2_case;
            break;
        case LOC_AS3_AGV3:
            joint_group_positions = jgp_gt_as3_agv3;
            break;
        case LOC_AS3_AGV4:
            joint_group_positions = jgp_gt_as3_agv4;
            break;
        case LOC_AS3_PCASE:
            joint_group_positions = jgp_gt_as3_pcase;
            break;
        case LOC_AS3_CASE:
            joint_group_positions = jgp_gt_as3_case;
            break;
        case LOC_AS4_AGV3:
            joint_group_positions = jgp_gt_as4_agv3;
            break;
        case LOC_AS4_AGV4:
            joint_group_positions = jgp_gt_as4_agv4;
            break;
        case LOC_AS4_PCASE:
            joint_group_positions = jgp_gt_as4_pcase;
            break;
        case LOC_AS4_CASE:
            joint_group_positions = jgp_gt_as4_case;
            break;
        default:
            break;
    }

    this->move_group.setJointValueTarget(joint_group_positions);
    bool success = (this->move_group.plan(this->my_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
        this->move_group.move();
    } else {
        printf("planning not successfull...");
    }
    

    printf("Hopefully done with move_torso (using string) now. JPG: %f, %f, %f\n", joint_group_positions[0], joint_group_positions[1], joint_group_positions[2]);

    this->sp_ = this->move_group.getCurrentPose();
    this->p_ = this->sp_.pose;

    return 1;
}

int Gantry_Torso::move_neutral() {  // Move torso to a neutral nearby location based on current location
    geometry_msgs::Pose target_pose;
    tf2::Quaternion q;
    int test = 0;
    switch (this->current_location)
    {
        case 0: // starting pose, move to initial position
            q.setRPY( 0, 0, M_PI/2 );  // Create this quaternion from roll/pitch/yaw (in radians)
            q.normalize();
            target_pose.position.x = -3.6;
            target_pose.position.y = 0.0;
            target_pose.position.z = 0.7;
            target_pose.orientation.x = q[0];
            target_pose.orientation.y = q[1];
            target_pose.orientation.z = q[2];
            target_pose.orientation.w = q[3];
            this->current_location = LOC_NEUTRAL_BIN;
            break;
        case LOC_BINS:
            target_pose = this->p_;
            target_pose.position.x -= 0.5;
            this->current_location = LOC_NEUTRAL_BIN;
            break;
        case LOC_AS:
            break;
        case LOC_NEUTRAL_AS:
            break;
        case LOC_NEUTRAL_BIN:
            break;
        default:  // Can this be used for both LOC_NEUTRAL_BIN and LOC_NEUTRAL_AS to basically do nothing?
            break;
    }
    test = this->move_torso(target_pose);
    if (test) {
        this->p_ = target_pose;
    }
    return test;
}

int Gantry_Torso::move_to_bin(const geometry_msgs::TransformStamped & dest_pose_) {
    std::vector<geometry_msgs::Pose> waypoints;
    int test = 0;
    if (this->current_location == LOC_BINS && 
        std::abs(this->p_.position.x - dest_pose_.transform.translation.x) < 0.8 &&
        std::abs(this->p_.position.y - dest_pose_.transform.translation.y) < 0.3) {
        // Use 0.8 for the x position tolerance above because the destination is the part and the torso should be 0.7 from the part on the x-axis
        // Use 0.3 for the y position tolerance above because the destination is the part and this y position indicates the part is in the same bin
        // Basically do nothing because we should already be in position for picking
        return 1;
    } else {
        test = this->move_neutral();
        if (test) {
            geometry_msgs::Pose target_pose = this->p_;
            waypoints.push_back(target_pose);  // Always start with the current pose before adding additional poses

            geometry_msgs::Pose target_pose0 = target_pose;
            target_pose0.position.y = dest_pose_.transform.translation.y;  // Just move to it on the y-axis
            waypoints.push_back(target_pose0); 

            geometry_msgs::Pose target_pose1 = target_pose0;
            target_pose1.position.x = dest_pose_.transform.translation.x;
            target_pose1.position.z = dest_pose_.transform.translation.z+.1;  // Move over part
            waypoints.push_back(target_pose1);

            double fraction = this->move_group.computeCartesianPath(waypoints, this->eef_step, this->jump_threshold, this->trajectory);
            this->my_plan_.trajectory_ = this->trajectory;
            this->move_group.execute(this->my_plan_);
            printf("done moving near location\n");

            this->p_ = target_pose1;
        }

        return test;
    }
}

int Gantry_Arm::pickup_part(const geometry_msgs::TransformStamped & dest_pose_) {  // Pick up the part at the location defined by the msg
    // First, enable the gripper
    this->enable_gripper();

    // Then move near the part
    int mg_counter = 0;
    std::vector<geometry_msgs::Pose> waypoints;
    this->sp_ = this->move_group.getCurrentPose();
    this->p_ = this->sp_.pose;
    geometry_msgs::Pose target_pose = this->p_;
    // waypoints.push_back(target_pose);  // Always start with the current pose before adding additional poses

    geometry_msgs::Pose target_pose0 = target_pose;
    target_pose0.position.x = dest_pose_.transform.translation.x;
    target_pose0.position.y = dest_pose_.transform.translation.y;
    target_pose0.position.z = dest_pose_.transform.translation.z + 0.1;  // Move over part; was static value "0.805" when picking a battery off a bin
    target_pose.orientation.x = dest_pose_.transform.rotation.x;
    target_pose.orientation.y = dest_pose_.transform.rotation.y;
    target_pose.orientation.z = dest_pose_.transform.rotation.z;
    target_pose.orientation.w = dest_pose_.transform.rotation.w;
    waypoints.push_back(target_pose0);
    geometry_msgs::Pose target_pose01 = target_pose0;
    target_pose0.position.z = dest_pose_.transform.translation.z + 0.03;
    waypoints.push_back(target_pose01);
    double fraction = this->move_group.computeCartesianPath(waypoints, this->eef_step, this->jump_threshold, this->trajectory, false);
    this->my_plan_.trajectory_ = this->trajectory;
    this->move_group.execute(this->my_plan_);
    while (this->trajectory_success < 1 && mg_counter < 10) {
        ros::Duration(0.25).sleep();
        mg_counter++;
    }
    printf("done moving near part\n");
    this->trajectory_success = 0;
    mg_counter = 0;

    geometry_msgs::Pose target_pose1 = target_pose0;
    int move_counter =  0;
    while (!this->gripper_attached && move_counter < 15) {
        target_pose1.position.z -= 0.005;  // Move closer over part
        waypoints.clear();
        waypoints.push_back(target_pose1);
        fraction = this->move_group.computeCartesianPath(waypoints, this->eef_step, this->jump_threshold, this->trajectory, false);
        this->my_plan_.trajectory_ = this->trajectory;
        this->move_group.execute(this->my_plan_);
        printf("done moving nearer part...\n");
        ros::Duration(0.25).sleep();
        move_counter++;
    }
    printf("hopefully part attached now\n");
    this->trajectory_success = 0;
    mg_counter = 0;

    waypoints.clear();
    geometry_msgs::Pose target_pose2 = target_pose1;
    target_pose2.position.z += 0.5;  // Move away from part
    waypoints.push_back(target_pose2);

    geometry_msgs::Pose target_pose3 = target_pose2;

    fraction = this->move_group.computeCartesianPath(waypoints, this->eef_step, this->jump_threshold, this->trajectory, false);
    this->my_plan_.trajectory_ = this->trajectory;
    this->move_group.execute(this->my_plan_);
    while (this->trajectory_success < 1 && mg_counter < 10) {
        ros::Duration(0.25).sleep();
        mg_counter++;
    }
    printf("hopefully part attached AND picked up...\n");
    this->trajectory_success = 0;
    this->p_ = target_pose3;

    return 1;
}

int Gantry_Arm::pickup_part_bins(const geometry_msgs::TransformStamped & dest_pose_) {  // Pick up the part at the location defined by the msg
    // First, enable the gripper
    this->enable_gripper();

    // Then move near the part
    int mg_counter = 0;
    std::vector<geometry_msgs::Pose> waypoints;
    this->sp_ = this->move_group.getCurrentPose();
    this->p_ = this->sp_.pose;
    geometry_msgs::Pose target_pose = this->p_;
    geometry_msgs::Pose starting_pose = this->p_;
    // waypoints.push_back(target_pose);  // Always start with the current pose before adding additional poses

    geometry_msgs::Pose target_pose0 = target_pose;
    target_pose0.position.x = dest_pose_.transform.translation.x;
    target_pose0.position.y = dest_pose_.transform.translation.y;
    target_pose0.position.z = dest_pose_.transform.translation.z + 0.1;  // Move over part; was static value "0.805" when picking a battery off a bin
    waypoints.push_back(target_pose0);
    geometry_msgs::Pose target_pose01 = target_pose0;
    target_pose01.position.z = dest_pose_.transform.translation.z + 0.03;
    waypoints.push_back(target_pose01);
    double fraction = this->move_group.computeCartesianPath(waypoints, this->eef_step, this->jump_threshold, this->trajectory, false);
    this->my_plan_.trajectory_ = this->trajectory;
    this->move_group.execute(this->my_plan_);
    while (this->trajectory_success < 1 && mg_counter < 10) {
        ros::Duration(0.25).sleep();
        mg_counter++;
    }
    printf("done moving near part\n");
    this->trajectory_success = 0;
    mg_counter = 0;

    geometry_msgs::Pose target_pose1;
    target_pose1 = this->move_group.getCurrentPose().pose;
    std::vector<geometry_msgs::Pose> pickup_poses(15);
    for (int x = 0; x < 15; x++) {
        target_pose1.position.z -= 0.005;
        pickup_poses.at(x) = target_pose1;
    }
    fraction = this->move_group.computeCartesianPath(pickup_poses, this->eef_step, this->jump_threshold, this->trajectory, false);
    this->my_plan_.trajectory_ = this->trajectory;
    setAvgCartesianSpeed(this->my_plan_, "gantry_arm_ee_link", 0.01, "/ariac/gantry/robot_description");
    this->check_joints = true;
    this->move_group.execute(this->my_plan_);

    int move_counter =  0;
    while (!this->gripper_attached && move_counter < 15) {
        ros::Duration(0.25).sleep();
        move_counter++;
    }
    printf("hopefully part attached now\n");
    this->trajectory_success = 0;
    mg_counter = 0;

    waypoints.clear();
    waypoints.push_back(starting_pose);

    fraction = this->move_group.computeCartesianPath(waypoints, this->eef_step, this->jump_threshold, this->trajectory, false);
    this->my_plan_.trajectory_ = this->trajectory;
    this->move_group.execute(this->my_plan_);
    while (this->trajectory_success < 1 && mg_counter < 10) {
        ros::Duration(0.25).sleep();
        mg_counter++;
    }
    printf("hopefully part attached AND picked up...\n");
    this->trajectory_success = 0;
    this->p_ = starting_pose;

    return 1;
}

void Gantry_Arm::gantry_arm_joint_state_callback(const sensor_msgs::JointState::ConstPtr & joint_state_msg) {
    if (this->check_joints && this->gripper_attached) {
        this->clear_arm_goal();
        this->check_joints = false;
        printf("\n\nIN CALLBACK! Stopping motion...\n\n");
    }
}

void Gantry_Arm::clear_arm_goal() {
    // Send an empty trajectory message to stop execution immediately
    trajectory_msgs::JointTrajectory msg;

    msg.joint_names.clear();
    
    this->gantry_arm_joint_trajectory_publisher_.publish(msg);  
}

int Gantry_Arm::drop_part(const geometry_msgs::TransformStamped & dest_pose_) {  // Drop the part at the location defined by the msg
    // First move near the destination
    int mg_counter = 0;
    std::vector<geometry_msgs::Pose> waypoints;
    this->sp_ = this->move_group.getCurrentPose();
    this->p_ = this->sp_.pose;
    geometry_msgs::Pose target_pose = this->p_;
    // waypoints.push_back(target_pose);  // Always start with the current pose before adding additional poses

    geometry_msgs::Pose target_pose0 = target_pose;
    target_pose0.position.x = dest_pose_.transform.translation.x;
    target_pose0.position.y = dest_pose_.transform.translation.y;
    target_pose0.position.z = dest_pose_.transform.translation.z + 0.2;  // Move over destination; was "z+0.1" in original code
    // target_pose0.orientation.x = dest_pose_.transform.rotation.x;
    // target_pose0.orientation.y = dest_pose_.transform.rotation.y;
    // target_pose0.orientation.z = dest_pose_.transform.rotation.z;
    // target_pose0.orientation.w = dest_pose_.transform.rotation.w;
    waypoints.push_back(target_pose0);
    double fraction = this->move_group.computeCartesianPath(waypoints, this->eef_step, this->jump_threshold, this->trajectory, false);
    this->my_plan_.trajectory_ = this->trajectory;
    this->move_group.execute(this->my_plan_);
    while (this->trajectory_success < 1 && mg_counter < 10) {
        ros::Duration(0.25).sleep();
        mg_counter++;
    }
    printf("done moving near destination, z: %f\n", target_pose0.position.z);
    this->trajectory_success = 0;
    mg_counter = 0;

    waypoints.clear();
    geometry_msgs::Pose target_pose01 = target_pose0;
    std::cout << "dest_pose_.transform.translation.z value before adding: " << dest_pose_.transform.translation.z << "\n";
    target_pose01.position.z = dest_pose_.transform.translation.z + 0.01;  // Move closer to drop position
    std::cout << "target_pose01 z value: " << target_pose01.position.z << "\n";
    waypoints.push_back(target_pose01);
    fraction = this->move_group.computeCartesianPath(waypoints, this->eef_step, this->jump_threshold, this->trajectory, false);
    this->my_plan_.trajectory_ = this->trajectory;
    this->move_group.execute(this->my_plan_);
    while (this->trajectory_success < 1 && mg_counter < 10) {
        ros::Duration(0.25).sleep();
        mg_counter++;
    }
    this->sp_ = this->move_group.getCurrentPose();
    this->p_ = this->sp_.pose;
    geometry_msgs::Pose target_posed = this->p_;
    printf("done lowering part to destination: %f\n", target_posed.position.z);
    this->trajectory_success = 0;
    mg_counter = 0;

    waypoints.clear();
    geometry_msgs::Pose target_pose01a = target_pose01;
    target_pose01a.position.z = dest_pose_.transform.translation.z + 0.01;  // Move closer to drop position
    waypoints.push_back(target_pose01a);
    fraction = this->move_group.computeCartesianPath(waypoints, this->eef_step, this->jump_threshold, this->trajectory, false);
    this->my_plan_.trajectory_ = this->trajectory;
    this->move_group.execute(this->my_plan_);
    while (this->trajectory_success < 1 && mg_counter < 10) {
        ros::Duration(0.25).sleep();
        mg_counter++;
    }
    this->sp_ = this->move_group.getCurrentPose();
    this->p_ = this->sp_.pose;
    target_posed = this->p_;
    printf("done lowering part to destination: %f\n", target_posed.position.z);
    this->trajectory_success = 0;
    mg_counter = 0;

    // Then, disable the gripper to drop the part
    this->disable_gripper();
    printf("hopefully part detached now\n");

    waypoints.clear();
    geometry_msgs::Pose target_pose01b = target_pose01a;
    target_pose01b.position.z += 0.1;  // Move away from part
    waypoints.push_back(target_pose01b);

    fraction = this->move_group.computeCartesianPath(waypoints, this->eef_step, this->jump_threshold, this->trajectory, false);
    this->my_plan_.trajectory_ = this->trajectory;
    this->move_group.execute(this->my_plan_);
    while (this->trajectory_success < 1 && mg_counter < 10) {
        ros::Duration(0.5).sleep();
        mg_counter++;
    }
    printf("hopefully part detached AND left in correct location...\n");
    this->trajectory_success = 0;
    this->sp_ = this->move_group.getCurrentPose();
    this->p_ = this->sp_.pose;

    return 1;
}

int Gantry_Arm::drop_part_agv(const geometry_msgs::TransformStamped & dest_pose_) {  // Drop the part at the location defined by the msg
    // First move near the destination
    int mg_counter = 0;
    std::vector<geometry_msgs::Pose> waypoints;
    this->sp_ = this->move_group.getCurrentPose();
    this->p_ = this->sp_.pose;
    geometry_msgs::Pose target_pose = this->p_;
    geometry_msgs::Pose starting_pose = this->p_;
    // waypoints.push_back(target_pose);  // Always start with the current pose before adding additional poses

    geometry_msgs::Pose target_pose0 = target_pose;
    target_pose0.position.x = dest_pose_.transform.translation.x;
    target_pose0.position.y = dest_pose_.transform.translation.y;
    target_pose0.position.z = dest_pose_.transform.translation.z + 0.2;  // Move over destination; was "z+0.1" in original code
    // target_pose.orientation.x = dest_pose_.transform.rotation.x;  // need to maintain original orientation else the gripper turns on its side for some reason
    // target_pose.orientation.y = dest_pose_.transform.rotation.y;
    // target_pose.orientation.z = dest_pose_.transform.rotation.z;
    // target_pose.orientation.w = dest_pose_.transform.rotation.w;
    waypoints.push_back(target_pose0);
    geometry_msgs::Pose target_pose01a = target_pose0;
    target_pose01a.position.z = dest_pose_.transform.translation.z + 0.02;
    waypoints.push_back(target_pose01a);
    double fraction = this->move_group.computeCartesianPath(waypoints, this->eef_step, this->jump_threshold, this->trajectory, false);
    this->my_plan_.trajectory_ = this->trajectory;
    this->move_group.execute(this->my_plan_);
    while (this->trajectory_success < 1 && mg_counter < 10) {
        ros::Duration(0.25).sleep();
        mg_counter++;
    }
    printf("done moving near destination, z: %f\n", target_pose0.position.z);
    this->trajectory_success = 0;
    mg_counter = 0;

    // Then, disable the gripper to drop the part
    this->disable_gripper();
    printf("hopefully part detached now\n");

    waypoints.clear();
    waypoints.push_back(starting_pose);

    fraction = this->move_group.computeCartesianPath(waypoints, this->eef_step, this->jump_threshold, this->trajectory, false);
    this->my_plan_.trajectory_ = this->trajectory;
    this->move_group.execute(this->my_plan_);
    while (this->trajectory_success < 1 && mg_counter < 10) {
        ros::Duration(0.25).sleep();
        mg_counter++;
    }
    printf("hopefully part detached AND left in correct location...\n");
    this->trajectory_success = 0;
    this->sp_ = this->move_group.getCurrentPose();
    this->p_ = this->sp_.pose;

    return 1;
}

int Gantry_Arm::ZeroArm() {
    // const robot_state::JointModelGroup* joint_model_group = this->move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GANTRY);
    current_state = this->move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    joint_group_positions = jgp_ga_as_agv;
    this->move_group.setJointValueTarget(joint_group_positions);

    bool success = (this->move_group.plan(this->my_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    this->move_group.move();
    printf("Visualizing plan 2 (joint space goal) %s\n", success ? "Move Success" : "FAILED");

    this->sp_ = this->move_group.getCurrentPose();
    this->p_ = this->sp_.pose;

    return 1;
}

int Gantry_Arm::CaseArm() {
    current_state = this->move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    joint_group_positions = jgp_ga_case;
    this->move_group.setJointValueTarget(joint_group_positions);

    bool success = (this->move_group.plan(this->my_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    this->move_group.move();
    
    this->sp_ = this->move_group.getCurrentPose();
    this->p_ = this->sp_.pose;

    return 1;
}
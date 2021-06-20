#include <map>
#include <vector>
#include "cmath"
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
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

int Kitting::move_near(const geometry_msgs::TransformStamped & dest_pose_) {
    std::vector<double> joint_group_positions;
    double xaxis = dest_pose_.transform.translation.x;
    double yaxis = dest_pose_.transform.translation.y;
    if (xaxis > -1.3) {
        joint_group_positions = jgp_kit_conveyor;
        joint_group_positions[0] = yaxis;
    } else {
        if (yaxis > 4.5) joint_group_positions = jgp_kit_agv1;
        else if (yaxis > 2.0) joint_group_positions = jgp_kit_bins14;
        else if (yaxis > 0.5) joint_group_positions = jgp_kit_agv2;
        else if (yaxis > -0.5) joint_group_positions = jgp_kit_badparts;
        else if (yaxis > -1.5) joint_group_positions = jgp_kit_agv3;
        else if (yaxis > -4.0) joint_group_positions = jgp_kit_bins58;
        else joint_group_positions = jgp_kit_agv4;
    }

    this->move_group.setJointValueTarget(joint_group_positions);
    bool success = (this->move_group.plan(this->my_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    this->move_group.move();

    printf("Hopefully done with move_near now.\n");

    this->sp_ = this->move_group.getCurrentPose();
    this->p_ = this->sp_.pose;

    return 1;
}

int Kitting::move_near(const geometry_msgs::Pose & dest_pose_) {
    std::vector<double> joint_group_positions;
    double xaxis = dest_pose_.position.x;
    double yaxis = dest_pose_.position.y;
    if (xaxis > -1.3) {
        joint_group_positions = jgp_kit_conveyor;
        joint_group_positions[0] = yaxis;
    } else {
        if (yaxis > 4.5) joint_group_positions = jgp_kit_agv1;
        else if (yaxis > 2.0) joint_group_positions = jgp_kit_bins14;
        else if (yaxis > 0.5) joint_group_positions = jgp_kit_agv2;
        else if (yaxis > -0.5) joint_group_positions = jgp_kit_badparts;
        else if (yaxis > -1.5) joint_group_positions = jgp_kit_agv3;
        else if (yaxis > -4.0) joint_group_positions = jgp_kit_bins58;
        else joint_group_positions = jgp_kit_agv4;
    }

    this->move_group.setJointValueTarget(joint_group_positions);
    bool success = (this->move_group.plan(this->my_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    this->move_group.move();

    printf("Hopefully done with move_near now.\n");

    this->sp_ = this->move_group.getCurrentPose();
    this->p_ = this->sp_.pose;

    return 1;
}

int Kitting::pickup_part(const geometry_msgs::TransformStamped & dest_pose_) {  // Pick up the part at the location defined by the msg
    // First, enable the gripper
    this->enable_gripper();

    // Then move near the part
    int mg_counter = 0;
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose target_pose = this->p_;
    geometry_msgs::Pose starting_pose = this->p_;

    geometry_msgs::Pose target_pose0 = target_pose;
    target_pose0.position.y = dest_pose_.transform.translation.y;  // Just move to it on the y-axis
    waypoints.push_back(target_pose0); 

    geometry_msgs::Pose target_pose1 = target_pose0;
    target_pose1.position.x = dest_pose_.transform.translation.x;
    target_pose1.position.z = dest_pose_.transform.translation.z+.1;  // Move over part
    waypoints.push_back(target_pose1);

    geometry_msgs::Pose target_pose2 = target_pose1;
    target_pose2.position.z = dest_pose_.transform.translation.z + 0.005;  // Move over part; was static value "0.805" when picking a battery off a bin
    waypoints.push_back(target_pose2);
    double fraction = this->move_group.computeCartesianPath(waypoints, this->eef_step, this->jump_threshold, this->trajectory);
    this->my_plan_.trajectory_ = this->trajectory;
    this->move_group.execute(this->my_plan_);
    while (this->trajectory_success < 1 && mg_counter < 10) {
        ros::Duration(0.5).sleep();
        mg_counter++;
    }
    printf("done moving near part\n");
    this->trajectory_success = 0;
    mg_counter = 0;

    geometry_msgs::Pose target_pose3;
    target_pose3 = this->move_group.getCurrentPose().pose;
    std::vector<geometry_msgs::Pose> pickup_poses(15);
    for (int x = 0; x < 15; x++) {
        target_pose3.position.z -= 0.005;
        pickup_poses.at(x) = target_pose3;
    }
    fraction = this->move_group.computeCartesianPath(pickup_poses, this->eef_step, this->jump_threshold, this->trajectory, false);
    this->my_plan_.trajectory_ = this->trajectory;
    setAvgCartesianSpeed(this->my_plan_, "ee_link", 0.01, "/ariac/kitting/robot_description");
    this->check_joints2 = true;
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
        ros::Duration(0.5).sleep();
        mg_counter++;
    }
    printf("hopefully part attached AND picked up...\n");
    this->trajectory_success = 0;
    this->p_ = starting_pose;

    return 1;
}

int Kitting::drop_part(const geometry_msgs::TransformStamped & dest_pose_) {  // Drop the part at the location defined by the msg
    // First move near the destination
    int mg_counter = 0;
    std::vector<geometry_msgs::Pose> waypoints;
    this->sp_ = this->move_group.getCurrentPose();
    this->p_ = this->sp_.pose;
    geometry_msgs::Pose target_pose = this->p_;
    // waypoints.push_back(target_pose);  // Always start with the current pose before adding additional poses

    geometry_msgs::Pose target_pose0 = target_pose;
    target_pose0.position.y = dest_pose_.transform.translation.y;  // Just move to it on the y-axis
    // target_pose0.orientation.x = dest_pose_.transform.rotation.x;
    // target_pose0.orientation.y = dest_pose_.transform.rotation.y;
    // target_pose0.orientation.z = dest_pose_.transform.rotation.z;
    // target_pose0.orientation.w = dest_pose_.transform.rotation.w;
    waypoints.push_back(target_pose0); 

    geometry_msgs::Pose target_pose1 = target_pose0;
    target_pose1.position.x = dest_pose_.transform.translation.x;
    target_pose1.position.z = dest_pose_.transform.translation.z+.1;  // Move over part
    waypoints.push_back(target_pose1);

    geometry_msgs::Pose target_pose2 = target_pose1;
    target_pose2.position.z = dest_pose_.transform.translation.z + 0.02;  // Move over destination; was "z+0.1" in original code
    // std::cout << "Transform.translation in drop_part (should be 0.904, then 0.824): " << target_pose1.position.z << ", " << target_pose2.position.z << "\n";
    // target_pose2.position.z = 0.89;  // More general position?
    waypoints.push_back(target_pose2);
    double fraction = this->move_group.computeCartesianPath(waypoints, this->eef_step, this->jump_threshold, this->trajectory, false);
    this->my_plan_.trajectory_ = this->trajectory;
    this->move_group.execute(this->my_plan_);
    while (this->trajectory_success < 1 && mg_counter < 10) {
        ros::Duration(0.25).sleep();
        mg_counter++;
    }
    printf("done moving near destination\n");
    this->trajectory_success = 0;
    mg_counter = 0;

    this->sp_ = this->move_group.getCurrentPose();
    this->p_ = this->sp_.pose;

    // std::cout << "Transform.translation in drop_part AFTER MOVE TO 0.824: " << this->p_.position.z << "\n";

    // ros::Duration(10).sleep();

    // Then, disable the gripper to drop the part
    this->disable_gripper();
    printf("hopefully part detached now\n");

    this->move_near(dest_pose_);

    printf("hopefully part detached AND left in correct location...\n");
    this->trajectory_success = 0;
    
    return 1;
}

int Kitting::move_bin_side() {
    current_state = this->move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(this->joint_model_group, joint_group_positions);
    std::cout << "In move_bin_side, joint positions: " << joint_group_positions[0] << ", " << joint_group_positions[1] << ", " << joint_group_positions[2] << ", " << joint_group_positions[3] << ", " << joint_group_positions[4] << ", " << joint_group_positions[5] << ", " << joint_group_positions[6] << "\n";
    joint_group_positions = jgp_kit_neutral;
    this->move_group.setJointValueTarget(joint_group_positions);

    bool success = (this->move_group.plan(this->my_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    this->move_group.move();
    printf("Visualizing plan 2 (joint space goal) %s\n", success ? "Move Success" : "FAILED");

    std::cout << "In move_bin_side, joint positions AFTER move: " << joint_group_positions[0] << ", " << joint_group_positions[1] << ", " << joint_group_positions[2] << ", " << joint_group_positions[3] << ", " << joint_group_positions[4] << ", " << joint_group_positions[5] << ", " << joint_group_positions[6] << "\n";

    this->sp_ = this->move_group.getCurrentPose();
    this->p_ = this->sp_.pose;

    printf("End effector pose->position (x,y,z): (%f,%f,%f)\n", this->p_.position.x, this->p_.position.y, this->p_.position.z);
    printf("End effector pose->orientation (x,y,z,w): (%f,%f,%f,%f)\n", this->p_.orientation.x, this->p_.orientation.y, this->p_.orientation.z, this->p_.orientation.w);

    return 1;
}

int Kitting::ZeroArm() {
    current_state = this->move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    joint_group_positions = jgp_kit_conveyor;
    this->move_group.setJointValueTarget(joint_group_positions);

    bool success = (this->move_group.plan(this->my_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    this->move_group.move();
    printf("Visualizing plan 2 (joint space goal) %s\n", success ? "Move Success" : "FAILED");

    this->sp_ = this->move_group.getCurrentPose();
    this->p_ = this->sp_.pose;

    return 1;
}

void Kitting::move_joints(const std::vector<std::vector<double>> & arm_joints, const std::vector<double> & times, const std::vector<double> & linear_stops) {
    trajectory_msgs::JointTrajectory msg;

    // Fill the names of the joints to be controlled.
    // Note that the vacuum_gripper_joint is not controllable.
    msg.joint_names.clear();
    msg.joint_names.push_back("shoulder_pan_joint");
    msg.joint_names.push_back("shoulder_lift_joint");
    msg.joint_names.push_back("elbow_joint");
    msg.joint_names.push_back("wrist_1_joint");
    msg.joint_names.push_back("wrist_2_joint");
    msg.joint_names.push_back("wrist_3_joint");
    msg.joint_names.push_back("linear_arm_actuator_joint");
    
    msg.points.resize(arm_joints.size());
    
    for(int i=0;i<arm_joints.size();i++){
      msg.points[i].positions.resize(msg.joint_names.size());
      for(int j=0;j<6;j++){
        msg.points[i].positions[j] = arm_joints[i][j];
      }
      msg.points[i].positions[6] = linear_stops[i];
      msg.points[i].time_from_start = ros::Duration(times[i]);
    }

    this->kitting_joint_trajectory_publisher_.publish(msg);
}

void Kitting::kitting_joint_state_callback(const sensor_msgs::JointState::ConstPtr & joint_state_msg) {
    
    if(this->watch_linac){
        sensor_msgs::JointState arm_current_joint_states_;
        arm_current_joint_states_ = *joint_state_msg;
        double arm_linear = arm_current_joint_states_.position[1]; //linear
        // std::cout << "In callback, destination: " << this->linac_destination << ", current: " <<  arm_linear << ", diff: " << std::abs(arm_linear- this->linac_destination) << "\n";
        if(std::abs(arm_linear- this->linac_destination) < .001) this->watch_linac = false;
    }
    if (this->check_joints && this->gripper_attached) {
        this->check_joints = false;
        printf("\n\nIN CALLBACK!\n\n");
        sensor_msgs::JointState arm_current_joint_states_;
        arm_current_joint_states_ = *joint_state_msg;
        double arm_linear = arm_current_joint_states_.position[1]; //here only, 1=linear actuator
        std::cout << arm_linear << "\n";
        // clear_arm_goal();  // Stop existing trajectory...works, but code in hw_example_node.cpp
        this->move_joints(std::vector<std::vector<double>>{
            {0.0299997, -1.33945, 1.94756, -2.17855, -1.57991, 0.00}
            }, std::vector<double>{0.75}, 
            std::vector<double>{ arm_linear });
        this->attached_ready = true;   
    }
    if (this->check_joints2 && this->gripper_attached) {
        this->clear_arm_goal();
        this->check_joints2 = false;
        printf("\n\nIN CALLBACK! Stopping motion...\n\n");
    }
}

void Kitting::clear_arm_goal() {
    // Send an empty trajectory message to stop execution immediately
    trajectory_msgs::JointTrajectory msg;

    msg.joint_names.clear();
    
    this->kitting_joint_trajectory_publisher_.publish(msg);  
}
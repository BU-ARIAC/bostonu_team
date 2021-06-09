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

    // std::vector<geometry_msgs::Pose> waypoints;
    // geometry_msgs::Pose target_pose = this->p_;
    // waypoints.push_back(target_pose);  // Always start with the current pose before adding additional poses

    // geometry_msgs::Pose target_pose0 = target_pose;
    // target_pose0.position.y = dest_pose_.transform.translation.y;  // Just move to it on the y-axis
    // waypoints.push_back(target_pose0); 

    // geometry_msgs::Pose target_pose1 = target_pose0;
    // target_pose1.position.x = dest_pose_.transform.translation.x;
    // target_pose1.position.z = dest_pose_.transform.translation.z+.1;  // Move over part
    // waypoints.push_back(target_pose1);

    // double fraction = this->move_group.computeCartesianPath(waypoints, this->eef_step, this->jump_threshold, this->trajectory);
    // this->my_plan_.trajectory_ = this->trajectory;
    // this->move_group.execute(this->my_plan_);
    // printf("done moving near location\n");

    // this->p_ = target_pose1;

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

    // std::vector<geometry_msgs::Pose> waypoints;
    // geometry_msgs::Pose target_pose = this->p_;
    // waypoints.push_back(target_pose);  // Always start with the current pose before adding additional poses

    // geometry_msgs::Pose target_pose0 = target_pose;
    // target_pose0.position.y = dest_pose_.position.y;  // Just move to it on the y-axis
    // waypoints.push_back(target_pose0); 

    // geometry_msgs::Pose target_pose1 = target_pose0;
    // target_pose1.position.x = dest_pose_.position.x;
    // target_pose1.position.z = dest_pose_.position.z+.1;  // Move over part
    // waypoints.push_back(target_pose1);

    // waypoints.push_back(dest_pose_);

    // double fraction = this->move_group.computeCartesianPath(waypoints, this->eef_step, this->jump_threshold, this->trajectory);
    // this->my_plan_.trajectory_ = this->trajectory;
    // this->move_group.execute(this->my_plan_);
    // printf("done moving near location\n");

    // this->p_ = target_pose1;

    return 1;
}

int Kitting::pickup_part(const geometry_msgs::TransformStamped & dest_pose_) {  // Pick up the part at the location defined by the msg
    // First, enable the gripper
    this->enable_gripper();

    // Then move near the part
    int mg_counter = 0;
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose target_pose = this->p_;
    // waypoints.push_back(target_pose);  // Always start with the current pose before adding additional poses

    geometry_msgs::Pose target_pose0 = target_pose;
    target_pose0.position.y = dest_pose_.transform.translation.y;  // Just move to it on the y-axis
    waypoints.push_back(target_pose0); 

    geometry_msgs::Pose target_pose1 = target_pose0;
    target_pose1.position.x = dest_pose_.transform.translation.x;
    target_pose1.position.z = dest_pose_.transform.translation.z+.1;  // Move over part
    waypoints.push_back(target_pose1);

    geometry_msgs::Pose target_pose2 = target_pose1;
    target_pose2.position.z = dest_pose_.transform.translation.z + 0.03;  // Move over part; was static value "0.805" when picking a battery off a bin
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

    geometry_msgs::Pose target_pose3 = target_pose2;
    int move_counter =  0;
    while (!this->gripper_attached && move_counter < 10) {
        target_pose3.position.z -= 0.002;  // Move closer over part
        waypoints.clear();
        waypoints.push_back(target_pose3);
        fraction = this->move_group.computeCartesianPath(waypoints, this->eef_step, this->jump_threshold, this->trajectory, false);
        this->my_plan_.trajectory_ = this->trajectory;
        this->move_group.execute(this->my_plan_);
        std::cout << "done moving nearer part... " << target_pose3.position.z << "\n";
        ros::Duration(0.25).sleep();
        move_counter++;
    }
    printf("hopefully part attached now\n");
    this->trajectory_success = 0;
    mg_counter = 0;

    waypoints.clear();
    geometry_msgs::Pose target_pose4 = target_pose3;
    target_pose4.position.z += 0.1;  // Move away from part
    waypoints.push_back(target_pose4);

    geometry_msgs::Pose target_pose5 = target_pose;
    // target_pose5.position.z += 0.1;  // Move more away from part
    // target_pose5.position.x += 0.1;
    waypoints.push_back(target_pose5);

    fraction = this->move_group.computeCartesianPath(waypoints, this->eef_step, this->jump_threshold, this->trajectory, false);
    this->my_plan_.trajectory_ = this->trajectory;
    this->move_group.execute(this->my_plan_);
    while (this->trajectory_success < 1 && mg_counter < 10) {
        ros::Duration(0.5).sleep();
        mg_counter++;
    }
    printf("hopefully part attached AND picked up...\n");
    this->trajectory_success = 0;
    this->p_ = target_pose5;

    return 1;
}

int Kitting::drop_part(const geometry_msgs::TransformStamped & dest_pose_) {  // Drop the part at the location defined by the msg
    // First move near the destination
    int mg_counter = 0;
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose target_pose = this->p_;
    // waypoints.push_back(target_pose);  // Always start with the current pose before adding additional poses

    geometry_msgs::Pose target_pose0 = target_pose;
    target_pose0.position.y = dest_pose_.transform.translation.y;  // Just move to it on the y-axis
    waypoints.push_back(target_pose0); 

    geometry_msgs::Pose target_pose1 = target_pose0;
    target_pose1.position.x = dest_pose_.transform.translation.x;
    target_pose1.position.z = dest_pose_.transform.translation.z+.1;  // Move over part
    waypoints.push_back(target_pose1);

    geometry_msgs::Pose target_pose2 = target_pose1;
    // target_pose2.position.z = dest_pose_.transform.translation.z + 0.08;  // Move over destination; was "z+0.1" in original code
    target_pose2.position.z = 0.89;  // More general position?
    waypoints.push_back(target_pose2);
    double fraction = this->move_group.computeCartesianPath(waypoints, this->eef_step, this->jump_threshold, this->trajectory, false);
    this->my_plan_.trajectory_ = this->trajectory;
    this->move_group.execute(this->my_plan_);
    while (this->trajectory_success < 1 && mg_counter < 10) {
        ros::Duration(0.5).sleep();
        mg_counter++;
    }
    printf("done moving near destination\n");
    this->trajectory_success = 0;
    mg_counter = 0;

    // Then, disable the gripper to drop the part
    this->disable_gripper();
    printf("hopefully part detached now\n");

    this->move_near(dest_pose_);

    printf("hopefully part detached AND left in correct location...\n");
    this->trajectory_success = 0;
    
    return 1;
}

// int Kitting::move_bin_side() {  // Move the kitting arm to the bin side of the linear actuator
//     // End effector pose->position (x,y,z): (-2.047874,-0.167802,1.382357)
//     // End effector pose->orientation (x,y,z,w): (-0.677523,-0.002440,0.735486,0.004078)
//     printf("In move_bin_side:\n");
//     printf("End effector pose->position (x,y,z): (%f,%f,%f)\n", this->p_.position.x, this->p_.position.y, this->p_.position.z);
//     printf("End effector pose->orientation (x,y,z,w): (%f,%f,%f,%f)\n", this->p_.orientation.x, this->p_.orientation.y, this->p_.orientation.z, this->p_.orientation.w);

//     // First, swing around to the agv side
//     // this->move_group.setEndEffectorLink("ee_link");  // Made no difference
//     this->move_group.setStartState(*this->move_group.getCurrentState());
//     int mg_counter = 0;
//     geometry_msgs::Pose target_pose;
//     std::cout << "in move_bin_side, p_.x = " << this->p_.position.x << "\n";
//     target_pose.position.x = -2.047874;
//     // target_pose.position.x = -2.26;
//     target_pose.position.y = -0.167802;
//     target_pose.position.z = 1.382357;
//     target_pose.orientation.x = -0.677523;
//     target_pose.orientation.y = -0.002440;
//     target_pose.orientation.z = 0.735486;
//     target_pose.orientation.w = 0.004078;
//     this->move_group.setPoseTarget(target_pose);
//     // this->move_group.setGoalTolerance(0.01);  // Made no difference
//     // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//     bool success = (this->move_group.plan(this->my_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     this->move_group.move();
//     while (this->move_group_success < 2 && mg_counter < 10) {
//         ros::Duration(0.5).sleep();
//         mg_counter++;
//     }
//     printf("move 1 done...\n");
//     this->move_group_success = 0;
//     // mg_counter = 0;

//     // target_pose.position.x = -1.3;  // Swing around to AGV side
//     // target_pose.position.y = 0.9;  // Swing around to AGV side
//     // this->move_group.setPoseTarget(target_pose);
//     // success = (this->move_group.plan(this->my_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     // this->move_group.move();
//     // while (this->move_group_success < 2 && mg_counter < 10) {
//     //     ros::Duration(0.5).sleep();
//     //     mg_counter++;
//     // }
//     // printf("move 2 done...\n");
//     // this->move_group_success = 0;
//     // mg_counter = 0;

//     // target_pose.position.x = -2.26;  // Swing around to AGV side
//     // this->move_group.setPoseTarget(target_pose);
//     // success = (this->move_group.plan(this->my_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     // this->move_group.move();
//     // while (this->move_group_success < 2 && mg_counter < 10) {
//     //     ros::Duration(0.5).sleep();
//     //     mg_counter++;
//     // }
//     // printf("swing to agv done...\n");
//     // this->move_group_success = 0;
//     this->p_ = target_pose;
//     return 1;
// }

int Kitting::move_bin_side() {
    // const robot_state::JointModelGroup* joint_model_group = this->move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP_KITTING);
    // moveit::core::RobotStatePtr current_state = this->move_group.getCurrentState();
    current_state = this->move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(this->joint_model_group, joint_group_positions);
    std::cout << "In move_bin_side, joint positions: " << joint_group_positions[0] << ", " << joint_group_positions[1] << ", " << joint_group_positions[2] << ", " << joint_group_positions[3] << ", " << joint_group_positions[4] << ", " << joint_group_positions[5] << ", " << joint_group_positions[6] << "\n";
    // joint_group_positions[0] = -4.60;  // Linear actuator
    // joint_group_positions[1] = 3.14;
    // joint_group_positions[2] = -1.38;
    // joint_group_positions[4] = 4.15;
    // joint_group_positions[5] = -1.58;
    joint_group_positions = jgp_kit_neutral;
    // Trying this to move ALLL THE WAY down to a part...
    // In main, post move, joint positions: 2.43539, 2.83001, -0.15041, 0.458681, 4.44882, -1.59301, -0.30887
    // joint_group_positions[0] = 2.43539;
    // joint_group_positions[1] = 2.83001;
    // joint_group_positions[2] = -0.15041;
    // joint_group_positions[3] = 0.458681;
    // joint_group_positions[4] = 4.44882;
    // joint_group_positions[5] = -1.59301;
    // joint_group_positions[6] = -0.30887;
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
    // const robot_state::JointModelGroup* joint_model_group = this->move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GANTRY);
    current_state = this->move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    // std::cout << "In zeroarm, joint positions: " << joint_group_positions[0] << ", " << joint_group_positions[1] << ", " << joint_group_positions[2] << ", " << joint_group_positions[3] << ", " << joint_group_positions[4] << ", " << joint_group_positions[5] << "\n";
    // joint_group_positions[3] = 2.39;
    // joint_group_positions[4] = -1.58;
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
    // if (this->check_joints && this->testing < 120) {
    //     this->testing+=1;
    //     printf("\n in callback: %d\n", this->testing);
    // } else if (this->check_joints && this->testing >= 120) {
    //     this->check_joints = false;
    //     printf("\n\nIN CALLBACK!\n\n");
    //     sensor_msgs::JointState arm_current_joint_states_;
    //     arm_current_joint_states_ = *joint_state_msg;
    //     double arm_linear = arm_current_joint_states_.position[1]; //linear
    //     // clear_arm_goal();  // Stop existing trajectory...works, but code in hw_example_node.cpp
    //     this->move_joints(std::vector<std::vector<double>>{
    //         {0.0299997, -1.33945, 1.94756, -2.17855, -1.57991, -0.00782045}
    //         }, std::vector<double>{0.5}, 
    //         std::vector<double>{ arm_linear });   
    // }
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
}

void Kitting::clear_arm_goal() {
    // Send an empty trajectory message to stop execution immediately
    trajectory_msgs::JointTrajectory msg;

    msg.joint_names.clear();
    
    this->kitting_joint_trajectory_publisher_.publish(msg);  
}
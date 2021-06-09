// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// %Tag(FULLTEXT)%
// %Tag(INCLUDE_STATEMENTS)%
#include <algorithm>
#include <vector>
#include <unistd.h>  // Temp to enable sleeping
#include <string>
#include <sstream>
#include <cmath>
#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <limits>
#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include <nist_gear/Proximity.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud.h>  // Used for the depth cameras
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <nist_gear/VacuumGripperControl.h>
#include <nist_gear/VacuumGripperState.h>
#include <nist_gear/AGVControl.h>
#include <nist_gear/AGVToAssemblyStation.h>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/LinearMath/Quaternion.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_msgs/ExecuteTrajectoryActionResult.h>

#include <moveit/kinematic_constraints/utils.h>

#include "part_mgmt.h"
#include "order_mgmt.h"
#include "robot_mgmt.h"
#include <moveit/robot_model_loader/robot_model_loader.h>

struct conveyor_part{
    ros::Time time_acquired;
    geometry_msgs::TransformStamped part_pose;
};
std::map<std::string,std::vector<conveyor_part>> conveyor_parts;
bool watch_linac = false;
double linac_destination;
std::map<std::string, int> bin_part_count;
std::map<std::string, int> conveyor_part_count;

// %EndTag(INCLUDE_STATEMENTS)%

// %Tag(START_COMP)%
/// Start the competition by waiting for and then calling the start ROS Service.
void start_competition(ros::NodeHandle & node) {
  // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient start_client =
    node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
  if (!start_client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }
  ROS_INFO("Requesting competition start...");
  std_srvs::Trigger srv;  // Combination of the "request" and the "response".
  start_client.call(srv);  // Call the start Service.
  if (!srv.response.success) {  // If not successful, print out why.
    ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
  } else {
    ROS_INFO("Competition started!");
  }
}
// %EndTag(START_COMP)%

void end_competition(ros::NodeHandle & node) {
  // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient start_client =
    node.serviceClient<std_srvs::Trigger>("/ariac/end_competition");
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
  if (!start_client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }
  ROS_INFO("Requesting competition end...");
  std_srvs::Trigger srv;  // Combination of the "request" and the "response".
  start_client.call(srv);  // Call the start Service.
  if (!srv.response.success) {  // If not successful, print out why.
    ROS_ERROR_STREAM("Failed to end the competition: " << srv.response.message);
  } else {
    ROS_INFO("Competition ended");
  }
}

void setAvgCartesianSpeed(moveit::planning_interface::MoveGroupInterface::Plan &plan, const std::string end_effector, const double speed)
{
    robot_model_loader::RobotModelLoader robot_model_loader("/ariac/kitting/robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
 
    int num_waypoints = plan.trajectory_.joint_trajectory.points.size();                                //gets the number of waypoints in the trajectory
    const std::vector<std::string> joint_names = plan.trajectory_.joint_trajectory.joint_names;    //gets the names of the joints being updated in the trajectory
 
    //set joint positions of zeroth waypoint
    kinematic_state->setVariablePositions(joint_names, plan.trajectory_.joint_trajectory.points.at(0).positions);
 
    Eigen::Affine3d current_end_effector_state = kinematic_state->getGlobalLinkTransform(end_effector);
    Eigen::Affine3d next_end_effector_state;
    double euclidean_distance, new_timestamp, old_timestamp, q1, q2, q3, dt1, dt2, v1, v2, a;
    trajectory_msgs::JointTrajectoryPoint *prev_waypoint, *curr_waypoint, *next_waypoint;
 
    for(int i = 0; i < num_waypoints - 1; i++)      //loop through all waypoints
    {
        curr_waypoint = &plan.trajectory_.joint_trajectory.points.at(i);
        next_waypoint = &plan.trajectory_.joint_trajectory.points.at(i+1);
         
        //set joints for next waypoint
        kinematic_state->setVariablePositions(joint_names, next_waypoint->positions);
 
        //do forward kinematics to get cartesian positions of end effector for next waypoint
        next_end_effector_state = kinematic_state->getGlobalLinkTransform(end_effector);
 
        //get euclidean distance between the two waypoints
        euclidean_distance = pow(pow(next_end_effector_state.translation()[0] - current_end_effector_state.translation()[0], 2) + 
                            pow(next_end_effector_state.translation()[1] - current_end_effector_state.translation()[1], 2) + 
                            pow(next_end_effector_state.translation()[2] - current_end_effector_state.translation()[2], 2), 0.5);
 
        new_timestamp = curr_waypoint->time_from_start.toSec() + (euclidean_distance / speed);      //start by printing out all 3 of these!
        old_timestamp = next_waypoint->time_from_start.toSec();
 
        //update next waypoint timestamp & joint velocities/accelerations if joint velocity/acceleration constraints allow
        if(new_timestamp > old_timestamp)
            next_waypoint->time_from_start.fromSec(new_timestamp);
        else
        {
            //ROS_WARN_NAMED("setAvgCartesianSpeed", "Average speed is too fast. Moving as fast as joint constraints allow.");
        }
         
        //update current_end_effector_state for next iteration
        current_end_effector_state = next_end_effector_state;
    }
     
    //now that timestamps are updated, update joint velocities/accelerations (used updateTrajectory from iterative_time_parameterization as a reference)
    for(int i = 0; i < num_waypoints; i++)
    {
        curr_waypoint = &plan.trajectory_.joint_trajectory.points.at(i);            //set current, previous & next waypoints
        if(i > 0)
            prev_waypoint = &plan.trajectory_.joint_trajectory.points.at(i-1);
        if(i < num_waypoints-1)
            next_waypoint = &plan.trajectory_.joint_trajectory.points.at(i+1);
 
        if(i == 0)          //update dt's based on waypoint (do this outside of loop to save time)
            dt1 = dt2 = next_waypoint->time_from_start.toSec() - curr_waypoint->time_from_start.toSec();
        else if(i < num_waypoints-1)
        {
            dt1 = curr_waypoint->time_from_start.toSec() - prev_waypoint->time_from_start.toSec();
            dt2 = next_waypoint->time_from_start.toSec() - curr_waypoint->time_from_start.toSec();
        }
        else
            dt1 = dt2 = curr_waypoint->time_from_start.toSec() - prev_waypoint->time_from_start.toSec();
 
        for(int j = 0; j < joint_names.size(); j++)     //loop through all joints in waypoint
        {
            if(i == 0)                      //first point
            {
                q1 = next_waypoint->positions.at(j);
                q2 = curr_waypoint->positions.at(j);
                q3 = q1;
            }
            else if(i < num_waypoints-1)    //middle points
            {
                q1 = prev_waypoint->positions.at(j);
                q2 = curr_waypoint->positions.at(j);
                q3 = next_waypoint->positions.at(j);
            }
            else                            //last point
            {
                q1 = prev_waypoint->positions.at(j);
                q2 = curr_waypoint->positions.at(j);
                q3 = q1;
            }
 
            if(dt1 == 0.0 || dt2 == 0.0)
                v1 = v2 = a = 0.0;
            else
            {
                v1 = (q2 - q1)/dt1;
                v2 = (q3 - q2)/dt2;
                a = 2.0*(v2 - v1)/(dt1+dt2);
            }
 
            //actually set the velocity and acceleration
            curr_waypoint->velocities.at(j) = (v1+v2)/2;
            curr_waypoint->accelerations.at(j) = a;
        }
    }
}

int pick_part(std::string part_type, Kitting &kitting){
  const double CONVEYOR_SPEED = .2;
  const double KITTING_ARM_SPEED = 1;
  conveyor_part part;
  bool part_found = false;
  double part_y = 0;
  double kitting_y = kitting.move_group.getCurrentPose().pose.position.y;
  
  //remove parts too far along the conveyor
  for(std::map<std::string,std::vector<conveyor_part>>::iterator it=conveyor_parts.begin(); it!=conveyor_parts.end(); ++it){
    for(std::vector<conveyor_part>::iterator vit = it->second.begin(); vit != it->second.end(); ){
      if(ros::Time::now().toSec() - vit->time_acquired.toSec() > 25){
        vit = it->second.erase(vit);
      }
      else ++vit;
    }
  }

  std::cout << "Part type, count, and part y position: " <<  part_type << ", " << conveyor_parts[part_type].size() << ", " << part.part_pose.transform.translation.y << "\n";

  if(conveyor_parts[part_type].size() != 0){
    part_found = true;
    part = conveyor_parts[part_type][0];
  }

  // for(int i = 0; i < conveyor_parts.size(); i++){
  //   if(conveyor_parts[i].part_type == part_type){
  //     part = conveyor_parts[i];
  //     part_found = true;
  //   } 
  // }
  if(!part_found) return -1;

  kitting.enable_gripper();

  part_y = part.part_pose.transform.translation.y;
  double part_location = part_y - CONVEYOR_SPEED*(ros::Time::now().toSec() - part.time_acquired.toSec());
  std::vector<geometry_msgs::Pose> kitting_poses;
  std::vector<geometry_msgs::Pose> pickup_poses(14);
  geometry_msgs::Pose kitting_pose = kitting.move_group.getCurrentPose().pose;
  double distance_to_part; 
  double catch_up_speed;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  const double eef_step_small = 0.01;  // tried 0.005, made no diff
  double fraction;
  moveit_msgs::RobotTrajectory trajectory;

  std::vector<double> joint_group_positions;
  kitting.current_state = kitting.move_group.getCurrentState();
  kitting.current_state->copyJointGroupPositions(kitting.joint_model_group, joint_group_positions);
  // std::cout << "joint 0: " << joint_group_positions[0] << "\n";
  double jgp0 = joint_group_positions[0];  // 0=linear actuator
  std::vector<double> temp = {-0.0121211, 0.043126, -0.933196, 2.05574, -2.69212, -1.58004, 0.0 };
  // std::cout << "jgp_kit 0: " << temp[0] << "\n";
  joint_group_positions = temp;
  joint_group_positions[0] = jgp0;
  kitting.move_group.setJointValueTarget(joint_group_positions);
  bool success = (kitting.move_group.plan(kitting.my_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  kitting.move_group.move();

  // kitting.current_state = kitting.move_group.getCurrentState();
  // kitting.current_state->copyJointGroupPositions(kitting.joint_model_group, joint_group_positions);
  // for(int i = 0; i < joint_group_positions.size(); i++){
  // std::cout << "Joint " << i << " : " << joint_group_positions[i] << "\n";

  // kitting_pose = kitting.move_group.getCurrentPose().pose;
  // std::cout << "Current z (should be 1.305): " << kitting_pose.position.z << "\n";
  // kitting.current_state = kitting.move_group.getCurrentState();
  // kitting.current_state->copyJointGroupPositions(kitting.joint_model_group, joint_group_positions);
  // std::cout << "Joints: " << joint_group_positions[0] << ", " << joint_group_positions[1] << ", " << joint_group_positions[2] << ", " << joint_group_positions[3] << ", " << joint_group_positions[4] << ", " << joint_group_positions[5] << ", " << joint_group_positions[6] << "\n";

  double guess_velocity = 0.9;
  double offset = .19429;
  double full_distance;
  kitting_pose = kitting.move_group.getCurrentPose().pose;
  part_location = part_y - CONVEYOR_SPEED*(ros::Time::now().toSec() - part.time_acquired.toSec());
  distance_to_part = part_location - kitting_pose.position.y;
  if(std::abs(distance_to_part) > 0){
    if(part_location < kitting_pose.position.y) catch_up_speed = guess_velocity - CONVEYOR_SPEED;
    else catch_up_speed = guess_velocity + CONVEYOR_SPEED;
    full_distance = guess_velocity*distance_to_part/catch_up_speed;
    kitting.current_state = kitting.move_group.getCurrentState();
    kitting.current_state->copyJointGroupPositions(kitting.joint_model_group, joint_group_positions);
    double jgp1 = joint_group_positions[0];  // 0=linear actuator
    std::cout << "Part location: " << part_location << ", kitting.y: " << kitting_pose.position.y << ", jgp1: " << jgp1 << "\n";
    kitting.linac_destination = jgp1 + full_distance - offset;

    if(kitting.linac_destination < -9){
      conveyor_parts[part_type].erase(conveyor_parts[part_type].begin());
      return -1;
    }

    kitting.move_joints(std::vector<std::vector<double>>{
      {0.043126, -0.933196, 2.05574, -2.69212, -1.58004, 0.0 }
      }, std::vector<double>{std::abs(full_distance)/0.9}, 
      std::vector<double>{kitting.linac_destination});
    kitting.watch_linac = true;
    while(kitting.watch_linac){
      ros::Duration(.1).sleep();
    }
    std::cout << "Done with joint-based long distance move\n";
  }

  guess_velocity = 0.6;
  kitting_pose = kitting.move_group.getCurrentPose().pose;
  part_location = part_y - CONVEYOR_SPEED*(ros::Time::now().toSec() - part.time_acquired.toSec());
  distance_to_part = part_location - kitting_pose.position.y;
  if(part_location < kitting_pose.position.y) catch_up_speed = guess_velocity - CONVEYOR_SPEED;
  else catch_up_speed = guess_velocity + CONVEYOR_SPEED;
  full_distance = guess_velocity*distance_to_part/catch_up_speed;
  double first_y_coord = kitting_pose.position.y;
  kitting_pose.position.y += full_distance;
  kitting_poses.clear();
  kitting_poses.push_back(kitting_pose);
  fraction = kitting.move_group.computeCartesianPath(kitting_poses,eef_step, jump_threshold, trajectory, false);
  int trajectory_size = trajectory.joint_trajectory.points.size();
  double estimated_time = trajectory.joint_trajectory.points[trajectory_size-1].time_from_start.toSec();
  // std::cout << "first_y_coord: " << first_y_coord << " full_distance: " << full_distance << "\n";
  //estimated_time == full_distance/guess_velocity;
  guess_velocity = std::abs(full_distance/estimated_time);
  // std::cout << "time: " << estimated_time << " new_vel: " << guess_velocity << "\n";

  part_location = part_y - CONVEYOR_SPEED*(ros::Time::now().toSec() - part.time_acquired.toSec());
  distance_to_part = part_location - first_y_coord;
  if(part_location < first_y_coord) catch_up_speed = guess_velocity - CONVEYOR_SPEED;
  else catch_up_speed = guess_velocity + CONVEYOR_SPEED;
  full_distance = guess_velocity*distance_to_part/catch_up_speed;
  kitting_pose.position.y = first_y_coord + full_distance;
  kitting_poses.clear();
  kitting_poses.push_back(kitting_pose);
  fraction = kitting.move_group.computeCartesianPath(kitting_poses,eef_step, jump_threshold, trajectory, false);
  std::cout << "first_y_coord: " << first_y_coord << " full_distance: " << full_distance << "\n";
  kitting.my_plan_.trajectory_ = trajectory;
  // std::cout << ros::Time::now() << '\n';
  kitting.move_group.execute(kitting.my_plan_);
  std::cout << "movement 2 complete\n";

  double part_height;
  if(part_type[9] == 'p') part_height = PUMP_HEIGHT;
  else if(part_type[9] == 's') part_height = SENSOR_HEIGHT; 
  else if(part_type[9] == 'b') part_height = BATTERY_HEIGHT;
  else part_height = REGULATOR_HEIGHT;

  kitting_pose = kitting.move_group.getCurrentPose().pose;
  kitting_pose.position.y -= .11;
  kitting_pose.position.z = .89 + part_height + 0.003;
  pickup_poses.at(0) = kitting_pose;
  geometry_msgs::Pose kitting_pose2 = kitting_pose;
  // kitting_pose2.position.y -= 0.1;
  // pickup_poses.at(1) = kitting_pose2;
  for (int x = 1; x < 13; x+=2) {
    kitting_pose2.position.y -= 0.05;
    pickup_poses.at(x) = kitting_pose2;
    kitting_pose2.position.y -= 0.025;
    kitting_pose2.position.z -= 0.002;  // Approach 1mm at a time in 0.25 second increments
    pickup_poses.at(x+1) = kitting_pose2;
  }

  kitting_pose2.position.y -= 0.025;
  kitting_pose2.position.z += 0.25;
  pickup_poses.at(13) = kitting_pose2;
  
  fraction = kitting.move_group.computeCartesianPath(pickup_poses,eef_step_small, jump_threshold, trajectory, false);
  kitting.my_plan_.trajectory_ = trajectory;
  std::cout << ros::Time::now() << '\n';
  setAvgCartesianSpeed(kitting.my_plan_, "ee_link", 0.23);
  // std::cout << "desired z value for picking: " << kitting_pose.position.z << "\n";
  kitting.check_joints = true;
  kitting.move_group.execute(kitting.my_plan_);
  std::cout << "movement 3 complete\n";
  // // kitting_pose = kitting.move_group.getCurrentPose().pose;
  // // std::cout << "ACTUAL z value for picking: " << kitting_pose.position.z << "\n";
  
  // LULZ the current position_tolerance = 0.0001! So none of the below makes a difference...
  // std::cout << "Current goal position tolerance: " << kitting.move_group.getGoalPositionTolerance() << "\n";
  // double position_tolerance = 0.001;
  // kitting.move_group.setGoalPositionTolerance(position_tolerance);
  while (!kitting.attached_ready) {
    ros::Duration(0.25).sleep();
  }
  conveyor_parts[part_type].erase(conveyor_parts[part_type].begin());
  ros::Duration(0.75).sleep();
  
  return 1;
}
/// Example class that can hold state and provide methods that handle incoming data.
class MyCompetitionClass
{
public:
  explicit MyCompetitionClass(ros::NodeHandle & node)
  : current_score_(0), assembly_has_been_zeroed_(false)
  {
    // %Tag(ADV_CMD)%
    assembly_torso_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
      "/ariac/gantry/gantry_controller/command", 10);

    assembly_arm_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
      "/ariac/gantry/gantry_arm_controller/command", 10);

    depth_as1_pcl2_pub_ = node.advertise<sensor_msgs::PointCloud2>("/ariac/depth_camera_as1/p2", 10);
    depth_as2_pcl2_pub_ = node.advertise<sensor_msgs::PointCloud2>("/ariac/depth_camera_as2/p2", 10);
    depth_as3b_pcl2_pub_ = node.advertise<sensor_msgs::PointCloud2>("/ariac/depth_camera_as3b/p2", 10);
    depth_as4_pcl2_pub_ = node.advertise<sensor_msgs::PointCloud2>("/ariac/depth_camera_as4/p2", 10);
    // %EndTag(ADV_CMD)%
  }

  /// Called when a new message is received.
  void current_score_callback(const std_msgs::Float32::ConstPtr & msg) {
    if (msg->data != current_score_)
    {
      ROS_INFO_STREAM("Score: " << msg->data);
    }
    current_score_ = msg->data;
  }

  /// Called when a new message is received.
  void competition_state_callback(const std_msgs::String::ConstPtr & msg) {
    if (msg->data == "done" && competition_state_ != "done")
    {
      ROS_INFO("Competition ended.");
    }
    competition_state_ = msg->data;
  }

  // %Tag(CB_CLASS)%
  void assembly_joint_state_callback(
    const sensor_msgs::JointState::ConstPtr & joint_state_msg)
  {
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Joint States assembly (throttled to 0.1 Hz):\n" << *joint_state_msg);
    // // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
    // assembly_current_joint_states_ = *joint_state_msg;
    // if (!assembly_has_been_zeroed_) {
    //   assembly_has_been_zeroed_ = true;
    //   ROS_INFO("Sending assembly arm to zero joint positions...");
    // //   send_arm_to_zero_state(assembly_arm_joint_trajectory_publisher_);
    // }
  }
  // %EndTag(CB_CLASS)%
  
  /// Called when a new LogicalCameraImage message is received.
  void logical_camera_callback(
    const nist_gear::LogicalCameraImage::ConstPtr & image_msg)
  {
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Logical camera: '" << image_msg->models.size() << "' objects.");
  }

  void depth_camera_as1_callback(
    const sensor_msgs::PointCloud::ConstPtr & point_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10, "Depth camera as1 published data...");
    // std::cout << "AS1 depth points: " << point_msg->points.size() << "\n";  // Turns out only 10K points are published each time...
    static bool test = sensor_msgs::convertPointCloudToPointCloud2(*point_msg,	depth_as1_current_pcl2_); 	
    depth_as1_pcl2_pub_.publish(depth_as1_current_pcl2_);
  }

  void depth_camera_as2_callback(
    const sensor_msgs::PointCloud::ConstPtr & point_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10, "Depth camera as2 published data...");
    // std::cout << "AS2 depth points: " << point_msg->points.size() << "\n";
    static bool test = sensor_msgs::convertPointCloudToPointCloud2(*point_msg,	depth_as2_current_pcl2_); 	
    depth_as2_pcl2_pub_.publish(depth_as2_current_pcl2_);
  }

  void depth_camera_as3b_callback(
    const sensor_msgs::PointCloud::ConstPtr & point_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10, "Depth camera as3b published data...");
    // std::cout << "AS3 depth points: " << point_msg->points.size() << "\n";
    static bool test = sensor_msgs::convertPointCloudToPointCloud2(*point_msg,	depth_as3b_current_pcl2_); 	
    depth_as3b_pcl2_pub_.publish(depth_as3b_current_pcl2_);
  }

  void depth_camera_as4_callback(
    const sensor_msgs::PointCloud::ConstPtr & point_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10, "Depth camera as4 published data...");
    // std::cout << "AS4 depth points: " << point_msg->points.size() << "\n";
    static bool test = sensor_msgs::convertPointCloudToPointCloud2(*point_msg,	depth_as4_current_pcl2_); 	
    depth_as4_pcl2_pub_.publish(depth_as4_current_pcl2_);
  }

  /// Called when a new LogicalCameraImage message is received over the conveyor.
  void logical_camera_callback2(
    const nist_gear::LogicalCameraImage::ConstPtr & image_msg)
  {
    if (!bb_read) {
      ROS_INFO_STREAM_THROTTLE(10,
        "Logical camera: '" << image_msg->models.size() << "' objects.");
      bb_read = true;
    }
  }
  
  /// Called when a new Proximity message is received.
  void breakbeam_callback(const nist_gear::Proximity::ConstPtr & msg) {
    if (msg->object_detected) {  // If there is an object in proximity.
      ROS_INFO("Break beam triggered.");
    } else {
      ROS_INFO("Break beam trigger complete, variable incremented.");
      breakbeam_triggered += 1;
      bb_read = false;
      
      conveyor_part new_part;
      new_part.time_acquired = msg->header.stamp;
      const nist_gear::LogicalCameraImage::ConstPtr & image_msg = ros::topic::waitForMessage<nist_gear::LogicalCameraImage>("/ariac/logical_camera_2");
      geometry_msgs::TransformStamped localPose;
      double lowest_y = INT_MAX;
      int lowest_index = 0;
      for(int i = 0; i < image_msg->models.size(); i++){
        if(image_msg->models[i].pose.position.y < lowest_y){
          lowest_y = image_msg->models[i].pose.position.y;
          lowest_index = i;
        }
      }
      std::string part_type = image_msg->models[lowest_index].type;
      
        std::map<std::string,int>::iterator part_itr = conveyor_part_count.find(part_type);
      if (part_itr == conveyor_part_count.end()){
        int new_count = 0;
        conveyor_part_count.insert(std::pair<std::string, int>(part_type,new_count));
      } 
      else{
        conveyor_part_count[part_type]+=1;
      }
      

      Part_Mgmt part_mgmt;
      geometry_msgs::TransformStamped partlocalPose;
      
      std::string part_header = "logical_camera_2_" + image_msg->models[lowest_index].type + "_" + std::to_string(conveyor_part_count[part_type] + bin_part_count[part_type]+1) + "_frame";
      // new_part.part_pose = part_mgmt.GetPartPose(part_header);

      // TESTING
      // --------
      localPose.header.frame_id = "logical_camera_2_frame";
      localPose.transform.translation.x = image_msg->models[lowest_index].pose.position.x;
      localPose.transform.translation.y = image_msg->models[lowest_index].pose.position.y;
      localPose.transform.translation.z = image_msg->models[lowest_index].pose.position.z;
      localPose.transform.rotation.x = image_msg->models[lowest_index].pose.orientation.x;
      localPose.transform.rotation.y = image_msg->models[lowest_index].pose.orientation.y;
      localPose.transform.rotation.z = image_msg->models[lowest_index].pose.orientation.z;
      localPose.transform.rotation.w = image_msg->models[lowest_index].pose.orientation.w;
      new_part.part_pose = part_mgmt.GetPartPose(localPose);
      // --------


      std::cout << new_part.part_pose.transform.translation.z << "\n";
      std::map<std::string,std::vector<conveyor_part>>::iterator itr;
      // for(itr=conveyor_parts.begin(); itr!= conveyor_parts.end(); ++itr){
      // }
      itr = conveyor_parts.find(part_type);
      if (itr == conveyor_parts.end()){
        std::vector<conveyor_part> new_vector;
        new_vector.push_back(new_part);
        conveyor_parts.insert(std::pair<std::string, std::vector<conveyor_part>>(part_type,new_vector));
      } 
      else{
        conveyor_parts[part_type].push_back(new_part);
      } 

      //conveyor_parts.push_back(new_part);
      //std::cout << conveyor_parts.size() << "hello \n";
      std::cout << part_type << conveyor_parts[part_type].size() << "\n";
    }
  }

int breakbeam_triggered = 0;

private:
  std::string competition_state_;
  double current_score_;
  ros::Publisher assembly_torso_joint_trajectory_publisher_;
  ros::Publisher assembly_arm_joint_trajectory_publisher_;
  ros::Publisher depth_as1_pcl2_pub_;
  ros::Publisher depth_as2_pcl2_pub_;
  ros::Publisher depth_as3b_pcl2_pub_;
  ros::Publisher depth_as4_pcl2_pub_;
  sensor_msgs::PointCloud2 depth_as1_current_pcl2_;
  sensor_msgs::PointCloud2 depth_as2_current_pcl2_;
  sensor_msgs::PointCloud2 depth_as3b_current_pcl2_;
  sensor_msgs::PointCloud2 depth_as4_current_pcl2_;
  sensor_msgs::JointState kitting_current_joint_states_;
  sensor_msgs::JointState assembly_current_joint_states_;
  bool assembly_has_been_zeroed_;
  bool bb_read = true;
};

// %Tag(MAIN)%
int main(int argc, char ** argv) {

  ROS_INFO("HERE HERE HERE");
  
  // Last argument is the default name of the node.
  ros::init(argc, argv, "hw_example_node");

  // First initialize the parts list
  Parts_List pl;

  // Call logical cameras over bins and get all parts
  Bin_Parts bp;

  // Then build the list of part counts in the bins
  pl.PopulateBinList(bp);

  //std::vector<std::string> bin_part_types = bp.GetPartTypes();
  //for(int i = 0; i < bin_part_types.size(); i++){
  //  bin_part_count.insert(std::pair<std::string, int>(bin_part_types[i],bp.PartCount(bin_part_types[i])));
  //}

  ros::NodeHandle node;

  // Now initialize the order class to receive orders when they're announced
  Orders orders(pl, bp);
  
  // Instance of custom class from above.
  MyCompetitionClass comp_class(node);

  // Subscribe to the '/ariac/current_score' topic.
  ros::Subscriber current_score_subscriber = node.subscribe(
    "/ariac/current_score", 10,
    &MyCompetitionClass::current_score_callback, &comp_class);

  // Subscribe to the '/ariac/competition_state' topic.
  ros::Subscriber competition_state_subscriber = node.subscribe(
    "/ariac/competition_state", 10,
    &MyCompetitionClass::competition_state_callback, &comp_class);

  // ros::Subscriber assembly_joint_state_subscriber = node.subscribe(
  //   "/ariac/gantry/joint_states", 10,
  //   &MyCompetitionClass::assembly_joint_state_callback, &comp_class);

  // %Tag(SUB_FUNC)%
  // Subscribe to the '/ariac/breakbeam_0_change' topic.
  ros::Subscriber breakbeam_subscriber = node.subscribe(
    "/ariac/breakbeam_0_change", 10,
    &MyCompetitionClass::breakbeam_callback, &comp_class);

  // Subscribe to the depth cameras.
  ros::Subscriber depth_camera_as1_subscriber = node.subscribe(
    "/ariac/depth_camera_as1", 2,
    &MyCompetitionClass::depth_camera_as1_callback, &comp_class);

  ros::Subscriber depth_camera_as2_subscriber = node.subscribe(
    "/ariac/depth_camera_as2", 2,
    &MyCompetitionClass::depth_camera_as2_callback, &comp_class);

  ros::Subscriber depth_camera_as3b_subscriber = node.subscribe(
    "/ariac/depth_camera_as3b", 2,
    &MyCompetitionClass::depth_camera_as3b_callback, &comp_class);

  ros::Subscriber depth_camera_as4_subscriber = node.subscribe(
    "/ariac/depth_camera_as4", 1,
    &MyCompetitionClass::depth_camera_as4_callback, &comp_class);

  ROS_INFO("Setup complete.");

  // Now start the competition (causes orders to be announced)
  start_competition(node);
  
  // FROM conveyor_picking-mcb
  // --------------------------
  ros::AsyncSpinner spinner(4);  // For moveit to not block all code execution
  spinner.start();  // For moveit to not block all code execution

  // Robots must be initialized AFTER the `spinner.start()` above
  Gantry_Arm robot_g_arm;  
  Gantry_Torso robot_g_torso;
  Kitting robot_kit;

  int zero = robot_kit.ZeroArm();
  zero = robot_g_arm.ZeroArm();

  robot_kit.add_collision_objects();
  robot_g_torso.add_collision_objects();
  robot_g_arm.add_collision_objects();

  Part_Mgmt part_mgmt;

  int test = 0;

  geometry_msgs::Pose target_pose1;
  // tf2_ros::Buffer buffer;
  // tf2_ros::TransformListener tfl(buffer);
  
  OrderPart order_part;
  // order_part = orders.getNextPart(order_kit_order);
  geometry_msgs::TransformStamped localPose, part_worldPose_curr, part_worldPose_dest;
  geometry_msgs::Pose target_pose;
  // int remaining_part_count = orders.order_list.size(); 
  bool part_picked = false;
  orders.update_parts_remain();
  // ros::Duration(7).sleep();

  while (orders.parts_remain_kit > 0 || orders.parts_remain_asm > 0) {
    if (orders.parts_remain_kit > 0) {
      if (comp_class.breakbeam_triggered > 0 && pl.list_part_count["needed"].size() > 0) {
        for (auto part : pl.list_part_count["needed"]) {
          int try_the_conveyor = pick_part(part.first, robot_kit);
          std::cout << "\n\nIN THE IF STATEMENT, FOR LOOP, return value: " << try_the_conveyor << "\n\n";
          if (try_the_conveyor > 0) {
            order_part = orders.getOrderPart(part.first);
            part_picked = true;
          } else {
            order_part = orders.getNextPart(order_kit_order);
            part_worldPose_curr = part_mgmt.GetPartPose(order_part.current_pose);
            std::cout << "current: " << part_worldPose_curr.transform.translation.x << ", " << part_worldPose_curr.transform.translation.y << ", " << part_worldPose_curr.transform.translation.z << "\n";
            part_picked = false;
          }
        }
      } else {
        // If parts remain on order, get the next part
        order_part = orders.getNextPart(order_kit_order);
        part_worldPose_curr = part_mgmt.GetPartPose(order_part.current_pose);
        std::cout << "current: " << part_worldPose_curr.transform.translation.x << ", " << part_worldPose_curr.transform.translation.y << ", " << part_worldPose_curr.transform.translation.z << "\n";
        part_picked = false;
      }
      std::cout << "data returned: " << order_part.order_number << ", " << order_part.part_type << ", " << order_part.agv << ", " << order_part.station << ", " << order_part.current_pose << "\n";
      
      // get the pose of the object in the tray from the order
      part_worldPose_dest = part_mgmt.GetPartPose(order_part.destination_pose);
      std::cout << "Transform.translation in " << order_part.agv << ": " << part_worldPose_dest.transform.translation.x << ", " << part_worldPose_dest.transform.translation.y << ", " << part_worldPose_dest.transform.translation.z << "\n";
      std::cout << "Transform.orientation in " << order_part.agv << ": " << part_worldPose_dest.transform.rotation.x << ", " << part_worldPose_dest.transform.rotation.y << ", " << part_worldPose_dest.transform.rotation.z << ", " << part_worldPose_dest.transform.rotation.w << "\n";

      if (part_picked) {
        // Move to the destination agv
        test = robot_kit.move_near(part_worldPose_dest);

        // Drop the part at the destination
        test = robot_kit.drop_part(part_worldPose_dest);

        // Remove the needed part from teh list
        test = pl.DecrementNeededPart(order_part.part_type);
      } else {
        if (part_worldPose_curr.transform.translation.x > -2.60) {
          // Use the kitting robot
          test = robot_kit.move_bin_side();

          // Move to the part in the bin
          test = robot_kit.move_near(part_worldPose_curr);

          // Pickup the part
          test = robot_kit.pickup_part(part_worldPose_curr);

          // Need to check if part needs to be flipped...and then add some way to flip a part...

          // Move to the destination agv
          test = robot_kit.move_near(part_worldPose_dest);

          // Drop the part at the destination
          test = robot_kit.drop_part(part_worldPose_dest);
        } else {
          // Use the gantry robot
          test = robot_g_arm.ZeroArm();
          
          // Logic to move to correct bin
          std::vector<double> joint_group_positions;
          double xaxis = part_worldPose_curr.transform.translation.x;
          double yaxis = part_worldPose_curr.transform.translation.y;
          if (xaxis > -2.2) {  
              // The bins closer to the conveyor
              if (yaxis > 3.08) joint_group_positions = jgp_gt_kit_bin1;
              else if (yaxis > 2.26) joint_group_positions = jgp_gt_kit_bin2;
              else if (yaxis > -2.86) joint_group_positions = jgp_gt_kit_bin6;
              else joint_group_positions = jgp_gt_kit_bin5;
          } else {
              // The bins further from the conveyor
              if (yaxis > 3.08) joint_group_positions = jgp_gt_kit_bin4;
              else if (yaxis > 2.26) joint_group_positions = jgp_gt_kit_bin3;
              else if (yaxis > -2.86) joint_group_positions = jgp_gt_kit_bin7;
              else joint_group_positions = jgp_gt_kit_bin8;
          }
          test = robot_g_torso.move_to(joint_group_positions);

          // The pose on the bin is to the bottom of the part, so need to add its height to it
          double part_height;
          if(order_part.part_type[9] == 'p') part_height = PUMP_HEIGHT;
          else if(order_part.part_type[9] == 's') part_height = SENSOR_HEIGHT; 
          else if(order_part.part_type[9] == 'b') part_height = BATTERY_HEIGHT;
          else part_height = REGULATOR_HEIGHT;
          part_worldPose_curr.transform.translation.z += part_height;
          test = robot_g_arm.pickup_part_bins(part_worldPose_curr);

          // NEED AGV LOGIC HERE
          if(order_part.agv[3] == '1') test = robot_g_torso.move_to(jgp_gt_kit_agv1);
          else if(order_part.agv[3] == '2') test = robot_g_torso.move_to(jgp_gt_kit_agv2);
          else if(order_part.agv[3] == '3') test = robot_g_torso.move_to(jgp_gt_kit_agv3);
          else test = robot_g_torso.move_to(jgp_gt_kit_agv4);

          // Again, the pose on the agv is to the bottom of the part, so need to add its height to it 
          part_worldPose_dest.transform.translation.z += part_height;
          test = robot_g_arm.drop_part_agv(part_worldPose_dest);

          // Move to an out of the way pose
          test = robot_g_torso.move_to(jgp_gt_prehandoff);

        }
        // Remove part from list of parts in bins
        bp.PartUsed(order_part.part_type);
      }
    } else {
      order_part = orders.getNextAssemblyPart();

      int station_id = station_numbers.find(order_part.station)->second;
      int agv_id = 0;

      geometry_msgs::TransformStamped localPose;
      for (int c=0; c < 2; c++) {
        std::string cam = station_cameras.find(order_part.station)->second[c];
        const nist_gear::LogicalCameraImage::ConstPtr & image_msg = ros::topic::waitForMessage<nist_gear::LogicalCameraImage>(cam);
        for(int i = 0; i < image_msg->models.size(); i++){
          if(image_msg->models[i].type == order_part.part_type){
            agv_id = (2*((station_id-1)/2))+c+1;
            localPose.header.frame_id = cam.substr(7,cam.size()-7)+"_frame";
            localPose.transform.translation.x = image_msg->models[i].pose.position.x;
            localPose.transform.translation.y = image_msg->models[i].pose.position.y;
            localPose.transform.translation.z = image_msg->models[i].pose.position.z;
            localPose.transform.rotation.x = image_msg->models[i].pose.orientation.x;
            localPose.transform.rotation.y = image_msg->models[i].pose.orientation.y;
            localPose.transform.rotation.z = image_msg->models[i].pose.orientation.z;
            localPose.transform.rotation.w = image_msg->models[i].pose.orientation.w;
            i = image_msg->models.size();
            c = 2;
          }
        }
      }

      test = robot_g_arm.ZeroArm();

      int asagv, aspcase, ascase;
      if (station_id == 1) {
        aspcase = LOC_AS1_PCASE;
        ascase = LOC_AS1_CASE;
        if (agv_id == 1) asagv = LOC_AS1_AGV1;
        else asagv = LOC_AS1_AGV2;
      } else if (station_id == 2) {
        aspcase = LOC_AS2_PCASE;
        ascase = LOC_AS2_CASE;
        if (agv_id == 1) asagv = LOC_AS2_AGV1;
        else asagv = LOC_AS2_AGV2;
      } else if (station_id == 3) {
        aspcase = LOC_AS3_PCASE;
        ascase = LOC_AS3_CASE;
        if (agv_id == 3) asagv = LOC_AS3_AGV3;
        else asagv = LOC_AS3_AGV4;
      } else if (station_id == 4) {
        aspcase = LOC_AS4_PCASE;
        ascase = LOC_AS4_CASE;
        if (agv_id == 3) asagv = LOC_AS4_AGV3;
        else asagv = LOC_AS4_AGV4;
      }

      test = robot_g_torso.move_torso(asagv);
      
      part_worldPose_curr = part_mgmt.GetPartPose(localPose);  // Pose on agv tray; z-value = BOTTOM of part on tray (hence need to add part height)
      // The pose on the bin is to the bottom of the part, so need to add its height to it
      double part_height;
      if(order_part.part_type[9] == 'p') part_height = PUMP_HEIGHT;
      else if(order_part.part_type[9] == 's') part_height = SENSOR_HEIGHT; 
      else if(order_part.part_type[9] == 'b') part_height = BATTERY_HEIGHT;
      else part_height = REGULATOR_HEIGHT; 
      part_worldPose_curr.transform.translation.z += part_height;
      std::cout << "current: " << part_worldPose_curr.transform.translation.x << ", " << part_worldPose_curr.transform.translation.y << ", " << part_worldPose_curr.transform.translation.z << "\n";

      test = robot_g_arm.pickup_part(part_worldPose_curr);

      // Assuming pickup successful, attach the part to the end effector for planning

      // Maybe add joint constraint to keep the ee facing downward for the placing operations?

      test = robot_g_torso.move_torso(aspcase);

      test = robot_g_arm.CaseArm();

      test = robot_g_torso.move_torso(ascase);

      part_worldPose_dest = part_mgmt.GetPartPose(order_part.destination_pose);

      test = robot_g_arm.drop_part(part_worldPose_dest);

      test = robot_g_torso.move_torso(asagv);

      test = robot_g_arm.ZeroArm();

    }

    // Remove the part from the order and get back the count of parts remianing on the order
    test = orders.UpdateOrder(order_part);

    if (test == 0) {
      // Order complete; submit it
      int order_submitted = orders.SubmitOrderShipment(order_part);
      printf("Order submitted, should return 1 (meaning success): %d\n", order_submitted);
    }

    orders.update_parts_remain();

  }

  // Now end the competition
  end_competition(node);

  


  // int bb_old = 0;
  // int bb_count = 0;
  // // while (comp_class.breakbeam_triggered < 2 && bb_count < 50) {
  // //   if (bb_old != comp_class.breakbeam_triggered) {
  //     ros::Duration(10).sleep();
  //     // Pickup first part
  //     pick_part("assembly_regulator_red",robot_kit);

  //     // Deliver first part and then pickup next part:
  //     // ---------------------------------------------
  //     // First, wait for the attachment and movement away from the conveyor to start 
  //     while (!robot_kit.attached_ready) {
  //       ros::Duration(0.25).sleep();
  //     }
  //     // Then wait for the movement away from the conveyor to complete (so the getCurrentPose call below returns the correct current pose!)
  //     if (robot_kit.attached_ready) {
  //       robot_kit.attached_ready = false;
  //       ros::Duration(0.75).sleep();
  //     }

  //     // Now deliver the part to the AGV
  //     geometry_msgs::Pose target_pose5 = robot_kit.move_group.getCurrentPose().pose;
  //     target_pose5.position.y = 4.67;  // Move down y to AGV
  //     robot_kit.move_group.setPoseTarget(target_pose5);
  //     bool success = (robot_kit.move_group.plan(robot_kit.my_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  //     robot_kit.move_group.move();
  //     printf("move to agv done...\n");

  //     geometry_msgs::Pose target_pose6 = target_pose5;
  //     target_pose6.position.x = -2.26;  // Swing around to AGV
  //     robot_kit.move_group.setPoseTarget(target_pose6);
  //     success = (robot_kit.move_group.plan(robot_kit.my_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  //     robot_kit.move_group.move();
  //     printf("swing to agv done...\n");

  //     geometry_msgs::Pose target_pose7 = target_pose6;
  //     target_pose7.position.z = 0.91;  // Lower to drop position
  //     robot_kit.move_group.setPoseTarget(target_pose7);
  //     success = (robot_kit.move_group.plan(robot_kit.my_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  //     robot_kit.move_group.move();
  //     printf("lower part done...\n");

  //     robot_kit.disable_gripper();

  //     geometry_msgs::Pose target_pose8 = target_pose7;
  //     target_pose8.position.z = 1.25;  // Raise arm
  //     target_pose8.position.x += 0.25;  // Move arm closer to body
  //     robot_kit.move_group.setPoseTarget(target_pose8);
  //     success = (robot_kit.move_group.plan(robot_kit.my_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  //     robot_kit.move_group.move();
  //     printf("part left on AGV...\n");

  //     // Now pickup the next part
  //     pick_part("assembly_regulator_red",robot_kit);




  //     // ---------------------------------------------------

  //  // }
  //    //ros::Duration(1).sleep();
  //   // printf("bb count: %d\n", comp_class.breakbeam_triggered);
  //   bb_count+=1;
 // }

  return 0;
}
// %EndTag(MAIN)%
// %EndTag(FULLTEXT)%

// OLD FROM main:
// -----------------

//   ros::AsyncSpinner spinner(1);  // Use async so moveit doesn't block all code execution
//   spinner.start();  

//   // Robots must be initialized AFTER the `spinner.start()` above
//   // Kitting robot_kit;
//   Gantry_Arm robot_g_arm;  
//   Gantry_Torso robot_g_torso;
//   Kitting robot_kit;
//   robot_g_torso.add_collision_objects();
//   robot_g_arm.add_collision_objects();
//   robot_kit.add_collision_objects();
  
//   // TESTING
//   // --------
//   int test = 0;
//   geometry_msgs::TransformStamped localPose, part_worldPose_source, part_worldPose_dest;
//   tf2::Quaternion q;

//   // FROM part_flipping_handoff
//   Part_Mgmt part_mgmt;

//   test = robot_g_arm.ZeroArm();

//   test = robot_g_torso.move_to(jgp_gt_kit_bin8);

//   // pick part - need pose of part on agv in a TransformStamped message, which means converting rpy to quaternion
//   q.setRPY( 0.0, 0.0, 0.00 );  // Create this quaternion from roll/pitch/yaw (in radians)
//   q.normalize();
//   std::cout << "The destination quaternion representation is: " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << "\n";
//   part_worldPose_source.header.frame_id = "world";
//   part_worldPose_source.transform.translation.x = -2.751691;
//   part_worldPose_source.transform.translation.y = -3.479923;
//   part_worldPose_source.transform.translation.z = 0.781468 + (PUMP_HEIGHT/2);
//   part_worldPose_source.transform.rotation.x = q[0];
//   part_worldPose_source.transform.rotation.y = q[1];
//   part_worldPose_source.transform.rotation.z = q[2];
//   part_worldPose_source.transform.rotation.w = q[3];

//   // Pickup the part
//   // FROM gantry_kitting
//   test = robot_g_arm.pickup_part(part_worldPose_source);

//   // Move to the destination agv
//   test = robot_g_torso.move_to(jgp_gt_kit_agv4);

//   // Drop part
//   // xyz: [0.1, 0.1, 0], rpy: [0, 0, 0]
//   localPose.header.frame_id = "kit_tray_4";
//   // std::cout << "Getting OrderPart, original pose in vect: " << dest_pose.position.x << "\n";
//   localPose.transform.translation.x = 0.1;
//   localPose.transform.translation.y = 0.1;
//   // FROM main
//   test = robot_kit.pickup_part(part_worldPose_source);

//   test = flip_part_handoff(robot_kit, robot_g_arm, robot_g_torso);

  
//   // FROM main
//   //int test = 0;
//   geometry_msgs::TransformStamped localPose, part_worldPose_source, part_worldPose_dest;
//   tf2::Quaternion q;

//   test = robot_g_arm.ZeroArm();

//   test = robot_g_torso.move_torso(LOC_AS3_AGV3);

//   // pick part - need pose of part on agv
//   // xyz: [0.0, 0.0, 0]
//   // rpy: [0, 0, 0]
//   q.setRPY( 0, 0, 0 );  // Create this quaternion from roll/pitch/yaw (in radians)
//   q.normalize();
//   std::cout << "The destination quaternion representation is: " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << "\n";
//   localPose.header.frame_id = "kit_tray_3";
//   localPose.transform.translation.x = 0.0;
//   localPose.transform.translation.y = 0.0;
//   localPose.transform.translation.z = 0.0;
//   localPose.transform.rotation.x = q[0];
//   localPose.transform.rotation.y = q[1];
//   localPose.transform.rotation.z = q[2];
//   localPose.transform.rotation.w = q[3];
//   part_worldPose_source = part_mgmt.GetPartPose(localPose);  // Pose on agv tray; z-value = BOTTOM of part on tray (hence need to add part height) 
//   part_worldPose_source.transform.translation.z += (BATTERY_HEIGHT);
//   std::cout << "Source pose.translation in case: " << part_worldPose_source.transform.translation.x << ", " << part_worldPose_source.transform.translation.y << ", " << part_worldPose_source.transform.translation.z << "\n";
//   std::cout << "Source pose.rotation in case: " << part_worldPose_source.transform.rotation.x << ", " << part_worldPose_source.transform.rotation.y << ", " << part_worldPose_source.transform.rotation.z << ", " << part_worldPose_source.transform.rotation.w << "\n";

//   test = robot_g_arm.pickup_part(part_worldPose_source);

//   // Assuming pickup successful, attach the part to the end effector for planning

//   // Maybe add joint constraint to keep the ee facing downward for the placing operations?
//   test = robot_g_arm.CaseArm();

//   test = robot_g_torso.move_torso(LOC_AS3_CASE);

//   // place part - need pose of part on briefcase
//   // xyz: [-0.032465, 0.174845, 0.15]
//   // rpy: [0, 0, 0]
//   localPose.header.frame_id = "briefcase_3";
//   localPose.transform.translation.x = -0.032465;
//   localPose.transform.translation.y = 0.174845;
//   localPose.transform.translation.z = 0.15;
//   localPose.transform.rotation.x = q[0];
//   localPose.transform.rotation.y = q[1];
//   localPose.transform.rotation.z = q[2];
//   localPose.transform.rotation.w = q[3];
//   part_worldPose_dest = part_mgmt.GetPartPose(localPose);  // Pose in briefcase; z-value = TOP of part installed in briefcase
//   std::cout << "Destination pose.translation in case: " << part_worldPose_dest.transform.translation.x << ", " << part_worldPose_dest.transform.translation.y << ", " << part_worldPose_dest.transform.translation.z << "\n";
//   std::cout << "Destination pose.rotation in case: " << part_worldPose_dest.transform.rotation.x << ", " << part_worldPose_dest.transform.rotation.y << ", " << part_worldPose_dest.transform.rotation.z << ", " << part_worldPose_dest.transform.rotation.w << "\n";

//   test = robot_g_arm.drop_part(part_worldPose_dest);

//   test = robot_g_torso.move_torso(LOC_AS3_AGV3);

//   test = robot_g_arm.ZeroArm();

//   // Now pick pump, too...
//   localPose.header.frame_id = "kit_tray_3";
//   localPose.transform.translation.x = 0.15;
//   localPose.transform.translation.y = -0.1;
//   // ABOVE from main
//   localPose.transform.translation.z = 0;
//   localPose.transform.rotation.x = q[0];
//   localPose.transform.rotation.y = q[1];
//   localPose.transform.rotation.z = q[2];
//   localPose.transform.rotation.w = q[3];
//   // FROM gantry_kitting
//   part_worldPose_dest = part_mgmt.GetPartPose(localPose);
//   std::cout << "The destination z is: " << part_worldPose_dest.transform.translation.z << "\n";
//   part_worldPose_dest.transform.translation.z += (PUMP_HEIGHT + 0.02);  // For some reason this is still too close somehow; adding extra 0.02 for safety
//   std::cout << "The destination z after adding pump height is: " << part_worldPose_dest.transform.translation.z << "\n";

//   test = robot_g_arm.drop_part(part_worldPose_dest);  
//   // FROM main
//   part_worldPose_source = part_mgmt.GetPartPose(localPose);  // Pose on agv tray; z-value = BOTTOM of part on tray (hence need to add part height) 
//   part_worldPose_source.transform.translation.z += (PUMP_HEIGHT);
//   std::cout << "Source pose.translation in case: " << part_worldPose_source.transform.translation.x << ", " << part_worldPose_source.transform.translation.y << ", " << part_worldPose_source.transform.translation.z << "\n";
//   std::cout << "Source pose.rotation in case: " << part_worldPose_source.transform.rotation.x << ", " << part_worldPose_source.transform.rotation.y << ", " << part_worldPose_source.transform.rotation.z << ", " << part_worldPose_source.transform.rotation.w << "\n";

//   test = robot_g_arm.pickup_part(part_worldPose_source);

//   test = robot_g_arm.CaseArm();

//   test = robot_g_torso.move_torso(LOC_AS3_CASE);

//   localPose.header.frame_id = "briefcase_3";
//   localPose.transform.translation.x = 0.032085;
//   localPose.transform.translation.y = -0.152835;
//   localPose.transform.translation.z = 0.15;
//   localPose.transform.rotation.x = q[0];
//   localPose.transform.rotation.y = q[1];
//   localPose.transform.rotation.z = q[2];
//   localPose.transform.rotation.w = q[3];
//   part_worldPose_dest = part_mgmt.GetPartPose(localPose);  // Pose in briefcase; z-value = TOP of part installed in briefcase
//   std::cout << "Destination pose.translation in case: " << part_worldPose_dest.transform.translation.x << ", " << part_worldPose_dest.transform.translation.y << ", " << part_worldPose_dest.transform.translation.z << "\n";
//   std::cout << "Destination pose.rotation in case: " << part_worldPose_dest.transform.rotation.x << ", " << part_worldPose_dest.transform.rotation.y << ", " << part_worldPose_dest.transform.rotation.z << ", " << part_worldPose_dest.transform.rotation.w << "\n";

//   test = robot_g_arm.drop_part(part_worldPose_dest);

//   test = robot_g_torso.move_torso(LOC_AS3_AGV3);

//   test = robot_g_arm.ZeroArm();
//   // ABOVE from main
  
//   // -------------
//   // TESTING DONE

//   return 0;
// }
// %EndTag(MAIN)%
// %EndTag(FULLTEXT)%


// WORKING PUMP ASSEMBLY CODE
// -----------------------------------------------------------
  // int test = 0;
  // geometry_msgs::TransformStamped localPose, part_worldPose_source, part_worldPose_dest;
  // tf2::Quaternion q;

  // test = robot_g_arm.ZeroArm();

  // test = robot_g_torso.move_torso(LOC_AS3_AGV3);

  // // pick part - need pose of part on agv
  // // xyz: [0.15, -0.1, 0]
  // // rpy: [0, 0, 0]
  // q.setRPY( 0, 0, 0 );  // Create this quaternion from roll/pitch/yaw (in radians)
  // q.normalize();
  // std::cout << "The destination quaternion representation is: " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << "\n";
  // localPose.header.frame_id = "kit_tray_3";
  // localPose.transform.translation.x = 0.15;
  // localPose.transform.translation.y = -0.1;
  // localPose.transform.translation.z = 0;
  // localPose.transform.rotation.x = q[0];
  // localPose.transform.rotation.y = q[1];
  // localPose.transform.rotation.z = q[2];
  // localPose.transform.rotation.w = q[3];
  // part_worldPose_source = part_mgmt.GetPartPose(localPose);  // Pose on agv tray; z-value = BOTTOM of part on tray (hence need to add part height) 
  // part_worldPose_source.transform.translation.z += (PUMP_HEIGHT);
  // std::cout << "Source pose.translation in case: " << part_worldPose_source.transform.translation.x << ", " << part_worldPose_source.transform.translation.y << ", " << part_worldPose_source.transform.translation.z << "\n";
  // std::cout << "Source pose.rotation in case: " << part_worldPose_source.transform.rotation.x << ", " << part_worldPose_source.transform.rotation.y << ", " << part_worldPose_source.transform.rotation.z << ", " << part_worldPose_source.transform.rotation.w << "\n";

  // test = robot_g_arm.pickup_part(part_worldPose_source);

  // // Assuming pickup successful, attach the part to the end effector for planning

  // // Maybe add joint constraint to keep the ee facing downward for the placing operations?

  // test = robot_g_arm.CaseArm();

  // test = robot_g_torso.move_torso(LOC_AS3_CASE);

  // // place part - need pose of part on briefcase
  // // xyz: [0.032085, -0.152835, 0.25]
  // // rpy: [0, 0, 0]
  // localPose.header.frame_id = "briefcase_3";
  // localPose.transform.translation.x = 0.032085;
  // localPose.transform.translation.y = -0.152835;
  // localPose.transform.translation.z = 0.15;
  // localPose.transform.rotation.x = q[0];
  // localPose.transform.rotation.y = q[1];
  // localPose.transform.rotation.z = q[2];
  // localPose.transform.rotation.w = q[3];
  // part_worldPose_dest = part_mgmt.GetPartPose(localPose);  // Pose in briefcase; z-value = TOP of part installed in briefcase
  // std::cout << "Destination pose.translation in case: " << part_worldPose_dest.transform.translation.x << ", " << part_worldPose_dest.transform.translation.y << ", " << part_worldPose_dest.transform.translation.z << "\n";
  // std::cout << "Destination pose.rotation in case: " << part_worldPose_dest.transform.rotation.x << ", " << part_worldPose_dest.transform.rotation.y << ", " << part_worldPose_dest.transform.rotation.z << ", " << part_worldPose_dest.transform.rotation.w << "\n";

  // test = robot_g_arm.drop_part(part_worldPose_dest);

  // test = robot_g_torso.move_torso(LOC_AS3_AGV3);

  // test = robot_g_arm.ZeroArm();
// -----------------------------------------------------------
// END -- WORKING PUMP ASSEMBLY CODE

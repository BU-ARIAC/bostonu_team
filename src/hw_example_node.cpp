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

#include <ros/ros.h>

#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include <nist_gear/Proximity.h>
#include <sensor_msgs/JointState.h>
// #include <sensor_msgs/Pointcloud.h>  // Used for the gantry-mounted depth camera...if we end up using it
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

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include "part_mgmt.h"
#include "order_mgmt.h"
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

/// Example class that can hold state and provide methods that handle incoming data.
class MyCompetitionClass
{
public:
  explicit MyCompetitionClass(ros::NodeHandle & node)
  : current_score_(0), kitting_has_been_zeroed_(false), assembly_has_been_zeroed_(false)
  {
    // %Tag(ADV_CMD)%
    kitting_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
      "/ariac/kitting/kitting_arm_controller/command", 10);

    assembly_torso_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
      "/ariac/gantry/gantry_controller/command", 10);

    assembly_arm_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
      "/ariac/gantry/gantry_arm_controller/command", 10);
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
  /// Called when a new JointState message is received.
  void kitting_joint_state_callback(
    const sensor_msgs::JointState::ConstPtr & joint_state_msg)
  {
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Joint States kitting (throttled to 0.1 Hz):\n" << *joint_state_msg);
    // // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
    // kitting_current_joint_states_ = *joint_state_msg;
    // if (!kitting_has_been_zeroed_) {
    //   kitting_has_been_zeroed_ = true;
    //   ROS_INFO("Sending kitting to zero joint positions...");
    // //   send_arm_to_zero_state(kitting_joint_trajectory_publisher_);
    // }
  }

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

  // %Tag(ARM_ZERO)%
  /// Create a JointTrajectory with all positions set to zero, and command the arm.
  void send_arm_to_zero_state(ros::Publisher & joint_trajectory_publisher) {
    // Create a message to send.
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
    // Create one point in the trajectory.
    msg.points.resize(1);
    // Resize the vector to the same length as the joint names.
    // Values are initialized to 0.
    msg.points[0].positions.resize(msg.joint_names.size(), 0.0);
    // How long to take getting to the point (floating point seconds).
    msg.points[0].time_from_start = ros::Duration(0.001);
    ROS_INFO_STREAM("Sending command:\n" << msg);
    joint_trajectory_publisher.publish(msg);
  }
  // %EndTag(ARM_ZERO)%

  /// Called when a new LogicalCameraImage message is received.
  void logical_camera_callback(
    const nist_gear::LogicalCameraImage::ConstPtr & image_msg)
  {
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Logical camera: '" << image_msg->models.size() << "' objects.");
  }

  /// Called when a new LogicalCameraImage message is received over the conveyor.
  void logical_camera_callback2(
    const nist_gear::LogicalCameraImage::ConstPtr & image_msg)
  {
    // if (!bb_read) {
    //   ROS_INFO_STREAM_THROTTLE(10,
    //     "Logical camera: '" << image_msg->models.size() << "' objects.");
    //   bb_read = true;
    // }
  }

  void kitting_gripper_callback(const nist_gear::VacuumGripperState::ConstPtr & gripper_msg){
    gripper_attached = gripper_msg->attached;
  }

  /// Called when a new Proximity message is received.
  void breakbeam_callback(const nist_gear::Proximity::ConstPtr & msg) {
    if (msg->object_detected) {  // If there is an object in proximity.
      ROS_INFO("Break beam triggered.");
    } else {
      ROS_INFO("Break beam trigger complete, variable incremented.");
      breakbeam_triggered += 1;
      bb_read = false;
    }
  }

  int breakbeam_triggered = 0;
  bool gripper_attached = false;

private:
  std::string competition_state_;
  double current_score_;
  ros::Publisher kitting_joint_trajectory_publisher_;
  ros::Publisher assembly_torso_joint_trajectory_publisher_;
  ros::Publisher assembly_arm_joint_trajectory_publisher_;
  sensor_msgs::JointState kitting_current_joint_states_;
  sensor_msgs::JointState assembly_current_joint_states_;
  bool kitting_has_been_zeroed_;
  bool assembly_has_been_zeroed_;
  bool bb_read = true;
};

// %Tag(MAIN)%
int main(int argc, char ** argv) {

  test_om();
  printf("Test msg 2, about to start node...\n");
  
  // Last argument is the default name of the node.
  ros::init(argc, argv, "hw_example_node");

  printf("Test msg 3, node started...\n");

// //   // First initialize the parts list
  Parts_List pl;

  printf("Test msg 4, pl created...\n");

// //   // Call logical cameras over bins and get all parts
  Bin_Parts bp;

  printf("Test msg 5, bp created...\n");

// //   // Then build the list of part counts in the bins
  pl.PopulateBinList(bp);

  printf("Test msg 6, bins populated...\n");

//   int test_dbp = pl.DecrementBinPart("assembly_regulator_red");
//   std::cout << "Decrement count of assembly_regulator_red, should be 3? " << std::to_string(test_dbp) << "\n";


// //   // Print out some part frames to verify they're correct...
//   std::vector<std::string> test = bp.GetFrames("assembly_regulator_red");
//   std::cout << test[0] << "\n" << bp.GetFrame("assembly_regulator_red") << "\n" << std::to_string(bp.PartCount("assembly_regulator_blue")) << "\n";

  ros::NodeHandle node;

  // printf("asm order should be 2: %d\n", order_asm_order);
  Orders orders(pl, bp);
  printf("orders max priority: %d\n", orders.max_priority);



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

  // Subscribe to the '/ariac/joint_states' topic.
  ros::Subscriber kitting_joint_state_subscriber = node.subscribe(
    "/ariac/kitting/joint_states", 10,
    &MyCompetitionClass::kitting_joint_state_callback, &comp_class);

  ros::Subscriber assembly_joint_state_subscriber = node.subscribe(
    "/ariac/gantry/joint_states", 10,
    &MyCompetitionClass::assembly_joint_state_callback, &comp_class);

  // %Tag(SUB_FUNC)%
  // Subscribe to the '/ariac/breakbeam_0_change' topic.
  ros::Subscriber breakbeam_subscriber = node.subscribe(
    "/ariac/breakbeam_0_change", 10,
    &MyCompetitionClass::breakbeam_callback, &comp_class);

  // Subscribe to the '/ariac/logical_camera_0' topic.
  ros::Subscriber logical_camera_subscriber = node.subscribe(
    "/ariac/logical_camera_0", 10,
    &MyCompetitionClass::logical_camera_callback, &comp_class);

  ros::Subscriber kitting_gripper_subscriber = node.subscribe(
    "/ariac/kitting/arm/gripper/state", 10,
    &MyCompetitionClass::kitting_gripper_callback, &comp_class);

  printf("Test msg 7, setup complete...\n");
  ROS_INFO("Setup complete.");
  start_competition(node);
//   ros::spin();  // This executes callbacks on new data until ctrl-c.
  ros::AsyncSpinner spinner(1);  // For moveit to not block all code execution
  spinner.start();  // For moveit to not block all code execution

  ros::NodeHandle node_kitting_("/ariac/kitting");
	
  static const std::string PLANNING_GROUP_KITTING = "kitting_arm";
  static const std::string PLANNING_DESC_KITTING = "/ariac/kitting/robot_description";
  static const std::string PLANNING_SCENE_NS_KITTING = "/ariac/kitting";

  moveit::planning_interface::MoveGroupInterface::Options loadOptions(PLANNING_GROUP_KITTING, PLANNING_DESC_KITTING, node_kitting_);
  moveit::planning_interface::MoveGroupInterface move_group(loadOptions);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface(PLANNING_SCENE_NS_KITTING);
  
  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP_KITTING);

  ros::NodeHandle node_gantry_("/ariac/gantry");
	
  static const std::string PLANNING_GROUP_GANTRY = "gantry_full";
  static const std::string PLANNING_DESC_GANTRY = "/ariac/gantry/robot_description";
  static const std::string PLANNING_SCENE_NS_GANTRY = "/ariac/gantry";

  moveit::planning_interface::MoveGroupInterface::Options loadOptions_ga(PLANNING_GROUP_GANTRY, PLANNING_DESC_GANTRY, node_gantry_);
  moveit::planning_interface::MoveGroupInterface move_group_ga(loadOptions_ga);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_ga(PLANNING_SCENE_NS_GANTRY);
  
  const robot_state::JointModelGroup* joint_model_group_ga = move_group_ga.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GANTRY);

  // We can print the name of the reference frame for this robot.
  ROS_INFO("Planning frame arm: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO("End effector link arm: %s", move_group.getEndEffectorLink().c_str());
  geometry_msgs::PoseStamped p = move_group.getCurrentPose();
  ROS_INFO("End effector pose->position (x,y,z): (%f,%f,%f)", p.pose.position.x, p.pose.position.y, p.pose.position.z);
  ROS_INFO("End effector pose->orientation (x,y,z,w): (%f,%f,%f,%f)", p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w);

  // geometry_msgs::PoseStamped moveit::planning_interface::MoveGroup::getCurrentPose 	( 	const std::string &  	end_effector_link = ""	) 	

  // We can get a list of all the groups in the robot:
  ROS_INFO("Available Planning Groups arm:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
  printf("\n");

  // We can print the name of the reference frame for this robot.
  ROS_INFO("Planning frame gantry: %s", move_group_ga.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO("End effector link gantry: %s", move_group_ga.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO("Available Planning Groups gantry:");
  std::copy(move_group_ga.getJointModelGroupNames().begin(), move_group_ga.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
  printf("\n");

  printf("Before setting a pose, I will try adding the conveyor as an obstacle in the scene\n");

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();
  collision_object.id = "conveyor";  // The id of the object is used to identify it
  // Define a box for the conveyor and add it to the world
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.68;
  primitive.dimensions[1] = 9.0;
  primitive.dimensions[2] = 0.93;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = -0.573076;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.5;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let’s add the collision object into the world

  printf("Add the collision object into the world\n");
  planning_scene_interface.addCollisionObjects(collision_objects);



  // printf("Trying a setPoseTarget operation...\n");

  // geometry_msgs::Pose target_pose1; //, target_pose2;
  // std::vector<geometry_msgs::Pose> target_poses;
  // printf("Message created\n");


  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener tfl(buffer);

  // ros::Time time = ...;
  ros::Duration timeout(1.0);

  nist_gear::VacuumGripperControl gripper_service_;
  nist_gear::VacuumGripperState gripper_status_;
  ros::ServiceClient gripper_client_;
  gripper_client_ = node_kitting_.serviceClient<nist_gear::VacuumGripperControl>("/ariac/kitting/arm/gripper/control");

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  printf("Test msg 8, some stuff done...\n");

//   // Below code used to read the conveyor breakbeam and the logical camera and then pick a part...not using right now
//   ////////////////////////////////////////////////////////////////////////////////////
//   // int bb_old = 0;
//   // while (comp_class.breakbeam_triggered < 2) {
//   //   if (bb_old != comp_class.breakbeam_triggered) {
//   //     // Below code works to get parts off the conveyor belt...well one part at least
//   //     // Cannot use right now....
//   //     ////////////////////////////////////////////////////////////////////////////////////
//   //     //call read conveyor logical
//   //     // const nist_gear::LogicalCameraImage::ConstPtr & image_msg = ros::topic::waitForMessage<nist_gear::LogicalCameraImage>("/ariac/logical_camera_2");
//   //     // for (auto model : image_msg->models) {
//   //     //   ROS_INFO_STREAM("model: " << model);
//   //     //   printf("position: (%f, %f, %f)\n", model.pose.position.x, model.pose.position.y, model.pose.position.z);
//   //     // }
//   //     // ROS_INFO_STREAM_THROTTLE(1,
//   //     //   "Logical camera: '" << image_msg->models.size() << "' objects.");
//   //     // bb_old = comp_class.breakbeam_triggered;
//   //     // printf("*** in if %d***\n", bb_old);

//   //     // nist_gear::VacuumGripperControl gripper_service_;
//   //     // nist_gear::VacuumGripperState gripper_status_;
//   //     // ros::ServiceClient gripper_client_;

//   //     // gripper_client_ = node_kitting_.serviceClient<nist_gear::VacuumGripperControl>("/ariac/kitting/arm/gripper/control");
//   //     // gripper_service_.request.enable = true;
//   //     // gripper_client_.call(gripper_service_);
//   //     // if (gripper_service_.response.success) {
//   //     //   ROS_INFO_STREAM("Gripper activated!");
//   //     // } else {
//   //     //   ROS_WARN_STREAM("Gripper activation failed!");
//   //     // }

//   //     // std::string product_frame = "logical_camera_2_" + image_msg->models[0].type + "_0_frame";

//   //     // geometry_msgs::TransformStamped tfGeom;
//   //     // try {
//   //     //     tfGeom = buffer.lookupTransform("world", "logical_camera_2_assembly_pump_blue_0_frame", ros::Time(0));
//   //     // } catch (tf2::TransformException &e) {
//   //     //     printf("ERROR\n");
//   //     // }
      
//   //     // ROS_INFO_STREAM("pose out: " << tfGeom);

//   //     // target_pose1.position.x = tfGeom.transform.translation.x;
//   //     // target_pose1.position.y = tfGeom.transform.translation.y - 0.52;
//   //     // target_pose1.position.z = tfGeom.transform.translation.z + 0.1;
//   //     // target_pose1.orientation.x = -0.0110788;
//   //     // target_pose1.orientation.y = 0.711468;
//   //     // target_pose1.orientation.z = 0.011894;
//   //     // target_pose1.orientation.w = 0.702532;
//   //     // move_group.setPoseTarget(target_pose1);
//   //     // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//   //     // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//   //     // move_group.move();
//   //     // printf("move 1 done...\n");

//   //     // std::vector<geometry_msgs::Pose> waypoints;
//   //     // waypoints.push_back(target_pose1);

//   //     // geometry_msgs::Pose target_pose3 = target_pose1;

//   //     // target_pose3.position.z -= 0.026;
//   //     // target_pose3.position.y -= 0.5;
//   //     // waypoints.push_back(target_pose3);  // down

//   //     // geometry_msgs::Pose target_pose4 = target_pose3;

//   //     // target_pose4.position.z += 0.35;
//   //     // target_pose4.position.y -= 0.25;
//   //     // waypoints.push_back(target_pose4);  // up

//   //     // // move_group.setMaxVelocityScalingFactor(0.001);
//   //     // moveit_msgs::RobotTrajectory trajectory;
//   //     // const double jump_threshold = 0.0;
//   //     // const double eef_step = 0.01;
//   //     // double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
//   //     // my_plan.trajectory_ = trajectory;
//   //     // move_group.execute(my_plan);
//   //     // printf("done picking up part\n");

//   //     // geometry_msgs::Pose target_pose5 = target_pose4;
//   //     // target_pose5.position.y = 4.67;  // Move down y to AGV
//   //     // move_group.setPoseTarget(target_pose5);
//   //     // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//   //     // move_group.move();
//   //     // printf("move to agv done...\n");

//   //     // geometry_msgs::Pose target_pose6 = target_pose5;
//   //     // target_pose6.position.x = -2.26;  // Swing around to AGV
//   //     // move_group.setPoseTarget(target_pose6);
//   //     // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//   //     // move_group.move();
//   //     // printf("swing to agv done...\n");

//   //     // geometry_msgs::Pose target_pose7 = target_pose6;
//   //     // target_pose7.position.z = 0.91;  // Lower to drop position
//   //     // move_group.setPoseTarget(target_pose7);
//   //     // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//   //     // move_group.move();
//   //     // printf("lower part done...\n");

//   //     // gripper_service_.request.enable = false;
//   //     // gripper_client_.call(gripper_service_);
//   //     // if (gripper_service_.response.success) {
//   //     //   ROS_INFO_STREAM("Gripper deactivated!");
//   //     // } else {
//   //     //   ROS_WARN_STREAM("Gripper deactivation failed!");
//   //     // }

//   //     // geometry_msgs::Pose target_pose8 = target_pose7;
//   //     // target_pose8.position.z = 1.25;  // Raise arm
//   //     // target_pose8.position.x += 0.25;  // Move arm closer to body
//   //     // move_group.setPoseTarget(target_pose8);
//   //     // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//   //     // move_group.move();
//   //     // printf("part left on AGV...\n");
//   //     ////////////////////////////////////////////////////////////////////////////////////



//   //     // Below code was used to use path planner for moving arm...except it kept failing to find a path in the 5 seconds allotted...
//   //     // So not using the cartesian paths below instead...
//   //     ////////////////////////////////////////////////////////////////////////////////////
//   //     // geometry_msgs::Pose target_pose9 = target_pose8;
//   //     // target_pose9.position.y = tfGeom2.transform.translation.y;  // Just move left
//   //     // move_group.setPoseTarget(target_pose9);
//   //     // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//   //     // move_group.move();
//   //     // printf("move near part bin\n");

//   //     // geometry_msgs::Pose target_pose10 = target_pose9;
//   //     // target_pose10.position.x = tfGeom2.transform.translation.x;
//   //     // target_pose10.position.z = tfGeom2.transform.translation.z+.9;  // Move over part
//   //     // move_group.setPoseTarget(target_pose10);
//   //     // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//   //     // move_group.move();
//   //     // printf("move over part\n");
//   //     ////////////////////////////////////////////////////////////////////////////////////

//   //   }
//   //   sleep(1);
//   //   // printf("bb count: %d\n", comp_class.breakbeam_triggered);
//   // }
//   ////////////////////////////////////////////////////////////////////////////////////
//   sleep(1);

  OrderPart order_part;
  order_part = orders.getNextPart(order_kit_order);
  geometry_msgs::TransformStamped tfGeom2;
  geometry_msgs::TransformStamped worldPose;

  printf("test msg 8a, order_part.part_count: %d\n", order_part.part_count);

  if (order_part.part_count > 0) {
  	std::cout << "test msg 8b, in order if: " << order_part.order_number << "\n";
    std::cout << "data returned: " << order_part.order_number << ", " << order_part.part_type << ", " << order_part.agv << ", " << order_part.station << ", " << order_part.current_pose << "\n";
    ros::Duration(1.0).sleep();
    try {
        tfGeom2 = buffer.lookupTransform("world", order_part.current_pose, ros::Time(0));
    } catch (tf2::TransformException &e) {
        printf("ERROR\n");
        printf("%s\n",e.what());
    }
    std::cout << "current: " << tfGeom2.transform.translation.x << ", " << tfGeom2.transform.translation.y << ", " << tfGeom2.transform.translation.z << "\n";
    // get the pose of the object in the tray from the order
    buffer.transform(order_part.destination_pose, worldPose, "world");
    
    std::cout << "Transform.translation in tray2: " << worldPose.transform.translation.x << ", " << worldPose.transform.translation.y << ", " << worldPose.transform.translation.z << "\n";
    std::cout << "Transform.orientation in tray2: " << worldPose.transform.rotation.x << ", " << worldPose.transform.rotation.y << ", " << worldPose.transform.rotation.z << ", " << worldPose.transform.rotation.w << "\n";
  }

  std::vector<geometry_msgs::Pose> waypoints;
  printf("Test msg 9, waypoint vector created...\n");

//   // ROS_INFO("End effector pose->position (x,y,z): (%f,%f,%f)", p.pose.position.x, p.pose.position.y, p.pose.position.z);
//   // ROS_INFO("End effector pose->orientation (x,y,z,w): (%f,%f,%f,%f)", p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w);

  // First, swing around to the agv side
  geometry_msgs::Pose target_pose;
  target_pose.position.x = p.pose.position.x;
  target_pose.position.y = p.pose.position.y;
  target_pose.position.z = p.pose.position.z + 0.3;
  target_pose.orientation.x = -0.0110788;
  target_pose.orientation.y = 0.711468;
  target_pose.orientation.z = 0.011894;
  target_pose.orientation.w = 0.702532;
  move_group.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();

  printf("Test msg 10, 1 move done?...\n");

  ros::Duration(1.0).sleep();

  geometry_msgs::Pose target_pose5 = target_pose;
  target_pose5.position.x = -1.3;  // Swing around to AGV side
  target_pose5.position.y = 0.9;  // Swing around to AGV side
  move_group.setPoseTarget(target_pose5);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();
  printf("move 2 done...\n");

  ros::Duration(1.0).sleep();

  geometry_msgs::Pose target_pose6 = target_pose5;
  target_pose6.position.x = -2.26;  // Swing around to AGV side
  move_group.setPoseTarget(target_pose6);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();
  printf("swing to agv done...\n");

  ros::Duration(1.0).sleep();

  waypoints.push_back(target_pose6);

  geometry_msgs::Pose target_pose1 = target_pose6;
  target_pose1.position.y = tfGeom2.transform.translation.y;  // Just move left
  waypoints.push_back(target_pose1); 

  geometry_msgs::Pose target_pose10 = target_pose1;
  target_pose10.position.x = tfGeom2.transform.translation.x;
  target_pose10.position.z = tfGeom2.transform.translation.z+.1;  // Move over part
  waypoints.push_back(target_pose10);

  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  my_plan.trajectory_ = trajectory;
  move_group.execute(my_plan);
  printf("done moving near part\n");

  gripper_service_.request.enable = true;
  gripper_client_.call(gripper_service_);
  if (gripper_service_.response.success) {
    ROS_INFO_STREAM("Gripper activated!");
  } else {
    ROS_WARN_STREAM("Gripper activation failed!");
  }
  printf("gripper activated, trying to go get teh part now...\n");

  waypoints.clear();
  target_pose10.position.z = 0.785;  // Move over part
  waypoints.push_back(target_pose10);
  fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  my_plan.trajectory_ = trajectory;
  move_group.execute(my_plan);
  printf("done moving near part 2\n");

  std::vector<geometry_msgs::Pose> waypoints3, waypoints4;
  geometry_msgs::Pose target_pose11 = target_pose10;
  while (!comp_class.gripper_attached) {
    ros::Duration(0.5).sleep();
    target_pose11.position.z -= 0.01;  // Move closer over part
    waypoints3.clear();
    waypoints3.push_back(target_pose11);
    fraction = move_group.computeCartesianPath(waypoints3, eef_step, jump_threshold, trajectory);
    my_plan.trajectory_ = trajectory;
    move_group.execute(my_plan);
    printf("done moving nearer part...\n");
  }

  geometry_msgs::Pose target_pose12 = target_pose11;
  target_pose12.position.z += 0.5;  // Move away from part
  waypoints4.push_back(target_pose12);

  geometry_msgs::Pose target_pose13 = target_pose12;
  target_pose13.position.z += 0.2;  // Move more away from part
  target_pose13.position.x += 0.5;
  waypoints4.push_back(target_pose13);

  fraction = move_group.computeCartesianPath(waypoints4, eef_step, jump_threshold, trajectory);
  my_plan.trajectory_ = trajectory;
  move_group.execute(my_plan);
  printf("hopefully part picked up...\n");

  ros::Duration(1.0).sleep();

  waypoints4.clear();
  geometry_msgs::Pose target_pose14 = target_pose13;
  target_pose14.position.y = worldPose.transform.translation.y;  // Move toward agv
  waypoints4.push_back(target_pose14);

  geometry_msgs::Pose target_pose15 = target_pose14;
  target_pose15.position.x = worldPose.transform.translation.x;  // Move toward agv
  waypoints4.push_back(target_pose15);

  geometry_msgs::Pose target_pose16 = target_pose15;
  target_pose16.position.x = worldPose.transform.translation.x;  // Move toward spot on agv
  target_pose16.position.y = worldPose.transform.translation.y;
  target_pose16.position.z = worldPose.transform.translation.z + 0.1;
  target_pose16.orientation.x = -0.0110788;
  target_pose16.orientation.y = 0.711468;
  target_pose16.orientation.z = 0.011894;
  target_pose16.orientation.w = 0.702532;
  waypoints4.push_back(target_pose16);

  fraction = move_group.computeCartesianPath(waypoints4, eef_step, jump_threshold, trajectory);
  my_plan.trajectory_ = trajectory;
  move_group.execute(my_plan);
  printf("hopefully part above agv...\n");

  gripper_service_.request.enable = false;
  gripper_client_.call(gripper_service_);
  if (gripper_service_.response.success) {
    ROS_INFO_STREAM("Gripper deactivated!");
  } else {
    ROS_WARN_STREAM("Gripper deactivation failed!");
  }
  printf("hopefully part on agv...\n");

  waypoints4.clear();
  geometry_msgs::Pose target_pose17 = target_pose16;
  target_pose17.position.z += 0.5;  // Move away from part
  waypoints4.push_back(target_pose17);

  geometry_msgs::Pose target_pose18 = target_pose17;
  target_pose18.position.z += 0.2;  // Move more away from part
  target_pose18.position.x += 0.5;
  waypoints4.push_back(target_pose18);

  fraction = move_group.computeCartesianPath(waypoints4, eef_step, jump_threshold, trajectory);
  my_plan.trajectory_ = trajectory;
  move_group.execute(my_plan);
  printf("hopefully moved away from part...\n");

  ros::ServiceClient submit_client = node.serviceClient<nist_gear::AGVToAssemblyStation>("/ariac/"+order_part.agv+"/submit_shipment");
  if (!submit_client.exists()) {
    ROS_INFO("Waiting for the client to be ready...");
    submit_client.waitForExistence();
    ROS_INFO("Service started.");
  }

  nist_gear::AGVToAssemblyStation srv;
  srv.request.shipment_type = order_part.order_number;
  srv.request.assembly_station_name = order_part.station;

  submit_client.call(srv);

  if (!srv.response.success) {
      ROS_ERROR_STREAM("Service failed!");
      printf("in submit shipment error");
  } else {
  	  printf("in submit shipment success");
      ROS_INFO("Service succeeded.");
  }

  return 0;
}
// %EndTag(MAIN)%
// %EndTag(FULLTEXT)%

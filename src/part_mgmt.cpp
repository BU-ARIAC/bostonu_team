#include <cstdio>
#include <string>
#include <ros/ros.h>
#include <nist_gear/LogicalCameraImage.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "part_mgmt.h"
#include "order_mgmt.h"

std::string bin_camera_0 = "logical_camera_0";
std::string bin_camera_1 = "logical_camera_1";
std::vector<std::string> bin_cams{ bin_camera_0, bin_camera_1 };

void test(){
    printf("IN TEST\n");
}

Bin_Parts::Bin_Parts() {
  // std::cout << "in bin parts initializer 1\n";
  for (auto cam : bin_cams) {
    // std::cout << "in bin parts initializer 2, cam: " << cam << "\n";
    int cam_counter = 1;
    std::map<std::string, int> cam_counters;
    const nist_gear::LogicalCameraImage::ConstPtr & image_msg = ros::topic::waitForMessage<nist_gear::LogicalCameraImage>("/ariac/" + cam);
    for (auto model : image_msg->models) {
      // std::cout << "in bin parts initializer 3, model.type: " << model.type << ", and x: " << model.pose.position.x << "\n";
      // if (model.pose.position.x < -2.6) {
        if (!cam_counters[model.type]) {
          std::pair<std::string, int> pair1;
          pair1.first = model.type;
          pair1.second = cam_counter;
          cam_counters.insert(pair1);
          cam_counters[model.type]++;
          // std::cout << "HERERERERERERERERE " << cam_counters.find(model.type)->second << "\n"; 
        }
        // std::cout << "cam counters: " << model.type << ": " << cam_counters[model.type] << " " << std::to_string(cam_counters[model.type]) << "\n";
        std::string frame = cam + "_" + model.type + "_" + std::to_string(cam_counters[model.type]) + "_frame";
        part_frames[model.type].emplace_back(frame);
        cam_counters[model.type]++;
        // std::cout << "cam counters: " << model.type << ": " << cam_counters[model.type] << " " << std::to_string(cam_counters[model.type]) << " frame: " << frame << "\n";
      // }
    }
  }
}

std::vector<std::string> Bin_Parts::GetFrames(const std::string &part_type) {
  return part_frames[part_type];
}

std::string Bin_Parts::GetFrame(const std::string &part_type) {
  return part_frames[part_type].back();
}

std::vector<std::string> Bin_Parts::GetPartTypes() {
  std::vector<std::string> keys;
  for(std::map<std::string, std::vector<std::string>>::iterator it = part_frames.begin(); it != part_frames.end(); ++it) {
    keys.push_back(it->first);
  }
  return keys;
}

int Bin_Parts::PartCount(const std::string &part_type) {
  return part_frames[part_type].size();
}

int Bin_Parts::PartUsed(const std::string &part_type) {
  part_frames[part_type].pop_back();
  // Return true so we know it's done
  return 1;
}

int Parts_List::PopulateBinList(Bin_Parts &bp) {
  std::vector<std::string> keys = bp.GetPartTypes();
  for (std::string key : keys){
    // std::cout << "in PopulateBinList for loop, key: " << key << "\n";
    this->list_part_count["bin"].insert( std::pair<std::string, int>(key, bp.PartCount(key)) );
  }
  return 1;
}

int Parts_List::DecrementBinPart(const std::string &part_type) {
  std::map<std::string, int>::iterator itr;
  itr = this->list_part_count["bin"].find(part_type);
  if (itr != this->list_part_count["bin"].end()) {
    if (itr->second > 0) {
      itr->second -= 1;
    } else {
      return -1;  
    }
  } else {
    return -1;
  }
  return this->list_part_count["bin"][part_type];
}

int Parts_List::IncrementNeededPart(const std::string &part_type) {
  std::map<std::string, int>::iterator itr;
  itr = this->list_part_count["needed"].find(part_type);
  if (itr != this->list_part_count["needed"].end()) {
    itr->second += 1;
  } else {
    this->list_part_count["needed"].insert( std::pair<std::string, int>(part_type, 1) );
  }
  return this->list_part_count["needed"][part_type];
}

int Parts_List::DecrementNeededPart(const std::string &part_type) {
  std::map<std::string, int>::iterator itr;
  itr = this->list_part_count["needed"].find(part_type);
  if (itr != this->list_part_count["needed"].end()) {
    if (itr->second > 1) {
      itr->second -= 1;
    } else if (itr->second == 1){
      itr = this->list_part_count["needed"].erase(itr);
      std::cout << " IN DecrementNeededPart, size should be 0? " << this->list_part_count["needed"].size() << "\n";
      return 0;
    }else {
      return -1;  
    }
  } else {
    return -1;
  }
  return this->list_part_count["needed"][part_type];
}

int Part_Mgmt::CheckPart(const std::string &agv) {
  int agv_id = agv_numbers.find(agv)->second;
  std::string qc_name = "quality_control_sensor_"+std::to_string(agv_id);
  const nist_gear::LogicalCameraImage::ConstPtr & image_msg = ros::topic::waitForMessage<nist_gear::LogicalCameraImage>("/ariac/" + qc_name);
  int ret = image_msg->models.size();  // Should report 0 if no bads parts, else the count of bad parts
  return ret;
}

geometry_msgs::TransformStamped Part_Mgmt::GetPartPose(const std::string & pose_frame) {  // Returns pose from input frame name to world
  geometry_msgs::TransformStamped part_pose_;
  ros::Time t = ros::Time(0);
  try {
    part_pose_ = this->buffer.lookupTransform("world", pose_frame, t, ros::Duration(5.0));
  } catch (tf2::TransformException &e) {
    printf("ERROR in GetPartPose by frame...\n");
    printf("%s\n",e.what());
  }
  return part_pose_;
}

geometry_msgs::TransformStamped Part_Mgmt::GetPartPose(const geometry_msgs::TransformStamped & pose_msg) {  // Returns pose from input Transform to world
  geometry_msgs::TransformStamped part_pose_;
  try {
    this->buffer.transform(pose_msg, part_pose_, "world", ros::Duration(5.0));
  } catch (tf2::TransformException &e) {
    printf("ERROR in GetPartPose by msg...\n");
    printf("%s\n",e.what());
  }
  return part_pose_;
}

Part_Mgmt::Part_Mgmt():
    tfl(buffer) {
  

}
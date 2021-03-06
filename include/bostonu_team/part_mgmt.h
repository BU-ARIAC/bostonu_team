#include <map>
#include <vector>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#pragma once

static const double BATTERY_HEIGHT = 0.054;
static const double REGULATOR_HEIGHT = .067;
static const double SENSOR_HEIGHT = .067;
static const double PUMP_HEIGHT = .115;

void test();

class Bin_Parts                       
{
  public:
    Bin_Parts();
    std::vector<std::string> GetFrames(const std::string &);
    std::string GetFrame(const std::string &);
    std::vector<std::string> GetPartTypes();
    int PartCount(const std::string &);  // returns how many of the (part_type) input there are
    int PartUsed(const std::string &);  // returns success when a used (part_type) is removed from the list of parts in a bin
  
  private:
    std::map<std::string, std::vector<std::string>> part_frames;  // part_type, <frames_for_part_type>
};

class Parts_List
{
  public:
    Parts_List(){
      list_part_count["bin"] = {};
      list_part_count["needed"] = {};
    };
    int PopulateBinList(Bin_Parts &);
    int DecrementBinPart(const std::string &);  // Takes in bin part list and part_type and reduces the count for that part_type; if 0 or part not listed, returns -1
    int IncrementNeededPart(const std::string &);  // Takes in part_type and increases the count for that part_type which is needed on the conveyor
    int DecrementNeededPart(const std::string &);  // Takes in part_type and reduces the count for that part_type in the needed parts list; returns count still needed (or -1 if part missing or 0 already)
  
  // private:
    std::map<std::string, std::map<std::string, int>> list_part_count;  // part list type (bin vs needed), <part_type, count>
};

class Part_Mgmt
{
  public:
    Part_Mgmt();
    int CheckPart(const std::string &);  //returns how many bad parts are detected on the agv name input
    geometry_msgs::TransformStamped GetPartPose(const std::string &);  // Returns pose from input frame name to world
    geometry_msgs::TransformStamped GetPartPose(const geometry_msgs::TransformStamped &);  // Returns pose from input Transform to world
  
  private:
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tfl;
};
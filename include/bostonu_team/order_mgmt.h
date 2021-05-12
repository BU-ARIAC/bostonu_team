#include <map>
#include <vector>
#include <nist_gear/Order.h>
#include <ros/ros.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "part_mgmt.h"

#pragma once

void test_om();

const int order_kit_order = 1;
const int order_asm_order = 2;
const std::map<std::string, int> agv_numbers = {{"agv1", 1}, {"agv2", 2}, {"agv3", 3}, {"agv4", 4}};

class OrderPart
{
  public:
    OrderPart() {
      part_count = 0;
    }
    std::string order_number;
    std::string order_shipment_number;
    int part_count;  // If a part in an order is available for picking, this value is 1, else it's 0 
    std::string part_type;
    std::string current_pose;
    geometry_msgs::TransformStamped destination_pose;
    std::string agv;
    std::string station;
    int idx;  // Index of the part in the part_type_pose_vect of the ordershipment
};

class OrderShipment                       
{
  public:
    OrderShipment() {
      part_type_pose_vect = {};
    }
    int order_shipment_type;
    std::string order_shipment_number;
    std::string order_shipment_agv;  // use -1 for assembly shipments
    std::string order_shipment_station;
    int priority;
    std::string order_shipment_status;
    // std::pair<std::string, geometry_msgs::Pose> part_type_pose;
    // std::vector<std::pair<std::string, geometry_msgs::Pose>> part_type_pose_vect;
    

    // Needs order_shipment_number, agv, station, priority, status, <part_type, part_pose>
    // Priority is based on WHEN an order is received (i.e., "the most recently received order is usually the highest priority")
    //   So, let's use 1 as the default priority and count up as orders are received
    //   Then the highest priority # is the highest priority order
    std::vector<std::pair<std::string, geometry_msgs::Pose>> part_type_pose_vect; 

  private:
};

class Orders                       
{
  public:
    Orders(Parts_List &, Bin_Parts &);
    std::map<std::string, std::vector<OrderShipment>> order_list;
    int max_priority;
    void order_callback(const nist_gear::Order::ConstPtr &);
    void process_received_orders();
    std::string findHighestPriorityOrder();
    int OrderNumberCheck(const std::string &);
    void allocateOrderPart(const std::string &);  // Takes in a part_type, checks if there are any available in the bin-based parts list and decrements if there are parts, else increments needed-part list
    OrderPart getNextPart(int);  // Assume it will return something by having the front end check size of order_list first
    int UpdateOrder(OrderPart);  // Takes in an OrderPart, removes the part from the order_shipment and returns # of parts still in order
    // Needs order_list=(order_number, <OrderShipment>), max_priority, open_shipments



  //   std::vector<std::string> GetFrames(const std::string &);
  //   std::string GetFrame(const std::string &);
  //   int PartCount(const std::string &);
  //   int PartUsed(const std::string &);
  
  private:
    ros::NodeHandle order_node;
    ros::Subscriber orders_subscriber;
    std::vector<nist_gear::Order> received_orders_;
    Parts_List *pl_;
    Bin_Parts *bp_;
  //   std::map<std::string, std::vector<std::string>> part_frames;
};
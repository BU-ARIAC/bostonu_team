#include <cstdio>
#include <string>
#include <geometry_msgs/Pose.h>
#include <nist_gear/Order.h>
#include <ros/ros.h>
#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/AGVToAssemblyStation.h>
#include <nist_gear/AssemblyStationSubmitShipment.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "order_mgmt.h"
#include "part_mgmt.h"

void test_om(){
    printf("IN TEST OM\n");
}

Orders::Orders(Parts_List &pl, Bin_Parts &bp) {
    max_priority = 0;  // meaning no orders have arrived yet
    order_list = {};
    pl_ = &pl;
    bp_ = &bp;
    // Subscribe to the '/ariac/orders' topic.
    orders_subscriber = order_node.subscribe("/ariac/orders", 10, &Orders::order_callback, this);
};

int Orders::OrderNumberCheck(const std::string & order_number) {
    std::map<std::string, std::vector<OrderShipment>>::iterator itr;
    std::cout << "In order number check...order number:" << order_number << ", order_list size: " << order_list.size() << "\n";
    itr = this->order_list.find(order_number);
    // std::cout << "In order number check...iterator.first:" << itr->first << "\n";
    if (itr != this->order_list.end()) {
        std::cout << "In order number check...in if\n";
        return 1;
    } else {
        std::cout << "In order number check...in else\n";
        return 0;
    }
    return 0;
}

// Called when a new Order message is received.
void Orders::order_callback(const nist_gear::Order::ConstPtr & order_msg) {
    ROS_INFO_STREAM("Received order:\n" << *order_msg);
    // std::cout << "Received order: \n" << *order_msg;
    received_orders_.push_back(*order_msg);
    process_received_orders();
}

void Orders::process_received_orders() {
    std::cout << "In order_callback...\n";
    std::pair<std::string, geometry_msgs::Pose> ptp_;
    std::vector<OrderShipment> osv = {};
    for (auto order : received_orders_) {
        std::cout << "In order_callback...processing order, kitting shipments len: " << order.kitting_shipments.size() << ", assembly shipments len: " << order.assembly_shipments.size() << "\n";
        std::string order_number = order.order_id;
        if (this->max_priority == 0 || order.kitting_shipments.size() > 0) {
            this->max_priority += 1;
        }
        if (order.kitting_shipments.size() > 0) {
            for (const auto &shipment : order.kitting_shipments){
                OrderShipment order_shipment;
                order_shipment.order_shipment_type = order_kit_order;
                order_shipment.priority = this->max_priority;   
                order_shipment.order_shipment_number = shipment.shipment_type;
                order_shipment.order_shipment_agv = shipment.agv_id;
                order_shipment.order_shipment_station = shipment.station_id;
                auto products = shipment.products;
                for (const auto &product: products){
                    ptp_.first = product.type;
                    ptp_.second = product.pose;
                    // std::cout << product.type << std::endl;
                    // std::cout << product.pose.position.x << " " << product.pose.position.y << " " << product.pose.position.z << std::endl;
                    order_shipment.part_type_pose_vect.emplace_back(ptp_);
                    allocateOrderPart(product.type);
                }
                osv.emplace_back(order_shipment);
            }
        }
        if (order.assembly_shipments.size() > 0) {
            // OrderShipment order_shipment;
            // order_shipment.order_shipment_type = order_asm_order;
            // order_shipment.priority = this->max_priority;
            // order_shipment.order_shipment_agv = "-1"; // No agv for asembly orders (agv already at station), so all get static value "-1"
            // auto a_shipments = order.assembly_shipments;
            for (const auto &shipment : order.assembly_shipments){
                OrderShipment order_shipment;
                order_shipment.order_shipment_type = order_asm_order;
                order_shipment.priority = this->max_priority;
                order_shipment.order_shipment_agv = "-1"; // No agv for asembly orders (agv already at station), so all get static value "-1"
                order_shipment.order_shipment_number = shipment.shipment_type;
                order_shipment.order_shipment_station = shipment.station_id;
                auto products = shipment.products;
                for (const auto &product: products){
                    ptp_.first = product.type;
                    ptp_.second = product.pose;
                    // std::cout << product.type << std::endl;
                    // std::cout << product.pose.position.x << " " << product.pose.position.y << " " << product.pose.position.z << std::endl;
                    order_shipment.part_type_pose_vect.emplace_back(ptp_);
                }
                osv.emplace_back(order_shipment);
            }
            // osv.emplace_back(order_shipment);
        }
        this->order_list.insert({order_number, osv});
        
        // for (auto ptp : order_shipment.part_type_pose_vect){
        //     std::cout << ptp.first << std::endl;
        //     std::cout << ptp.second.position.x << " " << ptp.second.position.y << " " << ptp.second.position.z << std::endl;
        // }
    }
    std::cout << "Order max priority: " << this->max_priority << "\n";
    received_orders_ = {};
}

void Orders::allocateOrderPart(const std::string & part_type) {
    if (pl_->DecrementBinPart(part_type) < 0) {
        pl_->IncrementNeededPart(part_type);
    }
    // if (pl_->list_part_count["bin"].size() > 0 ) {
    //     std::cout << "checking for part " << part_type << ", part list bin size = " << pl_->list_part_count["bin"].size() << "\n";
    // } else {
    //     std::cout << "checking for part " << part_type << ", no love on part list bin size, though...\n";
    // }
}

std::string Orders::findHighestPriorityOrder() {
    int highestPriority = 0;
    std::string orderNumber;
    std::map<std::string, std::vector<OrderShipment>>::iterator itr;
    for (itr = this->order_list.begin(); itr != this->order_list.end(); ++itr) {
        std::vector<OrderShipment>::iterator itrv;
        int itrv_count = itr->second.size();
        for (int i = 0; i < itrv_count; i++) {
            if (itr->second[i].part_type_pose_vect.size() > 0 && itr->second[i].priority > highestPriority) {
                orderNumber = itr->first;
                highestPriority = itr->second[i].priority;
            }
        }
    }
    return orderNumber;
}

OrderPart Orders::getNextPart(int shipment_type) {
    std::string highestPriorityOrder = findHighestPriorityOrder();
    std::cout << "In getNextPart, highestPriorityOrder: " << highestPriorityOrder << ", but using order_0 anyway...\n";
    // highestPriorityOrder = "order_0";  // MB: Remove for production
    OrderPart op_;
    for (auto ordershipment : this->order_list[highestPriorityOrder]) {
        // std::cout << "In getNextPart, in first for loop\n"; 
        if (ordershipment.order_shipment_type == shipment_type) {
            int partTypeCount = ordershipment.part_type_pose_vect.size();
            std::cout << "In getNextPart, partTypeCount: " << partTypeCount << "\n";
            for (int i = 0; i < partTypeCount; i++) {
                std::pair<std::string, geometry_msgs::Pose> ptp_pair = ordershipment.part_type_pose_vect[i];
                std::string part_type = ptp_pair.first;
                // std::cout << "In getNextPart, in second for loop, part type: " << part_type << ", and count: " << bp_->PartCount(part_type) << "\n";
                if (bp_->PartCount(part_type) > 0) {
                    // std::cout << "Somehow we're in the depth of getNextPart: " << part_type << "\n";
                    // Set values to return
                    op_.order_number = highestPriorityOrder;
                    op_.order_shipment_number = ordershipment.order_shipment_number;
                    op_.part_count = 1;
                    op_.part_type = ordershipment.part_type_pose_vect[i].first;
                    op_.agv = ordershipment.order_shipment_agv;
                    op_.station = ordershipment.order_shipment_station;
                    op_.idx = i;
                    // Get tf frame name of part's current position
                    op_.current_pose = bp_->GetFrame(op_.part_type);
                    // Build TransformStamped message of part's local pose on the agv (used to get world pose on agv later)
                    std::string agv = ordershipment.order_shipment_agv;
                    int agv_id = agv_numbers.find(agv)->second;
                    geometry_msgs::Pose dest_pose = ordershipment.part_type_pose_vect[i].second;
                    geometry_msgs::TransformStamped localPose;
                    localPose.header.frame_id = "kit_tray_" + std::to_string(agv_id);
                    std::cout << "Getting OrderPart, original pose position in vect: " << dest_pose.position.x << ", " << dest_pose.position.y << ", " << dest_pose.position.z << "\n";
                    std::cout << "Getting OrderPart, original pose orientatiuon in vect: " << dest_pose.orientation.x << ", " << dest_pose.orientation.y << ", " << dest_pose.orientation.z << ", " << dest_pose.orientation.w << "\n";
                    localPose.transform.translation.x = dest_pose.position.x;
                    localPose.transform.translation.y = dest_pose.position.y;
                    localPose.transform.translation.z = dest_pose.position.z;
                    localPose.transform.rotation.x = dest_pose.orientation.x;
                    localPose.transform.rotation.y = dest_pose.orientation.y;
                    localPose.transform.rotation.z = dest_pose.orientation.z;
                    localPose.transform.rotation.w = dest_pose.orientation.w;
                    op_.destination_pose = localPose;
                    i = partTypeCount; // end loop
                    return op_;  // Return and stop the for loop processing...
                }  
            }
        }
    }
    return op_;  
}

OrderPart Orders::getNextAssemblyPart() {
    std::string highestPriorityOrder = findHighestPriorityOrder();
    std::cout << "In getNextPart, highestPriorityOrder: " << highestPriorityOrder << ", but using order_0 anyway...\n";
    // highestPriorityOrder = "order_0";  // MB: Remove for production
    OrderPart op_;
    for (auto ordershipment : this->order_list[highestPriorityOrder]) {
        // std::cout << "In getNextPart, in first for loop\n"; 
        if (ordershipment.order_shipment_type == order_asm_order) {
            int partTypeCount = ordershipment.part_type_pose_vect.size();
            std::cout << "In getNextPart, partTypeCount: " << partTypeCount << "\n";
            for (int i = 0; i < partTypeCount; i++) {
                std::pair<std::string, geometry_msgs::Pose> ptp_pair = ordershipment.part_type_pose_vect[i];
                std::string part_type = ptp_pair.first;
                // std::cout << "In getNextPart, in second for loop, part type: " << part_type << ", and count: " << bp_->PartCount(part_type) << "\n";
                // if (bp_->PartCount(part_type) > 0) {
                    // std::cout << "Somehow we're in the depth of getNextPart: " << part_type << "\n";
                    // Set values to return
                    op_.order_number = highestPriorityOrder;
                    op_.order_shipment_number = ordershipment.order_shipment_number;
                    op_.part_count = 1;
                    op_.part_type = ordershipment.part_type_pose_vect[i].first;
                    op_.agv = ordershipment.order_shipment_agv;
                    op_.station = ordershipment.order_shipment_station;
                    op_.idx = i;
                    // Get tf frame name of part's current position
                    op_.current_pose = "";
                    // Build TransformStamped message of part's local pose on the agv (used to get world pose on agv later)
                    std::string station = ordershipment.order_shipment_station;
                    int station_id = station_numbers.find(station)->second;
                    geometry_msgs::Pose dest_pose = ordershipment.part_type_pose_vect[i].second;
                    geometry_msgs::TransformStamped localPose;
                    localPose.header.frame_id = "briefcase_" + std::to_string(station_id);
                    std::cout << "Getting OrderPart, original pose position in vect: " << dest_pose.position.x << ", " << dest_pose.position.y << ", " << dest_pose.position.z << "\n";
                    std::cout << "Getting OrderPart, original pose orientatiuon in vect: " << dest_pose.orientation.x << ", " << dest_pose.orientation.y << ", " << dest_pose.orientation.z << ", " << dest_pose.orientation.w << "\n";
                    localPose.transform.translation.x = dest_pose.position.x;
                    localPose.transform.translation.y = dest_pose.position.y;
                    localPose.transform.translation.z = dest_pose.position.z;
                    localPose.transform.rotation.x = dest_pose.orientation.x;
                    localPose.transform.rotation.y = dest_pose.orientation.y;
                    localPose.transform.rotation.z = dest_pose.orientation.z;
                    localPose.transform.rotation.w = dest_pose.orientation.w;
                    op_.destination_pose = localPose;
                    i = partTypeCount; // end loop
                    return op_;  // Return and stop the for loop processing...
                // }  
            }
        }
    }
    return op_;  
}

OrderPart Orders::getOrderPart(const std::string & part_type_input) {
    std::string highestPriorityOrder = findHighestPriorityOrder();
    // std::cout << "In getNextPart, highestPriorityOrder: " << highestPriorityOrder << ", but using order_0 anyway...\n";
    // highestPriorityOrder = "order_0";  // MB: Remove for production
    OrderPart op_;
    for (auto ordershipment : this->order_list[highestPriorityOrder]) {
        // std::cout << "In getNextPart, in first for loop\n"; 
        if (ordershipment.order_shipment_type == order_kit_order) {
            int partTypeCount = ordershipment.part_type_pose_vect.size();
            std::cout << "In getNextPart, partTypeCount: " << partTypeCount << "\n";
            for (int i = 0; i < partTypeCount; i++) {
                std::pair<std::string, geometry_msgs::Pose> ptp_pair = ordershipment.part_type_pose_vect[i];
                std::string part_type = ptp_pair.first;
                // std::cout << "In getNextPart, in second for loop, part type: " << part_type << ", and count: " << bp_->PartCount(part_type) << "\n";
                if (part_type == part_type_input) {
                    // std::cout << "Somehow we're in the depth of getNextPart: " << part_type << "\n";
                    // Set values to return
                    op_.order_number = highestPriorityOrder;
                    op_.order_shipment_number = ordershipment.order_shipment_number;
                    op_.part_count = 1;
                    op_.part_type = ordershipment.part_type_pose_vect[i].first;
                    op_.agv = ordershipment.order_shipment_agv;
                    op_.station = ordershipment.order_shipment_station;
                    op_.idx = i;
                    // Get tf frame name of part's current position
                    // op_.current_pose = bp_->GetFrame(op_.part_type);
                    // Build TransformStamped message of part's local pose on the agv (used to get world pose on agv later)
                    std::string agv = ordershipment.order_shipment_agv;
                    int agv_id = agv_numbers.find(agv)->second;
                    geometry_msgs::Pose dest_pose = ordershipment.part_type_pose_vect[i].second;
                    geometry_msgs::TransformStamped localPose;
                    localPose.header.frame_id = "kit_tray_" + std::to_string(agv_id);
                    std::cout << "Getting OrderPart, original pose position in vect: " << dest_pose.position.x << ", " << dest_pose.position.y << ", " << dest_pose.position.z << "\n";
                    std::cout << "Getting OrderPart, original pose orientatiuon in vect: " << dest_pose.orientation.x << ", " << dest_pose.orientation.y << ", " << dest_pose.orientation.z << ", " << dest_pose.orientation.w << "\n";
                    localPose.transform.translation.x = dest_pose.position.x;
                    localPose.transform.translation.y = dest_pose.position.y;
                    localPose.transform.translation.z = dest_pose.position.z;
                    localPose.transform.rotation.x = dest_pose.orientation.x;
                    localPose.transform.rotation.y = dest_pose.orientation.y;
                    localPose.transform.rotation.z = dest_pose.orientation.z;
                    localPose.transform.rotation.w = dest_pose.orientation.w;
                    op_.destination_pose = localPose;
                    i = partTypeCount; // end loop
                    return op_;  // Return and stop the for loop processing...
                }  
            }
        }
    }
    return op_;  
}

void Orders::update_parts_remain() {
    this->parts_remain_kit = 0;
    this->parts_remain_asm = 0;
    for (auto order : this->order_list) {
        for (auto os : order.second) {
            std::cout << " in update parts: " << os.order_shipment_type << ", " << os.part_type_pose_vect.size() << "\n";
            if (os.order_shipment_type == order_kit_order) this->parts_remain_kit += os.part_type_pose_vect.size();
            else this->parts_remain_asm += os.part_type_pose_vect.size();
        } 
    }
}

int Orders::UpdateOrder(OrderPart op_) {
    std::vector<OrderShipment>& osv_ = this->order_list[op_.order_number];  // Get '&' reference to original order in order_list, rather than making a copy of it
    int osv_size = osv_.size();
    int ret = -1;  // Basically this should never get returned...probably need error handling to deal with the case if it does, though
    for (int i=0; i < osv_size; i++) {
        if (osv_[i].order_shipment_number == op_.order_shipment_number) {
            std::cout << "In updateorder, order_shipment_number: " << osv_[i].order_shipment_number << "\n";
            std::cout << "In updateorder, order_shipment part_type_pose_vect size: " << osv_[i].part_type_pose_vect.size() << "\n";
            std::vector<std::pair<std::string, geometry_msgs::Pose>>::iterator it;
            it = osv_[i].part_type_pose_vect.begin() + op_.idx;
            std::cout << "In updateorder, original pose in vect: " << it->second.position.x << "\n";
            osv_[i].part_type_pose_vect.erase(it);
            std::cout << "In updateorder, this->order_shipment part_type_pose_vect size after delete: " << this->order_list[op_.order_number][i].part_type_pose_vect.size() << "\n";
            ret = osv_[i].part_type_pose_vect.size();
            i = osv_size;
        }
    }
    return ret; 
}

int Orders::SubmitOrderShipment(OrderPart op_) {
    int ret = 0;  // Returns 0 on failure, so hopefully this never happens...  /ariac/as{N}/submit_shipment 
    bool success = true;
    if (op_.order_shipment_number.find("kitting") != std::string::npos) {
        submit_client = order_node.serviceClient<nist_gear::AGVToAssemblyStation>("/ariac/"+op_.agv+"/submit_shipment");
        nist_gear::AGVToAssemblyStation srv;
        srv.request.shipment_type = op_.order_shipment_number;
        srv.request.assembly_station_name = op_.station;
        submit_client.call(srv);
        if (!srv.response.success) success = false;
    } else {
        submit_client = order_node.serviceClient<nist_gear::AssemblyStationSubmitShipment>("/ariac/"+op_.station+"/submit_shipment");
        nist_gear::AssemblyStationSubmitShipment srv;
        srv.request.shipment_type = op_.order_shipment_number;
        submit_client.call(srv);
        if (!srv.response.success) success = false;
    }
    
    // if (!submit_client.exists()) {
    //     ROS_INFO("Waiting for the client to be ready...");
    //     submit_client.waitForExistence();
    //     ROS_INFO("Service started.");
    // }

    // nist_gear::AGVToAssemblyStation srv;
    // srv.request.shipment_type = op_.order_number;
    // srv.request.assembly_station_name = op_.station;

    // submit_client.call(srv);

    if (!success) {
        ROS_ERROR_STREAM("Service failed!");
        printf("in submit shipment error\n");
    } else {
        ret = 1;
        printf("in submit shipment success\n");
        ROS_INFO("Service succeeded.");
        std::vector<OrderShipment>& osv_ = this->order_list[op_.order_number];
        int osv_size = osv_.size();
        for (int i=0; i < osv_size; i++) {
            if (osv_[i].order_shipment_number == op_.order_shipment_number) {
                osv_.erase(osv_.begin()+i);
            }
        }
    }
    return ret;
}
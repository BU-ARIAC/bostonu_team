#include <cstdio>
#include <string>
#include <geometry_msgs/Pose.h>
#include <nist_gear/Order.h>
#include <ros/ros.h>
#include <nist_gear/LogicalCameraImage.h>
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
            OrderShipment order_shipment;
            order_shipment.order_shipment_type = order_kit_order;
            order_shipment.priority = this->max_priority;
            auto shipments = order.kitting_shipments;
            for (const auto &shipment : shipments){
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
            }
            osv.emplace_back(order_shipment);
        }
        if (order.assembly_shipments.size() > 0) {
            OrderShipment order_shipment;
            order_shipment.order_shipment_type = order_asm_order;
            order_shipment.priority = this->max_priority;
            order_shipment.order_shipment_agv = "-1"; // No agv for asembly orders (agv already at station), so all get static value "-1"
            auto a_shipments = order.assembly_shipments;
            for (const auto &shipment : a_shipments){
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
            }
            osv.emplace_back(order_shipment);
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
        std::cout << "In getNextPart, in first for loop\n"; 
        if (ordershipment.order_shipment_type == shipment_type) {
            int partTypeCount = ordershipment.part_type_pose_vect.size();
            std::cout << "In getNextPart, partTypeCount: " << partTypeCount << "\n";
            for (int i = 0; i < partTypeCount; i++) {
                std::pair<std::string, geometry_msgs::Pose> ptp_pair = ordershipment.part_type_pose_vect[i];
                std::string part_type = ptp_pair.first;
                // std::cout << "In getNextPart, part_type: " << part_type << " and count? " << pl_->list_part_count["bin"].find(part_type)->second << "\n";
                // for(std::map<std::string,int>::iterator it = pl_->list_part_count["bin"].begin(); it != pl_->list_part_count["bin"].end(); ++it) {
                //   std::cout << "In getNextPart, all parts in list_part_count[bin]: \n";
                //   std::cout << "Key: " << it->first << "\n";
                //   std::cout << "Value: " << it->second << "\n";
                // }
                if (bp_->PartCount(part_type) > 0) {
                    std::cout << "Somehow we're in the depth of getNextPart: " << part_type << "\n";
                    // Set values to return
                    op_.order_number = highestPriorityOrder;
                    op_.order_shipment_number = ordershipment.order_shipment_number;
                    op_.part_count = 1;
                    op_.part_type = ordershipment.part_type_pose_vect[i].first;
                    op_.agv = ordershipment.order_shipment_agv;
                    op_.station = ordershipment.order_shipment_station;
                    // Get tf frame name of part's current position
                    op_.current_pose = bp_->GetFrame(op_.part_type);
                    // Build TransformStamped message of part's local pose on the agv (used to get world pose on agv later)
                    std::string agv = ordershipment.order_shipment_agv;
                    int agv_id = agv_numbers.find(agv)->second;
                    geometry_msgs::Pose dest_pose = ordershipment.part_type_pose_vect[i].second;
                    geometry_msgs::TransformStamped localPose;
                    localPose.header.frame_id = "kit_tray_" + std::to_string(agv_id);
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
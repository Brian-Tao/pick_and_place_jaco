#include "ros/ros.h"

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <gazebo_ros_link_attacher/Attach.h>

class pick_and_place{

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface* arm_group;
    moveit::planning_interface::MoveGroupInterface* gripper_group;
    const moveit::core::JointModelGroup* gripper_planning_group;
    ros::NodeHandle nh;
    ros::ServiceClient attach_client;
    ros::ServiceClient detach_client;

    void openGripper();
    void openGripper(trajectory_msgs::JointTrajectory& posture);
    void closeGripper();
    void closeGripper(trajectory_msgs::JointTrajectory& posture);

    bool attachGazeboObject();
    bool detachGazeboObject();
    bool attachRVizObject();
    bool detachRVizObject();

public:
    pick_and_place(ros::NodeHandle& nh, moveit::planning_interface::PlanningSceneInterface& psi);
    ~pick_and_place();

    bool pick();
    bool pickByStep();
    bool place();
    bool placeByStep();
    bool addCollision();

};
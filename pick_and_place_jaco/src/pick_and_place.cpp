#include "pick_and_place_jaco/pick_and_place.hpp"
#include <iostream>

pick_and_place::pick_and_place(ros::NodeHandle& nh, moveit::planning_interface::PlanningSceneInterface& psi){
    this->nh = nh;
    this->planning_scene_interface = psi;
    this->arm_group = new moveit::planning_interface::MoveGroupInterface("arm");
    this->gripper_group = new moveit::planning_interface::MoveGroupInterface("gripper");
    // this->gripper_planning_group = this->gripper_group->getCurrentState()->getJointModelGroup("gripper");
    // this->group->setPlanningTime(45.0);
    // this->arm_group->setPlannerId("RRTConnectkConfigDefault");
    this->attach_client = this->nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
    this->detach_client = this->nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
}

pick_and_place::~pick_and_place(){
    delete this->arm_group;
    delete this->gripper_group;
}

bool pick_and_place::addCollision(){
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    // first object to pick
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = this->arm_group->getPlanningFrame();
    collision_object.id = "box1";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.4;
    primitive.dimensions[1] = 0.2;
    primitive.dimensions[2] = 0.4;

    geometry_msgs::Pose box_pose;
    box_pose.position.x = 0.5;
    box_pose.position.y = 0;
    box_pose.position.z = 0.2;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    collision_objects.push_back(collision_object);

    //second object to place
    moveit_msgs::CollisionObject collision_object2;
    collision_object2.header.frame_id = this->arm_group->getPlanningFrame();
    collision_object2.id = "box2";

    shape_msgs::SolidPrimitive primitive2;
    primitive2.type = primitive2.BOX;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[0] = 0.4;
    primitive2.dimensions[1] = 0.2;
    primitive2.dimensions[2] = 0.4;

    geometry_msgs::Pose box_pose2;
    box_pose2.position.x = 0;
    box_pose2.position.y = 0.5;
    box_pose2.position.z = 0.2;

    collision_object2.primitives.push_back(primitive2);
    collision_object2.primitive_poses.push_back(box_pose2);
    collision_object2.operation = collision_object2.ADD;

    collision_objects.push_back(collision_object2);

    //third object to pick up
    moveit_msgs::CollisionObject to_pickup;
    to_pickup.header.frame_id = this->arm_group->getPlanningFrame();
    to_pickup.id = "to_pickup";

    shape_msgs::SolidPrimitive primitive3;
    primitive3.type = primitive3.BOX;
    primitive3.dimensions.resize(3);
    primitive3.dimensions[0] = 0.02;
    primitive3.dimensions[1] = 0.02;
    primitive3.dimensions[2] = 0.2;

    geometry_msgs::Pose to_pickup_pose;
    to_pickup_pose.position.x = 0.5;
    to_pickup_pose.position.y = 0;
    to_pickup_pose.position.z = 0.5;

    to_pickup.primitives.push_back(primitive3);
    to_pickup.primitive_poses.push_back(to_pickup_pose);
    to_pickup.operation = to_pickup.ADD;

    collision_objects.push_back(to_pickup);
    this->planning_scene_interface.addCollisionObjects(collision_objects);

    return true;
}

bool pick_and_place::pickByStep(){
    geometry_msgs::Pose target_pose1;
    tf2::Quaternion orientation;
    orientation.setRPY(-M_PI / 2, 0, -M_PI / 2);
    target_pose1.orientation = tf2::toMsg(orientation);
    target_pose1.position.x = 0.415;
    target_pose1.position.y = 0;
    target_pose1.position.z = 0.5;
    this->arm_group->setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (this->arm_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan (move arm) %s", success ? "" : "FAILED");
    this->arm_group->move();

    this->openGripper();

    geometry_msgs::Pose target_pose2 = target_pose1;
    target_pose2.position.x = 0.50;
    this->arm_group->setPoseTarget(target_pose2);
    success = (this->arm_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan (approach) %s", success ? "" : "FAILED");
    this->arm_group->move();

    this->closeGripper();

    this->attachRVizObject();
    this->attachGazeboObject();

    geometry_msgs::Pose target_pose3 = target_pose1;
    orientation.setRPY(-M_PI / 2, 0, -M_PI / 2);
    target_pose3.orientation = tf2::toMsg(orientation);
    target_pose3.position.z = 0.6;
    this->arm_group->setPoseTarget(target_pose3);
    success = (this->arm_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan (lift object) %s", success ? "" : "FAILED");
    this->arm_group->move();

    // geometry_msgs::Pose target_pose4 = target_pose1;
    // orientation.setRPY(-M_PI / 2, 0, 0);
    // target_pose4.orientation = tf2::toMsg(orientation);
    // this->arm_group->setPoseTarget(target_pose4);
    // success = (this->arm_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("tutorial", "Visualizing plan (try) %s", success ? "" : "FAILED");
    // this->arm_group->move();


    return true;
}

bool pick_and_place::placeByStep(){

    geometry_msgs::Pose target_pose1;
    tf2::Quaternion orientation;
    orientation.setRPY(-M_PI / 2, 0, 0);
    target_pose1.orientation = tf2::toMsg(orientation);
    target_pose1.position.x = 0.0;
    target_pose1.position.y = 0.5;
    target_pose1.position.z = 0.51;
    this->arm_group->setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (this->arm_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan (move arm) %s", success ? "" : "FAILED");
    this->arm_group->move();

    this->detachGazeboObject();
    this->detachRVizObject();

    this->openGripper();

    geometry_msgs::Pose target_pose2 = target_pose1;
    target_pose2.position.y = 0.3;
    this->arm_group->setPoseTarget(target_pose2);
    success = (this->arm_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan (retreat) %s", success ? "" : "FAILED");
    this->arm_group->move();

    return true;
}

bool pick_and_place::attachGazeboObject(){
    gazebo_ros_link_attacher::Attach attach_msg;
    attach_msg.request.model_name_1 = "j2n6s300";
    attach_msg.request.link_name_1 = "j2n6s300_link_6";
    attach_msg.request.model_name_2 = "to_pickup";
    attach_msg.request.link_name_2 = "link";

    return this->attach_client.call(attach_msg);
}

bool pick_and_place::detachGazeboObject(){
    gazebo_ros_link_attacher::Attach detach_msg;
    detach_msg.request.model_name_1 = "j2n6s300";
    detach_msg.request.link_name_1 = "j2n6s300_link_6";
    detach_msg.request.model_name_2 = "to_pickup";
    detach_msg.request.link_name_2 = "link";

    return this->detach_client.call(detach_msg);
}

bool pick_and_place::attachRVizObject(){
    std::vector<std::string> id_list;
    id_list.push_back("to_pickup");
    auto object_list = this->planning_scene_interface.getObjects(id_list);
    auto to_pickup_object = object_list["to_pickup"];
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = "j2n6s300_link_6";
    attached_object.object.header.frame_id = "world";
    attached_object.object = to_pickup_object;
    attached_object.object.operation = attached_object.object.ADD;

    this->planning_scene_interface.applyAttachedCollisionObject(attached_object);

}

bool pick_and_place::detachRVizObject(){
    moveit_msgs::AttachedCollisionObject detach_object;
    detach_object.object.id = "to_pickup";
    detach_object.link_name = "j2n6s300_link_6";
    detach_object.object.operation = detach_object.object.REMOVE;

    this->planning_scene_interface.applyAttachedCollisionObject(detach_object);
}

bool pick_and_place::pick(){

    geometry_msgs::Pose target_pose1;
    tf2::Quaternion orientation;
    orientation.setRPY(-M_PI / 2, 0, -M_PI / 2);
    target_pose1.orientation = tf2::toMsg(orientation);
    target_pose1.position.x = 0.415;
    target_pose1.position.y = 0;
    target_pose1.position.z = 0.5;
    this->arm_group->setPoseTarget(target_pose1);

    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // bool success = (this->arm_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("tutorial", "Visualizing plan (move arm) %s", success ? "" : "FAILED");
    // this->arm_group->move();

    std::vector<moveit_msgs::Grasp> grasps;
    // grasps.resize(1);

    moveit_msgs::Grasp grasp_pose;
    grasp_pose.id = "pick";
    grasp_pose.grasp_pose.header.frame_id = "world";
    grasp_pose.grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasp_pose.grasp_pose.pose.position.x = 0.45;
    grasp_pose.grasp_pose.pose.position.y = 0;
    grasp_pose.grasp_pose.pose.position.z = 0.5;

    grasp_pose.pre_grasp_approach.direction.header.frame_id = "world";
    grasp_pose.pre_grasp_approach.direction.vector.x = 1.0;
    grasp_pose.pre_grasp_approach.direction.vector.y = 0;
    grasp_pose.pre_grasp_approach.direction.vector.z = 0;
    grasp_pose.pre_grasp_approach.min_distance = 0.05;
    grasp_pose.pre_grasp_approach.desired_distance = 0.1;

    grasp_pose.post_grasp_retreat.direction.header.frame_id = "world";
    grasp_pose.post_grasp_retreat.direction.vector.z = 1.0;
    grasp_pose.post_grasp_retreat.direction.vector.x = 0.0;
    grasp_pose.post_grasp_retreat.direction.vector.y = 0.0;
    grasp_pose.post_grasp_retreat.min_distance = 0.02;
    grasp_pose.post_grasp_retreat.desired_distance = 0.25;

    openGripper(grasp_pose.pre_grasp_posture);
    closeGripper(grasp_pose.grasp_posture);
    grasps.push_back(grasp_pose);
    this->arm_group->setSupportSurfaceName("box1");
    this->arm_group->pick("to_pickup", grasps);

    

    // ros::WallDuration(1.0).sleep();
    // this->openGripper();

    // target_pose1.position.x = 0.45;
    // this->arm_group->setPoseTarget(target_pose1);
    // success = (this->arm_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("tutorial", "Visualizing plan (approach target) %s", success ? "" : "FAILED");
    // this->arm_group->move();

    // ros::WallDuration(1.0).sleep();
    // this->closeGripper();
    

    return true;

}

bool pick_and_place::place(){

    geometry_msgs::Pose target_pose1;
    tf2::Quaternion orientation;
    orientation.setRPY(-M_PI / 2, 0, -M_PI / 2);
    target_pose1.orientation = tf2::toMsg(orientation);
    target_pose1.position.x = 0.415;
    target_pose1.position.y = 0;
    target_pose1.position.z = 0.5;
    this->arm_group->setPoseTarget(target_pose1);

    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // bool success = (this->arm_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("tutorial", "Visualizing plan (move arm) %s", success ? "" : "FAILED");
    // this->arm_group->move();

    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    place_location[0].place_pose.header.frame_id = "world";
    orientation.setRPY(0, 0, M_PI / 2);
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

    place_location[0].place_pose.pose.position.x = 0;
    place_location[0].place_pose.pose.position.y = 0.5;
    place_location[0].place_pose.pose.position.z = 0.5;

    place_location[0].pre_place_approach.direction.header.frame_id = "world";
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.115;

    place_location[0].post_place_retreat.direction.header.frame_id = "world";
    place_location[0].post_place_retreat.direction.vector.y = -1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;

    openGripper(place_location[0].post_place_posture);

    this->arm_group->setSupportSurfaceName("box2");
    this->arm_group->place("to_pickup", place_location);
    return true;

}

void pick_and_place::openGripper(trajectory_msgs::JointTrajectory& posture){
    posture.joint_names.resize(3);
    posture.joint_names[0] = "j2n6s300_joint_finger_1";
    posture.joint_names[1] = "j2n6s300_joint_finger_2";
    posture.joint_names[2] = "j2n6s300_joint_finger_3";

    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(1);
    posture.points[0].positions.resize(3);
    posture.points[0].positions[0] = 0.0;
    posture.points[0].positions[1] = 0.0;
    posture.points[0].positions[2] = 0.0;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void pick_and_place::openGripper(){
    // moveit::core::RobotStatePtr current_state = this->gripper_group->getCurrentState();
    // std::cout << "opening the gripper" << std::endl;
    std::vector<double> joint_group_positions(3);
    // current_state->copyJointGroupPositions(gripper_planning_group, joint_group_positions);

    joint_group_positions[0] = 0.2;  // radians
    joint_group_positions[1] = 0.2;  // radians
    joint_group_positions[2] = 0.2;  // radians
    this->gripper_group->setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (this->gripper_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan (open gripper) %s", success ? "" : "FAILED");

    this->gripper_group->move();

}

void pick_and_place::closeGripper(trajectory_msgs::JointTrajectory& posture){
    posture.joint_names.resize(3);
    posture.joint_names[0] = "j2n6s300_joint_finger_1";
    posture.joint_names[1] = "j2n6s300_joint_finger_2";
    posture.joint_names[2] = "j2n6s300_joint_finger_3";

    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(1);
    posture.points[0].positions.resize(3);
    posture.points[0].positions[0] = 1.0;
    posture.points[0].positions[1] = 1.0;
    posture.points[0].positions[2] = 1.0;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void pick_and_place::closeGripper(){
    // moveit::core::RobotStatePtr current_state = this->gripper_group->getCurrentState();
    std::vector<double> joint_group_positions(3);
    // current_state->copyJointGroupPositions(gripper_planning_group, joint_group_positions);

    joint_group_positions[0] = 1.0;  // radians
    joint_group_positions[1] = 1.0;  // radians
    joint_group_positions[2] = 1.0;  // radians
    this->gripper_group->setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (this->gripper_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan (close gripper) %s", success ? "" : "FAILED");

    this->gripper_group->move();

}
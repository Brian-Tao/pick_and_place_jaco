#include "ros/ros.h"
#include "pick_and_place_jaco/pick_and_place.hpp"

int main(int argc, char** argv){
    
    ros::init(argc, argv, "pick_and_place_jaco");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    pick_and_place pick_and_place_object(nh, planning_scene_interface);

    pick_and_place_object.addCollision();
    
    ros::WallDuration(1.0).sleep();

    pick_and_place_object.pickByStep();

    ros::WallDuration(1.0).sleep();

    pick_and_place_object.placeByStep();

    ros::waitForShutdown();
    return 0;
}
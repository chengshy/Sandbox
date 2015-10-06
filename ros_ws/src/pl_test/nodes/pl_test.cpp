#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <tf/transform_listener.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/collision_detection/world.h>
#include <moveit/move_group/move_group_capability.h>
#include <moveit/macros/console_colors.h>
//#include <moveit_msgs/AttachedCollisionObject.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pl_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  robot_model_loader::RobotModelLoader robot_model_loader(ROBOT_DESCRIPTION);
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

  collision_detection::WorldPtr world(new collision_detection::World);
  shapes::ShapePtr ball(new shapes::Sphere(1.0));
  shapes::ShapePtr box(new shapes::Box(1,2,3));
  shapes::ShapePtr cyl(new shapes::Cylinder(2,3));

  world->addToObject("ball", ball, Eigen::Affine3d(Eigen::Translation3d(5,0,0)));
  bool ball_ok = world->hasObject("ball");
  ROS_INFO_STREAM(ball_ok);

  planning_scene::PlanningScenePtr ps(new planning_scene::PlanningScene(robot_model, world));

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  ps->checkCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 1: Current state is "
                  << (collision_result.collision ? "in" : "not in")
                  << " collision");
//  world->addToObject("box",box, Eigen::Affine3d(Eigen::Translation3d(0,5,0)));
//  moveit_msgs::PlanningScene ps_msg;
//  ps_msg.robot_state.is_diff = true;
//  ps->getPlanningSceneMsg(ps_msg);
//  ps->setPlanningSceneMsg(ps_msg);
//  psdp.publish(ps_msg);

//  boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener(ros::Duration(10.0)));
//  planning_scene_monitor::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION,tf));
//  if (psm->getPlanningScene())
//  {
//    psm->startSceneMonitor();
//    psm->startWorldGeometryMonitor();
//    psm->startStateMonitor();
//
//    bool debug = true;
//    move_group::MoveGroupExe mge(psm, debug);
//
//    psm->publishingDebugInformation(debug);
//    mge.status()
    ros::waitForShutdown();
//  }
//  else
//    ROS_ERROR("PlanningScene not configured");

  return 0;
}

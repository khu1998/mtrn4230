// spawn the red blocks on the conveyor belt
// and give them initial speed (by apply_body_wrench) to slide on conveyor

// ros communications:
// spawn model throught gazebo service: /gazebo/spawn_urdf_model
// initialize blocks speed: /gazebo/apply_body_wrench
// get urdf file path of blocks from parameter servicer
// publish all current blocks through topic: /current_blocks

#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SpawnModel.h>
#include <ros/ros.h>
#include <std_msgs/Int8MultiArray.h>
#include <urdf/model.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

int main(int argc, char **argv) {
  ros::init(argc, argv, "objects_spawner");
  ros::NodeHandle nh;
  srand(time(0));
  // service client for service /gazebo/spawn_urdf_model
  ros::ServiceClient spawn_client =
      nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
  gazebo_msgs::SpawnModel::Request spawn_model_req;
  gazebo_msgs::SpawnModel::Response spawn_model_resp;

  // ros::ServiceClient wrench_client =
  // nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
  // gazebo_msgs::ApplyBodyWrench::Request apply_wrench_req;
  // gazebo_msgs::ApplyBodyWrench::Response apply_wrench_resp;

  // publisher for current_blocks
  ros::Publisher current_blocks_publisher =
      nh.advertise<std_msgs::Int8MultiArray>("current_blocks", 1);
  std_msgs::Int8MultiArray current_blocks_msg;
  current_blocks_msg.data.clear();

  // make sure /gazebo/spawn_urdf_model service is service_ready
  bool service_ready = false;
  while (!service_ready) {
    service_ready = ros::service::exists("/gazebo/spawn_urdf_model", true);
    ROS_INFO("waiting for spawn_urdf_model service");
    ros::Duration(0.5).sleep();
  }
  ROS_INFO("spawn_urdf_model service is ready");

  // service_ready = false;
  // while (!service_ready) {
  //   service_ready = ros::service::exists("/gazebo/apply_body_wrench", true);
  //   ROS_INFO("waiting for apply_body_wrench service");
  //   ros::Duration(0.5).sleep();
  // }
  // ROS_INFO("apply_body_wrench service is ready");

  service_ready = false;
  while (!service_ready) {
    service_ready = ros::service::exists("/gazebo/set_model_state", true);
    ROS_INFO("waiting for set_model_state service");
    ros::Duration(0.5).sleep();
  }
  ROS_INFO("set_model_state service is ready");

  // get file path of blocks from parameter service
  std::string red_box_path;
  std::string blue_box_path;
  std::string yellow_box_path;
  std::string red_cylinder_path;
  std::string blue_cylinder_path;
  std::string yellow_cylinder_path;
  if (!nh.getParam("/red_box_path", red_box_path) ||
      !nh.getParam("/blue_box_path", blue_box_path) ||
      !nh.getParam("/yellow_box_path", yellow_box_path)) {
    ROS_ERROR("failed to get path to box and cylinder urdf files");
    return 1;
  } else {
    ROS_INFO("paths to box and cylinder urdf files have been found");
  }

  auto path_contents_to_string = [&](const std::string &path) {
    std::ifstream ifs(&path[0]);
    std::stringstream ss;
    ss << ifs.rdbuf();
    return ss.str();
  };

  std::string red_box_xml_str = path_contents_to_string(red_box_path);
  std::string blue_box_xml_str = path_contents_to_string(blue_box_path);
  std::string yellow_box_xml_str = path_contents_to_string(yellow_box_path);

  std::string red_cylinder_xml_str = path_contents_to_string(red_cylinder_path);
  std::string blue_cylinder_xml_str =
      path_contents_to_string(blue_cylinder_path);
  std::string yellow_cylinder_xml_str =
      path_contents_to_string(yellow_cylinder_path);

  // prepare the pawn model service message
  spawn_model_req.initial_pose.position.x = 0.0;
  spawn_model_req.initial_pose.position.y = 0.0;
  spawn_model_req.initial_pose.position.z = 0.2;
  spawn_model_req.initial_pose.orientation.x = 0.0;
  spawn_model_req.initial_pose.orientation.y = 0.0;
  spawn_model_req.initial_pose.orientation.z = 0.0;
  spawn_model_req.initial_pose.orientation.w = 1.0;
  spawn_model_req.reference_frame = "world";
  spawn_model_req.robot_namespace = "objects";

  // ros::Time time_temp(0, 0);
  // ros::Duration duration_temp(0, 1000000);
  // apply_wrench_req.wrench.force.x = -50;
  // apply_wrench_req.wrench.force.y = 0.0;
  // apply_wrench_req.wrench.force.z = 0.0;
  // apply_wrench_req.start_time = time_temp;
  // apply_wrench_req.duration = duration_temp;
  // apply_wrench_req.reference_frame = "world";
  const std::vector<std::tuple<std::string, double>> shapes = {
      std::make_tuple(red_box_xml_str, 0.4),
      std::make_tuple(blue_box_xml_str, 0),
      std::make_tuple(yellow_box_xml_str, -0.4)};
  std::size_t i = 0;
  while (ros::ok()) {
    for (const auto &shape : shapes) {
      // initialize model_name
      spawn_model_req.model_name = "object " + std::to_string(++i);
      spawn_model_req.model_xml = std::get<0>(shape);
      spawn_model_req.initial_pose.position.y = std::get<1>(shape);

      if (!spawn_client.call(spawn_model_req, spawn_model_resp)) {
        ROS_ERROR("failed to connect with gazebo server");
        return 1;
      }
      if (!spawn_model_resp.success) {
        ROS_ERROR("failed to spawn model");
        return 1;
      }

      // // prepare apply body wrench service message
      // apply_wrench_req.body_name = model_name + "::bottom";

      // // call apply body wrench service
      // if (!wrench_client.call(apply_wrench_req, apply_wrench_resp)) {
      //   ROS_ERROR("failed to connect with gazebo server");
      //   return 1;
      // }
      // if (!apply_wrench_resp.success) {
      //   ROS_ERROR("failed to initialize speed");
      //   return 1;
      // }
    }
    // publish current cylinder blocks status, all cylinder blocks will be
    // published no matter if it's successfully spawned, or successfully
    // initialized in speed
    current_blocks_publisher.publish(current_blocks_msg);

    // frequency control, spawn one cylinder in each loop; delay
    // time decides density of the cylinders
    ros::spinOnce();
    ros::Duration(2.0).sleep();
  }
  return 0;
}

// spawn the red blocks on the conveyor belt
// and give them initial speed (by apply_body_wrench) to slide on conveyor

// ros communications:
// spawn model throught gazebo service: /gazebo/spawn_urdf_model
// initialize blocks speed: /gazebo/apply_body_wrench
// get urdf file path of blocks from parameter servicer
// publish all current blocks through topic: /current_blocks

#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SpawnModel.h>
#include <ros/ros.h>
#include <std_msgs/Int8MultiArray.h>
#include <urdf/model.h>

#include <array>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

std::vector<std::string> get_spawn_objects_xml(const std::vector<std::string> &colours, const std::vector<std::string> &shapes, const ros::NodeHandle& nh);

int main(int argc, char **argv)
{
  unsigned int number_of_picks;
  int delay;
  std::vector<std::string> colours;
  std::vector<std::string> shapes;
  boost::program_options::options_description desc("Options");
  desc.add_options()
    ("help,h", "produce help message")
    ("delay,d", boost::program_options::value<int>(&delay)->default_value(10), "delay between object spawn. if delay is less than 0, user is prompted to press ENTER before object is spawned")
    ("colours,c", boost::program_options::value<std::vector<std::string>>(&colours), "colours to use to spawn objects. available colours: blue, red, yellow")
    ("number-of-picks,n", boost::program_options::value<unsigned int>(&number_of_picks)->default_value(10), "number of objects to be picked up by the robot")
    ("shapes,s", boost::program_options::value<std::vector<std::string>>(&shapes), "shapes to use to spawn objects. available shapes: box")
    ("random,r", "object spawn is random otherwise fixed location in the centre of the table")
  ;

  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << '\n';
    return 1;
  }

  const bool &random = vm.count("random");

  ros::init(argc, argv, "objects_spawner");
  ros::NodeHandle nh;
  srand(time(0));
  // service client for service /gazebo/spawn_urdf_model
  ros::ServiceClient spawn_client =
      nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
  gazebo_msgs::SpawnModel::Request spawn_model_req;
  gazebo_msgs::SpawnModel::Response spawn_model_resp;

  // publisher for current_blocks
  // ros::Publisher current_blocks_publisher =
  //     nh.advertise<std_msgs::Int8MultiArray>("current_blocks", 1);
  // std_msgs::Int8MultiArray current_blocks_msg;
  // current_blocks_msg.data.clear();

  // make sure /gazebo/spawn_urdf_model service is service_ready
  bool service_ready = false;
  while (!service_ready)
  {
    service_ready = ros::service::exists("/gazebo/spawn_urdf_model", true);
    ROS_INFO("waiting for spawn_urdf_model service");
    ros::Duration(0.5).sleep();
  }
  ROS_INFO("spawn_urdf_model service is ready");

  service_ready = false;
  while (!service_ready)
  {
    service_ready = ros::service::exists("/gazebo/set_model_state", true);
    ROS_INFO("waiting for set_model_state service");
    ros::Duration(0.5).sleep();
  }
  ROS_INFO("set_model_state service is ready");

  // prepare the spawn model service message
  spawn_model_req.initial_pose.position.x = 0.0;
  spawn_model_req.initial_pose.position.y = 0.0;
  spawn_model_req.initial_pose.position.z = 0.2;
  spawn_model_req.initial_pose.orientation.x = 0.0;
  spawn_model_req.initial_pose.orientation.y = 0.0;
  spawn_model_req.initial_pose.orientation.z = 0.0;
  spawn_model_req.initial_pose.orientation.w = 1.0;
  spawn_model_req.reference_frame = "world";
  spawn_model_req.robot_namespace = "spawn objects";

  // generate random numbers between 0 and 0.5 for spawning of blocks
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.0, 0.5);
  const auto& spawn_objects = get_spawn_objects_xml(colours, shapes, nh);
  while (ros::ok())
  {
    for (const auto &spawn_object : spawn_objects)
    {
      // initialize model_name
      spawn_model_req.model_name = "spawn object " + boost::lexical_cast<std::string>(boost::uuids::random_generator()());
      spawn_model_req.model_xml = spawn_object;
      if (random)
      {
        spawn_model_req.initial_pose.position.x = dis(gen);
        spawn_model_req.initial_pose.position.y = dis(gen);
      }

      if (!spawn_client.call(spawn_model_req, spawn_model_resp))
      {
        ROS_ERROR("failed to connect with gazebo server");
        return 1;
      }
      if (!spawn_model_resp.success)
      {
        ROS_ERROR("failed to spawn model");
        return 1;
      }

      // publish current cylinder blocks status, all cylinder blocks will be
      // published no matter if it's successfully spawned, or successfully
      // initialized in speed
      // current_blocks_publisher.publish(current_blocks_msg); // not sure if required

      // frequency control, spawn one cylinder in each loop; delay
      // time decides density of the cylinders
      ros::spinOnce();
      if (delay >= 0)
      {
        ros::Duration(delay).sleep();
      }
      else
      {
        std::cout << "Press ENTER to spawn a new object";
        std::cin.get();
      }
    }
  }
  return 0;
}

std::vector<std::string> get_spawn_objects_xml(const std::vector<std::string> &colours, const std::vector<std::string> &shapes, const ros::NodeHandle& nh)
{
  auto file_content_to_string = [&](const std::string &path) {
    std::ifstream ifs(&path[0]);
    std::stringstream ss;
    ss << ifs.rdbuf();
    return ss.str();
  };
  std::vector<std::string> spawn_objects_xml;
  std::string path;
  for (const auto &colour : colours)
  {
    for (const auto &shape : shapes)
    {
      if (!nh.getParam(boost::str(boost::format("/%1%_%2%_path") % colour % shape), path))
      {
        ROS_WARN_STREAM("failed to get path to urdf for " << colour << ' ' << shape);
        continue;
      }
      ROS_INFO_STREAM("path to urdf for " << colour << ' ' << shape << " file found");
      spawn_objects_xml.emplace_back(file_content_to_string(path));
    }
  }
  return spawn_objects_xml;
}
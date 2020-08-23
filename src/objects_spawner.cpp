// spawn the red blocks on the conveyor belt
// and give them initial speed (by apply_body_wrench) to slide on conveyor

// ros communications:
// spawn model throught gazebo service: /gazebo/spawn_urdf_model
// initialize blocks speed: /gazebo/apply_body_wrench
// get urdf file path of blocks from parameter servicer
// publish all current blocks through topic: /current_blocks

#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SpawnModel.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

static std::unordered_set<std::string>
    names;  // object names, only valid ones are kept, i.e. must be within
            // bounds of spawn location

static std::vector<std::string> get_spawn_objects_xml(
    const std::vector<std::string> &colours,
    const std::vector<std::string> &shapes, const ros::NodeHandle &nh);
static void update_objects(
    const gazebo_msgs::ModelStates &current_model_states);
static int getch();
static bool within_table_bounds(double x, double y, double z);

int main(int argc, char **argv) {
  unsigned int number_of_picks;
  int delay;
  std::vector<std::string> colours;
  std::vector<std::string> shapes;
  boost::program_options::options_description desc("Options");
  desc.add_options()("help,h", "produce help message")(
      "delay,d", boost::program_options::value(&delay)->default_value(10),
      "delay between object spawn. if delay is less than 0, user is prompted "
      "to press ENTER before object is spawned")(
      "colours,c",
      boost::program_options::value(&colours)->multitoken()->default_value(
          std::vector<std::string>{"red", "blue", "yellow"},
          "red, blue, yellow"),
      "colours to use to spawn objects. available colours: blue red yellow, if "
      "not specified defaults to all")(
      "number-of-picks,n",
      boost::program_options::value(&number_of_picks)->default_value(10),
      "number of objects to be picked up by the robot")(
      "shapes,s",
      boost::program_options::value(&shapes)->multitoken()->default_value(
          std::vector<std::string>{"box", "cylinder", "triangle"},
          "box, cylinder, triangle"),
      "shapes to use to spawn objects. available shapes: box cylinder "
      "triangle, if not specified, defaults to all")(
      "random,r",
      "object spawn is random otherwise fixed location in the "
      "centre of the table");

  boost::program_options::variables_map vm;
  boost::program_options::store(
      boost::program_options::parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);

  if (vm.count("help")) {
    std::cout << desc << '\n';
    return 0;
  }
  const bool &random = vm.count("random");

  ros::init(argc, argv, "objects_spawner");
  ros::NodeHandle nh;
  ros::ServiceClient spawn_client =
      nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
  ros::ServiceClient delete_client =
      nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
  ros::ServiceClient wrench_client =
      nh.serviceClient<gazebo_msgs::ApplyBodyWrench>(
          "/gazebo/apply_body_wrench");
  ros::Subscriber model_states_subscriber =
      nh.subscribe("/gazebo/model_states", 1, update_objects);
  gazebo_msgs::SpawnModel::Request spawn_model_req;
  gazebo_msgs::SpawnModel::Response spawn_model_resp;
  gazebo_msgs::DeleteModel::Request delete_model_req;
  gazebo_msgs::DeleteModel::Response delete_model_resp;
  gazebo_msgs::ApplyBodyWrench::Request apply_wrench_req;
  gazebo_msgs::ApplyBodyWrench::Response apply_wrench_resp;

  // make sure spawn model and delete model service is ready
  if (!ros::service::waitForService("/gazebo/spawn_urdf_model")) {
    ROS_ERROR("check for service /gazebo/spawn_urdf_model");
    return 1;
  }
  ROS_INFO("/gazebo/spawn_urdf_model service is ready");

  if (!ros::service::waitForService("/gazebo/delete_model")) {
    ROS_ERROR("check for service /gazebo/delete_model");
    return 1;
  }
  ROS_INFO("/gazebo/delete_model service is ready");

  if (!ros::service::waitForService("/gazebo/apply_body_wrench")) {
    ROS_ERROR("check for service /gazebo/apply_body_wrench");
    return 1;
  }
  ROS_INFO("/gazebo/apply_body_wrench service is ready");

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

  // prepare conveyor belt service
  apply_wrench_req.wrench.force.x = 0.0;
  apply_wrench_req.wrench.force.y = -1.0;
  apply_wrench_req.wrench.force.z = 0.0;
  apply_wrench_req.duration = ros::Duration(0.25);
  apply_wrench_req.reference_frame = "world";

  auto clear_table = [&]() -> void {
    for (auto name : names) {
      apply_wrench_req.body_name = name + "::base_link";
      apply_wrench_req.start_time = ros::Time::now();
      if (!wrench_client.call(apply_wrench_req, apply_wrench_resp)) {
        ROS_ERROR("failed to connect with gazebo server");
        std::exit(1);
      }
      if (!apply_wrench_resp.success) {
        ROS_ERROR("failed to clear table");
        std::exit(1);
      }
    }
    names.clear();
  };

  auto delete_objects = [&]() -> void {
    for (auto name : names) {
      delete_model_req.model_name = name;
      if (!delete_client.call(delete_model_req, delete_model_resp)) {
        ROS_ERROR("failed to connect with gazebo server");
        std::exit(1);
      }
      if (!delete_model_resp.success) {
        ROS_ERROR("failed to delete objects");
        std::exit(1);
      }
    }
    names.clear();
  };

  // generate random numbers between 0 and 0.5 for spawning of blocks
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> xdis(-0.45, 0.45);
  std::uniform_real_distribution<> ydis(0.2, 0.45);
  const auto &spawn_objects = get_spawn_objects_xml(colours, shapes, nh);
  while (ros::ok()) {
    for (const auto &spawn_object : spawn_objects) {
      if (delay < 0) {
        ROS_INFO_STREAM(
            "Press <ENTER> to spawn a new object, <d> to delete all objects, "
            "<c> to clear objects off table, or <q> to stop");
        auto ch = ::getch();
        ros::spinOnce();  // update model states
        switch (ch) {
          case 10:
            ROS_INFO_STREAM("Spawning new object...");
            break;
          case 99:
            ROS_INFO_STREAM("Clearing objects off table...");
            clear_table();
            ROS_INFO_STREAM("Finished clearing objects off table");
            continue;
            break;
          case 100:
            ROS_INFO_STREAM("Deleting all objects on table...");
            delete_objects();
            ROS_INFO_STREAM("Finished deleting objects off table");
            continue;
            break;
          case 113:
            ROS_INFO_STREAM("Quitting...");
            return 0;
            break;
          default:
            ROS_INFO_STREAM("Please press one of <ENTER>, <d>, <c>, or <q>");
            continue;
            break;
        }
      }

      // initialize model_name
      spawn_model_req.model_name =
          "object-" +
          boost::lexical_cast<std::string>(boost::uuids::random_generator()());
      spawn_model_req.model_xml = spawn_object;
      if (random) {
        spawn_model_req.initial_pose.position.x = xdis(gen);
        spawn_model_req.initial_pose.position.y = ydis(gen);
      }

      if (!spawn_client.call(spawn_model_req, spawn_model_resp)) {
        ROS_ERROR("failed to connect with gazebo server");
        return 1;
      }
      if (!spawn_model_resp.success) {
        ROS_ERROR("failed to spawn model");
        return 1;
      }

      ros::spinOnce();

      if (delay >= 0) {
        ros::Duration(delay).sleep();
      }
    }
  }
  return 0;
}

static std::vector<std::string> get_spawn_objects_xml(
    const std::vector<std::string> &colours,
    const std::vector<std::string> &shapes, const ros::NodeHandle &nh) {
  auto read_file = [&](const std::string &path) -> std::string {
    auto stream = std::ifstream(path, std::ios::in | std::ios::ate);
    auto file_size = stream.tellg();
    stream.seekg(0);
    std::string contents(file_size, 0);
    stream.read(&contents[0], contents.size());
    return contents;
  };
  std::vector<std::string> spawn_objects_xml;
  std::string path;
  for (const auto &colour : colours) {
    for (const auto &shape : shapes) {
      if (!nh.getParam(
              boost::str(boost::format("/%1%_%2%_path") % colour % shape),
              path)) {
        ROS_WARN_STREAM("failed to get path to urdf for " << colour << ' '
                                                          << shape);
        continue;
      }
      ROS_INFO_STREAM("path to urdf for " << colour << ' ' << shape
                                          << " file found");
      spawn_objects_xml.emplace_back(read_file(path));
    }
  }
  return spawn_objects_xml;
}

static void update_objects(
    const gazebo_msgs::ModelStates &current_model_states) {
  for (std::size_t i = 0; i < current_model_states.name.size(); ++i) {
    double x = current_model_states.pose[i].position.x;
    double y = current_model_states.pose[i].position.y;
    double z = current_model_states.pose[i].position.z;
    if (current_model_states.name[i].find("object-") == 0) {
      if (within_table_bounds(x, y, z)) {
        names.insert(current_model_states.name[i]);
      } else {
        auto it = names.find(current_model_states.name[i]);
        if (it != names.end()) {
          names.erase(it);
        }
      }
    }
  }
}

static int getch() {
  static struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);  // save old settings
  newt = oldt;
  newt.c_lflag &=
      ~ICANON & ~ECHO;  // disable buffering and echo of character in terminal
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // apply new settings
  int c = getchar();                        // read character (non-blocking)
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

static bool within_table_bounds(double x, double y, double z) {
  return (-0.5 <= x && x <= 0.5) && (-0.5 <= y && y <= 0.5);
}

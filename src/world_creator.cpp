#include "ros/ros.h"
#include <fstream>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

#define USAGE ""
void createWorld(std::string const *world_path, std::string const *map_path, int length, int width, int height)
{
  std::ofstream buff(world_path->c_str());
  buff << "<?xml version=\"1.0\" ?>\n"
       "<sdf version=\"1.4\">\n"
       "<world name=\"default\">\n"
       "<!-- A global light source -->"
       "<include>\n"
       "<uri>model://sun</uri>\n"
       "</include>\n";
  ROS_INFO("Writing started"); // выводим сообщение
  ROS_INFO("Started creating Gazebo world file");

  buff << "<model name=\"heightmap\">\n"
       "<static>true</static>\n"
       "<link name=\"link\">\n"
       "<collision name=\"collision1\">\n"
       "<geometry>\n"
       "<heightmap>\n"
       "<uri>\n"
       "file://";
  buff << map_path;
  buff << "</uri>\n"
       "<size>";
  buff << length;
       buff << " ";
       buff << width;
       buff << " ";
       buff << height;
       buff << "</size>\n"
       "<pos>0 0 0</pos>\n"
       "</heightmap>\n"
       "</geometry>\n"
       "</collision>\n"
       "<visual name=\"visual_abcedf1\">\n"
       "<geometry>\n"
       "<heightmap>\n"
       "<use_terrain_paging>false</use_terrain_paging>\n"
       "<texture>\n"
       "<diffuse>file://media/materials/textures/fungus_diffusespecular.png</diffuse>\n"
       "<normal>file://media/materials/textures/flat_normal.png</normal>\n"
       "<size>0.5</size>\n"
       "</texture>\n"
       "<texture>\n"
       "<diffuse>file://media/materials/textures/dirt_diffusespecular.png</diffuse>\n"
       "<normal>file://media/materials/textures/grass_normal.jpg</normal>\n"
       "<size>5</size>\n"
       "</texture>\n"
       "<texture>\n"
       "<diffuse>file://media/materials/textures/white.bmp</diffuse>\n"
       "<normal>file://media/materials/textures/white.bmp</normal>\n"
       "<size>1</size>\n"
       "</texture>\n"
       "<blend>\n"
       "<min_height>0.1</min_height>\n"
       "<fade_dist>0</fade_dist>\n"
       "</blend>\n"
       "<blend>\n"
       "<min_height>0.7</min_height>\n"
       "<fade_dist>0</fade_dist>\n"
       "</blend>\n"
       "<uri>\n"
       "file://";
  buff << map_path;
  buff << "</uri>\n"
       "<size>";
  buff << length;
       buff << " ";
       buff << width;
       buff << " ";
       buff << height;
       buff << "</size>\n"
       "<pos>0 0 0</pos>\n"
       "</heightmap>\n"
       "</geometry>\n"
       "</visual>\n"
       "</link>\n"
       "</model>\n";

  buff << "</world> \n </sdf>";
  buff.close();
  ROS_INFO("File closed");
}

void display_parameters(po::variables_map const *vm) {
  if (vm->count("l")) {
    std::cout << "World length was set to "
         << vm->at("w").as<int>() << "\n";
  } else {
    std::cout << "World length was not set. \n";
  }

  if (vm->count("w")) {
    std::cout << "World width was set to "
         << vm->at("w").as<int>() << "\n";
  } else {
    std::cout << "World width was not set. \n";
  }

  if (vm->count("h")) {
    std::cout << "World height was set to "
         << vm->at("h").as<int>() << "\n";
  } else {
    std::cout << "World height was not set. \n";
  }
}

bool check_parameters(po::variables_map const *vm) {
  if (vm->count("help")) {
    return false;
  }
  if (vm->count("f")) {
    std::cout << "Source image path was set to "
         << vm->at("f").as<std::string>() << "\n";
  } else {
    return false;
  }
  return true;
}

int main(int argc, char **argv)
{
  std::string map_path;
  int width=60;
  int length=60;
  int height=2;

  po::options_description desc("Allowed options");
  desc.add_options()
  ("help", "Show help message")
  ("f", po::value<std::string>(&map_path), "Path to source image")
  ("l", po::value<int>(&length)->default_value(60), "Generated heightmap length in meters.")
  ("w", po::value<int>(&width)->default_value(60), "Generated heightmap width in meters.")
  ("h", po::value<int>(&height)->default_value(2), "Generated heightmap height in meters.");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  if (!check_parameters(&vm)) {
    std::cout << desc << "\n";
    return 1;
  }
  display_parameters(&vm);

  size_t found = map_path.find_last_of('/');
  std::string world_path = map_path.substr(0, found);
  world_path += "/generated_world.world";
  //Initiate ROS
  ros::init(argc, argv, "world_creator");
  createWorld(&world_path, &map_path,length,width,height);
  return 0;
}

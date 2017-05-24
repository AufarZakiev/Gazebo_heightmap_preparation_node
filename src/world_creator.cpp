#include "ros/ros.h"
#include <fstream>
#include <string>
#include <iostream>

#define USAGE ""
void createWorld(std::string world_path, std::string map_path)  
  {
    std::ofstream buff(world_path.c_str());
    buff << "<?xml version=\"1.0\" ?>\n"
                  "<sdf version=\"1.4\">\n"
                  "<world name=\"default\">\n"
                  "<!-- A global light source -->"
                  "<include>\n"
                  "<uri>model://sun</uri>\n"
                  "</include>\n";
    buff << "<physics name=\"ode_200iters\" type=\"ode\" default=\"true\">\n"
          "<real_time_update_rate>250</real_time_update_rate>\n"
        "</physics>\n";
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
    buff << "60 60 2"; // arg width height
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
                "<diffuse>file://media/materials/textures/white.bmp</diffuse>\n"
                "<normal>file://media/materials/textures/white.bmp</normal>\n"
                "<size>1</size>\n"
              "</texture>\n"
              "<texture>\n"
                "<diffuse>file://media/materials/textures/dirt_diffusespecular.png</diffuse>\n"
                "<normal>file://media/materials/textures/grass_normal.jpg</normal>\n"
                "<size>5</size>\n"
              "</texture>\n"
              "<texture>\n"
                "<diffuse>file://media/materials/textures/fungus_diffusespecular.png</diffuse>\n"
                "<normal>file://media/materials/textures/flat_normal.png</normal>\n"
                "<size>0.5</size>\n"
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
    buff << "60 60 2"; // arg width height
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
int main(int argc, char **argv)
{
  std::string map_path="";
  for(int i=0; i<argc; i++)
  {
    if(!strcmp(argv[i], "-f"))
    {
      if(++i < argc){
        map_path = argv[i];
      }
      else
      {
        puts(USAGE);
        return 1;
      } 
    }
  } 
  size_t found = map_path.find_last_of("/");
  std::string world_path=map_path.substr(0,found);
  world_path+="/generated_world.world";
  //Initiate ROS
  ros::init(argc, argv, "world_creator");
  createWorld(world_path, map_path);
  return 0;
}

#include "ros/ros.h"
#include <fstream>
#include <Magick++.h>
#include <unistd.h>

using namespace Magick;

#define USAGE ""

bool fexists(std::string filename)
{
  std::ifstream ifile(filename.c_str());
  return ifile;
}

int main(int argc, char **argv)
{
  //Initiate ROS
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
	ros::init(argc, argv, "map_pgm2png");
	std::string save_path="";

	size_t found = map_path.find_last_of("/");
	save_path=map_path.substr(0,found);

	sleep(5);

	Image source_image(map_path);
	Image sub_image(source_image);

	sub_image.magick("png");
	sub_image.type(GrayscaleType);
	sub_image.negate();
	save_path+="/new_map_from_pgm.png";
	sub_image.write(save_path);
	return 0;
}

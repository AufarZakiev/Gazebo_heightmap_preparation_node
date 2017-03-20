#include "ros/ros.h"
#include <Magick++.h>

using namespace Magick;

#define USAGE "Usage: \n" \
"map_saver -f <map_path> \n"\
"map_saver -offset <x_offset> <y_offset> \n"\
"map_saver -w <min_resulting_width> \n"\
"map_saver -h <min_resulting_height> \n"


int main(int argc, char **argv)
{
  //Initiate ROS
	ros::init(argc, argv, "map_preparing");
	std::string map_path="";
	std::string save_path="";
	size_t x_offset=0;
	size_t y_offset=0;

	if(!strcmp(argv[1], "-f"))
	{
		if(2 < argc)
			map_path = argv[2];
		else
		{
			puts(USAGE);
			return 1;
		}
	}else {
		puts(USAGE);
		return 1;
	}

	size_t found = map_path.find_last_of("/");
	save_path=map_path.substr(0,found);

	Image source_image(map_path);
	size_t temp=0;
	size_t min_width=source_image.baseColumns();
	size_t min_height=source_image.baseRows();
	size_t max_size=0;

	for(int i=3; i<argc; i++)
	{
		if(!strcmp(argv[i], "-s"))
		{
			if(++i < argc){
				save_path = argv[i];
			}
			else
			{
				puts(USAGE);
				return 1;
			} 
		}
		else if(!strcmp(argv[i], "-offset"))
		{
			if(++i < argc){
				temp = atoi(argv[i]);
				if(temp<=source_image.baseColumns()){
					x_offset = temp;
				}
				++i;
				temp = atoi(argv[i]);
				if(temp<=source_image.baseRows()){
					y_offset = temp;
				}
			}
			else
			{
				puts(USAGE);
				return 1;
			} 
		}
		else if(!strcmp(argv[i], "-w"))
		{
			if(++i < argc){
				temp = atoi(argv[i]);
				if(temp<=source_image.baseColumns() && (temp<=(source_image.baseColumns()-x_offset))){
					min_width = temp;
					if(min_width>max_size){
						max_size=min_width;
					}
				}
			}
		}
		else if(!strcmp(argv[i], "-h"))
		{
			if(++i < argc){
				temp = atoi(argv[i]);
				if(temp<=source_image.baseRows() && (temp<=(source_image.baseRows()-y_offset))){
					min_height = temp;
					if(min_height>max_size){
						max_size=min_height;
					}
				}
			}
		}
	}

	Image sub_image(source_image);

	int calculated_size=1;
	while((calculated_size+1)<max_size){
		calculated_size=calculated_size*2;
	}
	calculated_size+=1; // Gazebo needs (2^n)+1 px size images  

	sub_image.magick("png");
	sub_image.chop(Geometry(x_offset,y_offset));
	sub_image.crop(Geometry(calculated_size,calculated_size));
	sub_image.type(GrayscaleType);
	save_path+="/new_map.png";
	sub_image.write(save_path);
	return 0;
}

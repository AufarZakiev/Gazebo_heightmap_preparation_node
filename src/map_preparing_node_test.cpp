#include "ros/ros.h"
#include <Magick++.h>
#include <map> 

using namespace Magick;

#define USAGE "Usage: \n" \
"map_saver -f <map_path> \n"\
"map_saver -offset <x_offset> <y_offset> \n"\
"map_saver -w <min_resulting_width> \n"\
"map_saver -h <min_resulting_height> \n"


bool is_point_exist(size_t row,size_t column,size_t width,size_t height){
	if((row>=height) || (column>=width) || (row==SIZE_MAX) || (column==SIZE_MAX) || (row==SIZE_MAX-1) || (column==SIZE_MAX-1) ) return false;
	return true;
}

int convert_to_index(size_t i,size_t column,size_t width){
	return (width*i)+column;
}

int main(int argc, char **argv)
{
  //Initiate ROS
	ros::init(argc, argv, "map_preparing_test");
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
	size_t max_size=400; //TODO с заполением пикселей при отсутствии обрезки

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

	size_t calculated_size=1;
	while((calculated_size+1)<max_size){
		calculated_size=calculated_size*2;
	}
	calculated_size+=1; // Gazebo needs (2^n)+1 px size images  

	ROS_INFO("Calculated size: %zu",calculated_size);

	sub_image.magick("png");
	sub_image.chop(Geometry(x_offset,y_offset));
	sub_image.crop(Geometry(calculated_size,calculated_size));
	sub_image.type(GrayscaleType);

	size_t w = sub_image.columns();
	ROS_INFO("Width: %zu",w);
	size_t h=sub_image.rows();
	ROS_INFO("Height: %zu",h);
	PixelPacket *pixels = sub_image.getPixels(0,0,w,h);
	Image filtered_image(sub_image);
	PixelPacket *filtered_pixels = filtered_image.getPixels(0,0,w,h);
	std::map<Color,int> neighbours;
	for(size_t i=0;i<h;i++){
		for(size_t j=0;j<w;j++){
			neighbours.clear();
			neighbours[pixels[convert_to_index(i,j,w)]]++;
			if(is_point_exist(i-1,j-1,w,h)){
				neighbours[pixels[convert_to_index(i-1,j-1,w)]]++;
			}
			if(is_point_exist(i-1,j,w,h)){
				neighbours[pixels[convert_to_index(i-1,j,w)]]++;
			}
			if(is_point_exist(i-1,j+1,w,h)){
				neighbours[pixels[convert_to_index(i-1,j+1,w)]]++;
			}
			if(is_point_exist(i,j-1,w,h)){
				neighbours[pixels[convert_to_index(i,j-1,w)]]++;
			}
			if(is_point_exist(i,j+1,w,h)){	
				neighbours[pixels[convert_to_index(i,j+1,w)]]++;
			}
			if(is_point_exist(i+1,j-1,w,h)){
				neighbours[pixels[convert_to_index(i+1,j-1,w)]]++;
			}
			if(is_point_exist(i+1,j,w,h)){
				neighbours[pixels[convert_to_index(i+1,j,w)]]++;
			}
			if(is_point_exist(i+1,j+1,w,h)){
				neighbours[pixels[convert_to_index(i+1,j+1,w)]]++;
			}
			Color max_value=neighbours.find(pixels[convert_to_index(i,j,w)])->first;
			int max_value_count=0;
			for (std::map<Color,int>::iterator it=neighbours.begin(); it!=neighbours.end(); ++it){
    			if (it->second>max_value_count) {
    				max_value_count=it->second;
    				max_value=it->first;
    			}
    		}

  			filtered_pixels[convert_to_index(i,j,w)]=max_value;
		}
	}
	for(size_t i=0;i<h;i++){
		for(size_t j=0;j<w;j++){
			if(is_point_exist(i-1,j,w,h) && is_point_exist(i+1,j,w,h)){
				if (pixels[convert_to_index(i-1,j,w)]==pixels[convert_to_index(i,j,w)] && 
					pixels[convert_to_index(i,j,w)]==pixels[convert_to_index(i+1,j,w)]){
					filtered_pixels[convert_to_index(i-1,j,w)]=pixels[convert_to_index(i-1,j,w)];
					filtered_pixels[convert_to_index(i,j,w)]=pixels[convert_to_index(i,j,w)];
					filtered_pixels[convert_to_index(i+1,j,w)]=pixels[convert_to_index(i+1,j,w)];
				}
			}
			if(is_point_exist(i,j-1,w,h) && is_point_exist(i,j+1,w,h)){
				if (pixels[convert_to_index(i,j-1,w)]==pixels[convert_to_index(i,j,w)] && 
					pixels[convert_to_index(i,j,w)]==pixels[convert_to_index(i,j+1,w)]){
					filtered_pixels[convert_to_index(i,j-1,w)]=pixels[convert_to_index(i,j-1,w)];
					filtered_pixels[convert_to_index(i,j,w)]=pixels[convert_to_index(i,j,w)];
					filtered_pixels[convert_to_index(i,j+1,w)]=pixels[convert_to_index(i,j+1,w)];
				}
			}
		}
	}
	save_path+="/new_map.png";
	filtered_image.write(save_path);
	return 0;
}
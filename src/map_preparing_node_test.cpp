#include "ros/ros.h"
#include <Magick++.h>

using namespace Magick;

#define USAGE "Usage: \n" \
"map_saver -f <map_path> \n"\
"map_saver -offset <x_offset> <y_offset> \n"\
"map_saver -w <min_resulting_width> \n"\
"map_saver -h <min_resulting_height> \n"


bool is_point_exist(size_t row,size_t column,size_t width,size_t height){
	if((row==height) || (column==width) || (row==SIZE_MAX) || (column==SIZE_MAX)) return false;
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
	size_t min_width=400;
	size_t min_height=400;
	size_t max_size=min_width;

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
	size_t row=0;
	size_t column=0;
	size_t temp_g=0;
	size_t temp_w=0;
	size_t temp_b=0;
	for(size_t i=0;i<h;i++){
		for(size_t j=0;j<w;j++){
			int white_count=0;
			int gray_count=0;
			int black_count=0;
			if(pixels[convert_to_index(i,j,w)]==Color(65278,65278,65278)){
	  			white_count+=1;
	  		}
	  		else if (pixels[convert_to_index(i,j,w)]==Color(0,0,0)){
	  			black_count+=1;
	  		}
	  		else {
	  			gray_count+=1;	  			
	  		}
			if (is_point_exist(i-1,j-1,w,h)){
		  		if(pixels[convert_to_index(i-1,j-1,w)]==Color(65278,65278,65278))
		  			white_count+=1;
		  		else if (pixels[convert_to_index(i-1,j-1,w)]==Color(0,0,0))
		  			black_count+=1;
		  		else gray_count+=1;
		  	}
		  	if (is_point_exist(i-1,j,w,h)){
		  		if(pixels[convert_to_index(i-1,j,w)]==Color(65278,65278,65278))
		  			white_count+=1;
		  		else if (pixels[convert_to_index(i-1,j,w)]==Color(0,0,0))
		  			black_count+=1;
		  		else gray_count+=1;
		  	}
		  	if (is_point_exist(i-1,j+1,w,h)) {
		  		if(pixels[convert_to_index(i-1,j+1,w)]==Color(65278,65278,65278))
		  			white_count+=1;
		  		else if (pixels[convert_to_index(i-1,j+1,w)]==Color(0,0,0))
		  			black_count+=1;
		  		else gray_count+=1;
		  	}
		  	//same i points
		  	if (is_point_exist(i,j-1,w,h)){
		  		if(pixels[convert_to_index(i,j-1,w)]==Color(65278,65278,65278))
		  			white_count+=1;
		  		else if (pixels[convert_to_index(i,j-1,w)]==Color(0,0,0))
		  			black_count+=1;
		  		else gray_count+=1;
		  	}
		  	if (is_point_exist(i,j+1,w,h)){
		  		if(pixels[convert_to_index(i,j+1,w)]==Color(65278,65278,65278))
		  			white_count+=1;
		  		else if (pixels[convert_to_index(i,j+1,w)]==Color(0,0,0))
		  			black_count+=1;
		  		else gray_count+=1;
		  	}
		  	//bottom three points
		  	if (is_point_exist(i+1,j-1,w,h)){
		  		if(pixels[convert_to_index(i+1,j-1,w)]==Color(65278,65278,65278))
		  			white_count+=1;
		  		else if (pixels[convert_to_index(i+1,j-1,w)]==Color(0,0,0))
		  			black_count+=1;
		  		else gray_count+=1;
		  	}
		  	if (is_point_exist(i+1,j,w,h)){
		  		if(pixels[convert_to_index(i+1,j,w)]==Color(65278,65278,65278))
		  			white_count+=1;
		  		else if (pixels[convert_to_index(i+1,j,w)]==Color(0,0,0))
		  			black_count+=1;
		  		else gray_count+=1;
		  	}
		  	if (is_point_exist(i+1,j+1,w,h)){
		  		if(pixels[convert_to_index(i+1,j+1,w)]==Color(65278,65278,65278))
		  			white_count+=1;
		  		else if (pixels[convert_to_index(i+1,j+1,w)]==Color(0,0,0))
		  			black_count+=1;
		  		else gray_count+=1;
		  	}
		  	if (white_count>black_count && white_count>gray_count){
		  		pixels[convert_to_index(i,j,w)]=Color(65278,65278,65278);
		  		temp_w++;
		  	} else if(gray_count>black_count && gray_count>white_count){
		  		pixels[convert_to_index(i,j,w)]=Color(52685,52685,52685);
		  		temp_g++;
		  	} else if(black_count>gray_count && black_count>white_count){
		  		pixels[convert_to_index(i,j,w)]=Color(0,0,0);
		  		temp_b++;
		  	}
		}
	}
	ROS_INFO("w: %zu",temp_w);
	ROS_INFO("g: %zu",temp_g);
	ROS_INFO("b: %zu",temp_b);
	sub_image.syncPixels();
	sub_image.negate();
	save_path+="/new_map.png";
	sub_image.write(save_path);
	return 0;
}
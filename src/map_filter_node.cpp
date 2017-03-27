#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

#define MAX_RANK 10
#define MIN_RANK 0
#define MINIMUM_PASSING_RANK 4

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<nav_msgs::OccupancyGrid>("/filtered_map", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/map", 1, &SubscribeAndPublish::callback, this);
  }

  void callback(const nav_msgs::OccupancyGrid &inputGrid)
  {
  	
    ROS_INFO("Started inputGrid header processing");

    size_t width=inputGrid.info.width;
    size_t height=inputGrid.info.height;

    ROS_INFO("Width: %zu", width);
    ROS_INFO("Hegiht: %zu", height);

    filtered_map.header=inputGrid.header;
    filtered_map.info=inputGrid.info;
    filtered_map.data=inputGrid.data;

    ROS_INFO("Preparing for grid filtering");
  	int ranks[width*height];
    for(int i=0;i<width*height;i++){
    	ranks[i]=MIN_RANK;
    }

    ROS_INFO("Started grid filtering");
    int count=0;
    int not_passed_count=0;
    ROS_INFO("not_passed_count = %d",not_passed_count);
	not_passed_count=0; 
	for(size_t i=0;i<height;++i){
	  	for(size_t j=0;j<width;++j){
	   		ranks[convert_to_index(i,j,width)]=compute_rank(i,j,width,height);
	    	count++;
	    	if(ranks[convert_to_index(i,j,width)]<MINIMUM_PASSING_RANK){
	    		filtered_map.data[convert_to_index(i,j,width)]=0;
	    		not_passed_count+=1;
	    	}
	   	}
	}
	ROS_INFO("Processed points count: %d", count);
	ROS_INFO("Processed not_passed_count points count: %d", not_passed_count);
	ROS_INFO("Finished inputGrid processing");

	ROS_INFO("Publishing...");
    pub_.publish(filtered_map);
    ROS_INFO("Finished publishing.");
  }

  int compute_rank(size_t row, size_t column,size_t width,size_t height){
  	int rank=0;
  	//top row points
  	int conv_row=row;
  	int conv_column=column;
  	//ROS_INFO("row = %zu",row);
  	//ROS_INFO("column = %d",conv_column);
  	if (is_point_exist(conv_row-1,conv_column-1,width,height)){
  		if(filtered_map.data[convert_to_index(conv_row-1,conv_column-1,width)]==-1)
  			return MAX_RANK;
  		if(filtered_map.data[convert_to_index(conv_row-1,conv_column-1,width)]==100)
  			rank+=1;
  	}
  	if (is_point_exist(conv_row-1,conv_column,width,height)){
  		if(filtered_map.data[convert_to_index(conv_row-1,conv_column,width)]==-1)
  			return MAX_RANK;
  		if(filtered_map.data[convert_to_index(conv_row-1,conv_column,width)]==100)
  			rank+=1;
  	}
  	if (is_point_exist(conv_row-1,conv_column+1,width,height)){
  		if(filtered_map.data[convert_to_index(conv_row-1,conv_column+1,width)]==-1)
  			return MAX_RANK;
  		if(filtered_map.data[convert_to_index(conv_row-1,conv_column+1,width)]==100)
  			rank+=1;
  	}
  	//same conv_row points
  	if (is_point_exist(conv_row,conv_column-1,width,height)){
  		if(filtered_map.data[convert_to_index(conv_row,conv_column-1,width)]==-1)
  			return MAX_RANK;
  		if(filtered_map.data[convert_to_index(conv_row,conv_column-1,width)]==100)
  			rank+=1;
  	}
  	if (is_point_exist(conv_row,conv_column+1,width,height)){
  		if(filtered_map.data[convert_to_index(conv_row,conv_column+1,width)]==-1)
  			return MAX_RANK;
  		if(filtered_map.data[convert_to_index(conv_row,conv_column+1,width)]==100)
  			rank+=1;
  	}
  	//bottom three points
  	if (is_point_exist(conv_row+1,conv_column-1,width,height)){
  		if(filtered_map.data[convert_to_index(conv_row+1,conv_column-1,width)]==-1)
  			return MAX_RANK;
  		if(filtered_map.data[convert_to_index(conv_row+1,conv_column-1,width)]==100)
  			rank+=1;
  	}
  	if (is_point_exist(conv_row+1,conv_column,width,height)){
  		if(filtered_map.data[convert_to_index(conv_row+1,conv_column,width)]==-1)
  			return MAX_RANK;
  		if(filtered_map.data[convert_to_index(conv_row+1,conv_column,width)]==100)
  			rank+=1;
  	}
  	if (is_point_exist(conv_row+1,conv_column+1,width,height)){
  		if(filtered_map.data[convert_to_index(conv_row+1,conv_column+1,width)]==-1)
  			return MAX_RANK;
  		if(filtered_map.data[convert_to_index(conv_row+1,conv_column+1,width)]==100)
  			rank+=1;
  	}
  	return rank;
  }

  bool is_point_exist(int row,int column,size_t width,size_t height){
  	if((row<0) || (column<0) || (row==height) || (column==width)) return false;
  	return true;
  }

  int convert_to_index(size_t i,size_t column,size_t width){
  	return (width*i)+column;
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;
  nav_msgs::OccupancyGrid filtered_map;
};

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "map_filter_node");;
  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPManager;

  ros::spin();

  return 0;
}

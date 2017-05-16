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

    ROS_INFO("Started grid filtering");
    int count=0;
    int not_passed_count=0;
    ROS_INFO("not_passed_count = %d",not_passed_count);
  	not_passed_count=0; 
  	for(size_t i=0;i<height;++i){
  	  	for(size_t j=0;j<width;++j){
  	   		
  	    		if (filtered_map.data[convert_to_index(i,j,width)]!=100) {
              filtered_map.data[convert_to_index(i,j,width)]++;
            }
  	   	}
  	}
  
  	ROS_INFO("Finished inputGrid processing");

  	ROS_INFO("Publishing...");
    pub_.publish(filtered_map);
    ROS_INFO("Finished publishing.");
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

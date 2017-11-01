#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/Pose.h"

int convert_to_index(size_t i,size_t column,size_t width){
	return (width*i)+column;
}

int convert_to_gmap_index(size_t i,size_t column,size_t width,size_t gmap_width){
	int diff=gmap_width-width;
	return (gmap_width*(i+diff))+column;
}

class SubscribeGmap
{
public:
  SubscribeGmap()
  {
    //Topic you want to publish
    pub_ = n_.advertise<nav_msgs::OccupancyGrid>("/map", 1);
    //Topic you want to subscribe
    sub_ = n_.subscribe("/gmapped_map", 1, &SubscribeGmap::callbackGmap, this);
  }

  void callbackGmap(const nav_msgs::OccupancyGrid &gmap)
  {
    ros::ServiceClient client = n_.serviceClient<nav_msgs::GetMap>("/static_map");
    nav_msgs::GetMap srv;
    client.call(srv);
    initial_map_ = srv.response.map;

    ROS_INFO("Started GMap processing");

    gmap_.header=gmap.header;
    gmap_.info=gmap.info;
    gmap_.data=gmap.data;

	  size_t gmap_width=gmap.info.width;
    size_t gmap_height=gmap.info.height;

    ROS_INFO("Gmap width: %zu", gmap_width);
    ROS_INFO("Gmap height: %zu", gmap_height);
    ROS_INFO("Gmap size: %zu", gmap_height*gmap_width);

	  ROS_INFO("Started initial map processing");

    size_t initial_map_width=initial_map_.info.width;
    size_t initial_map_height=initial_map_.info.height;

    ROS_INFO("Initial map width: %zu", initial_map_width);
    ROS_INFO("Initlal map height: %zu", initial_map_height);
    ROS_INFO("Initial map size: %zu", initial_map_height*initial_map_width);

    for(size_t i=0;i<initial_map_width;++i){
    	for(size_t j=0;j<initial_map_height;++j){
    		size_t map_index=convert_to_index(i,j,initial_map_width);
    		size_t gmap_index=convert_to_gmap_index(i,j,initial_map_width,gmap_width);
    		if(initial_map_.data[map_index]==-1 && gmap_.data[gmap_index]!=-1){
    			initial_map_.data[map_index]=gmap_.data[gmap_index];
    		} 
    	}
    }

    pub_.publish(initial_map_);
    ROS_INFO("Finished inputMap processing");
  }

private:
  ros::NodeHandle n_; 
  ros::Subscriber sub_;
  ros::Publisher pub_;
  nav_msgs::OccupancyGrid initial_map_;
  nav_msgs::OccupancyGrid gmap_;
};

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "maps_merging_node");;
  //Create an object of class SubscribeGMap that will take care of everything
  SubscribeGmap GmapManager();

  ros::spin();

  return 0;
}
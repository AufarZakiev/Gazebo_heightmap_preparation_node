#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Pose.h"

#define MAX_RANK 10
#define MIN_RANK 0
#define MINIMUM_PASSING_RANK 4

int convert_to_index(size_t i,size_t column,size_t width){
	return (width*i)+column;
}

int convert_to_gmap_index(size_t i,size_t column,size_t width,size_t gmap_width){
	int diff=gmap_width-width;
	return (gmap_width*(i+diff))+column;
}

class SubscribeMap
{
public:
  SubscribeMap()
  {
    //Topic you want to subscribe
    sub_ = n_.subscribe("/map", 1, &SubscribeMap::callbackMap, this);
  }

  void callbackMap(const nav_msgs::OccupancyGrid &initial_map)
  {
  	ROS_INFO("CallbackMap started");

    // size_t width=initial_map.info.width;
    // size_t height=initial_map.info.height;

    // ROS_INFO("Initial map width: %zu", width);
    // ROS_INFO("Initial map height: %zu", height);
    // ROS_INFO("Initial map size: %zu", height*width);

    double x_origin=initial_map.info.origin.position.x;
    double y_origin=initial_map.info.origin.position.y;
    double z_origin=initial_map.info.origin.position.z;
    float res=initial_map.info.resolution;
    ROS_INFO("Started grid info proccessing");
    ROS_INFO("Position: %f %f %f", x_origin,y_origin,z_origin);
    ROS_INFO("Resolution: %f", res);

    initial_map_.header=initial_map.header;
    initial_map_.info=initial_map.info;
    initial_map_.data=initial_map.data;
    ROS_INFO("CallbackMap finished");	
  }

  nav_msgs::OccupancyGrid getInitialGmap(){
  	return initial_map_;
  }

private:
  ros::NodeHandle n_; 
  ros::Subscriber sub_;
  nav_msgs::OccupancyGrid initial_map_;
};

class SubscribeGmap
{
public:
  SubscribeGmap(SubscribeMap *map_manager)
  {
  	map_manager_=map_manager;
    //Topic you want to publish
    pub_ = n_.advertise<nav_msgs::OccupancyGrid>("/map", 1);
    //Topic you want to subscribe
    sub_ = n_.subscribe("/gmapped_map", 1, &SubscribeGmap::callbackGmap, this);
  }

  void callbackGmap(const nav_msgs::OccupancyGrid &gmap)
  {
    initial_map_ = map_manager_->getInitialGmap();
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
	SubscribeMap *map_manager_;
  ros::NodeHandle n_; 
  ros::Subscriber sub_;
  ros::Publisher pub_;
  nav_msgs::OccupancyGrid initial_map_;
  nav_msgs::OccupancyGrid gmap_;
};

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "gmapped_map_processing_node");;
  //Create an object of class SubscribeMap that will take care of everything
  SubscribeMap MapManager;
  SubscribeGmap GmapManager(&MapManager);

  ros::spin();

  return 0;
}
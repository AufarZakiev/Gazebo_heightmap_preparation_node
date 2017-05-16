#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<nav_msgs::OccupancyGrid>("/filtered_map", 1);

    ROS_INFO("Started grid filling");
    size_t height=100;
    size_t width=100;
    filtered_map.info.width = width;
    filtered_map.info.height = height;
    filtered_map.info.resolution = 1.0;
    std::vector<signed char> ints;
    for(size_t i=0;i<height;++i){
        for(size_t j=0;j<width;++j){
            ints.push_back(-1);
        }
    }
    filtered_map.data = ints;
    ROS_INFO("Data: %d", filtered_map.data[1000]);

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
  nav_msgs::OccupancyGrid filtered_map;
};

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "map_filler_node");;
  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPManager;

  ros::spin();

  return 0;
}

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/Pose.h"

class PublishReachable
{
public:
  PublishReachable()
  {
    //Topic you want to publish
    pub_ = n_.advertise<nav_msgs::OccupancyGrid>("/reachable_map", 1);
    //Topic you want to subscribe
    sub_ = n_.subscribe("/map", 1, &PublishReachable::callbackReachable, this);
    service_ = n_.advertiseService("reachable_map",  &PublishReachable::reachable_map, this);
    temp = 30; // value for visualization purposes
  }

  bool reachable_map(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res) {
    //ignore empty request
    res.map = reachable_map_;
    //ROS_INFO("Reachable map sent.");
    return true;
  }

  void callbackReachable(const nav_msgs::OccupancyGrid &merged_map)
  {
    merged_map_ = merged_map;
    ROS_INFO("Started merged map processing");

    merged_map_.header = merged_map.header;
    merged_map_.info = merged_map.info;
    merged_map_.data = merged_map.data;

    size_t merged_map_width = merged_map_.info.width;
    size_t merged_map_height = merged_map_.info.height;

    ROS_INFO("Merged map width: %zu", merged_map_width);
    ROS_INFO("Merged height: %zu", merged_map_height);
    //ROS_INFO("Merged size: %zu", merged_map_height*merged_map_width);

    reachable_map_.header = merged_map.header;
    reachable_map_.info = merged_map.info;
    std::vector<signed char> empty;
    for (size_t i = 0; i < merged_map_width * merged_map_height; ++i) {
      empty.push_back(-1);
    }
    reachable_map_.data = empty;
    //ROS_INFO("Debug");
    ROS_INFO("First element: %d", reachable_map_.data[0]);
    for (size_t i = 0; i < merged_map_width; ++i) {
      for (size_t j = 0; j < merged_map_height; ++j) {
        if (merged_map_.data[convert_to_index(i, j)] == 0) {
          is_point_reachable(i, j, 0);
        }
      }
    }

    pub_.publish(reachable_map_);
    ROS_INFO("Finished merged map processing");
  }


private:
  ros::ServiceServer service_;
  ros::NodeHandle n_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  int temp;
  nav_msgs::OccupancyGrid reachable_map_;
  nav_msgs::OccupancyGrid merged_map_;
  int convert_to_index(size_t i, size_t column) {
    return (merged_map_.info.width * i) + column;
  }

  bool is_point_exist(size_t row, size_t column) {
    if ((row >= merged_map_.info.height) || (column >= merged_map_.info.width) || (row == SIZE_MAX) || (column == SIZE_MAX) || (row == SIZE_MAX - 1) || (column == SIZE_MAX - 1) ) return false;
    return true;
  }

  void is_point_reachable(size_t i, size_t j, int step) {
    step++;
    if (step < 5) {
      if (is_point_exist(i - 1, j) && merged_map_.data[convert_to_index(i - 1, j)] == -1) {
        reachable_map_.data[convert_to_index(i - 1, j)] = temp;
        is_point_reachable(i - 1, j, step);
      }
      if (is_point_exist(i, j - 1) && merged_map_.data[convert_to_index(i, j - 1)] == -1) {
        reachable_map_.data[convert_to_index(i, j - 1)] = temp;
        is_point_reachable(i, j - 1, step);
      }
      if (is_point_exist(i, j + 1) && merged_map_.data[convert_to_index(i, j + 1)] == -1) {
        reachable_map_.data[convert_to_index(i, j + 1)] = temp;
        is_point_reachable(i, j + 1, step);
      }
      if (is_point_exist(i + 1, j) && merged_map_.data[convert_to_index(i + 1, j)] == -1) {
        reachable_map_.data[convert_to_index(i + 1, j)] = temp;
        is_point_reachable(i + 1, j, step);
      }
    }
  }
};

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "reachable_publisher_node");;
  //Create an object of class SubscribeMap that will take care of everything
  PublishReachable ReachableManager;

  ros::spin();

  return 0;
}
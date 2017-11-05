#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "math.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include "nav_msgs/GetPlan.h"
#include "nav_msgs/GetMap.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class WeightedCell {
public:
  size_t weight;
  size_t cell_x;
  size_t cell_y;
  size_t free;
  WeightedCell(size_t x, size_t y, size_t w, size_t f) {
    cell_x = y;
    cell_y = x;
    weight = w;
    free = f;
  }
};

class WeightedCellCompare {
public:
  bool operator() (WeightedCell *l, WeightedCell *r) {
    if (l->weight < r->weight) {
      return true;
    } else {
      return false;
    }
  }
};

class SubscribeReachable
{
public:
  SubscribeReachable(): prev_goal(256, 256, 0, 0)
  {
    pub_ = n_.advertise<nav_msgs::OccupancyGrid>("/aimed_map", 1);
    //Topic you want to subscribe
    sub_ = n_.subscribe("/reachable_map", 1, &SubscribeReachable::callbackReachable, this);
    temp = 30;
    prev_pose = 0;
  }

  void callbackReachable(const nav_msgs::OccupancyGrid &reachable_map)
  {
    reachable_map_ = reachable_map;
    ROS_INFO("Started reachable map processing");

    reachable_map_.header = reachable_map.header;
    reachable_map_.info = reachable_map.info;
    reachable_map_.data = reachable_map.data;

    aimed_map_.header = reachable_map.header;
    aimed_map_.info = reachable_map.info;

    max_map_.header = reachable_map.header;
    max_map_.info = reachable_map.info;

    size_t reachable_map_width = reachable_map_.info.width;
    size_t reachable_map_height = reachable_map_.info.height;

    ROS_INFO("Reachable map width: %zu", reachable_map_width);
    ROS_INFO("Reachable height: %zu", reachable_map_height);

    ros::ServiceClient client_map = n_.serviceClient<nav_msgs::GetMap>("/static_map");
    nav_msgs::GetMap srv_map;
    client_map.call(srv_map);
    known_map_ = srv_map.response.map;

    ROS_INFO("Started known map processing");

    size_t known_map_width = known_map_.info.width;
    size_t known_map_height = known_map_.info.height;

    ROS_INFO("Known map width: %zu", known_map_width);
    ROS_INFO("Known map height: %zu", known_map_height);
    ROS_INFO("Known map size: %zu", known_map_height * known_map_width);
    WeightedCell maxCell(0, 0, 0, 0);
    move_base_msgs::MoveBaseGoal goal;
    geometry_msgs::PoseStamped start;
    std::vector<signed char> empty;
    for (size_t k = 0; k < known_map_width * known_map_height; ++k) {
      empty.push_back(-1);
    }
    if (globalpath.poses.size() == 0) {

      //std::priority_queue <WeightedCell*, std::vector<WeightedCell*>,WeightedCellCompare> goals_queue;
      //search for global goal

      for (size_t i = 0; i < known_map_height; ++i) {
        for (size_t j = 0; j < known_map_width; ++j) {
          aimed_map_.data = empty;
          if (known_map_.data[convert_to_index(i, j)] == 0) {
            size_t reachable_count = 0;
            size_t free_count = 0;
            for (int xi = -40; xi < 40; ++xi) {
              for (int yi = -40; yi < 40; ++yi) {
                if ((xi * xi + yi * yi) < 1600 &&
                    is_point_exist(i + xi, j + yi)) {
                  if (reachable_map_.data[convert_to_index(i + xi, j + yi)] == temp) {
                    reachable_count++;
                    aimed_map_.data[convert_to_index(i + xi, j + yi)] = 70;
                  } else {
                    free_count++;
                  }
                }
              }
            }
            if (reachable_count != 0 && reachable_count > maxCell.weight) {
              maxCell = WeightedCell(i, j, reachable_count, free_count);
              max_map_.data = aimed_map_.data;
            }
          }
        }
      }
      pub_.publish(max_map_);

      ROS_INFO("Top is: %zu", maxCell.weight);
      ROS_INFO("Top_x is: %zu", maxCell.cell_x);
      ROS_INFO("Top_y is: %zu", maxCell.cell_y);

      // search for small reachable regions
      size_t temp_x = abs(prev_goal.cell_x - maxCell.cell_x) * known_map_.info.resolution;
      size_t temp_y = abs(prev_goal.cell_y - maxCell.cell_y) * known_map_.info.resolution;
      double euclid_dist = sqrt(temp_x * temp_x + temp_y * temp_y);
      ROS_INFO("Euclid distance is: %f", euclid_dist);

      start.pose.position.x = (known_map_.info.resolution * prev_goal.cell_x) - 30.0;
      start.pose.position.y = (known_map_.info.resolution * prev_goal.cell_y) - 30.0;
      start.pose.position.z = 0.0;

      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();

      goal.target_pose.pose.position.x = (known_map_.info.resolution * maxCell.cell_x) - 30.0;
      goal.target_pose.pose.position.y = (known_map_.info.resolution * maxCell.cell_y) - 30.0;
      goal.target_pose.pose.position.z = 0.0;
      goal.target_pose.pose.orientation.x = 0.0;
      goal.target_pose.pose.orientation.y = 0.0;
      goal.target_pose.pose.orientation.z = 0.707106781;
      goal.target_pose.pose.orientation.w =  0.707106781;

      //ROS_INFO("Sending Global goal");
      //ac.sendGoal(goal);
      ros::ServiceClient client = n_.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
      nav_msgs::GetPlan srv;
      srv.request.start = start;
      srv.request.goal = goal.target_pose;
      srv.request.tolerance = 0.1;

      for (;;) {
        if (client.call(srv)) {
          ROS_INFO("Make plan succeded!");
          globalpath = srv.response.plan;
          break;
        } else {
          ROS_INFO("Make plan failed!");
        }
      }
    }

    MoveBaseClient ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(5.0))) {
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    WeightedCell maxLocalGoal(0, 0, 0, 0);
    WeightedCell maxLocalCell(0, 0, 0, 0);

    ROS_INFO("Calculating local goals: ");
    double search_dist = 2.0;
    size_t robot_radius = 4;
    if (globalpath.poses.size() != 0) {
      for (size_t k = prev_pose; k < globalpath.poses.size(); k++) {
        bool found = false;
        geometry_msgs::Pose pose = globalpath.poses[k].pose;
        //ROS_INFO("Path's position x %f", pose.position.x);
        //ROS_INFO("Path's position y %f", pose.position.y);
        size_t x = (size_t)((pose.position.x + 30.0) / known_map_.info.resolution);
        size_t y = (size_t)((pose.position.y + 30.0) / known_map_.info.resolution);
        ROS_INFO("Path's x %zu", x);
        ROS_INFO("Path's y %zu", y);
        size_t x1 = (size_t)((pose.position.x + 30.0 - search_dist) / known_map_.info.resolution);
        size_t x2 = (size_t)((pose.position.x + 30.0 + search_dist) / known_map_.info.resolution);
        size_t y1 = (size_t)((pose.position.y + 30.0 - search_dist) / known_map_.info.resolution);
        size_t y2 = (size_t)((pose.position.y + 30.0 + search_dist) / known_map_.info.resolution);
        // max_map_.data[convert_to_index(y1, x1)] = 70;
        // max_map_.data[convert_to_index(y1, x2)] = 70;
        // max_map_.data[convert_to_index(y2, x1)] = 70;
        // max_map_.data[convert_to_index(y2, x2)] = 70;
        //max_map_.data[convert_to_index(y, x)] = 100;
        //pub_.publish(max_map_);
        //ROS_INFO("Path's x1 %zu", x1);
        //ROS_INFO("Path's x2 %zu", x2);
        //ROS_INFO("Path's y1 %zu", y1);
        //ROS_INFO("Path's y2 %zu", y2);
        for (size_t i = y1; i < y2; ++i) {
          for (size_t j = x1; j < x2; ++j) {
            bool blocked = false;
            max_map_.data = empty;
            //pub_.publish(max_map_);
            if (reachable_map_.data[convert_to_index(i, j)] == temp) {
              // ROS_INFO("I, J: %zu,%zu", i, j);
              Line(i, j, y, x);
            }
          }
          if (found) {
            break;
            prev_pose = k;
          }
        }
        //ROS_INFO("LocalTop is: %zu", maxLocalGoal.weight);
        //ROS_INFO("LocalTopGoal x %zu", maxLocalGoal\\.cell_x);
        //ROS_INFO("LocalTopGoal y %zu", maxLocalGoal.cell_y);
        ROS_INFO("LocalTopCell x %zu", maxLocalCell.cell_x);
        ROS_INFO("LocalTopCell y %zu", maxLocalCell.cell_y);
        //max_map_.data[convert_to_index(maxLocalCell.cell_y, maxLocalCell.cell_x)] = 70;
        //pub_.publish(max_map_);
        //ROS_INFO("LocalTop_x is: %f", (known_map_.info.resolution * maxLocalGoal.cell_x) - 30.0);
        //ROS_INFO("LocalTop_y is: %f", (known_map_.info.resolution * maxLocalGoal.cell_y) - 30.0);

        while (!ac.waitForServer(ros::Duration(5.0))) {
          ROS_INFO("Waiting for the move_base action server to come up");
        }

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = (known_map_.info.resolution * maxLocalGoal.cell_x) - 30.0;
        goal.target_pose.pose.position.y = (known_map_.info.resolution * maxLocalGoal.cell_y) - 30.0;
        goal.target_pose.pose.position.z = 0.0;
        goal.target_pose.pose.orientation.x = 0.0;
        goal.target_pose.pose.orientation.y = 0.0;

        ROS_INFO("Sending Local goal");
        ac.sendGoal(goal);

        //ac.waitForResult();
        ros::ServiceClient client_map = n_.serviceClient<nav_msgs::GetMap>("/reachable_map");
        nav_msgs::GetMap srv_map;

        while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) { // if robot is still moving to the goal
          //check if the local cell is explored during movement to it
          client_map.call(srv_map);
          reachable_map_ = srv_map.response.map;
          if (reachable_map_.data[convert_to_index(maxLocalCell.cell_y, maxLocalCell.cell_x)] != temp) {
            ac.cancelGoal();
            ROS_INFO("Cell is explored till movement.");
            break;
          }
          //ROS_INFO("%s",ac.getState().toString().c_str());
        }
        ROS_INFO("You have reached Local goal");
      }
    }
  }

private:
  nav_msgs::Path globalpath;
  size_t prev_pose;
  ros::NodeHandle n_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  nav_msgs::OccupancyGrid reachable_map_;
  nav_msgs::OccupancyGrid known_map_;
  nav_msgs::OccupancyGrid aimed_map_;
  nav_msgs::OccupancyGrid max_map_;
  WeightedCell prev_goal;
  int temp;

  size_t convert_to_index(size_t row, size_t column) {
    return (known_map_.info.width * row) + column;
  }

  bool is_point_exist(size_t row, size_t column) {
    if ((row >= known_map_.info.height) || (column >= known_map_.info.width)) return false;
    return true;
  }


  void Line(size_t x1, size_t y1, size_t x2, size_t y2)
  {
    // Bresenham's line algorithm
    const bool steep = (fabs((int)y2 - (int)y1) > fabs((int)x2 - (int)x1));
    if (steep)
    {
      std::swap(x1, y1);
      std::swap(x2, y2);
    }

    if (x1 > x2)
    {
      std::swap(x1, x2);
      std::swap(y1, y2);
    }

    const float dx = (float)x2 - (float)x1;
    const float dy = (float)fabs(y2 - y1);

    float error = dx / 2.0f;
    const size_t ystep = (y1 < y2) ? 1 : -1;
    size_t y = y1;

    const size_t maxX = x2;

    for (size_t x = x1; x < maxX; x++)
    {
      if (steep)
      {
        max_map_.data[convert_to_index(y, x)] = 70;
      }
      else
      {
        max_map_.data[convert_to_index(x, y)] = 70;
      }

      error -= dy;
      if (error < 0)
      {
        y += ystep;
        error += dx;
      }
    }
    pub_.publish(max_map_);
  }
};

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "goals_publisher");;
  //Create an object of class SubscribeMap that will take care of everything
  SubscribeReachable ReachableManager;
  ros::spin();

  return 0;
}



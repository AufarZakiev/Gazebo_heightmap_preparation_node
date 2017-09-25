#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Pose.h"
#include "math.h"
#include <queue>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class SubscribeMap
{
public:
  SubscribeMap()
  {
    //Topic you want to subscribe
    sub_ = n_.subscribe("/map", 1, &SubscribeMap::callbackMap, this);
  }

  void callbackMap(const nav_msgs::OccupancyGrid &known_map)
  {
    ROS_INFO("CallbackMap started");

    // size_t width=known_map.info.width;
    // size_t height=known_map.info.height;

    // ROS_INFO("Initial map width: %zu", width);
    // ROS_INFO("Initial map height: %zu", height);
    // ROS_INFO("Initial map size: %zu", height*width);

    double x_origin=known_map.info.origin.position.x;
    double y_origin=known_map.info.origin.position.y;
    double z_origin=known_map.info.origin.position.z;
    float res=known_map.info.resolution;
    ROS_INFO("Started grid info proccessing");
    ROS_INFO("Position: %f %f %f", x_origin,y_origin,z_origin);
    ROS_INFO("Resolution: %f", res);

    known_map_.header=known_map.header;
    known_map_.info=known_map.info;
    known_map_.data=known_map.data;
    ROS_INFO("CallbackMap finished"); 
  }

  nav_msgs::OccupancyGrid getKnownGmap(){
    return known_map_;
  }

private:
  ros::NodeHandle n_; 
  ros::Subscriber sub_;
  nav_msgs::OccupancyGrid known_map_;
};

class WeightedCell{
public:
  size_t weight;
  size_t cell_x;
  size_t cell_y;
  size_t free;
  WeightedCell(size_t x,size_t y,size_t w,size_t f){
    cell_x=y;
    cell_y=x;
    weight=w;
    free=f;
  }
};

class WeightedCellCompare{
public:
  bool operator() (WeightedCell *l,WeightedCell *r){
    if (l->weight<r->weight){
      return true;
    }else{
      return false;
    }
  }
};

class SubscribeReachable
{
public:
  SubscribeReachable(SubscribeMap *map_manager):prev_goal(256,256,0,0)
  {
    pub_ = n_.advertise<nav_msgs::OccupancyGrid>("/aimed_map", 1);
    map_manager_=map_manager;
    //Topic you want to subscribe
    sub_ = n_.subscribe("/reachable_map", 1, &SubscribeReachable::callbackReachable, this);
    temp=30;
  }

  void callbackReachable(const nav_msgs::OccupancyGrid &reachable_map)
  {
    reachable_map_=reachable_map;
    ROS_INFO("Started reachable map processing");

    reachable_map_.header=reachable_map.header;
    reachable_map_.info=reachable_map.info;
    reachable_map_.data=reachable_map.data;

    aimed_map_.header=reachable_map.header;
    aimed_map_.info=reachable_map.info;

    max_map_.header=reachable_map.header;
    max_map_.info=reachable_map.info;

    size_t reachable_map_width=reachable_map_.info.width;
    size_t reachable_map_height=reachable_map_.info.height;

    ROS_INFO("Reachable map width: %zu", reachable_map_width);
    ROS_INFO("Reachable height: %zu", reachable_map_height);

    known_map_=map_manager_->getKnownGmap();

    ROS_INFO("Started known map processing");

    size_t known_map_width=known_map_.info.width;
    size_t known_map_height=known_map_.info.height;

    ROS_INFO("Known map width: %zu", known_map_width);
    ROS_INFO("Known map height: %zu", known_map_height);
    ROS_INFO("Known map size: %zu", known_map_height*known_map_width);

    std::vector<signed char> empty;
    for(size_t k=0;k<known_map_width*known_map_height;++k){
      empty.push_back(-1);
    }
    //std::priority_queue <WeightedCell*, std::vector<WeightedCell*>,WeightedCellCompare> goals_queue;
    WeightedCell maxCell(0,0,0,0);
    for(size_t i=0;i<known_map_height;++i){
      for(size_t j=0;j<known_map_width;++j){
        aimed_map_.data=empty;
        if (known_map_.data[convert_to_index(i,j)]==0){
          size_t reachable_count=0;
          size_t free_count=0;
          for(int xi=-40;xi<40;++xi){
            for(int yi=-40;yi<40;++yi){
              if(sqrt(xi*xi+yi*yi)<40.0 && 
                is_point_exist(i+xi,j+yi)){
                if(reachable_map_.data[convert_to_index(i+xi,j+yi)]==temp){
                  reachable_count++;
                  aimed_map_.data[convert_to_index(i+xi,j+yi)]=70;
                }else{
                  free_count++;
                }
              }
            }  
          }
          if(reachable_count!=0 && reachable_count>maxCell.weight){
            maxCell=WeightedCell(i,j,reachable_count,free_count);
            max_map_.data=aimed_map_.data;
          }
        }
      }
    }
    pub_.publish(max_map_);
    // ROS_INFO("Zeros is: %zu",zeros);
    // ROS_INFO("Exist is: %zu",exist);
    //ROS_INFO("Size is: %zu",goals_queue.size());
    //goals_queue.pop();

    ROS_INFO("Top is: %zu",maxCell.weight);
    ROS_INFO("Top_x is: %zu",maxCell.cell_x);
    ROS_INFO("Top_y is: %zu",maxCell.cell_y);
    //ROS_INFO("Free is: %zu",maxCell.free);

    //ROS_INFO("Value is: %d",known_map_.data[210714]);
    // for(size_t i=0;i<10;++i){
    //   ROS_INFO("Weight is: %zu",goals_queue.top()->weight);
    //   ROS_INFO("Top_x is: %f",(known_map_.info.resolution*goals_queue.top()->cell_x)-30.0);
    //   ROS_INFO("Top_y is: %f",(known_map_.info.resolution*goals_queue.top()->cell_y)-30.0);
    //   goals_queue.pop();
    // }

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);


    move_base_msgs::MoveBaseGoal goal;
    ROS_INFO("Prev goal x is: %zu",prev_goal.cell_x);
    ROS_INFO("Prev goal y is: %zu",prev_goal.cell_y);
    // serach for small reachable regions
    size_t temp_x=abs(prev_goal.cell_x-maxCell.cell_x)*known_map_.info.resolution;
    size_t temp_y=abs(prev_goal.cell_y-maxCell.cell_y)*known_map_.info.resolution;
    double boundary_dist = sqrt(temp_x*temp_x+temp_y*temp_y);
    ROS_INFO("Euclid distance is: %f",boundary_dist);

    WeightedCell maxLocalCell(0,0,0,0);
    for(size_t i=0;i<known_map_height;++i){
      for(size_t j=0;j<known_map_width;++j){
        temp_x=abs(prev_goal.cell_x-j)*known_map_.info.resolution;
        temp_y=abs(prev_goal.cell_y-i)*known_map_.info.resolution;
        if(sqrt(temp_x*temp_x+temp_y*temp_y)<boundary_dist){
          aimed_map_.data=empty;
          if (known_map_.data[convert_to_index(i,j)]==0){
            size_t reachable_count=0;
            size_t free_count=0;
            for(int xi=-40;xi<40;++xi){
              for(int yi=-40;yi<40;++yi){
                if(sqrt(xi*xi+yi*yi)<40.0 && 
                  is_point_exist(i+xi,j+yi)){
                  if(reachable_map_.data[convert_to_index(i+xi,j+yi)]==temp){
                    reachable_count++;
                    aimed_map_.data[convert_to_index(i+xi,j+yi)]=70;
                  }else{
                    free_count++;
                  }
                }
              }  
            }
            if(reachable_count!=0 && reachable_count>maxLocalCell.weight && free_count/reachable_count>10){
              maxLocalCell=WeightedCell(i,j,reachable_count,free_count);
              max_map_.data=aimed_map_.data;
            }
          }
        }
      }
    }
    pub_.publish(max_map_);
    // ROS_INFO("Zeros is: %zu",zeros);
    // ROS_INFO("Exist is: %zu",exist);
    //ROS_INFO("Size is: %zu",goals_queue.size());
    //goals_queue.pop();

    ROS_INFO("LocalTop is: %zu",maxLocalCell.weight);
    ROS_INFO("LocalTop_x is: %f",(known_map_.info.resolution*maxLocalCell.cell_x)-30.0);
    ROS_INFO("LocalTop_y is: %f",(known_map_.info.resolution*maxLocalCell.cell_y)-30.0);
    //ROS_INFO("Free is: %zu",maxCell.free);

    //ROS_INFO("Value is: %d",known_map_.data[210714]);
    // for(size_t i=0;i<10;++i){
    //   ROS_INFO("Weight is: %zu",goals_queue.top()->weight);
    //   ROS_INFO("Top_x is: %f",(known_map_.info.resolution*goals_queue.top()->cell_x)-30.0);
    //   ROS_INFO("Top_y is: %f",(known_map_.info.resolution*goals_queue.top()->cell_y)-30.0);
    //   goals_queue.pop();
    // }

    //tell the action client that we want to spin a thread by default
    //MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    //move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = (known_map_.info.resolution*maxLocalCell.cell_x)-30.0;
    goal.target_pose.pose.position.y = (known_map_.info.resolution*maxLocalCell.cell_y)-30.0;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.707106781;
    goal.target_pose.pose.orientation.w =  0.707106781;

    ROS_INFO("Sending Local goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = -0.707106781;
    goal.target_pose.pose.orientation.w =  0.707106781;

    ROS_INFO("Sending Local goal");
    ac.sendGoal(goal);

    ac.waitForResult();
    
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, we reached goal.");
    else
      ROS_INFO("The base failed to reach the goal.");

    prev_goal=maxLocalCell;


    // WeightedCell prev_goal=maxCell;

    // WeightedCell emptyCell(0,0,0,0);
    // maxCell=emptyCell;
    // for(size_t i=0;i<known_map_height;++i){
    //   for(size_t j=0;j<known_map_width;++j){
    //     aimed_map_.data=empty;
    //     if (known_map_.data[convert_to_index(i,j)]==0){
    //       size_t reachable_count=0;
    //       size_t free_count=0;
    //       for(int xi=-40;xi<40;++xi){
    //         for(int yi=-40;yi<40;++yi){
    //           if(sqrt(xi*xi+yi*yi)<40.0 && 
    //             is_point_exist(i+xi,j+yi)){
    //             if(reachable_map_.data[convert_to_index(i+xi,j+yi)]==temp){
    //               reachable_count++;
    //               aimed_map_.data[convert_to_index(i+xi,j+yi)]=70;
    //             }else{
    //               free_count++;
    //             }
    //           }
    //         }  
    //       }
    //       if(reachable_count!=0 && reachable_count>maxCell.weight){
    //         maxCell=WeightedCell(i,j,reachable_count,free_count);
    //         max_map_.data=aimed_map_.data;
    //       }
    //     }
    //   }
    // }

    // ROS_INFO("new Top is: %zu",maxCell.weight);
    // ROS_INFO("new Top_x is: %f",(known_map_.info.resolution*maxCell.cell_x)-30.0);
    // ROS_INFO("new Top_y is: %f",(known_map_.info.resolution*maxCell.cell_y)-30.0);
    
    
  }
private:
  SubscribeMap *map_manager_;
  ros::NodeHandle n_; 
  ros::Subscriber sub_;
  ros::Publisher pub_;
  nav_msgs::OccupancyGrid reachable_map_;
  nav_msgs::OccupancyGrid known_map_;
  nav_msgs::OccupancyGrid aimed_map_;
  nav_msgs::OccupancyGrid max_map_;
  WeightedCell prev_goal;
  int temp;

  size_t convert_to_index(size_t row,size_t column){
    return (known_map_.info.width*row)+column;
  }

  bool is_point_exist(size_t row,size_t column){
    // ROS_INFO("Row is: %zu",row);
    // ROS_INFO("Height is: %d",known_map_.info.height);
    // ROS_INFO("Exist? %s",(row>=known_map_.info.height)?"false":"true");
    if((row>=known_map_.info.height) || (column>=known_map_.info.width)) return false;
    return true;
  }
};

int main(int argc, char **argv)
{
    //Initiate ROS
  ros::init(argc, argv, "goals_publisher");;
    //Create an object of class SubscribeMap that will take care of everything
  SubscribeMap MapManager;
  SubscribeReachable ReachableManager(&MapManager);

  ros::spin();

  return 0;
}

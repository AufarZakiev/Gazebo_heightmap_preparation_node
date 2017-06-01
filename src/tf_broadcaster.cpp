#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Point.h>
#include <string>


void modelStateCallback(const gazebo_msgs::ModelStates &msg)
{
  ROS_INFO("I heard name: [%s]", msg.name[1].c_str());
  ROS_INFO("I heard position: [%f %f %f]", msg.pose[1].position.x,msg.pose[1].position.y,msg.pose[1].position.z);
  tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));
  
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_groundtruth_broadcaster");
  ros::NodeHandle node;

  ros::Subscriber sub = node.subscribe("/gazebo/model_states", 1000, modelStateCallback);
  ros::Rate rate(100.0);
  ros::spin();
  return 0;
};
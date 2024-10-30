
#include <dynamic_reconfigure/server.h>
#include <iostream>
#include <math.h>
#include <ros/package.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <string>
#include <thread>
#include <darknet_ros_msgs/ObjectCount.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include "geometry_msgs/PoseStamped.h"

/*CHANGE THESE VALUES ACCORDINGLY*/
#define MINDISTANCE 0.5
#define SPEED 0.2
#define ROTSPEED 0.3

using namespace ros;
class RobotControlNode
{
public:

  RobotControlNode()
      : private_nh("~")
  {
    // std::string cloud_topic = "objects";
    // pub_.advertise(nh_, cloud_topic.c_str(), 1);
    pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    sub_lidar = node.subscribe("/scan", 1, &RobotControlNode::laserCallback,this);
    sub_pose = node.subscribe("/slam_out_pose", 1, &RobotControlNode::poseCallback,this);
    // config_server_.setCallback(boost::bind(&ShapeFinderNode::dynReconfCallback, this, _1, _2));
    ros::NodeHandle private_nh("~");
    // private_nh.param("frame_id", tf_frame, std::string("pico_zense_depth_frame"));
    private_nh.param("frame_id", tf_frame, std::string("base_link"));
    command.linear.x = 0.0;
    command.linear.y = 0.0;
    command.linear.z = 0.0;
    command.angular.x = 0.0;
    command.angular.y = 0.0;
    command.angular.z = 0.0;
  }
  ~RobotControlNode() {}

void laserCallback(const sensor_msgs::LaserScan lms)
{
    if(trash_detected){
        command.linear.x = 0.0;
        command.angular.z = 0.0;
    }
    else{
        bool stuck = false;
        // std::cout<<lms.ranges.size()<<std::endl;
        for (int i = 0; i < lms.ranges.size(); i++)
        {
            if (i < 30 || i >330)
            {
                // printf("the range is: %f - %f \n",lms.ranges.size() * 0.4,lms.ranges.size() * 0.6);
                if (lms.ranges[i] < MINDISTANCE && lms.ranges[i] >0.0)
                {
                    // printf("Value [%d] is : %f \n",i,lms.ranges[i]);
                    stuck = true;
                }
            }
        }
        if (stuck)
        {
            // printf("I'm stuck\n");
            command.linear.x = 0.0;
            command.angular.z = ROTSPEED;
        }
        else
        {
            // printf("I'm not stuck\n");
            command.linear.x = SPEED;
            command.angular.z = 0.0;
        }
    }
    pub.publish(command);
}


void poseCallback(const geometry_msgs::PoseStamped pose)
{
    currentPose = pose;    
}

//   void
//   dynReconfCallback(robot_control::robot_control_nodeConfig &config, uint32_t level)
//   {
//      MeanK = config.MeanK;                                        //50
//   }

private:
  ros::NodeHandle node;
  ros::NodeHandle private_nh;
  std::string tf_frame = "base_scan";
  ros::Subscriber sub_lidar;
  ros::Subscriber sub_pose;
  ros::Publisher pub;
//   dynamic_reconfigure::Server<robot_control::robot_control_nodeConfig> config_server_;
  bool trash_detected;
  geometry_msgs::Twist command;
  geometry_msgs::PoseStamped currentPose;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_control");
  RobotControlNode rc;
  ros::spin();
}

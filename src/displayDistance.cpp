#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include "kinematic.h"
#include <eigen3/Eigen/Core>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <thread>

using std::vector;

class distance_display
{
  public:
    distance_display(ros::NodeHandle& nodeHandle)
    {
        handle = nodeHandle;
    }

  private:
    ros::NodeHandle handle;
    ros::Subscriber sub;
    RowVector6d leftJoints, rightJoints;
    // Matrix4d T_left, T_right;
    Vector3d TCP_left, TCP_right;
    int i;
    kinematic kinematics;
    double p2p_distance;
    Eigen::Isometry3d pose;
    // rviz_visual_tools::RvizVisualTools rvt;
  public:
    void callback(const sensor_msgs::JointStateConstPtr& msg)
    {
        for (i = 6; i < 12; i++)
        {   
            leftJoints(i-6) = msg->position.at(i);
        }
        for (i = 12; i < 18; i++)
        {
            rightJoints(i-12) = msg->position.at(i);
        }

        TCP_left = kinematics.fk(leftJoints, ER20).block(0,3,3,1);
        TCP_right = kinematics.fk(rightJoints, ER20).block(0,3,3,1);
        TCP_right(1) += 2;
        printf("Left: %f, Right: %f", TCP_left);
        // the distance between two TCPs
        p2p_distance = sqrt(pow(TCP_left(0) - TCP_right(0),2) + pow(TCP_left(1) - TCP_right(1),2) + pow(TCP_left(2) - TCP_right(2),2));     
    }
    
    void drawDistance()
    {
      using namespace rviz_visual_tools;
      rviz_visual_tools::RvizVisualTools rvt("left_base_link", "/rviz_marker");
      std::stringstream ss;
      ss<<p2p_distance;
      while (true)
      { // TODO something wrong with the two points and the distance between
        pose = Isometry3d::Identity();
        pose.translate((TCP_left + TCP_right)/2);
        rvt.publishText(pose, ss.str(), WHITE, XXLARGE, false);
        ss.clear();
        rvt.publishLine(TCP_left, TCP_right, YELLOW, LARGE);
        rvt.trigger();
        ros::Duration(1).sleep();
      }
    }

    void drawThread()
    { // NOTE take notes of C++ thread
      // https://blog.csdn.net/Shreck66/article/details/50409874
      std::thread draw_thread(std::bind(&distance_display::drawDistance, this));
      draw_thread.detach();
    }

};

/*
void callback(const sensor_msgs::JointStateConstPtr& msg, rviz_visual_tools::RvizVisualTools& rvt)
{
    for (i = 6; i < 12; i++)
    {   
        leftJoints(i-6) = msg->position.at(i);
    }
    for (i = 12; i < 18; i++)
    {
        rightJoints(i-12) = msg->position.at(i);
    }

    TCP_left = kinematics.fk(leftJoints, "ER20").block(0,3,3,1);
    TCP_right = kinematics.fk(rightJoints, "ER20").block(0,3,3,1);
    // the distance between two TCPs
    p2p_distance = sqrt(pow(TCP_left(0) - TCP_right(0),2) + pow(TCP_left(1) - TCP_right(1),2) + pow(TCP_left(2) - TCP_right(2),2));     
    using namespace rviz_visual_tools;
    rvt.publishLine(TCP_left, TCP_right, YELLOW, LARGE);
    rvt.trigger();
}
*/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "distance_display");
    ros::NodeHandle handle;

    distance_display dis(handle);
    
    ros::Subscriber sub = handle.subscribe("/joint_states", 1000, &distance_display::callback, &dis);
    
    dis.drawThread();
    
    ros::spin();
    return 0;
}
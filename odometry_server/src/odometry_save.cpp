/*
 * odometry_sub.cpp
 *
 *  Created on: 27.07.2017
 *      Author: mario
 */

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include "odometry_server/odom_save.h"

float 	x = 0.0;
float 	y = 0.0;
float 	w = 0.0;
float 	theta = 0.0;
tf::Pose pose;

ros::Publisher odom_save_pub;

void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    w = msg->pose.pose.orientation.w;
    tf::poseMsgToTF(msg->pose.pose,pose);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "odom_listener");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("odom", 1000, chatterCallback);

	odometry_server::odom_save e;

	ros::Rate loop_rate(10);
	int count = 0;

	odom_save_pub = n.advertise<odometry_server::odom_save>("odo_save", 100);

	while (ros::ok())
	  {
		theta = tf::getYaw(pose.getRotation())*180/3.14149265;
		e.nr = count;
		e.x = x;
		e.y = y;
		e.theta = theta;
	    ROS_INFO("Odometry Listener \n Nr: %i \n X: %f \n Y: %f \n Theta: %f",count,x,y,theta);
	    odom_save_pub.publish(e);

	    ros::spinOnce();

	    loop_rate.sleep();
	    ++count;
	  }

	return 0;
}



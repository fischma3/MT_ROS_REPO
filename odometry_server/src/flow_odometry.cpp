#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "odometry_server/OpticalFlow.h"



ros::Subscriber px4_flow_sub;

// pose estimate
double x = 0.0;
double y = 0.0;
double theta = 0.0;
double theta_degree = 0.0;

// position change
double dx = 0.0;
double dy = 0.0;
double dtheta = 0.0;
double dx_cam = 0.0;
double dy_cam = 0.0;
double dist_center = 0.0;

// camera position
double dist_cam = 1.0; // camera in front of center in m

// velocity estimate
double vx = 0.0;
double vy = -0.0;
double vtheta = 0.0;

// test variable

int counter = 0;
int counter_max = 5;

ros::Time current_time, last_time;

// functions used to check signed integer wraparound
int wrap(int a) {
	return ((a + 1) > a);
}

bool check_wrap() {
	return (wrap(~0u>>1));
}


/*!
 * \brief callback when camera position changes
 */
void px_comm_Callback(const px_comm::OpticalFlow::ConstPtr& msg)
{
	// delta time
	last_time = current_time;
	current_time = msg->header.stamp;
	double dt = (current_time - last_time).toSec();

	double cos_current = cos(theta);
	double sin_current = sin(theta);

	// camera position change
	dx_cam = msg->velocity_x*dt*1.0175;
	dy_cam = msg->velocity_y*dt*1.0175;

	// angle change of the vehicle
	dtheta = atan(-dy_cam/(dist_cam+dx_cam));

	// position change of the vehicle
	dist_center = sqrt(pow(dist_cam+dx_cam,2)+pow(-dy_cam,2))-dist_cam;
	dx = dist_center*cos_current;
	dy = dist_center*sin_current;

	// position of the vehicle
	x += dx;
	y += dy;
	theta += dtheta;

	// velocities
	vx = dx/dt;
	vy = dy/dt;
	vtheta = dtheta/dt;

	// angle in degrees
	theta_degree = theta*180/3.1415926;

	if (counter>=counter_max){
		ROS_INFO("\n Flow Odometra \n Position (m) x:%.3f y:%.3f Winkel: %.3f quality:%.3f freq:%.3f",
		 (float)x, (float)y, (float)theta_degree, (float)msg->quality, (float)(1/dt));
		counter = 0;
	}
	else{
		counter++;
	}
}

/*!
 * \param connects to a phidgets high speed encoder
 *        device which contains two or more encoders
 */
bool subscribe_to_px4_node()
{
	const int buffer_length = 100;
    bool success = true;
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    std::string topic_path = "px4flow/";
    nh.getParam("topic_path", topic_path);
    std::string px4_flow_name = "/opt_flow";
    nh.getParam("px4_flow_name", px4_flow_name);
    std::string px4_flow_topic_base = topic_path + px4_flow_name;
    nh.getParam("px4_topic", px4_flow_topic_base);

    // get the topic name
    std::string px4_flow_topic = px4_flow_topic_base;

    px4_flow_sub =
		n.subscribe(px4_flow_topic, buffer_length, px_comm_Callback);

    return(success);
}

int main(int argc, char** argv)
{
    if (!check_wrap()) {
        ROS_ERROR("This version of GCC does not support " \
				  "signed integer wraparound.  " \
				  "Try using the -fwrapv compile option.");
    }
    else {

        ros::init(argc, argv, "flow_odometry_server");

        ros::NodeHandle n;
        ros::NodeHandle nh("~");

        std::string name = "odom_flow";

        std::string reset_topic = "flow_odometry/reset";
        nh.getParam("reset_topic", reset_topic);

        n.setParam(reset_topic, false);

        ros::Publisher flow_odom_pub =
			n.advertise<nav_msgs::Odometry>(name, 50);
        tf::TransformBroadcaster flow_odom_broadcaster;

        // connect to the encoders
        bool subscribed_to_px4_node = false;
        subscribed_to_px4_node = subscribe_to_px4_node();

        if (subscribed_to_px4_node) {

            std::string base_link = "flow_base_link";
            nh.getParam("base_link", base_link);
            std::string frame_id = "odom_server";
            nh.getParam("frame_id", frame_id);

            int frequency = 100;
            nh.getParam("frequency", frequency);

            ros::Rate update_rate(frequency);
            while(ros::ok()){
                // reset the pose
                bool reset = false;
                n.getParam(reset_topic, reset);
                if (reset) {
                    x = 0;
                    y = 0;
                    theta = 0;
                    vx = 0;
                    vy = 0;
                    vtheta = 0;
                    n.setParam(reset_topic, false);
                }
   
                // since all odometry is 6DOF we'll need a
				// quaternion created from yaw
                geometry_msgs::Quaternion odom_quat =
					tf::createQuaternionMsgFromYaw(theta);
  
                // first, we'll publish the transform over tf
                geometry_msgs::TransformStamped flow_odom_trans;
                flow_odom_trans.header.stamp = current_time;
                flow_odom_trans.header.frame_id = frame_id;
                flow_odom_trans.child_frame_id = base_link;
  
                flow_odom_trans.transform.translation.x = x;
                flow_odom_trans.transform.translation.y = y;
                flow_odom_trans.transform.translation.z = 0.0;
                flow_odom_trans.transform.rotation = odom_quat;
   
                // send the transform
                flow_odom_broadcaster.sendTransform(flow_odom_trans);
  
                // next, we'll publish the odometry
				//  message over ROS
                nav_msgs::Odometry flow_odom;
                flow_odom.header.stamp = current_time;
                flow_odom.header.frame_id = frame_id;
  
                // set the position
                flow_odom.pose.pose.position.x = x;
                flow_odom.pose.pose.position.y = y;
                flow_odom.pose.pose.position.z = 0.0;
                flow_odom.pose.pose.orientation = odom_quat;
  
                // set the velocity
                flow_odom.child_frame_id = base_link;
                flow_odom.twist.twist.linear.x = vx;
                flow_odom.twist.twist.linear.y = vy;
                flow_odom.twist.twist.angular.z = vtheta;

                // publish the message
                flow_odom_pub.publish(flow_odom);

                ros::spinOnce();
                update_rate.sleep();
            }
        }
    }
}


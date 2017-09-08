/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Phidgets based wheel odometry for a differential drive system
 *  For use with a pair of Phidgets high speed encoders
 *  See http://www.ros.org/wiki/navigation/Tutorials/RobotSetup/Odom
 *  Copyright (c) 2010, Bob Mottram
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "odometry_server/encoder_params.h"

bool initialised = false;

// index numbers of left and right encoders
int encoder_index_left= 1;
int encoder_index_right= 0;

// normally on a differential drive system to when moving
//  forwards the wheels are rotating in opposite directions
int encoder_direction_left = -1;
int encoder_direction_right = 1;

// distance between the wheel centres
double wheelbase_mm = 585;
double wheelbase = 585;

// Parameters
double speed = 0.5;
double load = 0;
double pressure = 2.25;
double radius = 1.6;

// scale the parameters to the norm
double speed_norm = 1.5;
double scale_S = (speed-speed_norm)/1;

double load_norm = 60;
double scale_L = (load-load_norm)/30;

double pressure_norm = 2;
double scale_P = (pressure-pressure_norm)/0.125;

double radius_norm = 1.6;
double scale_R = (radius-radius_norm)/1.6;

// Model-parameters for encoder_counts_per_mm
double b1 	= 0.821804512652212;
double b2 	= -0.0383404381621055;
double b3 	= -0.465613814889989;
double b4 	= 0.224057530496622;
double b5 	= 0.00213701253458295;
double b6 	= 0.0258913824218897;
double b7 	= -0.0793366829978452;
double b8 	= -0.0571884441982908;
double b9 	= 0.0236794828033771;
double b10 	= -0.0284737562865976;
double b11 	= 0.0293395708140309;

double gain = 0.0;

// Model-parameters for wheelbase
double b1_1 	= 0.00315093866744196;
double b2_1 	= -0.118601922953205;
double b3_1 	= -0.00366367942635825;
double b4_1 	= 0.00504865334373268;
double b5_1 	= -0.000319371697885493;
double b6_1 	= 0.117623420342880;
double b7_1 	= 0.00351558170590245;
double b8_1 	= 0.0214152977778184;
double b9_1 	= 0.0157316373443995;
double b10_1 	= -0.00219193979613172;
double b11_1 	= -0.0654376106721517;

double gain_1 = 0.0;
double gain_r_l = 0.0;

// encoder counts
int current_encoder_count_left = 0;
int current_encoder_count_right = 0;
int previous_encoder_count_left = 0;
int previous_encoder_count_right = 0;

// keep track if the initial encoder values so that relative
// movement can be reported
int start_encoder_count_left = 0;
int start_encoder_count_right = 0;

// encoder counts per millimeter
double left_encoder_counts_per_mm = 15.51200879;//15.1651811;//15.3846;//1.537;15.402469338;
double right_encoder_counts_per_mm = 15.51200879;//15.1651811;//15.3846;//1.537;15.402469338;
double encoder_counts_per_mm = 15.51200879;

// counter to make the console outputs slower
int counter = 0;
int counter_max = 12;

ros::Subscriber encoders_sub;

// pose estimate
double x = 0.0;
double y = 0.0;
double theta = 0.0;
double winkel = 0.0;

// velocity estimate
double vx = 0.1;
double vy = -0.1;
double vtheta = 0.1;

double rotation_offset=0;

// functions used to check signed integer wraparound
int wrap(int a) {
	return ((a + 1) > a);
}

bool check_wrap() {
	return (wrap(~0u>>1));
}

// Update the left encoder count
void update_encoder_left(int count)
{
    current_encoder_count_left += count * encoder_direction_left;
    if (start_encoder_count_left == 0) {
        start_encoder_count_left = current_encoder_count_left;
    }
}

// Update the right encoder count
void update_encoder_right(int count)
{
    current_encoder_count_right += count * encoder_direction_right;
    if (start_encoder_count_right == 0) {
        start_encoder_count_right = current_encoder_count_right;
    }
}

/*!
 * \brief callback when the left or right encoder count changes
 * \param ptr encoder parameters
 */
void encoderCallback(const phidgets::encoder_params::ConstPtr& ptr)
{
    if (initialised) {
        phidgets::encoder_params e = *ptr;
        if (e.index == encoder_index_left) {
            update_encoder_left(e.count_change);
        }
        if (e.index == encoder_index_right) {
            update_encoder_right(e.count_change);
        }

    }
}

/*!
 * \param connects to a phidgets high speed encoder
 *        device which contains two or more encoders
 */
bool subscribe_to_encoders_by_index()
{
	const int buffer_length = 100;
    bool success = true;
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    std::string topic_path = "phidgets/";
    nh.getParam("topic_path", topic_path);
    std::string encodername = "encoder";
    nh.getParam("encodername", encodername);
    std::string encoder_topic_base = topic_path + encodername;
    nh.getParam("encoder_topic", encoder_topic_base);

    // get the topic name
    std::string encoder_topic = encoder_topic_base;

    encoders_sub =
		n.subscribe(encoder_topic, buffer_length, encoderCallback);

    return(success);
}

/*!
 * \brief updates the robot velocities
 * \param dt time elapsed in seconds
 */
void update_velocities(
					   double dt)
{
    if (dt > 0) {

        int curr_encoder_count_left =
			current_encoder_count_left;
        int curr_encoder_count_right =
			current_encoder_count_right;
        // Model for encoder_counts_per_mm
        gain = b1+b2*scale_S+b3*scale_L+b4*scale_P+b5*scale_S*scale_S+b6*scale_L*scale_L+b7*scale_P*scale_P+b8*scale_S*scale_L+b9*scale_S*scale_P+b10*scale_L*scale_P+b11*scale_S*scale_L*scale_P;
        gain = 1-gain/100;

        // Model to weight the right and the left wheel
        gain_r_l = b1_1+b2_1*scale_L/2+b3_1*scale_P/2+b4_1*scale_R+b5_1*scale_L/2*scale_P/2+b6_1*scale_L/2*scale_R+b7_1*scale_P/2*scale_R;

        // set the encoder counts per mm
        left_encoder_counts_per_mm = (1-gain_r_l)*gain*encoder_counts_per_mm;
		right_encoder_counts_per_mm = (1+gain_r_l)*gain*encoder_counts_per_mm;

		// Model for the wheelbase
		gain_1 =1+(b8_1+b9_1*scale_L/2+b10_1*scale_P/2+b11_1*scale_R);
		wheelbase = wheelbase_mm*gain_1;

        double cos_current = cos(theta);
        double sin_current = sin(theta);

        // relying on the compiler for wraparound here
        int dist_left_counts =
			curr_encoder_count_left -
			previous_encoder_count_left;
        int dist_right_counts =
			curr_encoder_count_right -
			previous_encoder_count_right;

		// convert from counts to standard units
		double dist_left_mm =
			(double)dist_left_counts /
			left_encoder_counts_per_mm;
		double dist_right_mm =
			(double)dist_right_counts /
			right_encoder_counts_per_mm;

		double dist_center = (dist_right_mm + dist_left_mm)/2;
		double right_left = dist_right_mm - dist_left_mm;

		double fraction = right_left / wheelbase;

		const double mm_to_m = 1.0 / 1000;

		if (dist_left_counts == dist_right_counts) {
			vx = dist_center*cos_current*mm_to_m;
			vy = dist_center*sin_current*mm_to_m;
			fraction = 0;
		}
		else{
			double a = wheelbase * (dist_right_mm + dist_left_mm) * 0.5 / right_left;
			vx = a * (sin(fraction + theta) - sin_current) * mm_to_m;
			vy = -a * (cos(fraction + theta) - cos_current) * mm_to_m;
		}
		vtheta = fraction;
		winkel = theta*180/3.1415926;

		if (counter>=counter_max){
			ROS_INFO("\n Position (m) x:%.3f y:%.3f Winkel: %.3f c_mm:%.6f",
			 (float)x, (float)y, (float)winkel);
			counter = 0;
		}
		else{
			counter++;
		}
        // store previous values
        previous_encoder_count_left = curr_encoder_count_left;
        previous_encoder_count_right = curr_encoder_count_right;
    }
}

int main(int argc, char** argv)
{
    if (!check_wrap()) {
        ROS_ERROR("This version of GCC does not support " \
				  "signed integer wraparound.  " \
				  "Try using the -fwrapv compile option.");
    }
    else {

        ros::init(argc, argv, "odometry_server");

        ros::NodeHandle n;
        ros::NodeHandle nh("~");

        std::string name = "odom";

        std::string reset_topic = "odometry/reset";
        nh.getParam("reset_topic", reset_topic);

        n.setParam(reset_topic, false);

        ros::Publisher odom_pub =
			n.advertise<nav_msgs::Odometry>(name, 50);
        tf::TransformBroadcaster odom_broadcaster;

        // connect to the encoders
        bool subscribed_to_encoders = false;
        subscribed_to_encoders = subscribe_to_encoders_by_index();

        if (subscribed_to_encoders) {

            std::string base_link = "base_link";
            nh.getParam("base_link", base_link);
            std::string frame_id = "odom_server";
            nh.getParam("frame_id", frame_id);

            int frequency = 100;
            nh.getParam("frequency", frequency);

            ros::Time current_time, last_time;
            current_time = ros::Time::now();
            last_time = ros::Time::now();

            initialised = true;

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
                    start_encoder_count_left = 0;
                    start_encoder_count_right = 0;
                }

                last_time = current_time;
                current_time = ros::Time::now();
                double dt = (current_time - last_time).toSec();
                // update the velocity estimate based upon
				// encoder values
                update_velocities(dt);

                // compute odometry in a typical way given
				// the velocities of the robot       
                x += vx;
                y += vy;
                theta += vtheta;
   
                // since all odometry is 6DOF we'll need a
				// quaternion created from yaw
                geometry_msgs::Quaternion odom_quat =
					tf::createQuaternionMsgFromYaw(theta);
  
                // first, we'll publish the transform over tf
                geometry_msgs::TransformStamped odom_trans;
                odom_trans.header.stamp = current_time;
                odom_trans.header.frame_id = frame_id;
                odom_trans.child_frame_id = base_link;
  
                odom_trans.transform.translation.x = x;
                odom_trans.transform.translation.y = y;
                odom_trans.transform.translation.z = 0.0;
                odom_trans.transform.rotation = odom_quat;
   
                // send the transform
                odom_broadcaster.sendTransform(odom_trans);
  
                // next, we'll publish the odometry
				//  message over ROS
                nav_msgs::Odometry odom;
                odom.header.stamp = current_time;
                odom.header.frame_id = frame_id;
  
                // set the position
                odom.pose.pose.position.x = x;
                odom.pose.pose.position.y = y;
                odom.pose.pose.position.z = 0.0;
                odom.pose.pose.orientation = odom_quat;
  
                // set the velocity
                odom.child_frame_id = base_link;
                odom.twist.twist.linear.x = vx/dt;
                odom.twist.twist.linear.y = vy/dt;
                odom.twist.twist.angular.z = vtheta/dt;

                // publish the message
                odom_pub.publish(odom);

                ros::spinOnce();
                update_rate.sleep();
            }
        }
    }
}


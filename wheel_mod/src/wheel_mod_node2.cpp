/*
 * wheel_mod_node.cpp
 *
 *  Created on: 02.11.2017
 *  Author: Mario
 */

#include <ros/ros.h>
#include <string>
#include <bt_p_sensor/ble_pressure.h>
#include <wheel_mod/radius.h>

bt_p_sensor::ble_pressure p;

const float g = 9.80665;

float m0_1 = 0.0;	 // mass [Kg]
float m0_2 = 0.0;	 // mass [Kg]
float m_1 = 0.0;
float m_2 = 0.0;

float p0_1 = 300000; // wheelpressure [Pa]
float p0_2 = 300000; // wheelpressure [Pa]
float p_1 = 0.0;
float p_2 = 0.0;

float T0_1 = 293.15; // wheeltemperature [K]
float T0_2 = 293.15; // wheeltemperature [K]

float r0_1 = 0.16;   // radius [m]
float r0_2 = 0.16;   // radius [m]
float r_1 = 0.0;
float r_2 = 0.0;

void sensorCallback(const bt_p_sensor::ble_pressure::ConstPtr& ptr)
{
  p = *ptr;
  p.temp_1 = p.temp_1+273.15;
  p.temp_2 = p.temp_2+273.15;
  p.press_1 = p.press_1*100000;
  p.press_2 = p.press_2*100000;
}

float temp_compensation(float p0, float T0, float p1, float T1)
{
  float p = 0.0;
  p = p1-p0*(1/(T0)*(T1-T0));
  return p;
}

float eff_radius(float p1, float r0)
{
  float r = 0.0;
  return r;
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"wheel_mod_node");

  ros::NodeHandle nh("~");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("pressure",100,sensorCallback);
  ros::Publisher radius_pub = n.advertise<wheel_mod::radius>("radius",100);

  std::string frame_id = "radius";
  nh.getParam("frame_id", frame_id);

  nh.getParam("m0_1", m0_1);
  nh.getParam("m0_2", m0_2);

  nh.getParam("p0_1", p0_1);
  nh.getParam("p0_2", p0_2);
	
  nh.getParam("T0_1", T0_1);
  nh.getParam("T0_2", T0_2);
	
  nh.getParam("r0_1", r0_1);
  nh.getParam("r0_2", r0_2);

  ros::Rate loop_rate(1);
  while(ros::ok())
  {
	  
    p_1	= temp_compensation(p0_1,T0_1,p.press_1,p.temp_1);
	p_2	= temp_compensation(p0_2,T0_2,p.press_2,p.temp_2);
	  
    ROS_INFO_STREAM("Druck 1: "<<p.press_1);
	ROS_INFO_STREAM("Druck 1 kompensiert: "<<p_1);
	  
	ROS_INFO_STREAM("Druck 2: "<<p.press_2);
	ROS_INFO_STREAM("Druck 2 kompensiert: "<<p_2);
	  
    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}

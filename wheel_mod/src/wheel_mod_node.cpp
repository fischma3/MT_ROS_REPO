/*
 * wheel_mod_node.cpp
 *
 *  Created on: 02.11.2017
 *  Author: Mario
 */

#include <ros/ros.h>
#include <string>
#include "bt_p_sensor/ble_pressure.h"
#include "wheel_mod/radius.h"

bt_p_sensor::ble_pressure p;
wheel_mod::radius r;

const float g = 9.80665;
const float Rm = 8.3144598;
const float pi = 3.14159265359;
const float ri = 0.11;

float n_1 = 0.0;
float V0_1 = 0.0;
float k1_1 = 0.0;

float n_2 = 0.0;
float V0_2 = 0.0;
float k1_2 = 0.0;

float m0_1 = 66.0;	 // mass [Kg]
float m0_2 = 66.0;	 // mass [Kg]
float m_1 = 0.0;
float m_2 = 0.0;

float p0_1 = 300000; // wheelpressure [Pa]
float p0_2 = 300000; // wheelpressure [Pa]
float p_1 = 300000;
float p_2 = 300000;

float T0_1 = 293.15; // wheeltemperature [K]
float T0_2 = 293.15; // wheeltemperature [K]

float r0_1 = 0.153;   // radius [m]
float r0_2 = 0.153;   // radius [m]
float ro0_1 = 0.16;   // radius [m]
float ro0_2 = 0.16;   // radius [m]
float dr_1 = 0.0;
float dr_2 = 0.0;
float r_1 = 0.16;
float r_2 = 0.16;

float p1 = 184.8;
float p2 = -72.83;
float p3 = 9.121;

int calibrate = 0;
int calib_mode = 0;
int samples = 10;
int counter = 0;
float p1_filt = 0.0;
float p2_filt = 0.0;
float T1_filt = 0.0;
float T2_filt = 0.0;

void sensorCallback(const bt_p_sensor::ble_pressure::ConstPtr& ptr)
{
  p = *ptr;
}

float temp_compensation(float p0, float T0, float p1, float T1)
{
  float p = 0.0;
  p = p1-p0*(1/(T0)*(T1-T0));
  return p;
}

float load_model(float p1, float p0)
{
  float m = 0.0;
  float p00 =       600.1;
  float p10 =     -0.2404;
  float p01 =   -0.003125;
  float p20 =   0.0001511;
  float p11 =   7.339e-07;
  float p02 =   4.111e-09;
  float p30 =  -4.873e-08;
  float p21 =  -2.545e-10;
  float p12 =    2.55e-13;
  float p40 =   2.433e-13;
  float p31 =   1.368e-13;
  float p22 =  -5.279e-16;
  float x = p1-p0;
  float y = p0;
  m = p00 + p10*x + p01*y + p20*pow(x,2) + p11*x*y + p02*pow(y,2) + p30*pow(x,3) + p21*pow(x,2)*y + p12*x*pow(y,2) + p40*pow(x,4) + p31*pow(x,3)*y + p22*pow(x,2)*pow(y,2);
  return m;
}

float radius_model(float T, float p, float V0, float k1, float n)
{
  float dV = 0.0;
  float dr = 0.0;

  dV = V0-(n*Rm*T)/p;
  //ROS_DEBUG_STREAM("dV : "<<dV);
  if (dV>=0)
  {
    dr = k1*sqrt(dV);
  //ROS_DEBUG_STREAM("dr : "<<dr);
  }
  return dr;
}

void calc_params()
{
// Sensor 1
  ro0_1 = sqrt(pow(r0_1,2)+pow(sqrt((2*m0_1*g/p0_1)/pi),2));
  ROS_DEBUG_STREAM("ro0_1 Sensor1 : "<<ro0_1);
  k1_1 = p1*pow(ro0_1,2)+p2*ro0_1+p3;
  ROS_DEBUG_STREAM("k1_1 Sensor1 : "<<k1_1);
  V0_1 = (pow(pi,2))/4*(ri+ro0_1)*pow((ro0_1-ri),2);
  ROS_DEBUG_STREAM("V0_1 Sensor1 : "<<V0_1);
  n_1 = p0_1*(V0_1-(pow(ro0_1-r0_1,2)/pow(k1_1,2)))/(Rm*T0_1);
  ROS_DEBUG_STREAM("n_1 Sensor1 : "<<n_1);

// Sensor 2
  ro0_2 = sqrt(pow(r0_2,2)+pow(sqrt((2*m0_2*g/p0_2)/pi),2));
  ROS_DEBUG_STREAM("ro0_2 Sensor2 : "<<ro0_2);
  k1_2 = p1*pow(ro0_2,2)+p2*ro0_2+p3;
  ROS_DEBUG_STREAM("k1_2 Sensor2 : "<<k1_2);
  V0_2 = (pow(pi,2))/4*(ri+ro0_2)*pow((ro0_2-ri),2);
  ROS_DEBUG_STREAM("V0_2 Sensor2 : "<<V0_2);
  n_2 = p0_2*(V0_2-(pow(ro0_2-r0_2,2)/pow(k1_2,2)))/(Rm*T0_2);
  ROS_DEBUG_STREAM("n_2 Sensor2 : "<<n_2);
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
  calc_params();

  ros::Rate loop_rate(1);
  while(ros::ok())
  {
      p_1	= temp_compensation(p0_1,T0_1,p.press_1,p.temp_1);
      p_2	= temp_compensation(p0_2,T0_2,p.press_2,p.temp_2);

      m_1 = load_model(p_1, p0_1);
      m_2 = load_model(p_2, p0_2);

      dr_1 = radius_model(p.temp_1,p.press_1,V0_1,k1_1,n_1);
      dr_2 = radius_model(p.temp_2,p.press_2,V0_2,k1_2,n_2);
      r.radius_1 = ro0_1-dr_1;
      r.radius_2 = 0;//ro0_2-dr_2;

    nh.getParam("calibrate", calibrate);
    if (calibrate==1)
    {
      ROS_INFO_STREAM("Calibration mode on");
      nh.setParam("calibrate", 0);
      calib_mode = 1;
    }

    switch (calib_mode) {
      case 0:
	radius_pub.publish(r);
        break;
      case 1:
	ROS_DEBUG_STREAM("Loading Parameters:");
	nh.getParam("samples", samples);

	nh.getParam("m0_1", m0_1);
    nh.getParam("m0_2", m0_2);
	
    nh.getParam("r0_1", r0_1);
    nh.getParam("r0_2", r0_2);

	ROS_DEBUG_STREAM("Wheel 1: ");
	ROS_DEBUG_STREAM("m0 = "<<m0_1);
	ROS_DEBUG_STREAM("r0 = "<<r0_1);

	ROS_DEBUG_STREAM("Wheel 2: ");
	ROS_DEBUG_STREAM("m0 = "<<m0_2);
	ROS_DEBUG_STREAM("r0 = "<<r0_2);

	ROS_INFO_STREAM("Calibrate over n-Samples = "<<samples);

	p1_filt = 0;
	p2_filt = 0;
	T1_filt = 0;
	T2_filt = 0;
	counter = 0;

	calib_mode = 2;
        break;
      case 2:
        p1_filt = p1_filt+p.press_1;
        p2_filt = p2_filt+p.press_2;
        T1_filt = T1_filt+p.temp_1;
        T2_filt = T2_filt+p.temp_2;
	counter++;
	if (counter>=samples)
	{
	  p1_filt = p1_filt/samples;
	  p2_filt = p2_filt/samples;
	  T1_filt = T1_filt/samples;
	  T2_filt = T2_filt/samples;
	  calib_mode = 3;
	}
        break;
      case 3:
        ROS_INFO_STREAM("Save Measurements");
	ROS_INFO_STREAM("Wheel 1: ");
	ROS_INFO_STREAM("Pressure: "<<p1_filt);
	ROS_INFO_STREAM("Temperature: "<<T1_filt);
	p0_1 = p1_filt;
	T0_1 = T1_filt;
	ROS_INFO_STREAM("Wheel 2: ");
	ROS_INFO_STREAM("Pressure: "<<p2_filt);
	ROS_INFO_STREAM("Temperature: "<<T2_filt);
        p0_2 = p2_filt;
        T0_2 = T2_filt;
	ROS_DEBUG_STREAM("Recalculate Parameters");
	calc_params();
	calib_mode = 0;
	ROS_INFO_STREAM("Calibration mode off");
        break;
      default:
        break;
    }

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}

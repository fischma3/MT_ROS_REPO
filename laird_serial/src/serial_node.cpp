/*
 * serial_node.cpp
 *
 *  Created on: 25.09.2017
 *      Author: mario
 */


#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Char.h>
#include <iostream>
#include <string>
#include <sstream>
#include <BLE_msgs/BLE_pressure.h>

serial::Serial ser;
BLE_msgs::BLE_pressure e;

void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;
    ros::NodeHandle n;

    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);
    ros::Publisher pressure_pub = n.advertise<BLE_msgs::BLE_pressure>("pressure",100);

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    std::string result,cHand,temp, outstr,sFail,connection;//, substring1, substring2;
    std::string substring[10];
    connection = "connection";
    cHand = "conHndl";
    sFail = "Fail";
    int state;
    state = 0;
    int cnHndl = 0;
    int atHndl = 0;
    float pressure = 0.0;
    float temperature = 0.0;
    ser.write("connect 01F2EB685E2BE7");
    ros::Rate loop_rate(10);
    while(ros::ok()){

    	ros::spinOnce();

        if(ser.available()){
            //ROS_INFO_STREAM("Reading from serial port");
            result = "";
            result = ser.read(ser.available());
            //ROS_INFO_STREAM("Read: " << result);
            temp = result;
            std::size_t pos =  temp.find(" ");
            int i = 0;
            while(!temp.empty()){
            	substring[i] = temp.substr(0,pos);
            	temp = temp.substr(pos+1);
            	//ROS_INFO_STREAM("sub:" << substring[i]);
            	pos =  temp.find(" ");
            	i++;
            	if(pos>=temp.length()){
            		pos =  temp.find("\n");
            		substring[i] = temp.substr(0,pos);
            		//ROS_INFO_STREAM("sub:" << substring[i]);
            		//ROS_INFO_STREAM("i:" << i);
            		break;
            	}
            }
            //ROS_INFO_STREAM("sub0"<<substring[0]);
            //ROS_INFO_STREAM("sub1"<<substring[1]);
            //ROS_INFO_STREAM("sub2"<<substring[2]);
            //ROS_INFO_STREAM("sub3"<<substring[3]);
            //ROS_INFO_STREAM("sub4"<<substring[4]);
            //ROS_INFO_STREAM("sub5"<<substring[5]);
            switch (state) {
				case 0:
					if (substring[0].compare(connection)==0){
						ROS_INFO_STREAM("Connected to Sensor with connection Handle"<<substring[2]);
						std::stringstream stream1(substring[2]);
						if(sscanf(substring[2].c_str(),"%d",&cnHndl)!=1){}
						ROS_INFO_STREAM("Connected to:"<<cnHndl);
						ROS_INFO_STREAM("Get Data from Attribute Handle 32");
						outstr = "pread";
						ser.write(outstr);
						state = 1;
					}
					else if(result.find(sFail)==0){
						ser.write("connect 01F2EB685E2BE7");
						ROS_INFO_STREAM("Failed!!!!");
					}
					else{
						ROS_INFO_STREAM("command not detected State 0");
						ROS_INFO_STREAM("Read: " << result);
						ROS_INFO_STREAM("command:" << substring[0]);
						pos =  temp.find(sFail);
						ROS_INFO_STREAM("pos:" << pos);
					}
					break;
				case 1:
					if (substring[0].compare(cHand)==0){
						ROS_INFO_STREAM("Connection Handle "<<substring[1]);
						ROS_INFO_STREAM("Attribute Handle "<<substring[3]);
						//ROS_INFO_STREAM("Temperature "<<substring[5]);
						//ROS_INFO_STREAM("Pressure "<<substring[7]);
						if(sscanf(substring[1].c_str(),"%d",&cnHndl)!=1){}
						if(sscanf(substring[3].c_str(),"%d",&atHndl)!=1){}
						temperature = ((float)strtoul(substring[5].c_str(),NULL,16)/100);
						pressure = ((float)strtoul(substring[7].c_str(),NULL,16)/1000000);
						ROS_INFO_STREAM("Temperature "<<temperature);
						ROS_INFO_STREAM("Pressure "<<pressure);
						ROS_INFO_STREAM("\n");
						e.atHndl=atHndl;
						e.conHndl=cnHndl;
						e.temperature=temperature;
						e.pressure= pressure;
						pressure_pub.publish(e);
					}
					else{
						ROS_INFO_STREAM("command not detected State 1");
						ROS_INFO_STREAM("Read: " << result);
						ROS_INFO_STREAM("command:" << substring[0]);
					}
					break;
				default:
					ROS_INFO_STREAM("State error");
					break;
			}
            ser.flush();
            read_pub.publish(result);
        }
        loop_rate.sleep();

    }
}

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
#include <bt_p_sensor/ble_pressure.h>
#include <bt_p_sensor/ble_pressure_single.h>
#include <ros/xmlrpc_manager.h>
#include <signal.h>

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void sigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1)
  {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    g_request_shutdown = 1; // Set flag
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
}

serial::Serial ser;
bt_p_sensor::ble_pressure e;
bt_p_sensor::ble_pressure_single es;

int main (int argc, char** argv){
	// Override SIGINT handler
    ros::init(argc, argv, "BT900_US_communication",ros::init_options::NoSigintHandler);
	signal(SIGINT, sigIntHandler);
	
	
    // Override XMLRPC shutdown
    ros::XMLRPCManager::instance()->unbind("shutdown");
    ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);
	
    ros::NodeHandle nh("~");
    ros::NodeHandle n;
    ros::Publisher pressure_pub = n.advertise<bt_p_sensor::ble_pressure>("pressure",100);
	ros::Publisher pressure_single_pub = n.advertise<bt_p_sensor::ble_pressure_single>("pressure_single",100);

	int main_state = 0;
	int next_state = 0;
	int con_fail_cnt = 0;
    int cnHndl = 0;
    int cnHndl_1 = 0;
    int cnHndl_2 = 0;
    int atHndl = 0;
    int sensor_1 = 0;
	int sensor_2 = 0;
	int output_slow_dev = 10;
	int output_slow_cnt = 1;
	float T_new[2];
	float T[2];
	float p_new[2];
	float p[2];
    std::string result, temp, outstr;
    std::string substring[10];
    std::string keywords_in[7];
    std::string keywords_out[9];

    keywords_in[0] = "error";
    keywords_in[1] = "fail";
    keywords_in[2] = "connection";
    keywords_in[3] = "write";
    keywords_in[4] = "read";
    keywords_in[5] = "sensors";
    keywords_in[6] = "exit";

    keywords_out[0] = "connect ";
    keywords_out[1] = "disconnect ";
    keywords_out[2] = "read ";
    keywords_out[3] = "aread ";
    keywords_out[4] = "stop ";
    keywords_out[5] = "write ";
    keywords_out[6] = "exit ";
    keywords_out[7] = "sensors ";
	keywords_out[8] = "ack";

    std::string port = "/dev/ttyUSB0";
    nh.getParam("port", port);

    int baudrate = 115200;
    nh.getParam("baudrate", baudrate);

    std::string frame_id = "p_sen_com";
    nh.getParam("frame_id", frame_id);

    int n_sensor = 2;
    nh.getParam("n_sensor", n_sensor);
	
	int filter = 0;
    nh.getParam("filter", filter);
	
	float alpha_p = 0.1;
    nh.getParam("alpha_p", alpha_p);
	
	float alpha_T = 0.1;
    nh.getParam("alpha_T", alpha_T);

    std::string sensor_1_addr = "01F2EB685E2BE7";
    nh.getParam("sensor_1", sensor_1_addr);

    std::string sensor_2_addr = "01F7B34480DDC8";
    nh.getParam("sensor_2", sensor_2_addr);

    try
    {
        ser.setPort(port);
        ser.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port "<<port);
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized with baudrate "<<baudrate);
    }else{
        return -1;
    }
	ROS_DEBUG_STREAM("Sende:"<<keywords_out[7]);
	ser.write(keywords_out[7]);

    ros::Rate loop_rate(10);

    while(!g_request_shutdown){		
		//------------------ Incomming Data Task---------------
        if(ser.available()){
            result = "";
            result = ser.read(ser.available());
            temp = result;
            std::size_t pos =  temp.find(" ");
            int i = 0;
            while(!temp.empty()){ // Read incomming Data
            	substring[i] = temp.substr(0,pos);
            	temp = temp.substr(pos+1);
            	pos =  temp.find(" ");
            	i++;
            	if(pos>=temp.length()){
            		pos =  temp.find("\n");
            		substring[i] = temp.substr(0,pos);
            		break;
            	}
            }
			switch (main_state){ // static
				case 0: // Check how many senors are connected
					if (substring[0].compare(keywords_in[5])==0){ //sensors
						std::stringstream stream1(substring[1]);
						if(sscanf(substring[1].c_str(),"%d",&sensor_1)!=1){}
						if(sscanf(substring[2].c_str(),"%d",&cnHndl_1)!=1){}
						if(sscanf(substring[3].c_str(),"%d",&sensor_2)!=1){}
						if(sscanf(substring[4].c_str(),"%d",&cnHndl_2)!=1){}
						// Check if all required sensors are connected, if not go to connection State
						ROS_DEBUG_STREAM("sensor_1:"<<sensor_1);
						ROS_DEBUG_STREAM("sensor_2:"<<sensor_2);
						if (sensor_1==0){
							next_state = 1; // Connect to sensor 1
						}
						else if ((sensor_2==0)&&(n_sensor==2)){
							next_state = 2; // Connect to sensor 2
						}
						else {
							next_state = 3; // Auto-Read data from sensors
						}
					}
					else{
						ROS_DEBUG_STREAM("Main state:"<<main_state);
						ROS_DEBUG_STREAM("Read: " << result);
						ROS_DEBUG_STREAM("command:" << substring[0]);
						ROS_DEBUG_STREAM("Sende:"<<keywords_out[7]);
						ser.write(keywords_out[7]);
						next_state = 0;
					}
					break;
				case 1: // Connect to sensor 1
					if (substring[0].compare(keywords_in[2])==0){ // connection
						ROS_INFO_STREAM("Connected to Sensor 1 with connection Handle "<<substring[1]);
						std::stringstream stream1(substring[1]);
						if(sscanf(substring[1].c_str(),"%d",&cnHndl_1)!=1){}
						sensor_1 = 1;
						con_fail_cnt = 0;
						ROS_DEBUG_STREAM("sensor_1:"<<sensor_1);
						ROS_DEBUG_STREAM("sensor_2:"<<sensor_2);
						if ((sensor_2==0)&&(n_sensor==2)){
							next_state = 2; // Connect to sensor 2
						}
						else{
							next_state = 3; // Auto-Read data from sensors
						}
					}
					else if(result.find(keywords_in[1])==0){ // fail
						ROS_DEBUG_STREAM("Sende:"<<keywords_out[0]+"1 "+sensor_1_addr);
						ser.write(keywords_out[0]+"1 "+sensor_1_addr);
						ROS_INFO_STREAM("Sensor 1 not found, try again...");
						con_fail_cnt++;
						next_state = 1; // Connect to sensor 1
					}
					else{
						ROS_DEBUG_STREAM("Main state:"<<main_state);
						ROS_DEBUG_STREAM("Read: " << result);
						ROS_DEBUG_STREAM("command:" << substring[0]);
						next_state = 0;
					}
					break;
				case 2: // Connect to sensor 2
					if (substring[0].compare(keywords_in[2])==0){ // connection
						ROS_INFO_STREAM("Connected to Sensor 2 with connection Handle "<<substring[1]);
						std::stringstream stream1(substring[1]);
						if(sscanf(substring[1].c_str(),"%d",&cnHndl_2)!=1){}
						sensor_2 = 1;
						con_fail_cnt = 0;
						next_state = 3; // Auto-Read data from sensors
					}
					else if(result.find(keywords_in[1])==0){ // fail
						ROS_DEBUG_STREAM("Sende:"<<keywords_out[0]+"2 "+sensor_2_addr);
						ser.write(keywords_out[0]+"2 "+sensor_2_addr);
						ROS_INFO_STREAM("Sensor 2 not found, try again...");
						con_fail_cnt++;
						next_state = 2; // Connect to sensor 2
					}
					else{
						ROS_DEBUG_STREAM("Main state:"<<main_state);
						ROS_DEBUG_STREAM("Read: " << result);
						ROS_DEBUG_STREAM("command:" << substring[0]);
						next_state = 0;
					}
					break;
				case 3: // Auto-Read data from sensors
					if (substring[0].compare(keywords_in[4])==0){
						if(sscanf(substring[1].c_str(),"%d",&cnHndl)!=1){}
						if(sscanf(substring[2].c_str(),"%d",&atHndl)!=1){}
						if ((((float)strtoul(substring[3].c_str(),NULL,16)/100)!=0)&&(((float)strtoul(substring[4].c_str(),NULL,16)/1000000)>=0.5)){
							if (cnHndl==cnHndl_1){
								es.sensor = 1;
							}
							else if (cnHndl==cnHndl_2){
								es.sensor = 2;
							}
							T_new[es.sensor] = ((float)strtoul(substring[3].c_str(),NULL,16)/100+273.15);
							p_new[es.sensor] = ((float)strtoul(substring[4].c_str(),NULL,16)/10);
							es.temp = T_new[es.sensor];
							es.press = p_new[es.sensor];
						}
						es.header.stamp = ros::Time::now();
						es.header.frame_id = frame_id;
						pressure_single_pub.publish(es);
						//ROS_DEBUG_STREAM("Sende:"<<keywords_out[8]);
						ser.write(keywords_out[8]); // ack
					}
					else if (substring[0].compare(keywords_in[2])==0){
						std::stringstream stream1(substring[1]);
						if(sscanf(substring[1].c_str(),"%d",&cnHndl)!=1){}
						if (cnHndl== cnHndl_1){
							ROS_WARN("Connection to Sensor 1 lost");
							cnHndl_1 = 0;
							next_state = 1; // Connect to sensor 1
						}
						else if (cnHndl==cnHndl_2){
							ROS_WARN("Connection to Sensor 2 lost");
							cnHndl_2 = 0;
							next_state = 2; // Connect to sensor 2
						}
					}
					else{
						ROS_DEBUG_STREAM("Main state:"<<main_state);
						ROS_DEBUG_STREAM("Read: " << result);
						ROS_DEBUG_STREAM("command:" << substring[0]);
						next_state = 0;
					}
					break;
			}
			if (substring[0].compare(keywords_in[0])==0){ //Error
				ROS_DEBUG_STREAM("Error: " << substring[1]+" "+substring[2]);
				next_state = 0;
			}
			ser.flush();
		}
		//------------------ END Incomming Data Task---------------
		
		//------------------ Transient Task---------------
		if (main_state != next_state){ 
			ROS_DEBUG_STREAM("Main state changed to "<<next_state);
			switch (next_state){ // transient
				case 0: // Check how many senors are connected
					ROS_DEBUG_STREAM("Sende:"<<keywords_out[7]);
					ser.write(keywords_out[7]); // sensors
					break;
				case 1: // Connect to sensor 1
					ROS_DEBUG_STREAM("Sende:"<<keywords_out[0]+"1 "+sensor_1_addr);
					ser.write(keywords_out[0]+"1 "+sensor_1_addr);
					break;
				case 2: // Connect to sensor 2
					ROS_DEBUG_STREAM("Sende:"<<keywords_out[0]+"2 "+sensor_2_addr);
					ser.write(keywords_out[0]+"2 "+sensor_2_addr);
					break;
				case 3: // Auto-Read data from sensors
					ROS_DEBUG_STREAM("Start Auto-Read Data");
					ROS_DEBUG_STREAM("Sende:"<<keywords_out[3]+"32");
					ser.write(keywords_out[3]+"32");
					break;
				default:
					ROS_WARN("P-Sensor-Node: Mainstate error");
					break;
			}
		}
		//------------------ END Transient Task---------------
		
		//------------------ Periodic Output Task---------------
		if (output_slow_cnt>output_slow_dev){
			nh.getParam("filter", filter);
			nh.getParam("alpha_T", alpha_T);
			nh.getParam("alpha_p", alpha_p);
			switch (main_state){ // periodic
				case 0: // Check how many senors are connected
					break;
				case 1: // Connect to sensor 1
					break;
				case 2: // Connect to sensor 2
					break;
				case 3: // Auto-Read data from sensors
					if (filter==1){
						T[1] = (float)alpha_T*T_new[1]+(float)(1-alpha_T)*T[1];
						p[1] = (float)alpha_p*p_new[1]+(float)(1-alpha_p)*p[1];
						T[2] = (float)alpha_T*T_new[2]+(float)(1-alpha_T)*T[2];
						p[2] = (float)alpha_p*p_new[2]+(float)(1-alpha_p)*p[2];
					}
					else{
						T[1] = T_new[1];
						p[1] = p_new[1];
						T[2] = T_new[2];
						p[2] = p_new[2];
					}
					e.header.stamp = ros::Time::now();
					e.header.frame_id = frame_id;
					e.temp_1 = T[1];
					e.press_1 = p[1];
					e.temp_2 = T[2];
					e.press_2 = p[2];
					e.n_sensors = n_sensor;
					pressure_pub.publish(e);
					break;
				default:
					break;
			}
		output_slow_cnt = 0;
		}
		output_slow_cnt++;
		//------------------ END Periodic Output Task---------------
		
		main_state = next_state;
		
		if (con_fail_cnt>=10){
			ROS_WARN("P-Sensor-Node: Connection to Sensor failed >= 10");
			con_fail_cnt = 0;
		}
		
		ros::spinOnce();
        loop_rate.sleep();

    } // END while(!g_request_shutdown)
	ser.flush();
	ROS_DEBUG_STREAM("Sende:"<<keywords_out[4]);
	ser.write(keywords_out[4]);
	ser.close();
	ros::shutdown();
}

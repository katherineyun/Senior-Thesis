/*
3 ros line commands that are most helpful: rostopic, rosmsg,and rosservice
*/

#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <stdlib.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>



using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace gazebo;

//Global Variables for interested values, services, and clients
// ros::Time t;
// gazebo_msgs::GetModelState targetPositionSrv;
// ros::ServiceClient targetPositionClient;

//Publisher
transport::PublisherPtr simulationControlPublisher;
transport::PublisherPtr modelDelPublisher;
transport::PublisherPtr modelModifyPublisher;
transport::PublisherPtr addModelPublisher;

class modelTaskCallback : public PortReader {
    virtual bool read(ConnectionReader& connection) {
    	/*
    	"in" is a bottle indicating the request 
    	-d, --delete
              Delete a model.

       -f, --spawn-file=arg
              Spawn model from SDF file.
              	-x, --pose-x arg x value
				-y, --pose-y arg y value
				-z, --pose-z arg z value
				-R, --pose-R arg roll in radians.
				-P, --pose-P arg pitch in radians.
				-Y, --pose-Y arg yaw in radians.

       -p, --pose
              Output model pose as a space separated 6-tuple: x y z roll pitch yaw.

        -m --modify
        	  Modify the position of the model (-x, -y, -z, -R, -P, -Y)
    	

    	*/
        Bottle in, out;

        in.read(connection); // get data
        string options=(in.get(0)).toString();
        string modelName=(in.get(1)).toString();
        ignition::math::Pose3d pose;
  		ignition::math::Vector3d rpy;
		pose.Pos().X((in.get(2)).asDouble());
		pose.Pos().Y((in.get(3)).asDouble());
		pose.Pos().Z((in.get(4)).asDouble());

		rpy.X((in.get(5)).asDouble());
		rpy.Y((in.get(6)).asDouble());
		rpy.Z((in.get(7)).asDouble());
		pose.Rot().Euler(rpy);

		string fileName=(in.get(8)).toString();
		bool good=false;
		ignition::math::Pose3d positionResponse;


        if (options=="d"){
        	msgs::Request *msg = msgs::CreateRequest("entity_delete", modelName); 
   			modelDelPublisher->Publish(*msg, true);
    		delete msg;
    		good=true;
        }
        else if (options=="p"){
        	boost::shared_ptr<msgs::Response> response = transport::request("default", "entity_info", modelName);
    		msgs::Model modelMsg;

		    if (response->has_serialized_data() &&
		        !response->serialized_data().empty() &&
		        modelMsg.ParseFromString(response->serialized_data())){
		    
			        positionResponse=gazebo::msgs::ConvertIgn(modelMsg.pose()); //we only need position

			    good=true;
		    }
		    else{
		      cout << "Unable to get info on model[" << modelName << "] in the world"<<endl;
		    }

        }
        else if (options=="f"){


			ifstream ifs(fileName.c_str());
			if (!ifs){
				cerr << "Error: Unable to open file[" << fileName << "]\n";
				good=false;
			}
			else good=true;

			sdf::SDFPtr sdf(new sdf::SDF());
			if (!sdf::init(sdf)){
				cerr << "Error: SDF parsing the xml failed" <<endl;
				good=false;
			}
			else good=true;

			if (!sdf::readFile(fileName, sdf))
			{
				cerr << "Error: SDF parsing the xml failed\n";
				good=false;
			}
			else good=true;

			if (good){
				
				sdf::ElementPtr modelElem = sdf->Root()->GetElement("model");

				if (!modelElem){
					cerr << "Unable to find <model> element.\n";
					good=false;
				}
				else{
					// Set the model name
					if (!modelName.empty())
					modelElem->GetAttribute("name")->SetFromString(modelName);
					msgs::Factory msg;
					msg.set_sdf(sdf->ToString());
					msgs::Set(msg.mutable_pose(), pose);
					addModelPublisher->Publish(msg, true);
					good=true;
				}

			}

			

        }
        else if (options=="m"){
			msgs::Model msg;
			msg.set_name(modelName);
			msgs::Set(msg.mutable_pose(), pose);
			modelModifyPublisher->Publish(msg, true);
			good=true;

        }
        else{
        	cout<<"Unknown command"<<endl;
        	good=false;
        }



		out.clear();
		if (good){
			if (options=="p"){
				out.addDouble(positionResponse.Pos()[0]);
				out.addDouble(positionResponse.Pos()[1]);
				out.addDouble(positionResponse.Pos()[2]);

			}
			else out.addInt(1);
		}
		else out.addInt(0);
		
        // respond
        ConnectionWriter *returnToSender = connection.getWriter();
        if (returnToSender!=NULL) {
            out.write(*returnToSender);
        }


        return true;
    }
};

modelTaskCallback modelTask;

class simulationControlCallback : public PortReader {
    virtual bool read(ConnectionReader& connection) {
    	/*
    	"in" is a bottle indicating the request
    	"out" is a bool indicating if the request is fullfilled 
    	-p, --pause=arg
              Pause/unpause simulation. 0=unpause, 1=pause.
        		e.g. the bottle[0]="p", bottle[1]=1 will pause the simulation

       -m, --multi-step=arg
              Step simulation mulitple iteration. arg=the number of steps you want an int

       -r, --reset-all
              Reset time and model poses
        */
        Bottle in, out; 
        gazebo::msgs::WorldControl worldControlMsg;
        in.read(connection); // get data
        string s=(in.get(0)).toString();
        int a=(in.get(1)).asInt();
        bool good = false;
        if (s=="p"){
        	if (a==1) worldControlMsg.set_pause(true);
        	else worldControlMsg.set_pause(false);
        	good=true;
        }
        else if (s=="m"){
        	worldControlMsg.set_multi_step(a);
        	good=true;
        }
        else if (s=="r"){
        	worldControlMsg.mutable_reset()->set_all(true);
        	good =true;
        }
        else {
        	cout<<"Unknown command"<<endl;
        }
        out.clear();
        if (good){
        	simulationControlPublisher->Publish(worldControlMsg);
			out.addInt(1);
		}
		else out.addInt(0);
		
        // respond
        ConnectionWriter *returnToSender = connection.getWriter();
        if (returnToSender!=NULL) {
            out.write(*returnToSender);
        }
        return true;
    }
};
simulationControlCallback simulationControl;




int main(int argc, char **argv)
{
// YARP RPC
	Network yarp;

	//YARP server RPC 
	Port modelTaskPort;            
    modelTaskPort.open("/gazeboInterface/modelTaskServer");     // Give it a name on the network.
    modelTaskPort.setReader(modelTask);  // register a callback function

    Port simulationControlPort;            
    simulationControlPort.open("/gazeboInterface/simulationControlServer");     // Give it a name on the network.
    simulationControlPort.setReader(simulationControl);  // register a callback function

	
/*	
// ROS Service Clients
    //initialzation of ros required for each Node
	ros::init(argc, argv, "clientsNode");	
	//handler for this node.
	ros::NodeHandle nh;	
	
	//initialized publisher ur3/command, buffer size of 10.
	//ros::Publisher pub_command=nh.advertise<ece470_ur3_driver::command>("ur3/command",10);
	// initialize subscriber to ur3/position and call function position_callback each time data is published
	ros::Subscriber sub_clock=nh.subscribe("/clock",1,clock_callback);
	//ros client for pausing the world
	//rosservice type /gazebo/pause_physics=std_srvs/Empty
	ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
	std_srvs::Empty srv;

	// ros client for getting target positions
	ros::ServiceClient targetPositionClient = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
*/
	

//Gazebo simlation control
	transport::init(); // it is needed for stand alone gazebo program
	transport::run(); // it is needed for stand alone gazebo program

	// create a new node 
	transport::NodePtr node(new transport::Node());
	// make sure the topic names are matched: the topic is /gazebo/default/world_control
	node->Init("default"); //so you should name it default and ~/world_control

	//simulation control
	simulationControlPublisher = node->Advertise<msgs::WorldControl>("~/world_control");
	//topic has a subscriber, it will try to connect to us automatically
  	simulationControlPublisher->WaitForConnection();

  	// model delete
  	modelDelPublisher = node->Advertise<msgs::Request>("~/request");
    modelDelPublisher->WaitForConnection();

    // modify model location
    modelModifyPublisher = node->Advertise<msgs::Model>("~/model/modify");
    modelModifyPublisher->WaitForConnection();

    // add model
    addModelPublisher = node->Advertise<msgs::Factory>("~/factory");
	addModelPublisher->WaitForConnection();
	
	
	
	while (1) {
		// will this matter ???
		usleep(1000);
	}  

	return 0;
}
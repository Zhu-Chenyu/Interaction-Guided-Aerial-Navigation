#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "NatNetTypes.h"
#include "NatNetClient.h"
#include "NatNetCAPI.h"

// Global variables for data sharing between callback and main loop
static geometry_msgs::msg::TransformStamped tf_msg;
static bool initialized = false;

enum objectIDs {
	OMNID1ID = 103,
	OMNID2ID = 104,
	OMNID3ID = 105,
	LEFTHANDID = 106,
	RIGHTHANDID = 107,
	VESTID = 108,
	WAND = 1015
};

void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* clockPointer);

void initialize_optitrack(int* ret, NatNetClient** mClient, rclcpp::Clock::SharedPtr clock);

void get_optitrack_data(NatNetClient* mClient);

void shutdown_optitrack(NatNetClient** mClient);

// Global TF broadcaster (will be set in main)
static std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

int main (int argc, char* argv[]) {
	rclcpp::init(argc, argv);

	auto optitrack_node = rclcpp::Node::make_shared("optitrack_publisher");
	auto optitrack_clock = optitrack_node->get_clock();
	
	// Create TF broadcaster
	tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(optitrack_node);
	
	rclcpp::Rate optitrack_rate(100);
	
	// initialize motiveClient and connect to server
	int result = ErrorCode_OK;
	NatNetClient* mClient = nullptr;
	initialize_optitrack(&result, &mClient, optitrack_clock);
	if (result != ErrorCode_OK) {
		shutdown_optitrack(&mClient);
		rclcpp::shutdown();
	    return 0;
	}
	
	// print rigid body's reference position and orientation
	get_optitrack_data(mClient);
	
	while(rclcpp::ok()) {
		if (initialized) {
			// Broadcast TF transform
			tf_broadcaster->sendTransform(tf_msg);
			rclcpp::spin_some(optitrack_node);
		}

		optitrack_rate.sleep();
	}

	// disconnect from optitrack server and shutdown rclcpp
	shutdown_optitrack(&mClient);
	std::cout << "Shut down" << std::endl;
	return 0;
}

void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* clockPointer) {

	// identify all rigid bodies (Omnids, person, and wand) in frame, write data to corresponding file

	for (int i = 0; i < data->nRigidBodies; i++) {
		bool trackingValid = data->RigidBodies[i].params & 0x01; // True if Rigid Body is detected in frame, false if occluded
		
		if (data->RigidBodies[i].ID == WAND) {
			// if you want to measure latency of message, refer to how it was implemented in Sampleclient C++'s DataHandler function

			// update message to most recent frame of data
			if (trackingValid) {
				if (!initialized) {
					initialized = true;
				}
				rclcpp::Clock* clock_ptr = static_cast<rclcpp::Clock*>(clockPointer);
				
				// Update TF transform message
				tf_msg.header.stamp = clock_ptr->now();
				tf_msg.header.frame_id = "optitrack";  // Parent frame (world/optitrack origin)
				tf_msg.child_frame_id = "imu_link";       // Child frame (tracked object)
				tf_msg.transform.translation.x = data->RigidBodies[i].x;
				tf_msg.transform.translation.y = data->RigidBodies[i].y;
				tf_msg.transform.translation.z = data->RigidBodies[i].z;
				tf_msg.transform.rotation.x = data->RigidBodies[i].qx;
				tf_msg.transform.rotation.y = data->RigidBodies[i].qy;
				tf_msg.transform.rotation.z = data->RigidBodies[i].qz;
				tf_msg.transform.rotation.w = data->RigidBodies[i].qw;
			}
			
			break;
		}
	}
}

void initialize_optitrack(int* ret, NatNetClient** mClient, rclcpp::Clock::SharedPtr clock) {
	// disconnect in case already connected before
    if (*mClient) {
    	(*mClient)->Disconnect();
    	delete *mClient;
    }
    
	// create Client Object and attach DataHandler to parse received data
    *mClient = new NatNetClient();
	(*mClient)->SetFrameReceivedCallback(DataHandler, clock.get());
    
	// set connection parameters. Modify connectionType and serverAddress to match the streaming settings on Motive
    sNatNetClientConnectParams g_connectParams;
    g_connectParams.connectionType = ConnectionType_Unicast; // change to ConnectionType_Multicast if you want multicast
    g_connectParams.serverAddress = "192.168.20.189";
    g_connectParams.localAddress = "192.168.20.135";
    
	// check if client-server connection was successfully established
    *ret = (*mClient)->Connect(g_connectParams);
    if (*ret != ErrorCode_OK) {
        std::cerr << "Failed to connect to OptiTrack server." << std::endl;
    } else {
        std::cout << "Successfully connected to OptiTrack server." << std::endl;
        
        // print server info and natnet SDK version
        /* this block of code is copied from the SampleClient.cpp downloaded from NatNet SDK 4.0.0 Ubuntu which can be accessed here: https://optitrack.com/support/downloads/developer-tools.html#natnet-sdk */
		sServerDescription g_serverDescription;
		memset( &g_serverDescription, 0, sizeof( g_serverDescription ) );
		*ret = (*mClient)->GetServerDescription( &g_serverDescription );
		if ( *ret != ErrorCode_OK || ! g_serverDescription.HostPresent )
		{
			std::cout << "Unable to connect to server. Host not present. Exiting." << std::endl;
		}
		std::cout << "[SampleClient] Server application info:" << std::endl;

		std::cout << "Application: " << g_serverDescription.szHostApp << " (ver. " << g_serverDescription.HostAppVersion[0] << "." << 
			g_serverDescription.HostAppVersion[1] << "." << g_serverDescription.HostAppVersion[2] << "." <<
			g_serverDescription.HostAppVersion[3] << std::endl;

		std::cout << "NatNet Version: " << g_serverDescription.NatNetVersion[0] << "." << g_serverDescription.NatNetVersion[1] << "." <<
			g_serverDescription.NatNetVersion[2] << "." << g_serverDescription.NatNetVersion[3] << std::endl;

		std::cout << "Client IP:" << g_connectParams.localAddress << std::endl;
		std::cout << "Server IP:" << g_connectParams.serverAddress << std::endl;
		std::cout << "Server Name:"<< g_serverDescription.szHostComputerName << std::endl << std::endl;
		/* copy pasted code ends here */
    }
}

void get_optitrack_data(NatNetClient* mClient) {
	if (!mClient) {
		std::cout << "Not Connected to Server!" << std::endl;
		return;
	}
	std::string ddMessage;
	// requesting Data Description
	sDataDescriptions* dataDescriptions = nullptr;
	int iResult = mClient->GetDataDescriptionList(&dataDescriptions);
	
	if (iResult != ErrorCode_OK || dataDescriptions == nullptr) {
	    ddMessage = "[new_optitrack]Unable to Retrieve Data Descriptions.";
	}
	else {
		int count = 0;
		// parse through each Data Description and extract relevant information
	    for (int i = 0; i < dataDescriptions->nDataDescriptions; i++) {
	    	if (dataDescriptions->arrDataDescriptions[i].type == Descriptor_RigidBody) {
				count++;
				sRigidBodyDescription* pRB = dataDescriptions->arrDataDescriptions[i].Data.RigidBodyDescription;
				ddMessage += pRB->szName;
				for (int j = 0; j < pRB->nMarkers; j++) {
					//message format: name, (x,y,z) positions of each marker corresponding to that rigid body
					ddMessage += "\t" + std::to_string(pRB->MarkerPositions[j][0]) + ", " + std::to_string(pRB->MarkerPositions[j][1]) + ", " + std::to_string(pRB->MarkerPositions[j][2]);
				}
				ddMessage += "\n";
	    	}
	    }
		// if there are no rigid bodies
		if (count == 0) {
			ddMessage = "No Rigid Bodies";
		}
	}

	if (dataDescriptions) {
		NatNet_FreeDescriptions(dataDescriptions);
	}
	
	std::cout << ddMessage << std::endl;
}

void shutdown_optitrack(NatNetClient** mClient) {
	// disconnect from server and free memory
    if (*mClient) {
		(*mClient)->SetFrameReceivedCallback(nullptr, nullptr);
		(*mClient)->Disconnect();
		delete *mClient;
		*mClient = nullptr;
    }
}
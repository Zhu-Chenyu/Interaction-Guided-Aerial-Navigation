#include <memory>
#include <string>
#include <sys/stat.h>
#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "NatNetTypes.h"
#include "NatNetClient.h"
#include "NatNetCAPI.h"

static std::shared_ptr<geometry_msgs::msg::PoseStamped> optitrack_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
static bool initialized = false;
static FILE* mainOmnidFile;
static FILE* omnid1Position;
static FILE* omnid2Position;
static FILE* omnid3Position;
static FILE* personPosition;
static FILE* allBodyRefOrientations;
enum objectIDs {
	OMNID1ID = 103,
	OMNID2ID = 104,
	OMNID3ID = 105,
	LEFTHANDID = 106,
	RIGHTHANDID = 107,
	VESTID = 108,
	WAND = 109
};

void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* clockPointer);

void initialize_optitrack(int* ret, NatNetClient** mClient, rclcpp::Clock::SharedPtr clock);

void get_optitrack_data(NatNetClient* mClient);

void shutdown_optitrack(NatNetClient** mClient);

void close_all_csvs();

void _WriteHeadingOnly(FILE* fp);

void _WriteData(FILE* fp, sFrameOfMocapData* data, char* name, bool trackingValid, int i);

int main (int argc, char* argv[]) {
	rclcpp::init(argc, argv);

	auto optitrack_node = rclcpp::Node::make_shared("optitrack_publisher");
	auto optitrack_clock = optitrack_node->get_clock();
	auto optitrack_qos = rclcpp::QoS(rclcpp::KeepAll()).reliable().durability_volatile();
	auto optitrack_pub = optitrack_node->create_publisher<geometry_msgs::msg::PoseStamped>("optitrack_data", optitrack_qos);
	rclcpp::Rate optitrack_rate(100);
	
	// initialize motiveClient and connect to server
	int result = ErrorCode_OK;
	NatNetClient* mClient = nullptr;
	initialize_optitrack(&result, &mClient, optitrack_clock);
	if (result != ErrorCode_OK) {
		shutdown_optitrack(&mClient);
		close_all_csvs();
		rclcpp::shutdown();
	    return 0;
	}
	
	// print rigid body's reference position and orientation
	get_optitrack_data(mClient);
	
	while(rclcpp::ok()) {
		if (initialized) {
			optitrack_pub->publish(*optitrack_msg);
			rclcpp::spin_some(optitrack_node);
		}

		optitrack_rate.sleep();
	}

	// disconnect from optitrack server and shutdown rclcpp
	shutdown_optitrack(&mClient);
	close_all_csvs();
	std::cout << "Shut down" << std::endl;
	return 0;
}

void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* clockPointer) {
	// identify all rigid bodies (Omnids, person, and wand) in frame, write data to corresponding file
	for (int i = 0; i < data->nRigidBodies; i++) {
		bool trackingValid = data->RigidBodies[i].params & 0x01; // True if Rigid Body is detected in frame, false if occluded
		char name[32];
		switch (data->RigidBodies[i].ID) {
			case OMNID1ID:
                strcpy(name, "Omnid1");
				if (omnid1Position) {
				    _WriteData(omnid1Position, data, name, trackingValid, i);
                }
				break;
			case OMNID2ID:
                strcpy(name, "Omnid2");
				if (omnid2Position) {
				    _WriteData(omnid2Position, data, name, trackingValid, i);
                }
				break;
			case OMNID3ID:
                strcpy(name, "Omnid3");
				if (omnid3Position) {
				    _WriteData(omnid3Position, data, name, trackingValid, i);
                }
				break;
			case LEFTHANDID:
                strcpy(name, "Left Hand");
				if (personPosition) {
				    _WriteData(personPosition, data, name, trackingValid, i);
                }
				break;
			case RIGHTHANDID:
                strcpy(name, "Right Hand");
				if (personPosition) {
				    _WriteData(personPosition, data, name, trackingValid, i);
                }
				break;
			case VESTID:
                strcpy(name, "Vest");
				if (personPosition) {
				    _WriteData(personPosition, data, name, trackingValid, i);
                }
				break;
			case WAND:
				strcpy(name, "Wand");
				if (personPosition) {
				    _WriteData(personPosition, data, name, trackingValid, i);
                }
				break;
			default:
                strcpy(name, "???");
				break;
		}
		
		if (mainOmnidFile) {
			_WriteData(mainOmnidFile, data, name, trackingValid, i);
		}
		
		if (data->RigidBodies[i].ID == WAND) {
			// if you want to measure latency of message, refer to how it was implemented in Sampleclient C++'s DataHandler function

			// update message to most recent frame of data
			if (trackingValid) {
				if (!initialized) {
					initialized = true;
				}

				rclcpp::Clock* clock_ptr = static_cast<rclcpp::Clock*>(clockPointer);
				optitrack_msg->header.stamp = clock_ptr->now();
				optitrack_msg->header.frame_id = "N/A";
				optitrack_msg->pose.position.x = data->RigidBodies[i].x;
				optitrack_msg->pose.position.y = data->RigidBodies[i].y;
				optitrack_msg->pose.position.z = data->RigidBodies[i].z;
				optitrack_msg->pose.orientation.x = data->RigidBodies[i].qx;
				optitrack_msg->pose.orientation.y = data->RigidBodies[i].qy;
				optitrack_msg->pose.orientation.z = data->RigidBodies[i].qz;
				optitrack_msg->pose.orientation.w = data->RigidBodies[i].qw;
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
    g_connectParams.localAddress = "192.168.20.134";
    
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
			g_serverDescription.NatNetVersion[2] <<"." << g_serverDescription.NatNetVersion[3] << std::endl;

		std::cout << "Client IP:" << g_connectParams.localAddress << std::endl;
		std::cout << "Server IP:" << g_connectParams.serverAddress << std::endl;
		std::cout << "Server Name:"<< g_serverDescription.szHostComputerName << std::endl << std::endl;
		/* copy pasted code ends here */

		mkdir("omnidCSVs", 0777);

		// open create csv files to write optitrack data inside
		std::string timestamp = std::to_string(std::time(nullptr));

		std::string mainOmnidPath = "omnidCSVs/MainOmnidFile_" + timestamp + ".csv";
		mainOmnidFile = fopen(mainOmnidPath.c_str(), "w");

		std::string omnid1Path = "omnidCSVs/Omnid1Position_" + timestamp + ".csv";
		omnid1Position = fopen(omnid1Path.c_str(), "w");

		std::string omnid2Path = "omnidCSVs/Omnid2Position_" + timestamp + ".csv";
		omnid2Position = fopen(omnid2Path.c_str(), "w");

		std::string omnid3Path = "omnidCSVs/Omnid3Position_" + timestamp + ".csv";
		omnid3Position = fopen(omnid3Path.c_str(), "w");

		std::string personPath = "omnidCSVs/PersonPosition_" + timestamp + ".csv";
		personPosition = fopen(personPath.c_str(), "w");

		std::string allBodyRefPath = "omnidCSVs/AllBodyRefOrientations_" + timestamp + ".csv";
		allBodyRefOrientations = fopen(allBodyRefPath.c_str(), "w");

		if (!mainOmnidFile || !omnid1Position || !omnid2Position || !omnid3Position || personPosition || allBodyRefOrientations) {
			printf("One or more CSV files could not be opened.  Exiting.\n");
			*ret = ErrorCode_InvalidOperation;
			return;
		}
		_WriteHeadingOnly(mainOmnidFile);
		_WriteHeadingOnly(omnid1Position);
		_WriteHeadingOnly(omnid2Position);
		_WriteHeadingOnly(omnid3Position);
		_WriteHeadingOnly(personPosition);
    }
}

void get_optitrack_data(NatNetClient* mClient) {
	if (!mClient) {
		std::cout << "Not Connected to Server!" << std::endl;
		return;
	}
	std::string message;
	// requesting Data Description
	sDataDescriptions* dataDescriptions = nullptr;
	int iResult = mClient->GetDataDescriptionList(&dataDescriptions);
	
	if (iResult != ErrorCode_OK || dataDescriptions == nullptr) {
	    message = "[new_optitrack] Unable to Retrieve Data Descriptions.";
	}
	else {
		int count = 0;
		// parse through each Data Description and extract relevant information
	    for (int i = 0; i < dataDescriptions->nDataDescriptions; i++) {
	    	if (dataDescriptions->arrDataDescriptions[i].type == Descriptor_RigidBody) {
				count++;
				sRigidBodyDescription* pRB = dataDescriptions->arrDataDescriptions[i].Data.RigidBodyDescription;
				message += pRB->szName;
				for (int j = 0; j < pRB->nMarkers; j++) {
					//message format: name, (x,y,z) positions of each marker corresponding to that rigid body
					message += "\t" + std::to_string(pRB->MarkerPositions[j][0]) + ", " + std::to_string(pRB->MarkerPositions[j][1]) + ", " + std::to_string(pRB->MarkerPositions[j][2]);
				}
				message += "\n";
	    	}
	    }
		// if there are no rigid bodies
		if (count == 0) {
			message = "No Rigid Bodies";
		}
	}
	
	if (dataDescriptions) {
		NatNet_FreeDescriptions(dataDescriptions);
	}
	if (allBodyRefOrientations) {
		fprintf(allBodyRefOrientations, "%s", message.c_str());
	}
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

void close_all_csvs() {
	if (omnid1Position) {
        fclose(omnid1Position);
        omnid1Position = nullptr;
    }

    if (omnid2Position) {
        fclose(omnid2Position);
        omnid2Position = nullptr;
    }

    if (omnid3Position) {
        fclose(omnid3Position);
        omnid3Position = nullptr;
    }

    if (personPosition) {
        fclose(personPosition);
        personPosition = nullptr;
    }

    if (mainOmnidFile) {
        fclose(mainOmnidFile);
        mainOmnidFile = nullptr;
    }

	if (allBodyRefOrientations) {
		fclose(allBodyRefOrientations);
		allBodyRefOrientations = nullptr;
	}
}

void _WriteHeadingOnly(FILE* fp) {
	if (fp) {
    	fprintf(fp, "Name\tMean Error\tValid?\tFrameID\tTimestamp (s)\tx (m)\ty (m)\tz (m)\tqx\tqy\tqz\tqw\tsoft\tsys\ttrans\tclient\n");
	}
}

void _WriteData(FILE* fp, sFrameOfMocapData* data, char* name, bool trackingValid, int i) {
	if (fp) {
		fprintf(fp, "%s\t%3.2f\t%d\t%d\t%3.2lf\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
		name,
		data->RigidBodies[i].MeanError,
		trackingValid,
		data->iFrame,
		data->fTimestamp,
		data->RigidBodies[i].x,
		data->RigidBodies[i].y,
		data->RigidBodies[i].z,
		data->RigidBodies[i].qx,
		data->RigidBodies[i].qy,
		data->RigidBodies[i].qz,
		data->RigidBodies[i].qw); //Format is Name, Mean Error, Valid?, FrameID, Timestamp, x, y, z, qx, qy, qz, qw
		fflush(fp);
	}
}
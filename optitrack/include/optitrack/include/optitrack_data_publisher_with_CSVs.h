/// @file
/// @brief parses data from OptiTrack and publishes data to topic
///
/// This code is to read the orientation of the wand controlling the Omnid's movmenets. 
/// On launch, this node initializes an OptiTrack Client and connects the client to the OptiTrack Motive server
/// with a Unicast connection. If the connection is successful, the client receives the data description, which
/// is the reference position orientation of the rigid bodies. Then the client receives the current position and orientation
/// of the rigid bodies detect in frame, and publishes this data onto a topic for the Omnids to read.
///
/// @section Publishers
/// 	(geometry_msgs/msg/PoseStamped) - A message containing the timestamp, xyz-position, and quaternion orientation of the wand
///
#pragma once

#include <memory>
#include <string>
#include <sys/stat.h>
#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "NatNetTypes.h"
#include "NatNetClient.h"

// global variables
std::shared_ptr<geometry_msgs::msg::PoseStamped> optitrack_msg;
bool initialized;
FILE* mainOmnidFile;
FILE* omnid1Position;
FILE* omnid2Position;
FILE* omnid3Position;
FILE* personPosition;
FILE* allBodyRefOrientations;
enum objectIDs;

/// @brief Handler for Rigid Body Position and Orientation data from OptiTrack
/// DataHandler receives a data packet and uses the information to calculate
/// the software, system, and total client latency. It then identifies the 
/// object by matching the received ID to the globally enumerated IDs.
/// Finally, it formats a message of the timestamp of the information, the Rigid Body 
/// Position and Orientation and publishes it to a topic.
/// @param data [in] Contains all information of the current frame OptiTrack cameras see
/// @param clockPointer [in] Contains clock of publisher node to get timestamp
/// @pre Global node and publisher must be initialized in main
/// @post Message is sent to topic
void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* clockPointer);

/// @brief Connects the Client Object to Optitrack's Motive Server
/// Link the DataHandler function to Client object to parse received data
/// @param ret [out] Either the client cannot connect to the server, it connects
/// but can't get server description, or it connects and retrieves server description successfully
/// @param mClient [out] motive client to be initialized
/// @param clock [in] pointer to the node's clock
/// @pre laptop running this code must be connected to the same network as
/// the computer that Motive is running on (should be NUMSR)
/// @pre g_connectParams.serverAddress is set to what is selected for
// Motive's Streaming Feature's Local Interface
/// @pre g_connectParams.localAddress is set to laptop's IP address
/// @pre since the version of Motive running in CRB is 3.0, we use NatNet SDK 4.0
/// @post prints out error message if failed or server info if succeeded
void initialize_optitrack(int* ret, NatNetClient** mClient, rclcpp::Clock::SharedPtr clock);

/// @brief Retrieves the reference position and orientation of rigid bodies in frame
/// Formats data into a message and prints it out
/// @param mClient [in] motive client used to send request to motive server
/// @post prints reference position and orientation of rigid bodies in frame
void get_optitrack_data(NatNetClient* mClient);

/// @brief Disconnects the server from the client
/// @param mClient [out] motive client to be deallocated
/// @post Global variable mClient is set to nullptr
void shutdown_optitrack(NatNetClient** mClient);

/// @brief Closes all csvs that is being written to
/// @post Global variables omnid1Position, omnid2Position, omnid3Position, personPosition, mainOmnidFile, allBodyRefOrientations are set to nullptr
void close_all_csvs();

/// @brief Writes file header
/// @param fp [in] Points to file whose header _WriteHeadingOnly writes
void _WriteHeadingOnly(FILE* fp);

/// @brief Writes data into file
/// @param fp [in] Points to file in which _WriteHData writes data
/// @param data [in] Contains all information of the current frame OptiTrack cameras see
/// @param name [in] Name of Rigid Body that's expected to be in camera frame
/// @param trackingValid [in] True if Rigid Body is detected in frame, false if Rigid Body is occluded
/// @param i [in] The ith Rigid Body in list of Rigid Bodies OptiTrack expects to detect in frame
void _WriteData(FILE* fp, sFrameOfMocapData* data, char* name, bool trackingValid, int i);
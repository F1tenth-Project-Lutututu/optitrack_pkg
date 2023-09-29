/*
    Copyright (c) 2017 Mobile Robots Laboratory at Poznan University of Technology:
    -Jan Wietrzykowski name.surname [at] put.poznan.pl

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/
// STD
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <termios.h>
#include <cstdio>
#include <iostream>
#include <memory>
#include <thread>
#include <ctime>

// boost
#include <boost/program_options.hpp>

// other
#include "NatNetLinux/NatNet.h"

#include "Mocap.hpp"




using namespace std;

Mocap::Mocap(std::string localAddressStr, std::string serverAddressStr) {
    localAddress = inet_addr( localAddressStr.c_str() );
    serverAddress = inet_addr( serverAddressStr.c_str() );
    
    init();
}

Mocap::Mocap(int argc, char **argv) {
    readOpts(argc, argv);
    
    init();
}

Mocap::~Mocap() {
    // Wait for threads to finish.
    frameListener->stop();
    commandListener->stop();
    frameListener->join();
    commandListener->join();

    // Epilogue
    close(sdData);
    close(sdCommand);
}

bool Mocap::getLatestPose(Eigen::Vector3d &retPos, Eigen::Quaterniond &retOrient) {
    MocapFrame mocapFrame(frameListener->pop(&lastMocapFrameValid).first);

    if (lastMocapFrameValid) {
        const std::vector<RigidBody> &rigidBodies = mocapFrame.rigidBodies();
        // assuming only 1 rigid body
	    //cout << rigidBodies.size() << endl;
        retPos.x() = rigidBodies.front().location().x;
        retPos.y() = rigidBodies.front().location().y;
        retPos.z() = rigidBodies.front().location().z;
        retOrient.x() = rigidBodies.front().orientation().qx;
        retOrient.y() = rigidBodies.front().orientation().qy;
        retOrient.z() = rigidBodies.front().orientation().qz;
        retOrient.w() = rigidBodies.front().orientation().qw;
    }

    return lastMocapFrameValid;
}

vectorPose Mocap::getLatestPoses() {
    MocapFrame mocapFrame(frameListener->pop(&lastMocapFrameValid).first);
    
    vectorPose ret;
    if (lastMocapFrameValid) {

//        cout << mocapFrame.cameraMidExposureTimestamp() << endl;

//        cout << mocapFrame.frameNum() << endl;

        const std::vector<RigidBody> &rigidBodies = mocapFrame.rigidBodies();

        for(size_t ir = 0; ir < rigidBodies.size(); ++ir){
            const RigidBody &rb = rigidBodies[ir];
            if(rb.trackingValid()) {
                Eigen::Vector3d t;
                Eigen::Quaterniond r;
                t.x() = rb.location().x;
                t.y() = rb.location().y;
                t.z() = rb.location().z;
                r.x() = rb.orientation().qx;
                r.y() = rb.orientation().qy;
                r.z() = rb.orientation().qz;
                r.w() = rb.orientation().qw;

                std::vector<Marker> markers;
                for(const LabeledMarker &labMark : mocapFrame.labeledMarkers()){
                    if(labMark.modelId() == rb.id()) {
//                        cout << labMark.markerId() << ": " << labMark.residual() << endl;
                        Marker curMark;
                        curMark.location(0) = labMark.location().x;
                        curMark.location(1) = labMark.location().y;
                        curMark.location(2) = labMark.location().z;
                        curMark.residual = labMark.residual();
                        curMark.occluded = labMark.occluded();
                        markers.push_back(curMark);
                    }
                }
    
                ret.emplace_back(rb.id(), t, r, mocapFrame.timestamp(), rb.meanError(), markers);
            }
        }
    }
    return ret;
}

void Mocap::readOpts(int argc, char **argv) {
    namespace po = boost::program_options;

    po::options_description desc("simple-example: demonstrates using NatNetLinux\nOptions");
    desc.add_options()
            ("help", "Display help message")
            ("local-addr,l", po::value<std::string>(), "Local IPv4 address")
            ("server-addr,s", po::value<std::string>(), "Server IPv4 address")
            ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc,argv,desc), vm);

    if(
            argc < 5 || vm.count("help") ||
            !vm.count("local-addr") ||
            !vm.count("server-addr")
            )
    {
        std::cout << desc << std::endl;
        exit(1);
    }

    localAddress = inet_addr( vm["local-addr"].as<std::string>().c_str() );
    serverAddress = inet_addr( vm["server-addr"].as<std::string>().c_str() );
}

void Mocap::init() {
    //NatNet
    lastMocapFrameValid = false;
    
    // Use this socket address to send commands to the server.
    struct sockaddr_in serverCommands = NatNet::createAddress(serverAddress, NatNet::commandPort);
    
    // Create sockets
    sdCommand = NatNet::createCommandSocket(localAddress);
    sdData = NatNet::createDataSocket(localAddress);
    
    // Start the CommandListener in a new thread.
    commandListener.reset(new CommandListener(sdCommand));
    commandListener->start();
    
    bool natNetVersionReveived = false;
    while(!natNetVersionReveived) {
        cout << "Sending ping to receive NetNet version" << endl;
        
        // Send a ping packet to the server so that it sends us the NatNet version
        // in its response to commandListener.
        NatNetPacket ping = NatNetPacket::pingPacket();
        ping.send(sdCommand, serverCommands);
    
        // wait 500 ms before sending another ping
        int waitTime = 500;
        // wait 50 ms between checks for response
        int checkTime = 50;
        
        int cnt = 0;
        // Wait here for ping response to give us the NatNet version.
        while(cnt < waitTime/checkTime && !natNetVersionReveived){
            this_thread::sleep_for(std::chrono::milliseconds(checkTime));
            natNetVersionReveived = commandListener->tryGetNatNetVersion(natNetMajor, natNetMinor);
            
            ++cnt;
        }
        
    }
    
    // Start up a FrameListener in a new thread.
    frameListener.reset(new FrameListener(sdData, natNetMajor, natNetMinor));
    frameListener->start();
}

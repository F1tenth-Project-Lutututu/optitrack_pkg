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

#ifndef RECORDING_OPTITRACK_MOCAP_HPP
#define RECORDING_OPTITRACK_MOCAP_HPP

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <memory>
#include <iostream>
#include <vector>

#include "NatNetLinux/CommandListener.h"
#include "NatNetLinux/FrameListener.h"

struct Marker{
    Marker() {}

    Marker(int id, const Eigen::Vector3d &location, double residual, bool occluded)
            : id(id), location(location), residual(residual), occluded(occluded) {}

    int id;
    Eigen::Vector3d location;
    double residual;
    bool occluded;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct Pose{
    Pose() {}
    
//    Pose(int id, const Eigen::Vector3d &t, const Eigen::Quaterniond &r) : id(id), t(t), r(r) {}
//
//    Pose(int id,
//         const Eigen::Vector3d &t,
//         const Eigen::Quaterniond &r,
//         uint64_t cameraMidExposureTimestamp)
//            : id(id), t(t), r(r), cameraMidExposureTimestamp(cameraMidExposureTimestamp)
//    {}

    Pose(int id,
         const Eigen::Vector3d &t,
         const Eigen::Quaterniond &r,
         double timestamp = 0,
         double meanError = 0,
         const std::vector<Marker> &markers = std::vector<Marker>())
            : id(id), t(t), r(r), timestamp(timestamp), meanError(meanError), markers(markers)
    {}


    int id;
    
    Eigen::Vector3d t;
    Eigen::Quaterniond r;
    
    double timestamp;

    double meanError;

    std::vector<Marker> markers;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef std::vector<Pose, Eigen::aligned_allocator<Pose> > vectorPose;

class Mocap {
public:
    Mocap(std::string localAddressStr, std::string serverAddressStr);
    
    Mocap(int argc, char* argv[]);

    ~Mocap();

    bool getLatestPose(Eigen::Vector3d& retPos,
                        Eigen::Quaterniond& retOrient);
    
    vectorPose getLatestPoses();
private:
    void readOpts(int argc, char* argv[]);

    void init();
    
    uint32_t localAddress;
    uint32_t serverAddress;

    // Version number of the NatNet protocol, as reported by the server.
    unsigned char natNetMajor;
    unsigned char natNetMinor;

    // Sockets
    int sdCommand;
    int sdData;

    std::shared_ptr<CommandListener> commandListener;
    std::shared_ptr<FrameListener> frameListener;

    bool lastMocapFrameValid;

//    MocapFrame lastMocapFrame;
};


#endif //RECORDING_OPTITRACK_MOCAP_HPP

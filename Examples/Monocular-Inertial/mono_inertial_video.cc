/**
 * This file is based on the GoPro ORB-SLAM3 Example
 *
 * Copyright (C) 2024 Luca Lach
 *
 * Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#include <chrono>
#include <fstream>
#include <iostream>

#include <opencv2/core/core.hpp>

#include <System.h>
#include <json.h>

using namespace std;
using json = nlohmann::json;

/*
Assumptions: 
data is of following format
{"samples": [s1, ..., sN]}

where si = [t, imu_frame_id, linear_accel, angular_vel]
t is scalar
imu_frame_id is string
linear_accel and angular_vel are 3-vectors

DATA IS ASSUMED TO BE ORDERED!!!
*/
bool readIMUData(
    const string &imuJsonPath,
    vector<double> &vTimeStamps,
    vector<cv::Point3f> &vAcc,
    vector<cv::Point3f> &vGyro)
    {
    std::ifstream ifs;

    cout << "reading IMU file ..." << endl;

    ifs.open(imuJsonPath.c_str());
    if (!ifs.is_open()) {
        cerr << "could not open file!" << endl;
        return false;
    }
    json j = json::parse(ifs);

    for (const auto s: j["samples"]){
        double t = (double) s[0];
        vTimeStamps.push_back(t);

        vAcc.emplace_back(cv::Point3f(
            (float) s[2][0],
            (float) s[2][1],
            (float) s[2][2]
        )); 

        vGyro.push_back(cv::Point3f(
            (float) s[3][0],
            (float) s[3][1],
            (float) s[3][2]
        ));
    }

    ifs.close();
    return true;
}

/*
Assumptions: 
data is of following format: {"samples": [t1, ..., tN]}, 
where ti are the video timestamps
*/
bool readVideoTimestamps(
    const string &videoStampsJsonPath,
    vector<double> &vTimeStamps)
    {
    std::ifstream ifs;

    cout << "reading video timestamps file ..." << endl;
    ifs.open(videoStampsJsonPath.c_str());
    if (!ifs.is_open()) {
        cerr << "could not open file!" << endl;
        return false;
    }
    json j = json::parse(ifs);

    for (const auto s: j["samples"]){
        vTimeStamps.push_back((double) s);
    }

    ifs.close();
    return true;
}

int main(int argc, char **argv) {
//   if (argc != 5) {
//     cerr << endl
//          << "Usage: ./mono_inertial_gopro_vi path_to_vocabulary path_to_settings path_to_video path_to_telemetry"
//          << endl;
//     return 1;
//   }

    string vocaPath("/home/slam/ORB3_SLAM/Vocabulary/ORBvoc.txt");
    string configPath("/home/slam/ORB3_SLAM/Examples/Monocular-Inertial/l515.yaml");
    string videoPath("/home/slam/ORB3_SLAM/data/video.avi");
    string videoStampsPath("/home/slam/ORB3_SLAM/data/video.json");
    string imuPath("/home/slam/ORB3_SLAM/data/imu.json");

    // read IMU data
    vector<double> imuTimestamps;
    vector<cv::Point3f> vAcc, vGyr;
    readIMUData(
        imuPath,
        imuTimestamps,
        vAcc,
        vGyr
    );
    cout << "got " << imuTimestamps.size() << " IMU measurements!" << endl;

    // read video timestamps
    vector<double> videoTimestamps;
    readVideoTimestamps(
        videoStampsPath,
        videoTimestamps
    );

    // read video
    cout << "opening video ..." << endl;
    cv::VideoCapture vid(videoPath);
    if (!vid.isOpened()) {
        std::cout << "Error opening video stream or file" << endl;
        return -1;
    }

    int nImages = vid.get(cv::CAP_PROP_FRAME_COUNT);
    double fps = vid.get(cv::CAP_PROP_FPS);
    double width = vid.get(cv::CAP_PROP_FRAME_WIDTH);
    double height = vid.get(cv::CAP_PROP_FRAME_HEIGHT);
    cout << "video has " << nImages << " frames at " << fps << " FPS → " << nImages/fps << "s" << endl;


    // initialize ORB3_SLAM
    cout << "initializing ORB3_SLAM ..." << endl;
    ORB_SLAM3::System SLAM(vocaPath, configPath, ORB_SLAM3::System::IMU_MONOCULAR, true);
    
    cout << "setup done!" << endl;

    int imuIdx = 0;

    vector<double> durations;
    vector<ORB_SLAM3::IMU::Point> IMUPoints;

    durations.reserve(nImages);
    IMUPoints.reserve(nImages);

    for (int i = 0; i < videoTimestamps.size(); i++){
        chrono::steady_clock::time_point start = chrono::steady_clock::now();

        // read frame from video
        cv::Mat im;
        vid.set(cv::CAP_PROP_POS_FRAMES, i);
        vid.read(im);

        // get timestamp corresponding to frame
        double vT = videoTimestamps[i];

        // store all IMU points between this frame and the one before 
        IMUPoints.clear();
        for (imuIdx; imuIdx < imuTimestamps.size(); imuIdx++){
            // shorthand for the index 
            int j = imuIdx;

            // break if we've reached the end or the next timestep will be after the frame timestep
            if (j == IMUPoints.size()-1 || imuTimestamps[j+1] > vT) break;
            
            // otherwise, append point
            IMUPoints.push_back(
                ORB_SLAM3::IMU::Point(
                    vAcc[j].x,
                    vAcc[j].y,
                    vAcc[j].z,
                    vGyr[j].x,
                    vGyr[j].y,
                    vGyr[j].z,
                    imuTimestamps[j]
                )
            );
        }
        imuIdx++; // breaking doesn't increase imuIdx

        // debug print to make sure times are correct
        // cout << fixed <<"frame #" << i << " at " << vT << " has " << IMUPoints.size() << " IMU measurements from " << IMUPoints.front() .t<< " to " << IMUPoints.back().t << endl;

        // track frame
        SLAM.TrackMonocular(im, vT, IMUPoints);

        durations.push_back(
            chrono::duration_cast<chrono::duration<double>>(
                chrono::steady_clock::now() - start
            ).count()
        );

        if (i % 10 == 0){
            cout << "ORB3_SLAM running at " << 1/durations.back() << "FPS\n";
        }
    }
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    // Tracking time statistics
    sort(videoTimestamps.begin(), videoTimestamps.end());
    float totaltime = 0;
    for (auto ni = 0; ni < videoTimestamps.size(); ni++) {
        totaltime += videoTimestamps[ni];
    }
    cout << "\n-------" << endl;
    cout << "median tracking time: " << videoTimestamps[nImages / 2] << endl;
    cout << "mean tracking time: " << totaltime / nImages << endl;
    cout << "-------" << endl << endl;

    cout << "all done!" << endl;
    return 0;
}
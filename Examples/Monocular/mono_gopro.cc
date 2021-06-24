/**
 * This file is part of ORB-SLAM3
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

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>

#include <opencv2/core/core.hpp>

#include <System.h>

using namespace std;

int main(int argc, char **argv) {
  if (argc != 4) {
    cerr << endl
         << "Usage: ./mono_tum path_to_vocabulary path_to_settings "
            "path_to_gopro_video"
         << endl;
    return 1;
  }

  // Retrieve paths to images
  vector<double> vTimestamps;
  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  cv::VideoCapture cap(argv[3]);
  // Check if camera opened successfully
  if (!cap.isOpened()) {
    std::cout << "Error opening video stream or file" << endl;
    return -1;
  }

  // Main loop

  int cnt_empty_frame = 0;
  int img_id = 0;
  int nImages = cap.get(cv::CAP_PROP_FRAME_COUNT);
  while (1) {
    cv::Mat im,im_track;
    bool success = cap.read(im);
    if (!success) {
      cnt_empty_frame++;
      std::cout<<"Empty frame...\n";
      if (cnt_empty_frame > 1000)
        break;
      continue;
    }
      im_track = im.clone();
      double tframe = cap.get(cv::CAP_PROP_POS_MSEC) * 1e-3;
      ++img_id;
      cv::resize(im_track, im_track, cv::Size(640, 360));

#ifdef COMPILEDWITHC11
      std::chrono::steady_clock::time_point t1 =
          std::chrono::steady_clock::now();
#else
      std::chrono::monotonic_clock::time_point t1 =
          std::chrono::monotonic_clock::now();
#endif

      // Pass the image to the SLAM system
      SLAM.TrackMonocular(im_track, tframe);

#ifdef COMPILEDWITHC11
      std::chrono::steady_clock::time_point t2 =
          std::chrono::steady_clock::now();
#else
      std::chrono::monotonic_clock::time_point t2 =
          std::chrono::monotonic_clock::now();
#endif

      double ttrack =
          std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
              .count();

//      vTimesTrack.push_back(ttrack);

//      // Wait to load the next frame
//      double T = 0;
//      if (img_id < nImages - 1)
//        T = vTimestamps[img_id + 1] - tframe;
//      else if (img_id > 0)
//        T = tframe - vTimestamps[img_id - 1];

//      if (ttrack < T)
//        usleep((T - ttrack) * 1e6);
      cv::waitKey(10);
  }

  //    // Stop all threads
  SLAM.Shutdown();

  // Tracking time statistics
  sort(vTimesTrack.begin(), vTimesTrack.end());
  float totaltime = 0;
  for (auto ni = 0; ni < vTimestamps.size(); ni++) {
    totaltime += vTimesTrack[ni];
  }
  cout << "-------" << endl << endl;
  cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
  cout << "mean tracking time: " << totaltime / nImages << endl;

  // Save camera trajectory
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  return 0;
}

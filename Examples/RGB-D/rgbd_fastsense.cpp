/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strPathRgb, const string &strPathDepth, const string &strPathTimes,
                vector<string> &vstrImageRgb, vector<string> &vstrImageDepth, vector<double> &vTimeStamps, int play_n);

int main(int argc, char **argv)
{
   if(argc != 7)
   {
      cerr << endl << "Usage: ./rgbd_fastsense path_to_vocabulary path_to_settings path_to_left_folder path_to_right_folder path_to_times_file play_n" << endl;
      return 1;
   }

   // Retrieve paths to images
   vector<string> vstrImageRgb;
   vector<string> vstrImageDepth;
   vector<double> vTimeStamp;
   LoadImages(string(argv[3]), string(argv[4]), string(argv[5]), vstrImageRgb, vstrImageDepth, vTimeStamp, stoi(string(argv[6])));

   if(vstrImageRgb.empty() || vstrImageDepth.empty())
   {
      cerr << "ERROR: No images in provided path." << endl;
      return 1;
   }

   if(vstrImageRgb.size()!=vstrImageDepth.size())
   {
      cerr << "ERROR: Different number of left and right images." << endl;
      return 1;
   }

   // Read rectification parameters
   cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
   if(!fsSettings.isOpened())
   {
      cerr << "ERROR: Wrong path to settings" << endl;
      return -1;
   }




   const int nImages = vstrImageRgb.size();

   // Create SLAM system. It initializes all system threads and gets ready to process frames.
   ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

   // Vector for tracking time statistics
   vector<float> vTimesTrack;
   vTimesTrack.resize(nImages);

   cout << endl << "-------" << endl;
   cout << "Start processing sequence ..." << endl;
   cout << "Images in the sequence: " << nImages << endl << endl;

   // Main loop
   cv::Mat imLeft, imRight, imLeftRect, imRightRect;
   for(int ni=0; ni<nImages; ni++)
   {
      // Read left and right images from file
      imLeft = cv::imread(vstrImageRgb[ni],CV_LOAD_IMAGE_UNCHANGED);
      imRight = cv::imread(vstrImageDepth[ni],CV_LOAD_IMAGE_UNCHANGED);

      if(imLeft.empty())
      {
         cerr << endl << "Failed to load image at: "
              << string(vstrImageRgb[ni]) << endl;
         return 1;
      }

      if(imRight.empty())
      {
         cerr << endl << "Failed to load image at: "
              << string(vstrImageDepth[ni]) << endl;
         return 1;
      }

//        cv::remap(imLeft,imLeftRect,M1l,M2l,cv::INTER_LINEAR);
//        cv::remap(imRight,imRightRect,M1r,M2r,cv::INTER_LINEAR);

      double tframe = vTimeStamp[ni];


#ifdef COMPILEDWITHC11
      std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
      std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
      cv::Mat dummy;
      // Pass the images to the SLAM system
//        SLAM.TrackStereo(imLeftRect,imRightRect,tframe, dummy, false, dummy);
      SLAM.TrackStereo(imLeft,imRight,tframe, dummy, false, dummy);

#ifdef COMPILEDWITHC11
      std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
      std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

      double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

      vTimesTrack[ni]=ttrack;

      // Wait to load the next frame
      double T=0;
      if(ni<nImages-1)
         T = vTimeStamp[ni+1]-tframe;
      else if(ni>0)
         T = tframe-vTimeStamp[ni-1];

      if(ttrack<T)
         usleep((T-ttrack)*1e6);
   }

   // Stop all threads
   SLAM.Shutdown();

   // Tracking time statistics
   sort(vTimesTrack.begin(),vTimesTrack.end());
   float totaltime = 0;
   for(int ni=0; ni<nImages; ni++)
   {
      totaltime+=vTimesTrack[ni];
   }
   cout << "-------" << endl << endl;
   cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
   cout << "mean tracking time: " << totaltime/nImages << endl;

   // Save camera trajectory
   SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");

//   while(1);

   return 0;
}

// play_n = 1 - play all images
// 2 - play every second image
// 3 - play every third image
void LoadImages(const string &strPathRgb, const string &strPathDepth, const string &strPathTimes,
                vector<string> &vstrImageRgb, vector<string> &vstrImageDepth, vector<double> &vTimeStamps, int play_n)
{
   ifstream fTimes;
   fTimes.open(strPathTimes.c_str());
   vTimeStamps.reserve(5000);
   vstrImageRgb.reserve(5000);
   vstrImageDepth.reserve(5000);

   int cnt = 1;

   while(!fTimes.eof())
   {
      string s;
      // read two lines from file
      getline(fTimes,s);
      getline(fTimes,s);
      if(!s.empty()) {
         if (cnt == play_n) {
            stringstream ss;
            ss << s;

            vstrImageRgb.push_back(strPathRgb + "/" + ss.str() + ".jpg");
            vstrImageDepth.push_back(strPathDepth + "/" + ss.str() + ".png");

            double t;
            ss >> t;
//            vTimeStamps.push_back(t / 1e9);
            vTimeStamps.push_back(t);
            cnt = 1;
         } else {
            cnt++;
         }
      }
   }
}

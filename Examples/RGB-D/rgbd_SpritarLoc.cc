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

#include <glog/logging.h>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include <System.h>
#include <boost/format.hpp>
#include <Config.hpp>

using namespace std;
cv::Mat PoseInv(cv::Mat Pose);

int main(int argc, char **argv)
{

    google::InitGoogleLogging("Sprirtar");
    google::SetStderrLogging(google::INFO);
#ifdef GBA_FRAME
    LOG(INFO)<<"-------------------GBA_FRAME-------------------";
#endif
    bool loadmap = false;
    bool keepMap = false;
    bool keepMapPoints = true;
    bool saveMap = true;
    bool fixInitFrame = true;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    string seq_name = "KinectCap_data_01";

    string mapSavePath = "/home/mxs/mxs/SLAMwork/ORB_SLAM2/Map";
    string mapSaveFile = mapSavePath+"/office_"+seq_name+".bin";
    string pointSaveFile = "/home/mxs/mxs/SLAMwork/ORB_SLAM2/Map/office_"+seq_name+"_Point.txt";

    string mapFile = mapSavePath+"/office_ALL.bin";
    string ImagePath = "/media/mxs/45a7155b-728d-4dad-a9b6-e1934286fb21/Dataset/SlamDataset/SpritarLoc/office/"+seq_name;

    int    start_index = 0;
    int    end_index = 8149;

    string vocFile = "/home/mxs/mxs/SLAMwork/ORB_SLAM2/SLAMConfigFiles/ORBvoc.txt";
    string yamlFile = "/home/mxs/mxs/SLAMwork/ORB_SLAM2/SLAMConfigFiles/my.yaml";

    system(std::string("rm -r "+ImagePath+"/pose").c_str());
    system(std::string("rm -r "+ImagePath+"/pose_GBA").c_str());
    system(std::string("rm -r "+ImagePath+"/points").c_str());
    system(std::string("rm -r "+ImagePath+"/points_ori").c_str());

    system(std::string("mkdir -p "+ImagePath+"/pose").c_str());
    system(std::string("mkdir -p "+ImagePath+"/pose_GBA").c_str());
    system(std::string("mkdir -p "+ImagePath+"/points").c_str());
    system(std::string("mkdir -p "+ImagePath+"/points_ori").c_str());

    system(std::string("mkdir -p "+mapSavePath).c_str());

    ORB_SLAM2::System SLAM(vocFile,yamlFile,mapFile,ORB_SLAM2::System::RGBD,true,loadmap,keepMap);


    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    // Main loop
    cv::Mat imRGB, imD;
    boost::format fmt ("%s/color/%06d.png");
    for ( int index = start_index; index < end_index; index ++ )
//    for ( int index = end_index-1; index >= start_index; index -- )
    {
        fmt = boost::format("%s/color/%06d.png");
        imRGB = cv::imread( (fmt%ImagePath.c_str()%index).str());
        std::cout<<(fmt%"[Image]:"%index).str()<<" .... ";

        fmt = boost::format("%s/depth/%06d.png");
        imD = cv::imread( (fmt%ImagePath.c_str()%index).str(), CV_LOAD_IMAGE_UNCHANGED);
//        std::cout<<(fmt%ImagePath.c_str()%index).str()<<std::endl;

        if(imRGB.empty() || imD.empty())
        {
            std::cout<<"Image is empty!\n";
            break;
        }

        cv::Mat tcw = SLAM.TrackRGBD( imRGB, imD, index  );
        if(tcw.empty())
            std::cout<<"LOST!"<<std::endl;
        else
            std::cout<<"OK!"<<std::endl;

    }
    std::cout<<"GlobalOptimize ... ";
//    std::vector<ORB_SLAM2::Frame*> res = SLAM.GlobalOptimize(100,keepMap,fixInitFrame);
    std::vector<ORB_SLAM2::Frame*> res = SLAM.GlobalOptimize(1000,keepMapPoints,fixInitFrame);
    std::cout<<"done!\n";

    std::cout<<"save pose ... ";
    for(int i=0; i<res.size(); i++)
    {
        int index = res[i]->mTimeStamp;
        fmt = boost::format("%s/pose_GBA/%06d.txt");
        std::ofstream of1( (fmt%ImagePath.c_str()%index).str(),std::ios::trunc | std::ios::out);
        of1<<cv::format(PoseInv(res[i]->mTcwGBA),cv::Formatter::FMT_CSV);
        of1.close();

        fmt = boost::format("%s/pose/%06d.txt");
        std::ofstream of2( (fmt%ImagePath.c_str()%index).str(),std::ios::trunc | std::ios::out);
        of2<<cv::format(PoseInv(res[i]->mTcw),cv::Formatter::FMT_CSV);
        of2.close();
    }
    std::cout<<"done!\n";

    std::cout<<"save points ... ";
    for(int i=0; i<res.size(); i++)
    {
        ORB_SLAM2::Frame* pF = res[i];
        int index = pF->mTimeStamp;

        fmt = boost::format("%s/points/%06d.txt");
        std::ofstream of1( (fmt%ImagePath.c_str()%index).str(),std::ios::trunc | std::ios::out);
         fmt = boost::format("%s/points_ori/%06d.txt");
        std::ofstream of2( (fmt%ImagePath.c_str()%index).str(),std::ios::trunc | std::ios::out);

        for(int p_i =0;p_i<pF->mvpMapPoints.size();p_i++)
        {
            ORB_SLAM2::MapPoint* mp =pF->mvpMapPoints[p_i];
            if(mp)
            {
                if(mp->isBad() || pF->mvbOutlier[i])
                    continue;
                of1<<mp->mnId<<std::endl;
                of2<<mp->mnId<<std::endl;
            }
        }
        of1.close();
        of2.close();

    }
    std::cout<<"done!\n";

    // Save camera trajectory
//    SLAM.SaveTrajectoryTUM("myCameraTrajectory1.txt");
//    SLAM.SaveKeyFrameTrajectoryTUM("myCameraTrajectory2.txt");

    // Save map
    if(saveMap)
    {
        std::cout<<"save map ... ";
        SLAM.SaveMap(mapSaveFile);
        SLAM.SaveMapPoints(pointSaveFile);
        std::cout<<"done!\n";
    }

    // Stop all threads
    SLAM.Shutdown();
    return 0;
}

// TCW --> TWC
cv::Mat PoseInv(cv::Mat Pose)
{
    cv::Mat camPos = cv::Mat::eye(4,4,CV_32FC1);
    cv::Mat mRcw = Pose.rowRange(0,3).colRange(0,3);
    cv::Mat mRwc = mRcw.t();
    cv::Mat mtcw = Pose.rowRange(0,3).col(3);
    cv::Mat mtwc = -mRcw.t()*mtcw;

    mRwc.copyTo(camPos.rowRange(0,3).colRange(0,3));
    mtwc.copyTo(camPos.rowRange(0,3).col(3));

    return camPos;
}

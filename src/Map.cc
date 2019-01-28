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

#include "stdafx.h"
#include "Map.h"

#include<mutex>

namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
}

template<class Archive>
void Map::save(Archive & ar, const unsigned int version)
{
    unique_lock<mutex> lock(mMutexMap);
#if defined(PRINT_TRACK_INFO)
	cout << "{INFO}Map save:" << endl;
	LOG("Map save:");
#endif

//    unsigned int test_data = TEST_DATA;
    int nItems = mspMapPoints.size();
    ar & nItems;
#if defined(PRINT_TRACK_INFO)
	cout << "{INFO}mspMapPoints size = " << nItems << endl;
	LOG("mspMapPoints size = %d", nItems);
#endif

    std::for_each(mspMapPoints.begin(), mspMapPoints.end(), [&ar](MapPoint* pMapPoint) {
        ar & *pMapPoint;
    });

    nItems = mspKeyFrames.size();
#if defined(PRINT_TRACK_INFO)
	cout << "{INFO}mspKeyFrames size = " << nItems << endl;
	LOG("mspKeyFrames size = %d", nItems);
#endif
    ar & nItems;
    std::for_each(mspKeyFrames.begin(), mspKeyFrames.end(), [&ar](KeyFrame* pKeyFrame) {
        ar & *pKeyFrame;
    });

    nItems = mvpKeyFrameOrigins.size();
#if defined(PRINT_TRACK_INFO)
	cout << "{INFO}mvpKeyFrameOrigins size = " << nItems << endl;
	LOG("mvpKeyFrameOrigins size = %d", nItems);
#endif
    ar & nItems;
    std::for_each(mvpKeyFrameOrigins.begin(), mvpKeyFrameOrigins.end(), [&ar](KeyFrame* pKeyFrameOrigin) {
        ar & *pKeyFrameOrigin;
    });
    // Pertaining to map drawing
    //nItems = mvpReferenceMapPoints.size();
    //cout << "$${INFO}mvpReferenceMapPoints size = %d " << nItems << endl;
    //ar & nItems;
    //std::for_each(mvpReferenceMapPoints.begin(), mvpReferenceMapPoints.end(), [&ar](MapPoint* pMapPointReference) {
    //    ar & *pMapPointReference;
    //});
    ar & const_cast<uint64_t &> (mnMaxKFid);
    ar & const_cast<uint64_t &>(mninit_id);
//    ar & test_data;
}

template<class Archive>
void Map::load(Archive & ar, const unsigned int version)
{
    unique_lock<mutex> lock(mMutexMap);
//    unsigned int test_data;

#if defined(PRINT_TRACK_INFO)
	cout << "{INFO}Map load:" << endl;
	LOG("Map load:");
#endif

    int nItems;
    ar & nItems;
#if defined(PRINT_TRACK_INFO)
	cout << "{INFO}mspMapPoints size = " << nItems << endl;
	LOG("mspMapPoints size = %d", nItems);
#endif

    for (int i = 0; i < nItems; ++i) {

        MapPoint* pMapPoint = new MapPoint();
        ar & *pMapPoint;
        mspMapPoints.insert(pMapPoint);
    }

    ar & nItems;
#if defined(PRINT_TRACK_INFO)
	cout << "{INFO}mspKeyFrames size = " << nItems << endl;
	LOG("mspKeyFrames size = %d", nItems);
#endif

    for (int i = 0; i < nItems; ++i) {

        KeyFrame* pKeyFrame = new KeyFrame;
        ar & *pKeyFrame;
        mspKeyFrames.insert(pKeyFrame);
    }


    ar & nItems;
#if defined(PRINT_TRACK_INFO)
	cout << "{INFO}mvpKeyFrameOrigins size = " << nItems << endl;
	LOG("mvpKeyFrameOrigins size = %d", nItems);
#endif

    for (int i = 0; i < nItems; ++i) {

        KeyFrame* pKeyFrame = new KeyFrame;
        ar & *pKeyFrame;
        /* TODO : VerifyHere*/
        mvpKeyFrameOrigins.push_back(*mspKeyFrames.begin());
    }

    ar & const_cast<uint64_t &> (mnMaxKFid);
    ar & const_cast<uint64_t &>(mninit_id);
//    ar & test_data;
#if defined(PRINT_TRACK_INFO)
	if (test_data == TEST_DATA)
	{
	    cout << ">>Map Loading Validated as True" << endl;
	    LOG("Map Loading Validated as True");
	}
	else
	{
	    cout << "ERROR Map Loading Validated as False: Got -" << test_data << " :( Check Load Save sequence" << endl;
	    LOG("Map Loading Validated as False: 0x%08x", test_data);
	}
#endif
}


// Explicit template instantiation
template void Map::save<portable_binary_oarchive>(portable_binary_oarchive &, const unsigned int);
template void Map::load<portable_binary_iarchive>(portable_binary_iarchive &, const unsigned int);

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

} //namespace ORB_SLAM

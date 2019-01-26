#ifndef STRUCTS_H
#define STRUCTS_H

#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "MapPoint.h"

namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;

struct MapPointStruct{
    std::pair<ushort, size_t> idx_mObservations[50];
    int idx_mpRefKF;
    int idx_mpReplaced;

    int n_mObservations;
};

struct KeyFrameStruct{
    ushort idx_mvpMapPoints[2500];
    std::pair<ushort, int> idx_mConnectedKeyFrameWeights[50];
    int idx_mpParent;
    ushort idx_mspChildrens[50];
    ushort idx_mspLoopEdges[50];

    int n_mvpMapPoints;
    int n_mConnectedKeyFrameWeights;
    int n_mspChildrens;
    int n_mspLoopEdges;
};

struct MapStruct{
    struct MapPointStruct MPSArray[5000];
    struct KeyFrameStruct KFSArray[200];
    ushort idx_RMPSArray[5000];
    ushort idx_KFOSArray[200];

    int n_MPS;
    int n_KFS;
    int n_RMPS;
    int n_KFOS;

    long unsigned int mnMaxKFid;
    int mnBigChangeIdx;
};

int PointKF2Index(KeyFrame* kf, vector<KeyFrame*> *pvpKFs);
int PointMP2Index(MapPoint* mp, vector<MapPoint*> *pvpMPs);
struct MapPointStruct GetMPStruct(MapPoint* pMP, vector<MapPoint*> *pvpMPs, vector<KeyFrame*> *pvpKFs);
struct KeyFrameStruct GetKFStruct(KeyFrame* pKF, vector<MapPoint*> *pvpMPs, vector<KeyFrame*> *pvpKFs);

}

#endif // STRUCTS_H


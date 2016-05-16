#ifndef VISION_OBSTACLEDETECTION_H_
#define VISION_OBSTACLEDETECTION_H_

#include <iostream>
#include <vector>
#include <algorithm>
#include <string.h>
#include <sstream>
#include <fstream>
#include "RobObsPose.h"

using namespace std;
using namespace RobObsPose;

class ObstacleDetection
{
public:
    ObstacleDetection();
    ~ObstacleDetection();
    int obsNum;
    vector<ObsPose> obsPoses;
    void ObstacleDetecting(const int obstacleMap[120][120], RobPose cRobotPos);
};

#endif

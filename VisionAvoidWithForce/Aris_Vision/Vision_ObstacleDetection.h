#ifndef VISION_OBSTACLEDETECTION_H_
#define VISION_OBSTACLEDETECTION_H_

#include <iostream>
#include <vector>
#include <algorithm>
#include <string.h>
#include <sstream>
#include <fstream>
#include "Vision_RobotPos.h"

using namespace std;

struct ObstaclePosition
{  
	double X;
	double Y;
	double radius;
};

class ObstacleDetection
{
public:
    ObstacleDetection();
    ~ObstacleDetection();
    int obsNum;
    float nextPosition[2];
    vector<ObstaclePosition> obsPoses;
    void ObstacleDetecting(const int obstacleMap[120][120]);
};

#endif

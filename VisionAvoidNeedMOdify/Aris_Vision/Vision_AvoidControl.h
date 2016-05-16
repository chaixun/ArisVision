#ifndef VISION_AVOIDCONTROL_H
#define VISION_AVOIDCONTROL_H

#include "Vision_ObstacleDetection.h"
#include "RobObsPose.h"
//#include "GaitMove.h"
#include <math.h>

using namespace RobObsPose;

struct SimpleWalkParam
{
    int stepNum = 1;
    double stepLength = 0.5;
    double walkDirection = M_PI;
};

class AvoidControl
{
public:
    SimpleWalkParam avoidWalkParam;
    AvoidControl(){}
    ~AvoidControl(){}
    RobPose nextRobotPos;
    void AvoidWalkControl(RobPose cTargetPos, RobPose cRobotPos, vector<ObsPose> cObstaclePoss);
};

#endif // VISION_AVOIDCONTROL_H

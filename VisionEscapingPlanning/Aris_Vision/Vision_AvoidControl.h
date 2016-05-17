#ifndef VISION_AVOIDCONTROL_H
#define VISION_AVOIDCONTROL_H

#include "Vision_ObstacleDetection.h"
#include "RobObsPose.h"
#include <math.h>

using namespace RobObsPose;

struct SimpleWalkParam
{
    int stepNum = 1;
    double stepLength = 0.5;
    double walkDirection = M_PI;
    double turnAngel = M_PI/6;
    double robHead = 90;
};

class AvoidControl
{
public:
    static ObsPose currentObs;
    static double WalkAngel;
    static bool rightTurn;
    SimpleWalkParam avoidWalkParam;
    AvoidControl(){}
    ~AvoidControl(){}
    RobPose nextRobotPos;
    void AvoidWalkControl(RobPose cRobotPos, vector<ObsPose> cObstaclePoss);
};

#endif // VISION_AVOIDCONTROL_H

#ifndef VISION_GAIT0_H
#define VISION_GAIT0_H

#include <Robot_Gait.h>

using namespace aris::dynamic;

enum robotMove
{
    nomove = 0,
    turn = 1,
    flatmove = 2,
    bodymove = 3,
    stepup = 4,
    stepdown = 5,
};

//struct VISION_WALK_PARAM :public Robots::GAIT_PARAM_BASE
//{
//    robotMove movetype = nomove;
//    int counter = 5000;
//    double turndata = 0;
//    double movedata[3] = {0, 0, 0};
//    double bodymovedata[3] = {0, 0, 0};
//    double stepupdata[6] = {0, 0, 0, 0, 0, 0};
//    double stepdowndata[6] = {0, 0, 0, 0, 0, 0};
//};

struct VISION_WALK_PARAM
{
    int count = 0;
    robotMove movetype = nomove;
    int totalCount = 5000;
    double turndata = 0;
    double movedata[3] = {0, 0, 0};
    double bodymovedata[3] = {0, 0, 0};
    double stepupdata[6] = {0, 0, 0, 0, 0, 0};
    double stepdowndata[6] = {0, 0, 0, 0, 0, 0};
};

int RobotVisionWalk(Robots::RobotBase &robot, const VISION_WALK_PARAM &param);

void RobotTurn(Robots::RobotBase &robot, const VISION_WALK_PARAM &pParam);

void RobotMove(Robots::RobotBase &robot, const VISION_WALK_PARAM &pParam);

void RobotBody(Robots::RobotBase &robot, const VISION_WALK_PARAM &pParam);

void RobotStepUp(Robots::RobotBase &robot, const VISION_WALK_PARAM &pParam);

void RobotStepDown(Robots::RobotBase &robot, const VISION_WALK_PARAM &pParam);

#endif // VISION_GAIT0_H

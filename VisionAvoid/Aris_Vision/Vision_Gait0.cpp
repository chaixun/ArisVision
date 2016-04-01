#include "Vision_Gait0.h"
#include <string.h>
#include <math.h>
#include <iostream>

#ifndef PI
#define PI 3.141592653589793
#endif

using namespace std;

int RobotVisionWalk(Robots::RobotBase &robot, const VISION_WALK_PARAM &param)
{
    Robots::WalkParam wk_param;

    switch(param.movetype)
    {
    case turn:
    {
        wk_param.alpha = 0;
        wk_param.beta = param.turndata*PI/180*2;
        wk_param.d = 0;
        wk_param.h = 0.05;
    }
        break;
    case flatmove:
    {
        if(param.movedata[0] != 0)
        {
            if(param.movedata[0] > 0)
            {
                wk_param.alpha = -PI/2;
                wk_param.d = param.movedata[0] * 2;
            }
            else
            {
                wk_param.alpha = PI/2;
                wk_param.d = -param.movedata[0] * 2;
            }
            wk_param.beta = 0;
            wk_param.h = 0.05;
        }
        else
        {
            wk_param.alpha = PI;
            wk_param.beta = 0;
            wk_param.d = param.movedata[2] * 2;
            wk_param.h = 0.05;
        }
    }
        break;
    default:
        break;
    }

    wk_param.n = 1;
    wk_param.count = param.count;
    wk_param.totalCount = param.totalCount;

    return Robots::walkGait(robot, wk_param);
}


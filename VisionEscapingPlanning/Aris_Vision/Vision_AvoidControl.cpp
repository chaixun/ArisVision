#include "Vision_AvoidControl.h"

ObsPose AvoidControl::currentObs = {-100, -100, -100, -100};
bool AvoidControl::rightTurn = true;
double AvoidControl::WalkAngel = M_PI/2;
bool AvoidControl::firstTurnLeft = true;

void AvoidControl::AvoidWalkControl(RobPose cRobotPos, vector<ObsPose> cObstaclePoses)
{
    memset(&avoidWalkParam, 0, sizeof(avoidWalkParam));

    int robWalkNum = 1;
    double robStepLength = 0.5;
    double robWalkAngel = M_PI;
    double robTurnAngel = 0;

    if(firstTurnLeft)
    {
        /*first turn left*/
        robWalkNum = 2;
        robStepLength = 0;
        robWalkAngel = 0;
        robTurnAngel = 20*M_PI/180;

        WalkAngel += (0.5 + robWalkNum - 1) * robTurnAngel;

        firstTurnLeft = false;
    }
    else
    {
        if(cObstaclePoses.size() == 0)
        {
            /*no obstacle move forward*/
            robWalkNum = 1;
            robStepLength = 0.5;
            robWalkAngel = M_PI/2 + M_PI/2;
            robTurnAngel = 0;
        }
        else
        {
            if(currentObs.x == cObstaclePoses.back().x&&currentObs.y == cObstaclePoses.back().y)
            {
                /*the same obstalce move forward*/
                robWalkNum = 1;
                robStepLength = 0.5;
                robWalkAngel = M_PI/2 + M_PI/2;
                robTurnAngel = 0;
            }
            else
            {
                /*not the same obstacle turn right or left 90*/
                currentObs = cObstaclePoses.back();
                robWalkNum = 2;
                if(rightTurn)
                {
                    /*turn right*/
                    robStepLength = 0;
                    robWalkAngel = 0;
                    robTurnAngel = -40*M_PI/180;
                    rightTurn = !rightTurn;

                    lObsPoses.push_back(currentObs);
                }
                else
                {
                    /*turn left*/
                    robStepLength = 0;
                    robWalkAngel = 0;
                    robTurnAngel = 40*M_PI/180;
                    rightTurn = !rightTurn;

                    rObsPoses.push_back(currentObs);
                }
                WalkAngel += (0.5 + robWalkNum - 1) * robTurnAngel;
            }
        }
    }
    avoidWalkParam.stepNum = robWalkNum;
    avoidWalkParam.stepLength = robStepLength;
    avoidWalkParam.walkDirection = robWalkAngel;
    avoidWalkParam.turnAngel = robTurnAngel;
    avoidWalkParam.robHead = WalkAngel/M_PI*180;

    RobPose tempRobotPos = cRobotPos;

    double robWalkVector[2] = {1*cos(WalkAngel), 1*sin(WalkAngel)};

    for(int i = 0; i < robWalkNum; i++)
    {
        if(i == 0)
        {
            tempRobotPos.x = tempRobotPos.x + 0.5*robStepLength*robWalkVector[0];
            tempRobotPos.y = tempRobotPos.y + 0.5*robStepLength*robWalkVector[1];
        }
        else
        {
            tempRobotPos.x = tempRobotPos.x + robStepLength*robWalkVector[0];
            tempRobotPos.y = tempRobotPos.y + robStepLength*robWalkVector[1];
        }
    }

    tempRobotPos.gama = WalkAngel - M_PI/2;

    nextRobotPos = tempRobotPos;
}

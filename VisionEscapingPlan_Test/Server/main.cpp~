#include <iostream>
#include <cstring>
#include <iomanip>
#include <bitset>
#include <map>
#include <string>
#include <vector>

using namespace std;

#include "GaitMove.h"
#include "EscapingPlanner.h"

#include "unistd.h"

using namespace RobObsPose;

int main(int argc, char *argv[])
{
    double realLeftObs[4][3] = {{-1, 3, 0.5}, {-1, 6, 0.25}, {-1, 8, 0.25}, {-1, 10, 0.25}};
    double realRightObs[3][3] = {{2, 3, 0.25}, {2, 6, 0.25}, {2, 9, 0.5}};

    robPoses.clear();

    RobPose startPose = {-1, 0, 0, 0, 0, 0};
    RobPose targetPose = {0, 11, 0, 0, 0, 0};

    robPoses.push_back(startPose);
    robPoses.push_back(targetPose);

    for(int i = 0; i < 4; i++)
    {
        ObsPose tempObs;
        tempObs.x = realLeftObs[i][0];
        tempObs.y = realLeftObs[i][1];
        tempObs.r = realLeftObs[i][2];
        lObsPoses.push_back(tempObs);
    }

    for(int i = 0; i < 3; i++)
    {
        ObsPose tempObs;
        tempObs.x = realRightObs[i][0];
        tempObs.y = realRightObs[i][1];
        tempObs.r = realRightObs[i][2];
        rObsPoses.push_back(tempObs);
    }

    EscapingPlanner escapingPlanner;
    escapingPlanner.GenEscapPath();

    for(int i = 0; i < 6000; i++)
    {
        double bodyPose[6], feetPosi[18];
        escapingPlanner.OutBodyandFeetTraj(bodyPose, feetPosi, i);
        robPoses.push_back(bodyPose);
        feetPoses.push_back(feetPosi);
        cout<<feetPosi[9]<<" "<<feetPosi[10]<<" "<<feetPosi[11]<<endl;
    }

    char a;
    cin>>a;
}

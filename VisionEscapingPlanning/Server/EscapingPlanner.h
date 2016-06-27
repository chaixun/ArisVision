#ifndef ESCAPINGPLANNER_H
#define ESCAPINGPLANNER_H

#include <math.h>
#include <iostream>
#include "Spline.h"
#include "RobObsPose.h"
#include "Eigen/Dense"

using namespace std;

using namespace Eigen;

using namespace RobObsPose;

class EscapingPlanner
{
public:
    struct Point2D
    {
        double x;
        double y;
    };

    enum PLANNER_STATE
    {
        NOTSTART = 1,
        GENPATHFINISHED = 2,
        GENBODYANDFOOTFINISHED = 3,
        GAITSTART = 4,
        PATHFOLLOWINGFINISHED = 5,
    };

    EscapingPlanner();
    ~EscapingPlanner();

    void PlannerStart(int timeNow);
    void GenEscapPath();
    void GenBodyandFeetPose();
    void OutBodyandFeetTraj(double bodyPose[6], double feetPosi[18], int timeNow);
    PLANNER_STATE GetPlannerState() const {return plannerState;}

private:
    PLANNER_STATE plannerState = NOTSTART;
    tk::spline splinePath;
    vector<Point2D> midPoints;
    vector<ObsPose> traLObsPoses;
    vector<ObsPose> traRObsPoses;
    double halfStep = 0.18;
    double difXTraj = 0.00001;
    vector<RobPose> bodyPoses;

    double bodyalpha = 0;
    double bodyX = 0;
    double bodyY = 0;

    vector<FootHold> feetPoses;
    vector<double> curveX;
    vector<double> curveY;
    int halfStepT = 2250;
    int timeStart = 0;
    int timeLast;
    bool leftSwing = false;

    void GetMidPoint(ObsPose lObsPose, ObsPose rObsPose, Point2D &midPoint);
    void SelMidPoint();
    void OutBodyPose();
    void OutFeetPosi();
    void OutFeetTraj(double feetTrajPosi[18], int timeCount, int numCycle);
};

#endif // ESCAPINGPLANNER_H

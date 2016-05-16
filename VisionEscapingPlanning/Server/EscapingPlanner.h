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
    int OutBodyandFeetTraj(double bodyPose[6], double feetPosi[18], int timeNow);
    PLANNER_STATE GetPlannerState() const {return plannerState;}

private:
    PLANNER_STATE plannerState = NOTSTART;
    tk::spline splinePath;
    vector<Point2D> midPoints;
    vector<ObsPose> traLObsPoses;
    vector<ObsPose> traRObsPoses;
    double halfStep = 0.3;
    double difXTraj = 0.045;
    vector<RobPose> bodyPoses;

    vector<FootHold> feetPoses;
    vector<double> curveX;
    vector<double> curveY;
    int halfStepT = 3000;
    int timeStart;
    int timeLast;
    bool leftSwing = false;

    void GetMidPoint(ObsPose lObsPose, ObsPose rObsPose, Point2D &midPoint);
    void SelMidPoint();
    void OutBodyPose();
    void OutFeetPosi();
    void OutFeetTraj(FootHold feetHold1, FootHold feetHold2, double feetTrajPosi[18], double timeCount);
};

#endif // ESCAPINGPLANNER_H

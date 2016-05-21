#include "EscapingPlanner.h"

double acc_even(int n, int i)
{
    return 1.0 / n / n  * i * i;
}
double dec_even(int n, int i)
{
    return 1.0 - 1.0 / n / n * (n - i)*(n - i);
}
double even(int n, int i)
{
    return 1.0 / n*i;
}

EscapingPlanner::EscapingPlanner()
{
    RobPose startBodyPose = {0, 0, 0, 0, 0, 0};
    bodyPoses.push_back(startBodyPose);
    //    FootHold startFoothold = {-0.3, 0.65, -0.45, 0, -0.3, -0.65, 0.3, 0.65, 0.45, 0, 0.3, -0.65};
    FootHold startFoothold = {0.65, 0.3, 0, 0.45, -0.65, 0.3, 0.65, -0.3, 0, -0.45, -0.65, -0.3};
    feetPoses.push_back(startFoothold);
    timeStart = 0;
}

EscapingPlanner::~EscapingPlanner()
{

}

void EscapingPlanner::PlannerStart(int timeNow)
{
    if(plannerState == GENBODYANDFOOTFINISHED)
    {
        timeStart = timeNow;
        plannerState = GAITSTART;
    }
}
void EscapingPlanner::GetMidPoint(ObsPose lObsPose, ObsPose rObsPose, Point2D &midPoint)
{
    double kLine = (rObsPose.y - lObsPose.y) / (rObsPose.x - lObsPose.x);
    double thetaLine = atan(kLine);
    double x1 = lObsPose.x + lObsPose.r * cos(thetaLine);
    double y1 = lObsPose.y + kLine * lObsPose.r * cos(thetaLine);
    double x2 = rObsPose.x - rObsPose.r * cos(thetaLine);
    double y2 = rObsPose.y - kLine * rObsPose.r * cos(thetaLine);

    midPoint.x = (x1 + x2) / 2;
    midPoint.y = (y1 + y2) / 2;
}

void EscapingPlanner::SelMidPoint()
{
    int lObsNum = lObsPoses.size();
    int rObsNum = rObsPoses.size();

    int pairObsNum = min(lObsNum, rObsNum);

    for (int i = 0; i < pairObsNum; i++)
    {
        Point2D tempMidpoint;
        GetMidPoint(lObsPoses[i], rObsPoses[i], tempMidpoint);
        midPoints.push_back(tempMidpoint);
        midPoints.push_back(tempMidpoint);
    }

    for (int i = 0; i < pairObsNum - 1; i++)
    {
        Point2D point1, point2;
        GetMidPoint(lObsPoses[i + 1], rObsPoses[i], point1);
        GetMidPoint(lObsPoses[i], rObsPoses[i + 1], point2);
        double dist1 = sqrt(pow((point1.x - midPoints[i * 2].x), 2) + pow((point1.y - midPoints[i * 2].y), 2))
                + sqrt(pow((point1.x - midPoints[(i + 1)* 2].x), 2) + pow((point1.y - midPoints[(i + 1) * 2 ].y), 2));
        double dist2 = sqrt(pow((point2.x - midPoints[i * 2 ].x), 2) + pow((point2.y - midPoints[i * 2].y), 2))
                + sqrt(pow((point2.x - midPoints[(i + 1) * 2].x), 2) + pow((point2.y - midPoints[(i + 1) * 2].y), 2));
        if (dist1 >= dist2)
        {
            midPoints[i * 2 + 1] = point2;
        }
        else
        {
            midPoints[i * 2 + 1] = point1;
        }
    }

    if (lObsNum - rObsNum != 0)
    {
        Point2D tempPoint;
        GetMidPoint(lObsPoses[lObsNum - 1], rObsPoses[rObsNum - 1], tempPoint);
        midPoints[2 * pairObsNum - 1] = tempPoint;
    }
    else
    {
        midPoints.pop_back();
    }
            for (unsigned int i = 0; i < midPoints.size(); i++)
            {
                cout<<midPoints[i].x<<" "<<midPoints[i].y<<endl;
            }
}

void EscapingPlanner::GenEscapPath()
{
    //reverse target to start
    double theta = -M_PI/2 + robPoses.back().gama;
    double deltaX = robPoses.back().x;
    double deltaY = robPoses.back().y;

    SelMidPoint();

    curveX.push_back(robPoses.front().x);
    curveY.push_back(robPoses.front().y);

    for (unsigned int i = 0; i < midPoints.size(); i++)
    {
        Point2D tempPoint = midPoints[i];
        curveX.push_back(tempPoint.x);
        curveY.push_back(tempPoint.y);
    }

    curveX.push_back(robPoses.back().x);
    curveY.push_back(robPoses.back().y);

    reverse(curveX.begin(), curveX.end());
    reverse(curveY.begin(), curveY.end());

    for (unsigned int i = 0; i < curveX.size(); i++)
    {
        double tempX = cos(theta) * curveX[i] + sin(theta) * curveY[i] + (-deltaX*cos(theta) - deltaY*sin(theta));
        double tempY = -sin(theta) * curveX[i] + cos(theta) * curveY[i] + (deltaX*sin(theta) - deltaY*cos(theta));
        curveX[i] = tempX;
        curveY[i] = tempY;

        //cout<<cos(theta)<<" "<<sin(theta)<<" "<<(-deltaX*cos(theta) - deltaY*sin(theta))<<endl;
        //cout<<-sin(theta)<<" "<<cos(theta)<<" "<<(deltaX*sin(theta) - deltaY*cos(theta))<<endl;
        //cout<<curveX[i]<<" "<<curveY[i]<<endl;
    }

   // start slope 0, target slope 0 or -robPoses.back().gama

    splinePath.set_boundary(tk::spline::bd_type::first_deriv, 0, tk::spline::bd_type::first_deriv, -robPoses.back().gama, false);
    splinePath.set_points(curveX, curveY);
    GenBodyandFeetPose();
    //    cout<<splinePath(2)<<endl;
    //    cout<<splinePath.getSlope(2)<<endl;
}

void EscapingPlanner::GenBodyandFeetPose()
{
    OutBodyPose();
    OutFeetPosi();
    plannerState = GENBODYANDFOOTFINISHED;
}

void EscapingPlanner::OutBodyPose()
{
    double curveLength = 0;

    for (double x = curveX.front(); x <= curveX.back();  x = x + difXTraj)
    {
        double difYTraj = splinePath(x + difXTraj) - splinePath(x);

        curveLength += sqrt(pow(difXTraj,2) + pow(difYTraj, 2));

        if (bodyPoses.size() == 1 && curveLength > 0.5*(halfStep - difXTraj))
        {
            RobPose tempBodyPose;
            tempBodyPose.x = x;
            tempBodyPose.y = splinePath(x);
            tempBodyPose.alpha = splinePath.getSlope(x);
            bodyPoses.push_back(tempBodyPose);
            curveLength = 0;
        }

        if (bodyPoses.size() > 1 && curveLength > (halfStep - difXTraj))
        {
            RobPose tempBodyPose;
            tempBodyPose.x = x;
            tempBodyPose.y = splinePath(x);
            tempBodyPose.alpha = splinePath.getSlope(x);
            bodyPoses.push_back(tempBodyPose);
            curveLength = 0;
        }
    }

    RobPose targetBodyPose;
    targetBodyPose.x = curveX.back();
    targetBodyPose.y = curveY.back();
    targetBodyPose.alpha = splinePath.getSlope(curveX.back());
    bodyPoses.push_back(targetBodyPose);

    //    for(int i = 0; i < bodyPoses.size(); i++)
    //    {
    //        cout<<bodyPoses[i].x<<" "<<bodyPoses[i].y<<" "<<bodyPoses[i].alpha<<endl;
    //    }
}

void EscapingPlanner::OutFeetPosi()
{
    MatrixXd nextLF(3, 3), nextRF(3, 3);

    for (unsigned int i = 1; i < bodyPoses.size(); i++)
    {
        MatrixXd LF(3, 3), RF(3, 3);
        LF << -0.3, -0.3, 0.45,
                0.65, -0.65, 0,
                1, 1, 1;
        RF <<  -0.45, 0.3, 0.3,
                0, -0.65, 0.65,
                1, 1, 1;

        double theta = -M_PI / 2 + atan(bodyPoses[i].alpha);
        double xRobPos = bodyPoses[i].x;
        double yRobPos = bodyPoses[i].y;

        Matrix3d transT;
        transT << cos(theta), -sin(theta), xRobPos,
                sin(theta), cos(theta), yRobPos,
                0, 0, 1;

        FootHold tempFootHold = feetPoses.back();

        if(leftSwing)
        {
            LF.row(1) += MatrixXd::Ones(1,3) * halfStep / 2;
            nextLF = transT * LF;
            RF.row(1) -= MatrixXd::Ones(1,3) * halfStep / 2;
            nextRF = transT * RF;
            leftSwing = !leftSwing;

            tempFootHold.feetHold[0] = nextLF(0, 0);
            tempFootHold.feetHold[1] = nextLF(1, 0);
            tempFootHold.feetHold[4] = nextLF(0, 1);
            tempFootHold.feetHold[5] = nextLF(1, 1);
            tempFootHold.feetHold[8] = nextLF(0, 2);
            tempFootHold.feetHold[9] = nextLF(1, 2);
        }
        else
        {
            LF.row(1) -= MatrixXd::Ones(1,3) * halfStep / 2;
            nextLF = transT * LF;
            RF.row(1) += MatrixXd::Ones(1,3) * halfStep / 2;
            nextRF = transT * RF;
            leftSwing = !leftSwing;

            tempFootHold.feetHold[2] = nextRF(0, 0);
            tempFootHold.feetHold[3] = nextRF(1, 0);
            tempFootHold.feetHold[6] = nextRF(0, 2);
            tempFootHold.feetHold[7] = nextRF(1, 2);
            tempFootHold.feetHold[10] = nextRF(0, 1);
            tempFootHold.feetHold[11] = nextRF(1, 1);
        }
        feetPoses.push_back(tempFootHold);
        //                cout<<feetPoses[i].feetHold[0]<<" "<<feetPoses[i].feetHold[1]
        //                                             <<" "<<feetPoses[i].feetHold[6]<<" "<<feetPoses[i].feetHold[7]<<" "<<endl;
    }
}

void EscapingPlanner::OutFeetTraj(FootHold feetHold1, FootHold feetHold2, double feetTrajPosi[18], double timeCount)
{

    const double s = -(M_PI / 2)*cos(M_PI * (timeCount + 1) / halfStepT) + M_PI / 2;
    //    const double s = M_PI * timeCount / halfStepT;
    //    cout<<s<<endl;
    //    cout<<feetHold1.feetHold[0]<<" "<<feetHold2.feetHold[0]<<" LF"<<endl;
    //cout<<feetHold1.feetHold[6]<<" "<<feetHold2.feetHold[6]<<" RF-1"<<endl;
    //    cout<<feetHold1.feetHold[7]<<" "<<feetHold2.feetHold[7]<<" RF-2"<<endl;

    for(int i = 0; i < 6; i++)
    {
        Point2D foot1 = {feetHold1.feetHold[i * 2], feetHold1.feetHold[i * 2 + 1]};
        Point2D foot2 = {feetHold2.feetHold[i * 2], feetHold2.feetHold[i * 2 + 1]};

        if(foot1.x != foot2.x)
        {
            Point2D difFoot = {foot2.x - foot1.x, foot2.y - foot1.y};
            double ellipL = sqrt(difFoot.x * difFoot.x + difFoot.y * difFoot.y);
            double ellipH = 0.04;
            double theta = atan2(difFoot.y, difFoot.x);
            double x = foot1.x + (ellipL / 2 - ellipL / 2 * cos(s)) * cos(theta);
            double y = foot1.y + (ellipL / 2 - ellipL / 2 * cos(s)) * sin(theta);
            double z = ellipH * sin(s);
            feetTrajPosi[i * 3 + 0] = x;
            feetTrajPosi[i * 3 + 1] = y;
            feetTrajPosi[i * 3 + 2] = z;
        }
        else
        {
            double x = foot1.x;
            double y = foot1.y;
            double z = 0;
            feetTrajPosi[i * 3 + 0] = x;
            feetTrajPosi[i * 3 + 1] = y;
            feetTrajPosi[i * 3 + 2] = z;
        }
    }
    //cout<<feetTrajPosi[0]<<" "<<feetTrajPosi[1]<<" "<<feetTrajPosi[2]<<endl;
    //cout<<feetTrajPosi[9]<<" "<<feetTrajPosi[10]<<" "<<feetTrajPosi[11]<<endl;
}

int EscapingPlanner::OutBodyandFeetTraj(double bodyPose[6], double feetPosi[18], int timeNow)
{
    if(plannerState == GAITSTART)
    {
        double cbodyPose[6] = {0};
        double cFeetPosi[18] = {0};

        int iInCycle = (timeNow - timeStart) % halfStepT;
        int numCycle = (timeNow - timeStart) / halfStepT;

        if(timeNow - timeStart <= halfStepT)
        {
            bodyalpha0 = splinePath.getSlope(bodyX);
            bodyX += double(iInCycle/halfStepT) * bodySpeed * cos(atan(bodyalpha0)) * 0.001;
            bodyY += double(iInCycle/halfStepT) * bodySpeed * sin(atan(bodyalpha0)) * 0.001;
            bodyalpha1 = splinePath.getSlope(bodyX);

            cbodyPose[0] = bodyX;
            cbodyPose[1] = bodyY;
            cbodyPose[3] = bodyalpha1;
            OutFeetTraj(feetPoses[numCycle], feetPoses[numCycle + 1], cFeetPosi, iInCycle);
        }
        else if(timeNow - timeStart <= (bodyPoses.size() - 2) * halfStepT)
        {
            bodyalpha0 = splinePath.getSlope(bodyX);
            bodyX += bodySpeed * cos(atan(bodyalpha0)) * 0.001;
            bodyY += bodySpeed * sin(atan(bodyalpha0)) * 0.001;
            bodyalpha1 = splinePath.getSlope(bodyX);

            cbodyPose[0] = bodyX;
            cbodyPose[1] = bodyY;
            cbodyPose[3] = bodyalpha1;
            OutFeetTraj(feetPoses[numCycle], feetPoses[numCycle + 1], cFeetPosi, iInCycle);
        }
        else if(timeNow - timeStart <= (bodyPoses.size() - 1) * halfStepT)
        {
            bodyalpha0 = splinePath.getSlope(bodyX);
            bodyX += (1 - iInCycle / halfStepT) * bodySpeed * cos(atan(bodyalpha0)) * 0.001;
            bodyY += (1 - iInCycle / halfStepT) * bodySpeed * sin(atan(bodyalpha0)) * 0.001;
            bodyalpha1 = splinePath.getSlope(bodyX);

            cbodyPose[0] = bodyX;
            cbodyPose[1] = bodyY;
            cbodyPose[3] = bodyalpha1;
            OutFeetTraj(feetPoses[numCycle], feetPoses[numCycle + 1], cFeetPosi, iInCycle);
            // cout<<"ending "<<endl;
        }

        for(int i = 0; i < 6; i++)
        {
            feetPosi[i * 3] = -cFeetPosi[i * 3 + 1];
            feetPosi[i * 3 + 1] = cFeetPosi[i * 3 + 2] - 0.85;
            feetPosi[i * 3 + 2] = -cFeetPosi[i * 3];
        }

        bodyPose[0] = -cbodyPose[1];
        bodyPose[1] = cbodyPose[2];
        bodyPose[2] = -cbodyPose[0];
        bodyPose[3] = M_PI / 2;
        bodyPose[4] = cbodyPose[3];
        bodyPose[5] = -M_PI / 2;

        if(timeNow - timeStart == (bodyPoses.size() - 1) * halfStepT)
        {
            plannerState = PATHFOLLOWINGFINISHED;
        }
    }
}

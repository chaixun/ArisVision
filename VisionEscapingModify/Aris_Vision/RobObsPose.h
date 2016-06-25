#ifndef ROBOBSPOSE_H
#define ROBOBSPOSE_H

#include <vector>

using namespace std;

namespace RobObsPose
{

struct RobPose
{
    double x;
    double y;
    double z;
    double alpha;
    double beta;
    double gama;
};

struct FootHold
{
    double feetHold[18];
};

struct ObsPose
{
    double x;
    double y;
    double h;
    double r;
};

extern vector<RobPose> robPoses;
extern vector<ObsPose> lObsPoses;
extern vector<ObsPose> rObsPoses;
}
#endif // ROBOBSPOSE_H

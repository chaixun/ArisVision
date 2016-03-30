#ifndef ARIS_VISION_H_
#define ARIS_VISION_H_

#define linux 1

#include "Aris_Sensor.h"
#include <memory>
#include <stdlib.h>
#include <iostream>
#include <cstring>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

namespace Aris
{

namespace Sensor
{

struct GridMap
{
    bool isStepOK;
    float X, Y, Z;
    float pointNum;
    float normalVector;
    float planePara[4];
    float planeDegree;
};

struct VISION_DATA
{
    unsigned long long timeStamp;
    unsigned short depthMap[480*640];
    float pointCloud[480][640][3];
    GridMap pGridMap[120][120];
    float gridMap[120][120];
};

class KINECT_BASE: public SENSOR_BASE<VISION_DATA>
{
public:
    KINECT_BASE();
    virtual ~KINECT_BASE();

protected:
    virtual void Initiate();
    virtual void UpdateData(VISION_DATA &data);
    virtual void Release();

private:
    class KINECT_BASE_STRUCT;
    std::auto_ptr<KINECT_BASE_STRUCT> mKinectStruct;
};

class KINECT: public KINECT_BASE
{
public:
    KINECT();
    ~KINECT();
private:
    virtual void UpdateData(VISION_DATA &data);
};

}
}

#endif // ARIS_VISION_H_

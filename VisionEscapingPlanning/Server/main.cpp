#include <iostream>
#include <cstring>
#include <iomanip>
#include <bitset>
#include <map>
#include <string>
#include <vector>

using namespace std;

#include <aris.h>
#include <Robot_Gait.h>
#include <Robot_Type_I.h>
#include "AvoidMove.h"

#include "GaitMove.h"

#include "rtdk.h"
#include "unistd.h"

using namespace aris::core;
using namespace RobObsPose;


int main(int argc, char *argv[])
{   
    VisionAvoid::VisionAvoidWrapper::KinectStart();

//    robPoses.clear();

//        RobPose startPose = {-1, 0, 0, 0, 0, 0};
//        RobPose targetPose = {0, 11, 0, 0, 0, 0};

//        double realLeftObs[4][3] = {{-1, 3, 0.5}, {-1, 6, 0.25}, {-1, 8, 0.25}, {-1, 10, 0.25}};
//        double realRightObs[3][3] = {{2, 3, 0.25}, {2, 6, 0.25}, {2, 9, 0.5}};

//        RobPose startPose = {0, 0, 0, 0, 0, 0};
//        RobPose targetPose = {0, 11, 0, 0, 0, 0};

//        double realLeftObs[4][3] = {{-1, 3, 0.25}, {-1, 6, 0.25}, {-1, 8, 0.25}, {-1, 10, 0.25}};
//        double realRightObs[3][3] = {{1, 3, 0.25}, {1, 6, 0.25}, {1, 9, 0.25}};

//    RobPose startPose = {0, 0, 0, 0, 0, 0};
//    RobPose targetPose = {0, 5, 0, 0, 0, 0};

//    robPoses.push_back(startPose);
//    robPoses.push_back(targetPose);

//    double realLeftObs[3][3] = {{-1, 2, 0.25}, {-1, 4, 0.25}};
//    double realRightObs[2][3] = {{1, 2, 0.25}, {1, 4, 0.55}};

//    for(int i = 0; i < 2; i++)
//    {
//        ObsPose tempObs;
//        tempObs.x = realLeftObs[i][0];
//        tempObs.y = realLeftObs[i][1];
//        tempObs.r = realLeftObs[i][2];
//        lObsPoses.push_back(tempObs);
//    }

//    for(int i = 0; i < 2; i++)
//    {
//        ObsPose tempObs;
//        tempObs.x = realRightObs[i][0];
//        tempObs.y = realRightObs[i][1];
//        tempObs.r = realRightObs[i][2];
//        rObsPoses.push_back(tempObs);
//    }

    std::string xml_address;

    if (argc <= 1)
    {
        std::cout << "you did not type in robot name, in this case ROBOT-III will start" << std::endl;
        //xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml";
        xml_address = "/home/hex/ArisVision/VisionEscapingPlanning/Robot_III.xml";
    }
    else if (std::string(argv[1]) == "III")
    {
        //xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml";
        xml_address = "/home/hex/ArisVision/VisionAvoid/Robot_III.xml";
    }
    else if (std::string(argv[1]) == "VIII")
    {
        //xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_VIII/Robot_VIII.xml";
        xml_address = "/home/hex/ArisVision/VisionAvoid/Robot_VIII.xml";
    }
    else
    {
        throw std::runtime_error("invalid robot name, please type in III or VIII");
    }

    auto &rs = aris::server::ControlServer::instance();

    rs.createModel<Robots::RobotTypeI>();
    rs.loadXml(xml_address.c_str());
    rs.addCmd("en", Robots::basicParse, nullptr);
    rs.addCmd("ds", Robots::basicParse, nullptr);
    rs.addCmd("hm", Robots::basicParse, nullptr);
    rs.addCmd("rc", Robots::recoverParse, Robots::recoverGait);
    rs.addCmd("wk", Robots::walkParse, Robots::walkGait);
    rs.addCmd("ro", Robots::resetOriginParse, Robots::resetOriginGait);

    rs.addCmd("ewk", Escaping::EscapingGaitWrapper::escapingParse, Escaping::EscapingGaitWrapper::escapingGait);
    rs.addCmd("vwk", VisionAvoid::VisionAvoidWrapper::visionWalkParse, VisionAvoid::VisionAvoidWrapper::visionWalk);
    rs.addCmd("swk", VisionAvoid::VisionAvoidWrapper::stopVisionWalkParse, VisionAvoid::VisionAvoidWrapper::visionWalk);

    rs.open();

    rs.setOnExit([&]()
    {
        aris::core::XmlDocument xml_doc;
        xml_doc.LoadFile(xml_address.c_str());
        auto model_xml_ele = xml_doc.RootElement()->FirstChildElement("Model");
        if (!model_xml_ele)
            throw std::runtime_error("can't find Model element in xml file");
        rs.model().saveXml(*model_xml_ele);

        aris::core::stopMsgLoop();
    });

    aris::core::runMsgLoop();

    return 0;
}

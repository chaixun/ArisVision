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

    robPoses.clear();

    RobPose startPose = {0, 0, 0, 0, 0, 0};
    RobPose targetPose = {0.625, 4.7476, 0, 0, 0, M_PI/6};

    robPoses.push_back(startPose);
    robPoses.push_back(targetPose);

    double realLeftObs[3][3] = {{-0.0250001, 1.7375, 0.353774}, {-0.435825, 2.52891, 0.257694}, {-0.237949, 4.04323, 0.285318}};
    double realRightObs[3][3] = {{1.4408, 2.2066, 0.290743}, {1.577, 3.41446, 0.251558}, {1.5515, 4.56942, 0.313249}};

    for(int i = 0; i < 3; i ++)
    {
        ObsPose realObs1, virtualObs1, realObs2, virtualObs2;

        realObs1.x = realLeftObs[i][0];
        realObs1.y = realLeftObs[i][1];
        realObs1.r = realLeftObs[i][2];
        lObsPoses.push_back(realObs1);

        virtualObs1.x = realObs1.x + realObs1.r + 0.2 + 0.9 + 0.2 + 0.3;
        virtualObs1.y = realObs1.y;
        virtualObs1.r = 0.3;
        rObsPoses.push_back(virtualObs1);

        realObs2.x = realRightObs[i][0];
        realObs2.y = realRightObs[i][1];
        realObs2.r = realRightObs[i][2];
        rObsPoses.push_back(realObs2);

        virtualObs2.x = realObs2.x - realObs2.r - 0.2 - 0.9 - 0.2 - 0.3;
        virtualObs2.y = realObs2.y;
        virtualObs2.r = 0.3;
        lObsPoses.push_back(virtualObs2);
    }

    std::string xml_address;

    if (argc <= 1)
    {
        std::cout << "you did not type in robot name, in this case ROBOT-III will start" << std::endl;
        //xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml";
        xml_address = "/home/hex/ArisVision/VisionEscapingModify/Robot_III.xml";
    }
    else if (std::string(argv[1]) == "III")
    {
        //xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml";
        xml_address = "/home/hex/ArisVision/VisionEscapingModify/Robot_III.xml";
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

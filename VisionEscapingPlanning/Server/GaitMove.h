#ifndef GAITMOVE_H
#define GAITMOVE_H

#include <iostream>

#include <aris.h>
#include <Robot_Gait.h>
#include <Robot_Base.h>
#include "EscapingPlanner.h"
#include "rtdk.h"

using namespace std;

namespace Escaping
{

enum GAIT_CMD
{
    NOCMD = 0,
    GENERPATHANDPOSE = 1,
    RUNGAIT = 3,
    STOP = 4
};

struct EscapingGaitParam final: public aris::server::GaitParamBase
{
};

class EscapingGaitWrapper
{
public:
    static double bodyPose[6];
    static double feetPosi[18];
    static EscapingPlanner escapingPlanner;
    static GAIT_CMD gaitCommand;

    static auto escapingParse(const string &cmd, const map<string, string> &param, aris::core::Msg &msg)->void;
    static auto escapingGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;
};

static EscapingGaitWrapper wrapper;
}


#endif // GAITMOVE_H

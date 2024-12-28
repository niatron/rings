#pragma once
#include <Fusion/FusionAll.h>

using namespace adsk::core;
using namespace adsk::fusion;

bool start();
Ptr<Component> drawGear(
    Ptr<Design> design,
    double diametralPitch,
    int numTeeth,
    double thickness,
    double rootFilletRad,
    double pressureAngle,
    double backlash,
    double holeDiam,
    double twist = 0,
    Ptr<Component> component = nullptr);
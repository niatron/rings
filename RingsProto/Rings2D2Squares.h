#pragma once

#include "FusionEnvironment.h"
#include "BasePart.h"
#include "RoofPart.h"
#include "VolfDownPart.h"
#include "VolfUpPart.h"

using namespace adsk::core;
using namespace adsk::fusion;

class Rings2D2Squares
{
    class Params
    {
    public:
        double baseOuterRadius;
        double baseInnerRadius;
        double baseWayHeight;
        double baseWallHeight;
    };

public:
    BasePart basePart;
    RoofPart roofPart;
    VolfUpPart volfUpPart;
    VolfDownPart volfDownPart;

    double lineVolfCount = 1;
    double cornerVolfCount = 2;
    double squareMiddleSize = 5; //length bitween centers of paralel line ways
    double moovableClearence = ABS_MOOVABLE_CLEARNCE;
    double unmoovableClearence = ABS_UNMOOVABLE_CLEARNCE;
    double verticalEdgeFilletRadius = 0.24;
    double horizontalEdgeFilletRadius = 0.12;
private:
    Ptr<ConstructionAxis> leftAxis = nullptr;
    Ptr<ConstructionAxis> rightAxis = nullptr;

public:
    Rings2D2Squares();
private:
    double getVolfRadius();
    double getLineLength();
    double getCornerOuterRadius();
    double getSquareShift();
    
    Ptr<Point3D> getLeftCenterPoint();
    Ptr<Point3D> getRightCenterPoint();
public:
    void createBodies(Ptr<Component> component);
};

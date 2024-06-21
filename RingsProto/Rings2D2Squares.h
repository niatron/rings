#pragma once

#include "FusionEnvironment.h"
#include "BasePart.h"
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
    double lineVolfCount = 1;
    double cornerVolfCount = 2;
    double volfLegRadius = 0.3;
    double volfLegThickness = 0.5;
    double volfLegHoleRadius = 0.14;
    double volfHeadThickness = 0.5;
    double squareMiddleSize = 5; //length bitween centers of paralel line ways
    double wallThickness = 0.16;
    double magnetRadius = 0.25;
    double floorThickness = 0.3;
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
    
    Ptr<Sketch> createSketchRings(Ptr<Component> component, double volfRadius, int count = -1);
    
    Ptr<BRepBody> createPairedSquares(Ptr<Component> component, double size, double cornerHeightOuter, double rotateAngel, double thickness, double height);
    
    Ptr<BRepBody> createVolfDownBody(Ptr<Component> component, VolfDownPart volfDownPart);
public:
    void createBodies(Ptr<Component> component);
};

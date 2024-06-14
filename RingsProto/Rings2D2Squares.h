#pragma once

#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>

#include "ComponentUtils.h"

using namespace adsk::core;
using namespace adsk::fusion;

#define RAD_90 M_PI / 2.0
#define RAD_180 M_PI
#define RAD_360 M_PI * 2.0
#define RAD_45 M_PI / 4.0

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

    class VolfDownParams
    {
    public:
        double radius;
        double height;
        double middleRadius = 0;
        double middleHeight = 0;
        double holeRadius = 0;
        double holeHeight = 0;
        double holeUpRadius = 0;
        double holeUpHeight = 0;
        double holeDownRadius = 0;
        double holeDownHeight = 0;
        Ptr<Point3D> centerPoint;
        double zMoveShift = 0;

        Ptr<BRepBody> createBody(Ptr<Component> component);
    };

    class VolfUpParams
    {
    public:
        double radius;
        double height;
        double middleRadius = 0;
        double middleHeight = 0;
        double holeRadius = 0;
        double holeHeight = 0;
        double holeUpRadius = 0;
        double holeUpHeight = 0;
        double holeDownRadius = 0;
        double holeDownHeight = 0;
        double cuttedSphreCuttingHeight = 0;
        double cuttedSphreRadius = 0;
        Ptr<Point3D> centerPoint;
        double zMoveShift = 0;

        Ptr<BRepBody> createBody(Ptr<Component> component);
    };

public:
    double lineVolfCount = 1;
    double cornerVolfCount = 2;
    double volfLegRadius = 0.3;
    double volfLegThickness = 0.5;
    double volfLegHoleRadius = 0.14;
    double volfHeadThickness = 0.2;
    double squareMiddleSize = 5; //length bitween centers of paralel line ways
    double wallThickness = 0.16;
    double magnetRadius = 0.25;
    double floorThickness = 0.2;
    double moovableClearence = ABS_MOOVABLE_CLEARNCE;
    double unmoovableClearence = ABS_UNMOOVABLE_CLEARNCE;
    double verticalEdgeFilletRadius = 0.12;
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
    
    Ptr<BRepBody> createVolfDownBody(Ptr<Component> component, VolfDownParams volfDownParams);
public:
    void createBodies(Ptr<Component> component);
};

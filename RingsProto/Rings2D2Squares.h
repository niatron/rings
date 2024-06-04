#pragma once

#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>

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

public:
    double lineVolfCount = 1;
    double cornerVolfCount = 2;
    double volfLegRadius = 0.3;
    double volfLegThickness = 0.2;
    double volfLegHoleRadius = 0.14;
    double volfHeadThickness = 0.2;
    double squareSize = 7;
    double wallThickness = 0.12;
    double magnetRadius = 0.3;
    double floorThickness = 0.2;
    double moovableClearence = 0.04;
    double unmoovableClearence = 0.02;
private:
    double volfRadiusWithoutClearance;
    double volfRadius;
    Ptr<ConstructionAxis> leftAxis = nullptr;
    Ptr<ConstructionAxis> rightAxis = nullptr;

public:
    Rings2D2Squares();
private:
    double getVolfSegmentAngelRad();
    double getVolfRadius();
    double getLineLength();
    double getCornerlHeight();
    double getSquareShift();
    Ptr<Point3D> getLeftCenterPoint();
    Ptr<Point3D> getRightCenterPoint();
    Ptr<Sketch> createSketchRings(Ptr<Component> component, double volfRadius, int count = -1);
    Ptr<Sketch> createSketchBase(Ptr<Component> component);
    Ptr<BRepBody> createSector(Ptr<Component> component, Ptr<Point3D> centerPoint, double radius, double angel, double startAngel, double height);
    Ptr<BRepBody> createPairedSquares(Ptr<Component> component, double size, double cornerHeightOuter, double rotateAngel, double thickness, double height);
    Ptr<BRepBody> createVolfCilinderPart(Ptr<Component> component, double radius, double height);
public:
    void createBodies(Ptr<Component> component);
};

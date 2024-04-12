#pragma once

#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>

using namespace adsk::core;
using namespace adsk::fusion;

#define RAD_90 M_PI / 2.0
#define RAD_180 M_PI
#define RAD_360 M_PI * 2.0
#define RAD_144 M_PI * 0.8

class Rings2D2Circles
{
    class Params
    {
    public:
        double baseOuterRadius;
    };
    int volfCount = 10;
    int crossVolfCount = 4;
    double volfRadiusWithoutClearance;
    double volfRadius;
    double circleRadius = 6;
    double wallThickness = 0.12;
    double floorThickness = 0.12;
    double moovableClearence = 0.04;
    double unmoovableClearence = 0.02;
    Ptr<ConstructionAxis> leftAxis = nullptr;
    
public:
    Rings2D2Circles();
private:
    void Recount();
    void setVolfCount(int count);
    void setCrossVolfCount(int count);
    double getVolfSegmentAngelRad();
    double getVolfRadius();
    double getCircleShift();
    Ptr<Point3D> getLeftCenterPoint();
    Ptr<Point3D> getRightCenterPoint();
    Ptr<Sketch> createSketchRings(Ptr<Component> component);
    Ptr<Sketch> createSketchBase(Ptr<Component> component);
    Ptr<BRepBody> createPairedCircles(Ptr<Component> component, double radiusOut, double thicknes, double height);
public:
    void createBodies(Ptr<Component> component);
};

#pragma once
#include "FusionEnvironment.h"

class SketcherHelper
{

};

class Sketcher
{
public:
    
    static void AddSquareCurves(Ptr<Sketch> sketch, Ptr<Point3D> center, double size, double cornerOuterRadius, double rotateAngel);
    static void AddCirclesOnSquare(Ptr<Sketch> sketch, Ptr<Point3D> center, double lineLength, double cornerMiddleRadius, double circleRadius, double circlesOnSquarePeriodRadius, double rotateAngel);

    static Ptr<BRepBody> CreateSquareBody(Ptr<Component> component, Ptr<Point3D> center, double size, double cornerOuterRadius, double rotateAngel, double thickness, double height);
    
};
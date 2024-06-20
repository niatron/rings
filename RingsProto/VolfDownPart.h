#pragma once
#include "FusionEnvironment.h"

class VolfDownPart
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
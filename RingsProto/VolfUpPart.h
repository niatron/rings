#pragma once
#include "FusionEnvironment.h"



class VolfUpPart
{
public:
    enum Form { convex, concave, straight };
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
    double filletRadius = 0;
    Form form = straight;
    double concaveHeight = 0;
    double concaveRadius = 0;
    double convexRadius = 0;

    Ptr<BRepBody> createBody(Ptr<Component> component);
private:
    bool edgeIsInHole(Ptr<BRepEdge> edge);
};
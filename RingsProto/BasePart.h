#pragma once
#include "FusionEnvironment.h"
#include "Sketcher.h"

class BasePart
{
public:
    double lineLength;
    double cornerMiddleRadius;
    double outerWidth;
    double innerWidth;
    double height;
    double wallThickness;
    double floorThickness;
    double circlesOnSquareRadius;
    double circlesOnSquarePeriodRadius;

    Ptr<Point3D> leftCenterPoint;
    Ptr<Point3D> rightCenterPoint;
    double zMoveShift = 0;
    double topEdgeFilletRadius = 0;
    double verticalEdgeFilletRadius = 0;
    double otherEdgeFilletRadius = 0;

    Ptr<BRepBody> createBody(Ptr<Component> component);
    Ptr<Sketch> createCirclesSketch(Ptr<Component> component, double circleRadius, int count);
protected:
    Ptr<BRepBody> createSquareBody(Ptr<Component> component, Ptr<Point3D> center, double size, double cornerOuterRadius, double rotateAngel, double thickness, double height);
    Ptr<BRepBody> createPairedSquares(Ptr<Component> component, double size, double cornerOuterRadius, double rotateAngel, double thickness, double height);

    void filletBody(Ptr<Component> component, Ptr<BRepBody> body);
};
#include "Rings2D2Squares.h"
#include "Geometry.h"
#include "ComponentUtils.h"

#define _USE_MATH_DEFINES
#include <math.h>

using namespace adsk::core;
using namespace adsk::fusion;

Rings2D2Squares::Rings2D2Squares()
{
}

double Rings2D2Squares::getVolfRadius()
{
    return squareSize / (2.0 * lineVolfCount + 2.0 / tan(RAD_180 / (4.0 * cornerVolfCount)));
}

double Rings2D2Squares::getLineLength()
{
    return lineVolfCount * getVolfRadius() * 2.0;
}

double Rings2D2Squares::getCornerlHeight()
{
    return (squareSize - getLineLength()) / 2.0;
}

double Rings2D2Squares::getSquareShift()
{
    auto sizeByVolfCenter = squareSize - 2.0 * getVolfRadius();
    return sizeByVolfCenter / (2.0 * sqrt(2.0));
}

Ptr<Point3D> Rings2D2Squares::getLeftCenterPoint()
{
    return Point3D::create(-getSquareShift());
}

Ptr<Point3D> Rings2D2Squares::getRightCenterPoint()
{
    return Point3D::create(getSquareShift());
}


void addSquareCurves(Ptr<Sketch> sketch, Ptr<Point3D> center, double size, double cornerHeight, double rotateAngel)
{
    auto length = 2.0 * cornerHeight + size;

    auto p1 = Point3D::create(center->x() - size / 2, center->y() + cornerHeight + size / 2);
    auto p2 = Point3D::create(p1->x() + size, p1->y());
    auto p3 = Point3D::create(p2->x() + cornerHeight, p2->y() - cornerHeight);
    auto p4 = Point3D::create(p3->x(), p3->y() - size);
    auto p5 = Point3D::create(p2->x(), p2->y() - length);
    auto p6 = Point3D::create(p1->x(), p1->y() - length);
    auto p7 = Point3D::create(p4->x() - length, p4->y());
    auto p8 = Point3D::create(p3->x() - length, p3->y());

    auto curves = createObjectCollection({
        AddLine(sketch, p1, p2),
        AddArc(sketch, Point3D::create(p2->x(), p3->y()), p3, p2),
        AddLine(sketch, p3, p4),
        AddArc(sketch, Point3D::create(p5->x(), p4->y()), p5, p4),
        AddLine(sketch, p5, p6),
        AddArc(sketch, Point3D::create(p6->x(), p7->y()), p7, p6),
        AddLine(sketch, p7, p8),
        AddArc(sketch, Point3D::create(p1->x(), p8->y()), p1, p8)
        });

    Rotate(sketch, rotateAngel, center, curves);
}

Ptr<BRepBody> createSquareBody(Ptr<Component> component, Ptr<Point3D> center, double size, double cornerHeightOuter, double rotateAngel, double thickness, double height)
{
    auto sketch = CreateSketch(component, component->xYConstructionPlane(), "SquareSketch");
    addSquareCurves(sketch, center, size, cornerHeightOuter, rotateAngel);
    addSquareCurves(sketch, center, size, cornerHeightOuter - thickness, rotateAngel);
    return Extrude(component, sketch, height)->bodies()->item(0);
}

Ptr<BRepBody> Rings2D2Squares::createPairedSquares(Ptr<Component> component, double size, double cornerHeightOuter, double rotateAngel, double thickness, double height)
{
    auto bodyLeft = createSquareBody(component, getLeftCenterPoint(), size, cornerHeightOuter, rotateAngel, thickness, height);
    auto bodyRight = createSquareBody(component, getRightCenterPoint(), size, cornerHeightOuter, rotateAngel, thickness, height);
    return Combine(component, JoinFeatureOperation, bodyLeft, bodyRight);
}

void Rings2D2Squares::createBodies(Ptr<Component> component)
{
    if (leftAxis == nullptr)
        leftAxis = AddConstructionAxis(component, getLeftCenterPoint(), Vector3D::create(0, 0, 1));
    if (rightAxis == nullptr)
        rightAxis = AddConstructionAxis(component, getRightCenterPoint(), Vector3D::create(0, 0, 1));
    createPairedSquares(component, getLineLength(), getCornerlHeight(), RAD_45, getVolfRadius() * 2.0, 1.5);
}
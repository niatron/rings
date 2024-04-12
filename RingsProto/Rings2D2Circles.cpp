#include "Rings2D2Circles.h"
#include "Geometry.h"
#include "ComponentUtils.h"

#define _USE_MATH_DEFINES
#include <math.h>

using namespace adsk::core;
using namespace adsk::fusion;

Rings2D2Circles::Rings2D2Circles()
{
    Recount();
}

void Rings2D2Circles::Recount()
{
    volfRadiusWithoutClearance = getVolfRadius();
    volfRadius = volfRadiusWithoutClearance - moovableClearence / volfCount;
    leftAxis = nullptr;
}

void Rings2D2Circles::setVolfCount(int count)
{
    volfCount = count;
    Recount();
}

void Rings2D2Circles::setCrossVolfCount(int count)
{
    crossVolfCount = count;
    Recount();
}

double Rings2D2Circles::getVolfSegmentAngelRad()
{
    return FULL_CIRCLE_RAD / volfCount;
}

double Rings2D2Circles::getVolfRadius()
{
    return GetTriangleSideLength(circleRadius, circleRadius, getVolfSegmentAngelRad()) / 2.0;
}

double Rings2D2Circles::getCircleShift()
{
    auto angel = (crossVolfCount - 1) * getVolfSegmentAngelRad() / 2.0;
    return GetRightTriangleLegByHypotenuseAndAdjacentAngle(circleRadius, angel);
}

Ptr<Point3D> Rings2D2Circles::getLeftCenterPoint()
{
    return Point3D::create(-getCircleShift());
}

Ptr<Point3D> Rings2D2Circles::getRightCenterPoint()
{
    return Point3D::create(getCircleShift());
}

Ptr<Sketch> Rings2D2Circles::createSketchRings(Ptr<Component> component)
{
    auto sketch = CreateSketch(component, component->xYConstructionPlane(), "RingsSketch");

    auto leftRotate = crossVolfCount % 2 == 0 ? getVolfSegmentAngelRad() / 2.0 : 0;
    auto rightRotate = crossVolfCount % 2 == volfCount % 2 ? getVolfSegmentAngelRad() / 2.0 : 0;
    for (int i = 0; i < volfCount; i++)
    {
        AddCircle(sketch, GetCirclePoint(getLeftCenterPoint(), circleRadius, getVolfSegmentAngelRad() * i + leftRotate), volfRadius);
        AddCircle(sketch, GetCirclePoint(getRightCenterPoint(), circleRadius, getVolfSegmentAngelRad() * i + rightRotate), volfRadius);
    }

    return sketch;
}

Ptr<Sketch> Rings2D2Circles::createSketchBase(Ptr<Component> component)
{
    auto sketch = CreateSketch(component, component->xYConstructionPlane(), "BaseSketch");

    auto radius = circleRadius + volfRadius + wallThickness;
    auto croosHeight = sqrt(pow(radius, 2) - pow(getLeftCenterPoint()->distanceTo(GetCenterPoint()), 2));
    auto leftStartPoint = Point3D::create(0, croosHeight);
    auto leftEndPoint = Point3D::create(0, -croosHeight);

    AddArc(sketch, getLeftCenterPoint(), leftStartPoint, leftEndPoint);
    AddArc(sketch, getRightCenterPoint(), leftEndPoint, leftStartPoint);

    return sketch;
}

Ptr<BRepBody> Rings2D2Circles::createPairedCircles(Ptr<Component> component, double radiusOut, double thicknes, double height)
{
    auto sketch = CreateSketch(component, component->xZConstructionPlane(), "PairedCirclesSketch");

    auto centerShift = getLeftCenterPoint()->distanceTo(GetCenterPoint());

    auto line1 = AddLine(sketch, Point3D::create(-centerShift + radiusOut), Point3D::create(-centerShift + radiusOut, height));
    auto line2 = AddLine(sketch, line1->endSketchPoint(), Point3D::create(line1->endSketchPoint()->geometry()->x() + thicknes, line1->endSketchPoint()->geometry()->y()));
    auto line3 = AddLine(sketch, line2->endSketchPoint(), Point3D::create(line2->endSketchPoint()->geometry()->x(), 0));
    auto line4 = AddLine(sketch, line3->endSketchPoint(), line1->startSketchPoint());

    auto leftBody = Revolve(component, sketch, leftAxis, FULL_CIRCLE_RAD)->bodies()->item(0);
    auto rightBody = Move(component, leftBody, component->xConstructionAxis(), getLeftCenterPoint()->distanceTo(getRightCenterPoint()), true);

    auto body = Combine(component, JoinFeatureOperation, leftBody, rightBody);

    return body;
}

void Rings2D2Circles::createBodies(Ptr<Component> component)
{
    if (leftAxis == nullptr)
        leftAxis = AddConstructionAxis(component, getLeftCenterPoint(), Vector3D::create(0, 0, 1));
    createSketchRings(component);
    createSketchBase(component);

    Params params;

    params.baseOuterRadius = circleRadius + volfRadius + wallThickness;

    createPairedCircles(component, 0, params.baseOuterRadius, floorThickness);
    createPairedCircles(component, params.baseOuterRadius - wallThickness, wallThickness, 0.4);
}
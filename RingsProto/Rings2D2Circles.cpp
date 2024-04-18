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

Ptr<Sketch> Rings2D2Circles::createSketchRings(Ptr<Component> component, double volfRadius)
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

    auto line1 = AddLine(sketch, Point3D::create(-centerShift + radiusOut, 0), Point3D::create(-centerShift + radiusOut, -height));
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
    createSketchRings(component, volfRadius);
    auto magnetSketch = createSketchRings(component, magnetRadius);
    //createSketchBase(component);

    Params params;

    params.baseOuterRadius = circleRadius + volfRadius + wallThickness + moovableClearence;
    params.baseInnerRadius = circleRadius - volfRadius - wallThickness - moovableClearence;

    auto baseBody = createPairedCircles(component, params.baseInnerRadius, params.baseOuterRadius - params.baseInnerRadius, floorThickness);
    auto baseOuterWallBody = createPairedCircles(component, params.baseOuterRadius - wallThickness, wallThickness, baseWallHeight);
    baseBody = Combine(component, JoinFeatureOperation, baseBody, baseOuterWallBody);
    auto baseInnerWallBody = createPairedCircles(component, params.baseInnerRadius, wallThickness, baseWallHeight);
    baseBody = Combine(component, JoinFeatureOperation, baseBody, baseInnerWallBody);
    auto baseWayBody = createPairedCircles(component, params.baseInnerRadius, params.baseOuterRadius - params.baseInnerRadius, floorThickness * 2.0);
    baseBody = Combine(component, JoinFeatureOperation, baseBody, baseWayBody);
    auto cutWayBody = createPairedCircles(component, params.baseInnerRadius + wallThickness, params.baseOuterRadius - params.baseInnerRadius - 2.0 * wallThickness, baseWallHeight);
    cutWayBody = Move(component, cutWayBody, leftAxis, floorThickness * 2.0);
    baseBody = Combine(component, CutFeatureOperation, baseBody, cutWayBody);

    for (int i = 0; i < magnetSketch->profiles()->count(); i++)
    {
        auto magnetBody = Extrude(component, magnetSketch->profiles()->item(i), floorThickness * 3.0, false);
        baseBody = Combine(component, CutFeatureOperation, baseBody, magnetBody->bodies()->item(0));
    }


    auto roofOuterInnerRadius = circleRadius + volfLegRadius + moovableClearence;
    auto roofOuterOuterRadius = params.baseOuterRadius + wallThickness + unmoovableClearence;

    auto roofOuterBody = createPairedCircles(component, roofOuterInnerRadius, roofOuterOuterRadius - roofOuterInnerRadius, floorThickness);
    auto roofOuterWallBody = createPairedCircles(component, roofOuterOuterRadius - wallThickness, wallThickness, baseWallHeight + floorThickness);
    roofOuterBody = Move(component, roofOuterBody, leftAxis, baseWallHeight + unmoovableClearence);
    roofOuterBody = Combine(component, JoinFeatureOperation, roofOuterBody, roofOuterWallBody);


    auto roofInnerInnerRadius = params.baseInnerRadius - wallThickness - unmoovableClearence;
    auto roofInnerOuterRadius = circleRadius - volfLegRadius - moovableClearence;

    auto roofInnerBody = createPairedCircles(component, roofInnerInnerRadius, roofInnerOuterRadius - roofInnerInnerRadius, floorThickness);
    auto roofInnerWallBody = createPairedCircles(component, roofInnerInnerRadius, wallThickness, baseWallHeight + floorThickness);
    roofInnerBody = Move(component, roofInnerBody, leftAxis, baseWallHeight + unmoovableClearence);
    roofInnerBody = Combine(component, JoinFeatureOperation, roofInnerBody, roofInnerWallBody);

    auto roofBody = Combine(component, JoinFeatureOperation, roofInnerBody, roofOuterBody);

    auto roofCutWayBody = createPairedCircles(component, roofInnerInnerRadius + wallThickness, roofOuterOuterRadius - roofInnerInnerRadius - 2.0 * wallThickness, baseWallHeight + unmoovableClearence);
    auto roofClearenceCutWayBody = createPairedCircles(component, roofInnerOuterRadius, roofOuterInnerRadius - roofInnerOuterRadius, baseWallHeight + unmoovableClearence + floorThickness);
    roofCutWayBody = Combine(component, JoinFeatureOperation, roofCutWayBody, roofClearenceCutWayBody);

    Combine(component, CutFeatureOperation, roofBody, roofCutWayBody);
}
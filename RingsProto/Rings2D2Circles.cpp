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

Ptr<Sketch> Rings2D2Circles::createSketchRings(Ptr<Component> component, double volfRadius, int count)
{
    if (count < 0 || count > 2 * volfCount)
        count = 2 * volfCount;
    auto sketch = CreateSketch(component, component->xYConstructionPlane(), "RingsSketch");

    auto leftRotate = crossVolfCount % 2 == 0 ? getVolfSegmentAngelRad() / 2.0 : 0;
    auto rightRotate = crossVolfCount % 2 == volfCount % 2 ? getVolfSegmentAngelRad() / 2.0 : 0;
    for (int i = 0, k = 0; i < volfCount; i++)
    {
        if (k == count)
            break;
        AddCircle(sketch, GetCirclePoint(getLeftCenterPoint(), circleRadius, getVolfSegmentAngelRad() * i + leftRotate), volfRadius);
        k++;
        if (k == count)
            break;
        AddCircle(sketch, GetCirclePoint(getRightCenterPoint(), circleRadius, getVolfSegmentAngelRad() * i + rightRotate), volfRadius);
        k++;
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

Ptr<BRepBody> Rings2D2Circles::createSector(Ptr<Component> component, Ptr<Point3D> centerPoint, double radius, double angel, double startAngel, double height)
{
    auto sketch = CreateSketch(component, component->xYConstructionPlane(), "SectorSketch");

    auto startPoint = GetCirclePoint(centerPoint, radius, startAngel);
    auto endPoint = GetCirclePoint(centerPoint, radius, startAngel + angel);
    
    AddLine(sketch, centerPoint, startPoint);
    AddLine(sketch, centerPoint, endPoint);
    AddArc(sketch, centerPoint, startPoint, endPoint);

    return Extrude(component, sketch, height)->bodies()->item(0);
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
Ptr<BRepBody> Rings2D2Circles::createVolfCilinderPart(Ptr<Component> component, double radius, double height)
{
    auto sketch = createSketchRings(component, radius, 1);
    return Extrude(component, sketch->profiles()->item(0), height, false)->bodies()->item(0);
}

void Rings2D2Circles::createBodies(Ptr<Component> component)
{
    if (leftAxis == nullptr)
        leftAxis = AddConstructionAxis(component, getLeftCenterPoint(), Vector3D::create(0, 0, 1));
    if (rightAxis == nullptr)
        rightAxis = AddConstructionAxis(component, getRightCenterPoint(), Vector3D::create(0, 0, 1));
    //createSketchRings(component, volfRadius);
    auto magnetSketch = createSketchRings(component, magnetRadius);
    //createSketchBase(component);

    Params params;

    params.baseOuterRadius = circleRadius + volfRadius + wallThickness + moovableClearence * 0.25;
    params.baseInnerRadius = circleRadius - volfRadius - wallThickness - moovableClearence * 0.75;
    params.baseWayHeight = floorThickness * 1.0;
    params.baseWallHeight = params.baseWayHeight + volfLegThickness + 2.0 * moovableClearence;

    auto baseBody = createPairedCircles(component, params.baseInnerRadius, params.baseOuterRadius - params.baseInnerRadius, floorThickness);
    auto baseOuterWallBody = createPairedCircles(component, params.baseOuterRadius - wallThickness, wallThickness, params.baseWallHeight);
    baseBody = Combine(component, JoinFeatureOperation, baseBody, baseOuterWallBody);
    auto baseInnerWallBody = createPairedCircles(component, params.baseInnerRadius, wallThickness, params.baseWallHeight);
    baseBody = Combine(component, JoinFeatureOperation, baseBody, baseInnerWallBody);
    auto baseWayBody = createPairedCircles(component, params.baseInnerRadius, params.baseOuterRadius - params.baseInnerRadius, params.baseWayHeight);
    baseBody = Combine(component, JoinFeatureOperation, baseBody, baseWayBody);
    auto cutWayBody = createPairedCircles(component, params.baseInnerRadius + wallThickness, params.baseOuterRadius - params.baseInnerRadius - 2.0 * wallThickness, params.baseWallHeight);
    cutWayBody = Move(component, cutWayBody, leftAxis, params.baseWayHeight);
    baseBody = Combine(component, CutFeatureOperation, baseBody, cutWayBody);


    //------------------inspect
    auto sketch1 = CreateSketch(component, component->xYConstructionPlane(), "RingsSketch");
    AddCircle(sketch1, getLeftCenterPoint(), params.baseOuterRadius - wallThickness);
    Extrude(component, sketch1, params.baseWallHeight);
    auto sketch2 = CreateSketch(component, component->xYConstructionPlane(), "RingsSketch");
    AddCircle(sketch2, getRightCenterPoint(), params.baseOuterRadius - wallThickness);
    Extrude(component, sketch2, params.baseWallHeight);
    //-------------------

    for (int i = 0; i < magnetSketch->profiles()->count(); i++)
    {
        auto magnetBody = Extrude(component, magnetSketch->profiles()->item(i), floorThickness * 3.0, false);
        baseBody = Combine(component, CutFeatureOperation, baseBody, magnetBody->bodies()->item(0));
    }


    auto roofOuterInnerRadius = circleRadius + volfLegRadius + moovableClearence * 0.25;
    auto roofOuterOuterRadius = params.baseOuterRadius + wallThickness + unmoovableClearence;
    auto roofInnerInnerRadius = params.baseInnerRadius - wallThickness - unmoovableClearence;
    auto roofInnerOuterRadius = circleRadius - volfLegRadius - moovableClearence * 0.75;

    auto roofBody = createPairedCircles(component, roofInnerInnerRadius, roofOuterOuterRadius - roofInnerInnerRadius, params.baseWallHeight + floorThickness + unmoovableClearence);

    auto roofCutWayBody = createPairedCircles(component, roofInnerInnerRadius + wallThickness, roofOuterOuterRadius - roofInnerInnerRadius - 2.0 * wallThickness, params.baseWallHeight + unmoovableClearence);
    auto roofClearenceCutWayBody = createPairedCircles(component, roofInnerOuterRadius, roofOuterInnerRadius - roofInnerOuterRadius, params.baseWallHeight + unmoovableClearence + floorThickness);
    roofCutWayBody = Combine(component, JoinFeatureOperation, roofCutWayBody, roofClearenceCutWayBody);

    Combine(component, CutFeatureOperation, roofBody, roofCutWayBody);


    
    auto sectorAngel = (circleRadius * RAD_360 / volfCount - 2.0 * volfLegRadius - unmoovableClearence) / RAD_360;
    auto sectorBody = createSector(component, getLeftCenterPoint(), roofOuterInnerRadius, sectorAngel, RAD_90, params.baseWallHeight + unmoovableClearence + floorThickness);
    auto sectorItersectBody = createPairedCircles(component, roofInnerOuterRadius, roofOuterInnerRadius - roofInnerOuterRadius, params.baseWallHeight + unmoovableClearence + floorThickness);
    sectorBody = Combine(component, IntersectFeatureOperation, sectorBody, sectorItersectBody);


    auto volfDownBody = createVolfCilinderPart(component, volfRadius, volfLegThickness);
    volfDownBody = Move(component, volfDownBody, leftAxis, params.baseWayHeight + moovableClearence);

    auto volfLegBody = createVolfCilinderPart(component, volfLegRadius, floorThickness + 2.0 * moovableClearence);
    volfLegBody = Move(component, volfLegBody, leftAxis, params.baseWayHeight + moovableClearence + volfLegThickness);
    volfDownBody = Combine(component, JoinFeatureOperation, volfDownBody, volfLegBody);
    
    auto volfLegHoleBody = createVolfCilinderPart(component, volfLegHoleRadius, volfLegThickness + floorThickness + 2.0 * moovableClearence);
    volfLegHoleBody = Move(component, volfLegHoleBody, leftAxis, params.baseWayHeight + moovableClearence);
    volfDownBody = Combine(component, CutFeatureOperation, volfDownBody, volfLegHoleBody);


    auto volfUpBody = createVolfCilinderPart(component, volfRadius, volfHeadThickness);
    auto volfUpHoleBody = createVolfCilinderPart(component, volfLegHoleRadius + moovableClearence, volfHeadThickness);
    volfUpBody = Combine(component, CutFeatureOperation, volfUpBody, volfUpHoleBody);
    volfUpBody = Move(component, volfUpBody, leftAxis, params.baseWallHeight + unmoovableClearence + floorThickness + moovableClearence);
    
    
    auto head2 = Rotate(component, volfUpBody, leftAxis, getVolfSegmentAngelRad(), true);
    auto head3 = Rotate(component, head2, leftAxis, getVolfSegmentAngelRad(), true);
    auto head4 = Rotate(component, head2, rightAxis, getVolfSegmentAngelRad(), true);
    auto head5 = Rotate(component, head2, rightAxis, -getVolfSegmentAngelRad(), true);
}
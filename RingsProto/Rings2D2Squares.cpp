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
    // innerCornerRadius + volfRadius = middleCornerRadius
    // volfDiametr = 2 * volfRadius;
    // volfDiametr^2 = middleCornerRadius^2 + middleCornerRadius^2 - 2 * middleCornerRadius * middleCornerRadius * cos(2*pi / 4*cornerVolfCount) //теорема косинусов
    // volfDiametr = middleCornerRadius * sqrt(2)*sqrt(1 - cos(2*pi / 4*cornerVolfCount))
    // rate = sqrt(2)*sqrt(1 - cos(2*pi / 4*cornerVolfCount))
    // volfDiametr = middleCornerRadius * rate
    // middleCornerRadius = (squareMiddleSize - lineVolfCount * volfDiametr)/2
    // volfDiametr = (squareMiddleSize - lineVolfCount * volfDiametr) * rate / 2
    // volfDiametr = (squareMiddleSize * rate / 2) / (1 + lineVolfCount * rate / 2)
    
    auto rate = sqrt(2.0) * sqrt(1 - cos(RAD_360 / (4.0 * cornerVolfCount)));
    auto volfDiametr = (squareMiddleSize * rate / 2.0) / (1.0 + lineVolfCount * rate / 2.0);
    return volfDiametr / 2.0;
}

double Rings2D2Squares::getLineLength()
{
    return lineVolfCount * getVolfRadius() * 2.0;
}

double Rings2D2Squares::getCornerOuterRadius()
{
    return (squareMiddleSize - getLineLength()) / 2.0 + getVolfRadius();
}

double Rings2D2Squares::getSquareShift()
{
    auto sizeByVolfCenter = squareMiddleSize;
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


void addSquareCurves(Ptr<Sketch> sketch, Ptr<Point3D> center, double size, double cornerOuterRadius, double rotateAngel)
{
    auto length = 2.0 * cornerOuterRadius + size;

    auto p1 = Point3D::create(center->x() - size / 2, center->y() + cornerOuterRadius + size / 2);
    auto p2 = Point3D::create(p1->x() + size, p1->y());
    auto p3 = Point3D::create(p2->x() + cornerOuterRadius, p2->y() - cornerOuterRadius);
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

void addSquareVolfsCircles(Ptr<Sketch> sketch, Ptr<Point3D> center, double size, double cornerMiddleRadius, double volfRadius, double volfFullRadius, double rotateAngel)
{
    auto length = 2.0 * cornerMiddleRadius + size;

    auto p1 = Point3D::create(center->x() - size / 2, center->y() + cornerMiddleRadius + size / 2);
    auto p2 = Point3D::create(p1->x() + size, p1->y());
    auto p3 = Point3D::create(p2->x() + cornerMiddleRadius, p2->y() - cornerMiddleRadius);
    auto rotatePointRightUp = Point3D::create(p2->x(), p3->y());

    for (int i = 0; i < 4; i++)
    {
        auto onLineCircle = AddCircle(sketch, Point3D::create(p1->x() + volfFullRadius, p1->y()), volfRadius);
        auto onCornerCircle1 = AddCircle(sketch, p2, volfRadius);
        Rotate(sketch, -RAD_45, rotatePointRightUp, { onCornerCircle1 });
        auto onCornerCircle2 = AddCircle(sketch, p2, volfRadius);
        Rotate(sketch, -RAD_45 / 2.0, rotatePointRightUp, { onCornerCircle1,onCornerCircle2 });
        Rotate(sketch, -RAD_90, center);
    }

    Rotate(sketch, rotateAngel, center);
}

Ptr<BRepBody> createSquareBody(Ptr<Component> component, Ptr<Point3D> center, double size, double cornerOuterRadius, double rotateAngel, double thickness, double height)
{
    auto sketch = CreateSketch(component, component->xYConstructionPlane(), "SquareSketch");
    addSquareCurves(sketch, center, size, cornerOuterRadius, rotateAngel);
    addSquareCurves(sketch, center, size, cornerOuterRadius - thickness, rotateAngel);
    return Extrude(component, sketch, height)->bodies()->item(0);
}

Ptr<BRepBody> Rings2D2Squares::createPairedSquares(Ptr<Component> component, double size, double cornerOuterRadius, double rotateAngel, double thickness, double height)
{
    auto bodyLeft = createSquareBody(component, getLeftCenterPoint(), size, cornerOuterRadius, rotateAngel, thickness, height);
    auto bodyRight = createSquareBody(component, getRightCenterPoint(), size, cornerOuterRadius, rotateAngel, thickness, height);
    return Combine(component, JoinFeatureOperation, bodyLeft, bodyRight);
}

Ptr<Sketch> Rings2D2Squares::createSketchRings(Ptr<Component> component, double volfRadius, int count)
{
    auto leftSketch = CreateSketch(component, component->xYConstructionPlane(), "SquareSketch");
    addSquareVolfsCircles(leftSketch, getLeftCenterPoint(), getLineLength(), getCornerOuterRadius() - getVolfRadius(), volfRadius, getVolfRadius(), RAD_45);
    auto rightSketch = CreateSketch(component, component->xYConstructionPlane(), "SquareSketch");
    addSquareVolfsCircles(rightSketch, getRightCenterPoint(), getLineLength(), getCornerOuterRadius() - getVolfRadius(), volfRadius, getVolfRadius(), RAD_45);
    for (int i = 0; i < rightSketch->sketchCurves()->sketchCircles()->count(); i++)
    {
        auto circle = rightSketch->sketchCurves()->sketchCircles()->item(i)->geometry();
        if (circle->center()->y() != 0)
            AddCircle(leftSketch, circle->center(), circle->radius());
    }
    rightSketch->deleteMe();

    return leftSketch;
}

Ptr<BRepBody> Rings2D2Squares::createVolfCilinderPart(Ptr<Component> component, double radius, double height)
{
    auto sketch = createSketchRings(component, radius, 1);
    return Extrude(component, sketch->profiles()->item(0), height, false)->bodies()->item(0);
}

void Rings2D2Squares::createBodies(Ptr<Component> component)
{
    if (leftAxis == nullptr)
        leftAxis = AddConstructionAxis(component, getLeftCenterPoint(), Vector3D::create(0, 0, 1));
    if (rightAxis == nullptr)
        rightAxis = AddConstructionAxis(component, getRightCenterPoint(), Vector3D::create(0, 0, 1));

    auto magnetSketch = createSketchRings(component, magnetRadius, 0);
    auto volfsSketch = createSketchRings(component, getVolfRadius(), 0);
    volfsSketch->isLightBulbOn(false);

    //---------------- bas body ---------------
    auto lineLength = getLineLength();
    auto volfDiameter = getVolfRadius() * 2.0;
    auto params_baseOuterRadius = getCornerOuterRadius() + wallThickness + moovableClearence * 0.25;
    auto params_baseInnerRadius = getCornerOuterRadius() - volfDiameter - wallThickness - moovableClearence * 0.75;
    auto params_baseWayHeight = floorThickness * 1.0;
    auto params_baseWallHeight = params_baseWayHeight + volfLegThickness + 2.0 * moovableClearence;

    auto baseBody = createPairedSquares(component, lineLength, params_baseOuterRadius, RAD_45, params_baseOuterRadius - params_baseInnerRadius, params_baseWallHeight);
    auto cutWayBody = createPairedSquares(component, lineLength, params_baseOuterRadius - wallThickness, RAD_45, params_baseOuterRadius - params_baseInnerRadius - 2.0 * wallThickness, params_baseWallHeight);
    cutWayBody = Move(component, cutWayBody, leftAxis, params_baseWayHeight);
    baseBody = Combine(component, CutFeatureOperation, baseBody, cutWayBody);

    for (int i = 0; i < magnetSketch->profiles()->count(); i++)
    {
        auto magnetBody = Extrude(component, magnetSketch->profiles()->item(i), floorThickness * 3.0, false);
        baseBody = Combine(component, CutFeatureOperation, baseBody, magnetBody->bodies()->item(0));
    }


    //---------------- roof bodies ---------------
    auto cornerMiddleRadius = getCornerOuterRadius() - getVolfRadius();
    auto roofOuterInnerRadius = cornerMiddleRadius + volfLegRadius + moovableClearence * 0.25;
    auto roofOuterOuterRadius = params_baseOuterRadius + wallThickness + unmoovableClearence * 0.4;
    auto roofInnerInnerRadius = params_baseInnerRadius - wallThickness - unmoovableClearence * 0.4;
    auto roofInnerOuterRadius = cornerMiddleRadius - volfLegRadius - moovableClearence * 0.75;

    auto roofBody = createPairedSquares(component, lineLength, roofOuterOuterRadius, RAD_45, roofOuterOuterRadius - roofInnerInnerRadius, params_baseWallHeight + floorThickness + unmoovableClearence);

    auto roofCutWayBody = createPairedSquares(component, lineLength, roofOuterOuterRadius - wallThickness, RAD_45, roofOuterOuterRadius - roofInnerInnerRadius - 2.0 * wallThickness, params_baseWallHeight);
    auto roofClearenceCutWayBody = createPairedSquares(component, lineLength, roofOuterInnerRadius, RAD_45, roofOuterInnerRadius - roofInnerOuterRadius, params_baseWallHeight + unmoovableClearence + floorThickness);
    roofCutWayBody = Combine(component, JoinFeatureOperation, roofCutWayBody, roofClearenceCutWayBody);

    Combine(component, CutFeatureOperation, roofBody, roofCutWayBody);

    //---------------- volf ---------------

    for (int i = 0; i < volfsSketch->sketchCurves()->sketchCircles()->count() && i < 2; i++)
    {
        auto volfRadius = getVolfRadius() - moovableClearence / (cornerVolfCount * 4.0 + lineVolfCount * 4.0);
        auto volfCenter = volfsSketch->sketchCurves()->sketchCircles()->item(i)->centerSketchPoint()->geometry();

        auto volfDownBody = CreateCylinder(component, volfCenter, volfRadius, volfLegThickness);
        volfDownBody = Move(component, volfDownBody, leftAxis, params_baseWayHeight + moovableClearence);

        auto volfLegBody = CreateCylinder(component, volfCenter, volfLegRadius, floorThickness + 2.0 * moovableClearence);
        volfLegBody = Move(component, volfLegBody, leftAxis, params_baseWayHeight + moovableClearence + volfLegThickness);
        volfDownBody = Combine(component, JoinFeatureOperation, volfDownBody, volfLegBody);

        auto volfLegHoleBody = CreateCylinder(component, volfCenter, volfLegHoleRadius, volfLegThickness + floorThickness + 2.0 * moovableClearence);
        volfLegHoleBody = Move(component, volfLegHoleBody, leftAxis, params_baseWayHeight + moovableClearence);
        volfDownBody = Combine(component, CutFeatureOperation, volfDownBody, volfLegHoleBody);

        //if (i > 0)
        //    continue;
        auto volfUpBody = CreateCylinder(component, volfCenter, volfRadius, volfHeadThickness);
        auto volfUpHoleBody = CreateCylinder(component, volfCenter, volfLegHoleRadius + moovableClearence, volfHeadThickness);
        volfUpBody = Combine(component, CutFeatureOperation, volfUpBody, volfUpHoleBody);
        volfUpBody = Move(component, volfUpBody, leftAxis, params_baseWallHeight + unmoovableClearence + floorThickness + moovableClearence);
        volfUpBody->isLightBulbOn(false);
    }

    //---------------- filleting ---------------
    for (int i = 0; i < component->bRepBodies()->count(); i++)
    {
        auto body = component->bRepBodies()->item(i);
        auto edges = GetEdges(body, [=](Ptr<BRepEdge> edge) {return Equal(abs(edge->startVertex()->geometry()->z() - edge->endVertex()->geometry()->z()), edge->length(), 0.01) && !edge->isDegenerate(); });
        Fillet(component, edges, verticalEdgeFilletRadius);
        auto edges2 = GetEdges(body, [=](Ptr<BRepEdge> edge) {return Equal(abs(edge->startVertex()->geometry()->z() - edge->endVertex()->geometry()->z()), 0, 0.01) && !edge->isDegenerate(); });
        Fillet(component, edges2, verticalEdgeFilletRadius / 2.0);
    }
}
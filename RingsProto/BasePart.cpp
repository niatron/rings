#include "BasePart.h"

void BasePart::addSquareCurves(Ptr<Sketch> sketch, Ptr<Point3D> center, double size, double cornerOuterRadius, double rotateAngel)
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

Ptr<BRepBody> BasePart::createSquareBody(Ptr<Component> component, Ptr<Point3D> center, double size, double cornerOuterRadius, double rotateAngel, double thickness, double height)
{
    auto sketch = CreateSketch(component, component->xYConstructionPlane(), "SquareSketch");
    addSquareCurves(sketch, center, size, cornerOuterRadius, rotateAngel);
    addSquareCurves(sketch, center, size, cornerOuterRadius - thickness, rotateAngel);
    return Extrude(component, sketch, height)->bodies()->item(0);
}

Ptr<BRepBody> BasePart::createPairedSquares(Ptr<Component> component, double size, double cornerOuterRadius, double rotateAngel, double thickness, double height)
{
    auto bodyLeft = createSquareBody(component, leftCenterPoint, size, cornerOuterRadius, rotateAngel, thickness, height);
    auto bodyRight = createSquareBody(component, rightCenterPoint, size, cornerOuterRadius, rotateAngel, thickness, height);
    return Combine(component, JoinFeatureOperation, bodyLeft, bodyRight);
}

void BasePart::addCirclesOnSquare(Ptr<Sketch> sketch, Ptr<Point3D> center, double lineLength, double cornerMiddleRadius, double circleRadius, double circlesOnSquarePeriodRadius, double rotateAngel)
{
    auto p1 = Point3D::create(center->x() - lineLength / 2, center->y() + cornerMiddleRadius + lineLength / 2);
    auto p2 = Point3D::create(p1->x() + lineLength, p1->y());
    auto p3 = Point3D::create(p2->x() + cornerMiddleRadius, p2->y() - cornerMiddleRadius);
    auto rotatePointRightUp = Point3D::create(p2->x(), p3->y());

    for (int i = 0; i < 4; i++)
    {
        auto onLineCircle = AddCircle(sketch, Point3D::create(p1->x() + circlesOnSquarePeriodRadius, p1->y()), circleRadius);
        auto onCornerCircle1 = AddCircle(sketch, p2, circleRadius);
        Rotate(sketch, -RAD_45, rotatePointRightUp, { onCornerCircle1 });
        auto onCornerCircle2 = AddCircle(sketch, p2, circleRadius);
        Rotate(sketch, -RAD_45 / 2.0, rotatePointRightUp, { onCornerCircle1,onCornerCircle2 });
        Rotate(sketch, -RAD_90, center);
    }

    Rotate(sketch, rotateAngel, center);
}

Ptr<Sketch> BasePart::createCirclesSketch(Ptr<Component> component, double circleRadius, int count)
{
    auto leftSketch = CreateSketch(component, component->xYConstructionPlane(), "CirclesSketch");
    addCirclesOnSquare(leftSketch, leftCenterPoint, lineLength, cornerMiddleRadius, circleRadius, circlesOnSquarePeriodRadius, RAD_45);
    auto rightSketch = CreateSketch(component, component->xYConstructionPlane(), "CirclesSketch");
    addCirclesOnSquare(rightSketch, rightCenterPoint, lineLength, cornerMiddleRadius, circleRadius, circlesOnSquarePeriodRadius, RAD_45);
    for (int i = 0; i < rightSketch->sketchCurves()->sketchCircles()->count(); i++)
    {
        auto circle = rightSketch->sketchCurves()->sketchCircles()->item(i)->geometry();
        if (circle->center()->y() != 0)
            AddCircle(leftSketch, circle->center(), circle->radius());
    }
    rightSketch->deleteMe();

    return leftSketch;
}

Ptr<BRepBody> BasePart::createBody(Ptr<Component> component)
{
    auto cornerOuterRadius = cornerMiddleRadius + outerWidth;
    auto width = outerWidth + innerWidth;

    auto body = createPairedSquares(component, lineLength, cornerOuterRadius, RAD_45, width, height);
    auto cutWayBody = createPairedSquares(component, lineLength, cornerOuterRadius - wallThickness, RAD_45, width - 2.0 * wallThickness, height);
    cutWayBody = Move(component, cutWayBody, component->zConstructionAxis(), floorThickness);
    body = Combine(component, CutFeatureOperation, body, cutWayBody);

    auto magnetSketch = createCirclesSketch(component, circlesOnSquareRadius, 0);

    for (int i = 0; i < magnetSketch->profiles()->count(); i++)
    {
        auto magnetBody = Extrude(component, magnetSketch->profiles()->item(i), floorThickness * 3.0, false);
        body = Combine(component, CutFeatureOperation, body, magnetBody->bodies()->item(0));
    }

    auto edges = GetEdges(body, [=](Ptr<BRepEdge> edge) {return EdgeIsVerticalLine(edge) && abs(edge->endVertex()->geometry()->x()) <= 0.01 && abs(edge->endVertex()->geometry()->y()) < width; });
    auto minY = GetMin(edges, [=](Ptr<BRepEdge> edge) {return abs(edge->endVertex()->geometry()->y()); });
    edges = GetEdges(edges, [=](Ptr<BRepEdge> edge) {return abs(edge->endVertex()->geometry()->y()) < minY + 0.01; });
    Fillet(component, edges, verticalEdgeFilletRadius / 2.0);

    edges = GetEdges(body, EdgeIsVerticalLine);
    Fillet(component, edges, verticalEdgeFilletRadius);

    edges = GetEdges(body, [=](Ptr<BRepEdge> edge) {return EdgeIsHorizontal(edge) && edge->endVertex()->geometry()->z() > floorThickness * 1.1; });
    Fillet(component, edges, wallTopEdgeFilletRadius);

    edges = GetEdges(body, EdgeIsHorizontal);
    Fillet(component, edges, otherEdgeFilletRadius);

    return body;
}
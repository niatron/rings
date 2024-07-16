#include "BasePart.h"



Ptr<BRepBody> BasePart::createPairedSquares(Ptr<Component> component, double size, double cornerOuterRadius, double rotateAngel, double thickness, double height)
{
    auto bodyLeft = Sketcher::CreateSquareBody(component, leftCenterPoint, size, cornerOuterRadius, rotateAngel, thickness, height);
    auto bodyRight = Sketcher::CreateSquareBody(component, rightCenterPoint, size, cornerOuterRadius, rotateAngel, thickness, height);
    return Combine(component, JoinFeatureOperation, bodyLeft, bodyRight);
}


Ptr<Sketch> BasePart::createCirclesSketch(Ptr<Component> component, double circleRadius, int count)
{
    auto leftSketch = CreateSketch(component, component->xYConstructionPlane(), "CirclesSketch");
    Sketcher::AddCirclesOnSquare(leftSketch, leftCenterPoint, lineLength, cornerMiddleRadius, circleRadius, circlesOnSquarePeriodRadius, RAD_45);
    auto rightSketch = CreateSketch(component, component->xYConstructionPlane(), "CirclesSketch");
    Sketcher::AddCirclesOnSquare(rightSketch, rightCenterPoint, lineLength, cornerMiddleRadius, circleRadius, circlesOnSquarePeriodRadius, RAD_45);
    for (int i = 0; i < rightSketch->sketchCurves()->sketchCircles()->count(); i++)
    {
        auto circle = rightSketch->sketchCurves()->sketchCircles()->item(i)->geometry();
        if (circle->center()->y() != 0)
            AddCircle(leftSketch, circle->center(), circle->radius());
    }
    rightSketch->deleteMe();

    return leftSketch;
}

void BasePart::filletBody(Ptr<Component> component, Ptr<BRepBody> body)
{
    auto edges = GetEdges(body, [=](Ptr<BRepEdge> edge) {return EdgeIsVerticalLine(edge) && abs(edge->endVertex()->geometry()->x()) <= 0.01 && abs(edge->endVertex()->geometry()->y()) < innerWidth + outerWidth; });
    auto minY = GetMin(edges, [=](Ptr<BRepEdge> edge) {return abs(edge->endVertex()->geometry()->y()); });
    edges = GetEdges(edges, [=](Ptr<BRepEdge> edge) {return abs(edge->endVertex()->geometry()->y()) < minY + 0.01; });
    Fillet(component, edges, verticalEdgeFilletRadius / 2.0);

    edges = GetEdges(body, EdgeIsVerticalLine);
    Fillet(component, edges, verticalEdgeFilletRadius);

    edges = GetEdges(body, [=](Ptr<BRepEdge> edge) {return EdgeIsHorizontal(edge) && edge->endVertex()->geometry()->z() > floorThickness * 1.1; });
    Fillet(component, edges, topEdgeFilletRadius);

    edges = GetEdges(body, [=](Ptr<BRepEdge> edge) {return EdgeIsHorizontal(edge) && Equal(edge->length(), GetCircleLength(circlesOnSquareRadius), 0.1); });
    Fillet(component, edges, floorThickness / 3.0);

    edges = GetEdges(body, EdgeIsHorizontal);
    Fillet(component, edges, otherEdgeFilletRadius);
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

    filletBody(component, body);

    return body;
}
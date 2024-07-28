#include "RoofPart.h"

Ptr<ObjectCollection> RoofPart::createBodies(Ptr<Component> component)
{
    auto cornerOuterRadius = cornerMiddleRadius + outerWidth;
    auto width = outerWidth + innerWidth;
    auto separationCornerOuterRadius = cornerMiddleRadius + separationOuterWidth;
    auto separationWidth = separationOuterWidth + separationInnerWidth;

    auto body = createPairedSquares(component, lineLength, cornerOuterRadius, RAD_45, width, height);
    auto centerBody = CreateCylinder(component, Point3D::create(), width, height);
    body = Combine(component, JoinFeatureOperation, body, centerBody);

    auto cutBody = createPairedSquares(component, lineLength, cornerOuterRadius - wallThickness, RAD_45, width - 2.0 * wallThickness, height - floorThickness);
    auto separationCutBody = createPairedSquares(component, lineLength, separationCornerOuterRadius, RAD_45, separationWidth, height);
    cutBody = Combine(component, JoinFeatureOperation, cutBody, separationCutBody);
    auto substrateCutBody = CreateCylinder(component, Point3D::create(), 2.0 * (lineLength + cornerOuterRadius * 2.0), downTrimmingThicknes);
    cutBody = Combine(component, JoinFeatureOperation, cutBody, substrateCutBody);
    
    auto feature = CreateCombineFeature(component, CutFeatureOperation, body, cutBody);

    auto result = ObjectCollection::create();
    auto smallShift = 0.1;
    auto outerDistance = lineLength / 2.0 + cornerOuterRadius;
    auto innerDistance = lineLength / 2.0 + cornerOuterRadius - width;
    auto leftUpCenterPoint = Point3D::create(leftCenterPoint->x(), leftCenterPoint->y(), height - smallShift);
    auto rightUpCenterPoint = Point3D::create(rightCenterPoint->x(), rightCenterPoint->y(), height - smallShift);
    
    for (int i = 0; i < feature->bodies()->count(); i++)
    {
        auto body = feature->bodies()->item(i);
        if (body->pointContainment(Point3D::create(0, 0, height - smallShift)) == PointInsidePointContainment)
        {
            body->name("CenterRoofBody");
        }
        else if (body->pointContainment(GetCirclePoint(leftUpCenterPoint, outerDistance - smallShift, RAD_90 + RAD_45, true)) == PointInsidePointContainment)
        {
            body->name("MainRoofBody");
        }
        else if (body->pointContainment(GetCirclePoint(leftUpCenterPoint, innerDistance + smallShift, RAD_90 + RAD_45, true)) == PointInsidePointContainment)
        {
            body->name("LeftSideRoofBody");
        }
        else if (body->pointContainment(GetCirclePoint(rightUpCenterPoint, innerDistance + smallShift, RAD_45, true)) == PointInsidePointContainment)
        {
            body->name("RightSideRoofBody");
        }
        result->add(body);
        filletBody(component, body);
    }

    return result;
}
bool RoofPart::edgeOnInnerCorner(Ptr<BRepEdge> edge)
{
    auto body = edge->body();
    auto point = edge->endVertex()->geometry();
    auto upPoint = Point3D::create(point->x(), point->y(), point->z() + floorThickness / 2.0);
    auto downPoint = Point3D::create(point->x(), point->y(), point->z() - (height - floorThickness) / 2.0);
    return EdgeIsHorizontal(edge) && Equal(point->z(), height - floorThickness, 0.01) && BodyContainPoint(body, upPoint) && BodyContainPoint(body, downPoint);
}

void RoofPart::filletBody(Ptr<Component> component, Ptr<BRepBody> body)
{
    auto edges = GetEdges(body, [=](Ptr<BRepEdge> edge) {return EdgeIsVerticalLine(edge) && abs(edge->endVertex()->geometry()->x()) <= 0.01 && abs(edge->endVertex()->geometry()->y()) < innerWidth + outerWidth; });
    auto minY = GetMin(edges, [=](Ptr<BRepEdge> edge) {return abs(edge->endVertex()->geometry()->y()); });
    edges = GetEdges(edges, [=](Ptr<BRepEdge> edge) {return abs(edge->endVertex()->geometry()->y()) < minY + 0.01; });
    Fillet(component, edges, verticalEdgeFilletRadius / 2.0);

    edges = GetEdges(body, EdgeIsVerticalLine);
    Fillet(component, edges, verticalEdgeFilletRadius);

    edges = GetEdges(body, [=](Ptr<BRepEdge> edge) {return edgeOnInnerCorner(edge); });
    Fillet(component, edges, otherEdgeFilletRadius);

    edges = GetEdges(body, [=](Ptr<BRepEdge> edge) {return EdgeIsHorizontal(edge) && Equal(edge->length(), GetCircleLength(circlesOnSquareRadius), 0.1); });
    Fillet(component, edges, floorThickness / 3.0);

    edges = GetEdges(body, [=](Ptr<BRepEdge> edge) {return EdgeIsHorizontal(edge) && edge->endVertex()->geometry()->z() > floorThickness * 1.1; });
    Fillet(component, edges, topEdgeFilletRadius);

    edges = GetEdges(body, EdgeIsHorizontal);
    Fillet(component, edges, otherEdgeFilletRadius);
}
#include "RectangledBasePart.h"

Ptr<BRepBody> RectangledBasePart::createBody(Ptr<Component> component)
{
    auto cornerOuterRadius = cornerMiddleRadius + outerWidth;
    auto width = outerWidth + innerWidth;

    auto body = createPairedSquares(component, lineLength, cornerOuterRadius, RAD_45, width, height);

    auto outerBox = body->boundingBox();
    auto box = CutShell(body->boundingBox(), cuttingShellThickness);

    auto cutWayBody = createPairedSquares(component, lineLength, cornerOuterRadius - wallThickness, RAD_45, width - 2.0 * wallThickness, height);
    cutWayBody = Move(component, cutWayBody, component->zConstructionAxis(), floorThickness);
    auto cutShellBody = CreateBox(component, outerBox->minPoint(), outerBox->maxPoint(), height, 0, width - wallThickness);
    cutWayBody = Combine(component, JoinFeatureOperation, cutWayBody, cutShellBody);
    body = Combine(component, CutFeatureOperation, body, cutWayBody);
    
    auto floorBody = CreateBox(component, box->minPoint(), box->maxPoint(), floorThickness, cornerFilletRadius);
    body = Combine(component, JoinFeatureOperation, body, floorBody);

    auto intersectBody = CreateBox(component, box->minPoint(), box->maxPoint(), height);
    body = Combine(component, IntersectFeatureOperation, body, intersectBody);

    auto createHalForMagnet = false;
    if (createHalForMagnet)
    {
        auto magnetSketch = createCirclesSketch(component, circlesOnSquareRadius, 0);

        for (int i = 0; i < magnetSketch->profiles()->count(); i++)
        {
            auto magnetBody = Extrude(component, magnetSketch->profiles()->item(i), floorThickness * 3.0, false);
            body = Combine(component, CutFeatureOperation, body, magnetBody->bodies()->item(0));
        }
    }

    if (!isPapaCenterPart)
    {
        PariedSquaresPart innerWallPart;

        innerWallPart.lineLength = lineLength;
        innerWallPart.cornerOuterRadius = cornerOuterRadius - width + wallThickness * 2.0;
        innerWallPart.rotateAngel = RAD_45;
        innerWallPart.height = height;
        innerWallPart.thickness = wallThickness;
        innerWallPart.leftCenterPoint = leftCenterPoint;
        innerWallPart.rightCenterPoint = rightCenterPoint;
        innerWallPart.initialize(component);

        auto centerJoinBody = innerWallPart.createInnerWallBody();
        auto centerBox = centerJoinBody->boundingBox();
        auto centerCutBody = CreateBox(component, centerBox->minPoint(), centerBox->maxPoint(), height);
        centerCutBody = Move(component, centerCutBody, component->zConstructionAxis(), floorThickness);
        body = Combine(component, CutFeatureOperation, body, centerCutBody);
        body = Combine(component, JoinFeatureOperation, body, centerJoinBody);
    }

    auto shift = cornerFilletRadius - cornerFilletRadius / sqrt(2.0) + linkingPart.radius / sqrt(2.0) + 0.03;
    std::vector<Ptr<Point3D>> points;
    auto top = box->maxPoint()->y();
    auto right = box->maxPoint()->x();
    auto left = box->minPoint()->x();
    auto down = box->minPoint()->y();
    points.push_back(Point3D::create(right - shift, top - shift));
    points.push_back(Point3D::create(right - shift, down + shift));
    points.push_back(Point3D::create(left + shift, top - shift));
    points.push_back(Point3D::create(left + shift, down + shift));
    for (auto point : points)
    {
        linkingPart.center = point;
        linkingPart.joinedBody = body;
        body = linkingPart.createBody(component);
    }

    auto upCentralLinkerBody = CreateCylinder(component, Point3D::create(0, top - centralLinkerRadius - wallThickness / 3.0), centralLinkerRadius, height);
    body = Combine(component, JoinFeatureOperation, body, upCentralLinkerBody);
    auto downCentralLinkerBody = CreateCylinder(component, Point3D::create(0, down + centralLinkerRadius + wallThickness / 3.0), centralLinkerRadius, height);
    body = Combine(component, JoinFeatureOperation, body, downCentralLinkerBody);

    filletBody(component, body);

    return body;
}

void RectangledBasePart::filletBody(Ptr<Component> component, Ptr<BRepBody> body)
{
    auto edges = GetEdges(body, [=](Ptr<BRepEdge> edge) {return EdgeIsVerticalLine(edge) && abs(edge->endVertex()->geometry()->x()) <= 0.01 && abs(edge->endVertex()->geometry()->y()) < innerWidth + outerWidth; });
    auto minY = GetMin(edges, [=](Ptr<BRepEdge> edge) {return abs(edge->endVertex()->geometry()->y()); });
    edges = GetEdges(edges, [=](Ptr<BRepEdge> edge) {return abs(edge->endVertex()->geometry()->y()) < minY + 0.01; });
    Fillet(component, edges, verticalEdgeFilletRadius / 2.0);

    edges = GetEdges(body, EdgeIsVerticalLine);
    Fillet(component, edges, verticalEdgeFilletRadius);

    edges = GetEdges(body, [=](Ptr<BRepEdge> edge) {return EdgeIsHorizontal(edge) && Equal(edge->length(), GetCircleLength(linkingPart.radius), 0.02) && edge->endVertex()->geometry()->z() > floorThickness * 1.1; });
    Fillet(component, edges, topEdgeFilletRadius * 2.0);

    edges = GetEdges(body, [=](Ptr<BRepEdge> edge) {return EdgeIsHorizontal(edge) && edge->endVertex()->geometry()->z() > floorThickness * 1.1; });
    Fillet(component, edges, topEdgeFilletRadius);

    edges = GetEdges(body, [=](Ptr<BRepEdge> edge) {return EdgeIsHorizontal(edge) && Equal(edge->length(), GetCircleLength(circlesOnSquareRadius), 0.1); });
    Fillet(component, edges, floorThickness / 3.0);

    edges = GetEdges(body, EdgeIsHorizontal);
    Fillet(component, edges, otherEdgeFilletRadius);
}
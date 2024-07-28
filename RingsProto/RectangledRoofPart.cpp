#include "RectangledRoofPart.h"

Ptr<ObjectCollection> RectangledRoofPart::createBodies(Ptr<Component> component)
{
    auto cornerOuterRadius = cornerMiddleRadius + outerWidth;
    auto width = outerWidth + innerWidth;
    auto separationCornerOuterRadius = cornerMiddleRadius + separationOuterWidth;
    auto separationWidth = separationOuterWidth + separationInnerWidth;

    auto body = createPairedSquares(component, lineLength, cornerOuterRadius, RAD_45, width, height);
    auto box = body->boundingBox();
    auto boundWallBody = CreateBox(component, box->minPoint(), box->maxPoint(), height, cornerFilletRadius, wallThickness);
    auto roofBody = CreateBox(component, box->minPoint(), box->maxPoint(), floorThickness, cornerFilletRadius);
    roofBody = Move(component, roofBody, component->zConstructionAxis(), height - floorThickness);
    body = Combine(component, JoinFeatureOperation, body, boundWallBody);
    body = Combine(component, JoinFeatureOperation, body, roofBody);
    auto centerUpBound = CreateBound(box->maxPoint()->y(), box->maxPoint()->y() - width, leftCenterPoint->x(), rightCenterPoint->x());
    auto centerDownBound = CreateBound(box->minPoint()->y(), box->minPoint()->y() + width, leftCenterPoint->x(), rightCenterPoint->x());
    auto centerUpBody = CreateBox(component, centerUpBound->minPoint(), centerUpBound->maxPoint(), height);
    auto centerDownBody = CreateBox(component, centerDownBound->minPoint(), centerDownBound->maxPoint(), height);
    body = Combine(component, JoinFeatureOperation, body, centerUpBody);
    body = Combine(component, JoinFeatureOperation, body, centerDownBody);

    auto cutBody = createPairedSquares(component, lineLength, cornerOuterRadius - wallThickness, RAD_45, width - 2.0 * wallThickness, height - floorThickness);
    auto separationCutBody = createPairedSquares(component, lineLength, separationCornerOuterRadius, RAD_45, separationWidth, height);
    cutBody = Combine(component, JoinFeatureOperation, cutBody, separationCutBody);
    auto substrateCutBody = CreateCylinder(component, Point3D::create(), 2.0 * (lineLength + cornerOuterRadius * 2.0), downTrimmingThicknes);
    cutBody = Combine(component, JoinFeatureOperation, cutBody, substrateCutBody);
    auto deepBox = CutShell(box, wallThickness);
    auto deepCutBody = CreateBox(component, deepBox->minPoint(), deepBox->maxPoint(), deepThickness, cornerFilletRadius - wallThickness);
    cutBody = Combine(component, JoinFeatureOperation, cutBody, deepCutBody);
    auto feature = CreateCombineFeature(component, CutFeatureOperation, body, cutBody);

    auto result = ObjectCollection::create();
    auto smallShift = 0.1;
    auto outerDistance = lineLength / 2.0 + cornerOuterRadius;
    auto innerDistance = lineLength / 2.0 + cornerOuterRadius - width;
    auto leftUpCenterPoint = Point3D::create(leftCenterPoint->x(), leftCenterPoint->y(), height - smallShift);
    auto rightUpCenterPoint = Point3D::create(rightCenterPoint->x(), rightCenterPoint->y(), height - smallShift);

    auto mainBody = body;
    auto centerBody = body;
    for (int i = 0; i < feature->bodies()->count(); i++)
    {
        auto body = feature->bodies()->item(i);
        if (body->pointContainment(Point3D::create(0, 0, height - smallShift)) == PointInsidePointContainment)
        {
            body->name("CenterRoofBody");
            centerBody = body;
        }
        else if (body->pointContainment(GetCirclePoint(leftUpCenterPoint, outerDistance - smallShift, RAD_90 + RAD_45, true)) == PointInsidePointContainment)
        {
            body->name("MainRoofBody");
            mainBody = body;
        }
        else if (body->pointContainment(GetCirclePoint(leftUpCenterPoint, innerDistance + smallShift, RAD_90 + RAD_45, true)) == PointInsidePointContainment)
        {
            body->name("LeftSideRoofBody");
            result->add(body);
        }
        else if (body->pointContainment(GetCirclePoint(rightUpCenterPoint, innerDistance + smallShift, RAD_45, true)) == PointInsidePointContainment)
        {
            body->name("RightSideRoofBody");
            result->add(body);
        }
    }

    if (isPapaCenterPart)
    {
        PariedSquaresPart innerWallPart;

        innerWallPart.lineLength = lineLength;
        innerWallPart.cornerOuterRadius = cornerOuterRadius - width + wallThickness;
        innerWallPart.rotateAngel = RAD_45;
        innerWallPart.height = height - deepThickness;
        innerWallPart.thickness = wallThickness;
        innerWallPart.leftCenterPoint = leftCenterPoint;
        innerWallPart.rightCenterPoint = rightCenterPoint;
        innerWallPart.initialize(component);

        auto centerBox = centerBody->boundingBox();
        auto centerCutBody = CreateBox(component, centerBox->minPoint(), centerBox->maxPoint(), height - deepThickness);
        centerBody = Combine(component, CutFeatureOperation, centerBody, centerCutBody);
        auto centerJoinBody = innerWallPart.createCenterBody();
        centerJoinBody = Move(component, centerJoinBody, component->zConstructionAxis(), deepThickness);
        centerBody = Combine(component, JoinFeatureOperation, centerBody, centerJoinBody);
    }

    mainBody = addLinkersToMainBody(component, mainBody);
    mainBody->name("MainRoofBody");
    result->add(mainBody);
    
    result->add(centerBody);

    for (auto b : result)
        filletBody(component, b);

    return result;
}

Ptr<BRepBody> RectangledRoofPart::addLinkersToMainBody(Ptr<Component> component, Ptr<BRepBody>& body)
{
    auto box = body->boundingBox();
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
    auto centralLinkerShift = centralLinkerRadius + wallThickness + wallThickness / 3.0 - 0.01;
    auto upCentralLinkerBody = CreateCylinder(component, Point3D::create(0, top - centralLinkerShift), centralLinkerRadius, height - floorThickness);
    body = Combine(component, CutFeatureOperation, body, upCentralLinkerBody);
    auto downCentralLinkerBody = CreateCylinder(component, Point3D::create(0, down + centralLinkerShift), centralLinkerRadius, height - floorThickness);
    body = Combine(component, CutFeatureOperation, body, downCentralLinkerBody);

    return body;
}

double RectangledRoofPart::getTop()
{
    auto cornerOuterRadius = cornerMiddleRadius + outerWidth;
    return lineLength / sqrt(2.0) + cornerOuterRadius;
}

double RectangledRoofPart::getRight()
{
    return rightCenterPoint->x() + getTop();
}

bool RectangledRoofPart::isEedgeOnOuterRectangle(Ptr<BRepEdge> edge)
{
    auto point1 = edge->endVertex()->geometry();
    auto point2 = edge->startVertex()->geometry();
    auto delta = 0.02;
    return EdgeIsHorizontal(edge) &&
        (Equal(abs(point1->x()), getRight(), delta) || Equal(abs(point1->y()), getTop(), delta)) &&
        (Equal(abs(point2->x()), getRight(), delta) || Equal(abs(point2->y()), getTop(), delta));
}

void RectangledRoofPart::filletBody(Ptr<Component> component, Ptr<BRepBody> body)
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

    edges = GetEdges(body, [=](Ptr<BRepEdge> edge) {return isEedgeOnOuterRectangle(edge) && edge->endVertex()->geometry()->z() > floorThickness * 1.1; });
    Fillet(component, edges, topEdgeFilletRadius * 2.5);

    edges = GetEdges(body, [=](Ptr<BRepEdge> edge) {return EdgeIsHorizontal(edge) && edge->endVertex()->geometry()->z() > floorThickness * 1.1; });
    Fillet(component, edges, topEdgeFilletRadius);

    edges = GetEdges(body, EdgeIsHorizontal);
    Fillet(component, edges, otherEdgeFilletRadius);
}
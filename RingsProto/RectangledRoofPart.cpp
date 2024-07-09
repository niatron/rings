#include "RectangledRoofPart.h"

Ptr<ObjectCollection> RectangledRoofPart::createBodies(Ptr<Component> component)
{
    auto cornerOuterRadius = cornerMiddleRadius + outerWidth;
    auto width = outerWidth + innerWidth;
    auto separationCornerOuterRadius = cornerMiddleRadius + separationOuterWidth;
    auto separationWidth = separationOuterWidth + separationInnerWidth;

    auto body = createPairedSquares(component, lineLength, cornerOuterRadius, RAD_45, width, height);
    auto box = body->boundingBox();
    auto boxBody = CreateBox(component, box->minPoint(), box->maxPoint(), height);
    auto edges = GetEdges(boxBody, EdgeIsVerticalLine);
    Fillet(component, edges, cornerFilletRadius);
    body = Combine(component, JoinFeatureOperation, body, boxBody);

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
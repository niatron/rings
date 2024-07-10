#include "RectangledBasePart.h"

Ptr<BRepBody> RectangledBasePart::createBody(Ptr<Component> component)
{
    auto cornerOuterRadius = cornerMiddleRadius + outerWidth;
    auto width = outerWidth + innerWidth;
    auto top = lineLength / sqrt(2.0) + cornerOuterRadius;

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

    auto magnetSketch = createCirclesSketch(component, circlesOnSquareRadius, 0);

    for (int i = 0; i < magnetSketch->profiles()->count(); i++)
    {
        auto magnetBody = Extrude(component, magnetSketch->profiles()->item(i), floorThickness * 3.0, false);
        body = Combine(component, CutFeatureOperation, body, magnetBody->bodies()->item(0));
    }
    

    //filletBody(component, body);

    return body;
}
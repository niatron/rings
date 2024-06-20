#include "VolfDownPart.h"

Ptr<BRepBody> VolfDownPart::createBody(Ptr<Component> component)
{
    auto body = CreateCylinder(component, centerPoint, radius, height);

    if (middleRadius > 0 && middleHeight > 0)
    {
        auto middleBody = CreateCylinder(component, centerPoint, middleRadius, middleHeight);
        middleBody = Move(component, middleBody, component->zConstructionAxis(), height);
        body = Combine(component, JoinFeatureOperation, body, middleBody);
    }

    if (holeRadius > 0 && holeHeight > 0)
    {
        auto holeBody = CreateCylinder(component, centerPoint, holeRadius, holeHeight);
        body = Combine(component, CutFeatureOperation, body, holeBody);
    }

    if (holeDownRadius > 0 && holeDownHeight > 0)
    {
        auto holeBody = CreateCylinder(component, centerPoint, holeDownRadius, holeDownHeight);
        body = Combine(component, CutFeatureOperation, body, holeBody);
    }

    if (holeUpRadius > 0 && holeUpHeight > 0)
    {
        auto holeBody = CreateCylinder(component, centerPoint, holeUpRadius, holeUpHeight);
        holeBody = Move(component, holeBody, component->zConstructionAxis(), height + middleHeight - holeUpHeight);
        body = Combine(component, CutFeatureOperation, body, holeBody);
    }
    body = Move(component, body, component->zConstructionAxis(), zMoveShift);
    return body;
}
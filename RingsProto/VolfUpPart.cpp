#include "VolfUpPart.h"

Ptr<BRepBody> VolfUpPart::createBody(Ptr<Component> component)
{
    auto body = CreateCylinder(component, centerPoint, radius, height);

    if (middleRadius > 0 && middleHeight > 0)
    {
        auto middleBody = CreateCylinder(component, centerPoint, middleRadius, middleHeight);
        body = Move(component, body, component->zConstructionAxis(), middleHeight);
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

    if (cuttedSphreRadius > 0 && cuttedSphreCuttingHeight > 0)
    {
        auto sphereBody = CreateSphere(component, Point3D::create(centerPoint->x(), centerPoint->y(), height + middleHeight + cuttedSphreRadius - cuttedSphreCuttingHeight), cuttedSphreRadius);
        body = Combine(component, CutFeatureOperation, body, sphereBody);
    }

    body = Move(component, body, component->zConstructionAxis(), zMoveShift - middleHeight);
    return body;
}
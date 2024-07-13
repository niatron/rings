#pragma once
#include "RoofPart.h"
#include "LinkingPart.h"

class RectangledRoofPart : public RoofPart
{
public:
    double deepThickness;
    double cornerFilletRadius;
    double centralLinkerRadius;
    LinkingPart linkingPart;
    Ptr<ObjectCollection> createBodies(Ptr<Component> component);
private:
    Ptr<BRepBody> createBody(Ptr<Component> component);
    Ptr<BRepBody> addLinkersToMainBody(Ptr<Component> component, Ptr<BRepBody>& body);
};
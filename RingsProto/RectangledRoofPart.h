#pragma once
#include "RoofPart.h"

class RectangledRoofPart : public RoofPart
{
public:
    double deepThickness;
    double cornerFilletRadius;
    Ptr<ObjectCollection> createBodies(Ptr<Component> component);
private:
    Ptr<BRepBody> createBody(Ptr<Component> component);

};
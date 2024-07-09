#pragma once
#include "RoofPart.h"

class RectangledRoofPart : public RoofPart
{
public:
    double cornerFilletRadius = 1.0;
    Ptr<ObjectCollection> createBodies(Ptr<Component> component);
private:
    Ptr<BRepBody> createBody(Ptr<Component> component);

};
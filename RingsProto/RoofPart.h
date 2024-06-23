#pragma once
#include "BasePart.h"

class RoofPart : public BasePart
{
public:
    double separationInnerWidth;
    double separationOuterWidth;
    double downTrimmingThicknes;
    Ptr<ObjectCollection> createBodies(Ptr<Component> component);
    bool edgeOnInnerCorner(Ptr<BRepEdge> edge);
    void filletBody(Ptr<Component> component, Ptr<BRepBody> body);
private:
    Ptr<BRepBody> createBody(Ptr<Component> component);
    
};
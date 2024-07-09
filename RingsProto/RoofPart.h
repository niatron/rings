#pragma once
#include "BasePart.h"

class RoofPart : public BasePart
{
public:
    double separationInnerWidth;
    double separationOuterWidth;
    double downTrimmingThicknes;
    Ptr<ObjectCollection> createBodies(Ptr<Component> component);
protected:
    bool edgeOnInnerCorner(Ptr<BRepEdge> edge);
    void filletBody(Ptr<Component> component, Ptr<BRepBody> body);
    Ptr<BRepBody> createBody(Ptr<Component> component);
    
};
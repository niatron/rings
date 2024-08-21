#pragma once
#include "RoofPart.h"
#include "LinkingPart.h"
#include "PariedSquaresPart.h"

class RectangledRoofPart : public RoofPart
{
public:
    double deepThickness;
    double cornerFilletRadius;
    double centralLinkerRadius;
    bool isPapaCenterPart = true;
    LinkingPart linkingPart;
    Ptr<ObjectCollection> createBodies(Ptr<Component> component, std::vector<Ptr<Point3D>> linkerPoints);
protected:
    Ptr<BRepBody> createBody(Ptr<Component> component);
    Ptr<BRepBody> addLinkersToMainBody(Ptr<Component> component, Ptr<BRepBody>& body, std::vector<Ptr<Point3D>> linkerPoints);
    void filletBody(Ptr<Component> component, Ptr<BRepBody> body);
    bool isEedgeOnOuterRectangle(Ptr<BRepEdge> edge);
    double getTop();
    double getRight();
};
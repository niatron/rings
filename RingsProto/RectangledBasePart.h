#pragma once
#include "BasePart.h"
#include "LinkingPart.h"
#include "PariedSquaresPart.h"

class RectangledBasePart : public BasePart
{
public:
    double cornerFilletRadius;
    double cuttingShellThickness;
    double centralLinkerRadius;
    bool isPapaCenterPart = false;
    LinkingPart linkingPart;

    Ptr<BRepBody> createBody(Ptr<Component> component);
protected:
    void filletBody(Ptr<Component> component, Ptr<BRepBody> body);
};
#pragma once
#include "BasePart.h"
#include "LinkingPart.h"

class RectangledBasePart : public BasePart
{
public:
    double cornerFilletRadius;
    double cuttingShellThickness;
    double centralLinkerRadius;
    LinkingPart linkingPart;

    Ptr<BRepBody> createBody(Ptr<Component> component);
};
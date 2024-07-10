#pragma once
#include "BasePart.h"

class RectangledBasePart : public BasePart
{
public:
    double cornerFilletRadius;
    double cuttingShellThickness;
    Ptr<BRepBody> createBody(Ptr<Component> component);

};
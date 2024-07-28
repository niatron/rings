#pragma once
#include "Sketcher.h"
#include "PariedSquaresPart.h"


class PariedSquaresWithOuterRectanglePart : public PariedSquaresPart
{
public:
    double rectangleWidth;
    double rectangleHeight;
    double rectangleCornerRadius;

    void initialize(Ptr<Component> component)
    {
        PariedSquaresPart::initialize(component);
        AddRectangle(sketch, Point3D::create(), rectangleWidth, rectangleHeight, rectangleCornerRadius);
    }

    bool isRectangleProfile(Ptr<Profile> profile)
    {
        return !isCenterProfile(profile) && !isInnerWallProfile(profile) && !isOuterWallProfile(profile) && !isLeftCenterProfile(profile) && !isRightCenterProfile(profile);
    }

    Ptr<BRepBody> createRectangleBody()
    {
        auto profiles = GetProfiles(sketch, [=](Ptr<Profile> profile) {return isRectangleProfile(profile); });
        return Extrude(component, profiles, height)->bodies()->item(0);
    }
};
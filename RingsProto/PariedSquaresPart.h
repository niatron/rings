#pragma once
#include "Sketcher.h"


class PariedSquaresPart
{
private:
    Ptr<Component> component;
    Ptr<Sketch> sketch;
    Ptr<Profile> centralUpProfile;
public:
    Ptr<Point3D> leftCenterPoint;
    Ptr<Point3D> rightCenterPoint;
    double lineLength;
    double cornerOuterRadius;
    double rotateAngel;
    double thickness;
    double height;
    
    void initialize(Ptr<Component> component)
    {
        this->component = component;
        sketch = CreateSketch(component, component->xYConstructionPlane(), "PariedSquaresSketch");
        Sketcher::AddSquareCurves(sketch, leftCenterPoint, lineLength, cornerOuterRadius, rotateAngel);
        Sketcher::AddSquareCurves(sketch, leftCenterPoint, lineLength, cornerOuterRadius - thickness, rotateAngel);
        Sketcher::AddSquareCurves(sketch, rightCenterPoint, lineLength, cornerOuterRadius, rotateAngel);
        Sketcher::AddSquareCurves(sketch, rightCenterPoint, lineLength, cornerOuterRadius - thickness, rotateAngel);

        centralUpProfile = GetProfiles(sketch, [=](Ptr<Profile> profile) {return profile->face()->centroid()->y() > 0.01; })->item(0);
    }

    bool isCenterProfile(Ptr<Profile> profile)
    {
        return Equal(profile->face()->centroid(), Point3D::create(), 0.01);
    }
    bool isInnerWallProfile(Ptr<Profile> profile)
    {
        auto projectionLength = lineLength / sqrt(2.0) + cornerOuterRadius;
        auto box = profile->face()->boundingBox();
        return !isCenterProfile(profile) && abs(box->maxPoint()->y()) < projectionLength - thickness - 0.01;
    }
    bool isOuterWallProfile(Ptr<Profile> profile)
    {
        auto projectionLength = lineLength / sqrt(2.0) + cornerOuterRadius;
        auto centroid = profile->face()->centroid();
        auto box = profile->face()->boundingBox();
        return !isCenterProfile(profile) &&
            (!Equal(centroid->y(), 0, 0.01) || abs(box->maxPoint()->y()) > projectionLength - 0.01);
    }
    bool isLeftCenterProfile(Ptr<Profile> profile)
    {
        auto centroid = profile->face()->centroid();
        return !isCenterProfile(profile) && !isInnerWallProfile(profile) && !isOuterWallProfile(profile) && centroid->x() < 0;
    }
    bool isRightCenterProfile(Ptr<Profile> profile)
    {
        auto centroid = profile->face()->centroid();
        return !isCenterProfile(profile) && !isInnerWallProfile(profile) && !isOuterWallProfile(profile) && centroid->x() > 0;
    }

    Ptr<BRepBody> createCenterBody()
    {
        auto profiles = GetProfiles(sketch, [=](Ptr<Profile> profile) {return isCenterProfile(profile); });
        return Extrude(component, profiles, height);
    }
    Ptr<BRepBody> createInnerWallBody()
    {
        auto profiles = GetProfiles(sketch, [=](Ptr<Profile> profile) {return isInnerWallProfile(profile); });
        return Extrude(component, profiles, height);
    }
    Ptr<BRepBody> createOuterWallBody()
    {
        auto profiles = GetProfiles(sketch, [=](Ptr<Profile> profile) {return isOuterWallProfile(profile); });
        return Extrude(component, profiles, height);
    }
    Ptr<BRepBody> createLeftCenterBody()
    {
        auto profiles = GetProfiles(sketch, [=](Ptr<Profile> profile) {return isLeftCenterProfile(profile); });
        return Extrude(component, profiles, height);
    }
    Ptr<BRepBody> createRightCenterBody()
    {
        auto profiles = GetProfiles(sketch, [=](Ptr<Profile> profile) {return isRightCenterProfile(profile); });
        return Extrude(component, profiles, height);
    }
};
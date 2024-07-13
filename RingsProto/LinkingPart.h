#pragma once
#include "FusionEnvironment.h"

class LinkingPart
{
public:
    enum FloorStates { Top, Down };
    
    Ptr<Point3D> center;
    double radius;
    double height;
    double z;
    double wallThickness;
    double floorThickness;
    double floorHoleRadius;
    FloorStates floorState = Top;
    bool isReverse = false;
    Ptr<BRepBody> joinedBody = nullptr;
  
    Ptr<BRepBody> createBody(Ptr<Component> component)
    {
        auto direction = isReverse ? -1 : 1;
        
        auto body = createCylinder(component, center, radius, height, z);
        if (joinedBody != nullptr)
            body = Combine(component, JoinFeatureOperation, body, joinedBody);
        auto cutBody = createCylinder(component, center, radius - wallThickness, height - floorThickness, z + direction * (floorState == Top ? 0 : floorThickness));
        body = Combine(component, CutFeatureOperation, body, cutBody);
        if (floorHoleRadius > 0 && floorThickness > 0)
        {
            
            auto cutFloorBody = createCylinder(component, center, floorHoleRadius, floorThickness, z + direction * (floorState == Top ? height - floorThickness : 0));
            body = Combine(component, CutFeatureOperation, body, cutFloorBody);
        }
        return body;
    }
private:
    Ptr<BRepBody> createCylinder(Ptr<Component> component, Ptr<Point3D> center, double radius, double height, double z)
    {
        auto body = CreateCylinder(component, center, radius, height);
        if (!isReverse && z != 0)
            body = Move(component, body, component->zConstructionAxis(), z);
        else if (isReverse && z - height != 0)
            body = Move(component, body, component->zConstructionAxis(), z - height);
        return body;
    }
};
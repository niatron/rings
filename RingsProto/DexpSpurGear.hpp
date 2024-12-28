#pragma once
#include "SpurGear.hpp"

void createBody(Ptr<Design> design)
{
    double printerHorizontalDelta = 0.04;
    double diametralPitch = 24.8;
    int numTeeth = 66;
    double thickness = 0.84;
    double rootFilletRad = 0;
    double pressureAngle = 0.4;
    double backlash = 0.0;
    double holeDiam = 0;
    double twist = 2 * M_PI / 66 / 2.1;

    double ribsCount = 6;

    double towerRadius = 0.90 + printerHorizontalDelta;
    double towerHeight = 4.88;
    double towerWallThickness = 0.25;
    double towerShift = 0.9;
    double towerRibs6Shift = sin(RAD_360 / ribsCount) * towerRadius + 0.03;
    double towerRibs6Bottom = 2.5;
    double towerMatizRadius = 0.08 + printerHorizontalDelta;

    double ringRadius = 1.16;
    double ringHeight = 0.96;

    
    auto component = drawGear(design, diametralPitch, numTeeth, thickness, rootFilletRad, pressureAngle, backlash, holeDiam, twist, design->rootComponent());
    auto zAxis = component->zConstructionAxis();

    auto body = component->bRepBodies()->item(0);

    auto towerBody = CreateCylinder(component, Point3D::create(), towerRadius, towerHeight);
    towerBody = Move(component, towerBody, zAxis, -towerShift);
    body = Combine(component, JoinFeatureOperation, body, towerBody);

    auto ringBody = CreateCylinder(component, Point3D::create(), ringRadius, ringHeight);
    ringBody = Move(component, ringBody, zAxis, -(ringHeight - thickness) / 2);
    body = Combine(component, JoinFeatureOperation, body, ringBody);

    auto ribs6CutBody = CreateBox(component, Point3D::create(towerRibs6Shift, towerRadius), Point3D::create(towerRibs6Shift + towerRadius, -towerRadius), towerHeight);
    Move(component, ribs6CutBody, zAxis, towerRibs6Bottom);
    
    for (int i = 1; i <= ribsCount; i++)
    {
        auto cutBody = Rotate(component, ribs6CutBody, zAxis, i * RAD_360 / ribsCount, i < ribsCount);
        body = Combine(component, CutFeatureOperation, body, cutBody);
    }

    auto centerCutBody = CreateCylinder(component, Point3D::create(), towerRadius - towerWallThickness, towerHeight - towerWallThickness);
    centerCutBody = Move(component, centerCutBody, zAxis, -towerShift);
    body = Combine(component, CutFeatureOperation, body, centerCutBody);

    auto metizCutBody = CreateCylinder(component, Point3D::create(), towerMatizRadius, towerHeight);
    body = Combine(component, CutFeatureOperation, body, metizCutBody);
}
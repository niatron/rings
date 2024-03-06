#pragma once
#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>
//#include <CAM/CAMAll.h>
#include <functional>

using namespace adsk::core;
using namespace adsk::fusion;
//using namespace adsk::cam;

Ptr<Point3D> GetCenterPoint();
Ptr<Point3D> GetCirclePoint(double radius, double angel);
Ptr<SketchArc> AddArc(Ptr<Sketch> sketch, Ptr<Point3D> circleCentr, double radius, double length, double angel, bool angelIsCenterOfArc = true);
Ptr<RevolveFeature> RevolveSketch(Ptr<Component> component, Ptr<Sketch> sketch, Ptr<ConstructionAxis> axis, double angelRad);
Ptr<ExtrudeFeature> ExtrudeSketch(Ptr<Component> component, Ptr<Sketch> sketch, double distance, bool isSymetric = false);
Ptr<BRepBody> Rotate(Ptr<Component> component, Ptr<BRepBody> body, Ptr<ConstructionAxis> axis, double angel, bool createCopy = false);
Ptr<BRepBody> Combine(Ptr<Component> component, FeatureOperations operation, Ptr<BRepBody> body1, Ptr<BRepBody> body2);
Ptr<FilletFeature> Fillet(Ptr<Component> component, Ptr<ObjectCollection> edges, double val);
Ptr<ObjectCollection> GetEdges(Ptr<BRepBody> body, std::function <bool(Ptr<BRepEdge>)> isGoodEdge);
Ptr<Vector3D> ConstructionAxisToVector3D(Ptr<ConstructionAxis> axis);
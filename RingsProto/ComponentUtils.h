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
bool Equal(double a, double b, double delta = 0.001);
bool Equal(Ptr<Point3D> p1, Ptr<Point3D> p2, double delta = 0.001);
bool Equal(Ptr<SketchLine> line, Ptr<BRepEdge> edge, double delta = 0.001);
bool EqualAny(Ptr<BRepEdge> edge, Ptr<SketchLines> lines, double delta = 0.001);
void MessageBox(std::string message);

Ptr<ConstructionPoint> AddConstructionPoint(Ptr<Component> component, Ptr<Base> point);
Ptr<ConstructionAxis> AddConstructionAxis(Ptr<Component> component, Ptr<Point3D> point, Ptr<Vector3D> vector);
Ptr<ConstructionAxis> AddConstructionAxis(Ptr<Component> component, Ptr<Vector3D> vector);

Ptr<SketchArc> AddArc(Ptr<Sketch> sketch, Ptr<Point3D> circleCentr, double radius, double length, double angel, bool angelIsCenterOfArc = true);
Ptr<SketchLine> AddLine(Ptr<Sketch> sketch, Ptr<Base> startPoint, Ptr<Base> endPoint);
Ptr<SketchLine> AddLine(Ptr<Sketch> sketch, double startPointX, double startPointY, double startPointZ, double endPointX, double endPointY, double endPointZ);

Ptr<RevolveFeature> Revolve(Ptr<Component> component, Ptr<Profile> profile, Ptr<ConstructionAxis> axis, double angelRad);
Ptr<RevolveFeature> Revolve(Ptr<Component> component, Ptr<Sketch> sketch, Ptr<ConstructionAxis> axis, double angelRad);
Ptr<ExtrudeFeature> Extrude(Ptr<Component> component, Ptr<Profile> profile, double distance, bool isSymetric = false);
Ptr<ExtrudeFeature> Extrude(Ptr<Component> component, Ptr<Sketch> sketch, double distance, bool isSymetric = false);

Ptr<BRepBody> Rotate(Ptr<Component> component, Ptr<BRepBody> body, Ptr<ConstructionAxis> axis, double angel, bool createCopy = false);
Ptr<BRepBody> Combine(Ptr<Component> component, FeatureOperations operation, Ptr<BRepBody> body1, Ptr<BRepBody> body2);
Ptr<FilletFeature> Fillet(Ptr<Component> component, Ptr<ObjectCollection> edges, double val);

Ptr<ObjectCollection> GetEdges(Ptr<BRepBody> body, std::function <bool(Ptr<BRepEdge>)> isGoodEdge);

Ptr<Vector3D> ConstructionAxisToVector3D(Ptr<ConstructionAxis> axis);

//template <typename T> std::vector<T> Where(std::vector<T> collection, std::function <bool(T)> isGoodItem);
//template <class T> std::vector<Ptr<T>> ToVector(Ptr<ObjectCollection> collection);
template <typename T> std::vector<T> Where(std::vector<T> collection, std::function <bool(T)> isGoodItem)
{
    std::vector<T> result;
    for (int i = 0; i < collection.size(); i++)
        if (isGoodItem(collection[i]))
            result.push_back(collection[i]);
    return result;
}

template <class T, class C> std::vector<Ptr<T>> ToVector(C collection) 
{
    std::vector<Ptr<T>> result;
    
    for (int i = 0; i < collection->count(); i++)
        result.push_back(collection->item(i));
    return result;
}
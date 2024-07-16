#pragma once

#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>
//#include <CAM/CAMAll.h>
#include <functional>
#define _USE_MATH_DEFINES
#include <math.h>
#include <initializer_list>
#include "Geometry.h"

using namespace adsk::core;
using namespace adsk::fusion;
//using namespace adsk::cam;

#define RAD_45  M_PI * 0.25
#define RAD_90  M_PI * 0.5
#define RAD_180 M_PI * 1.0
#define RAD_360 M_PI * 2.0

#define PLA_MOOVABLE_CLEARNCE 0.04
#define PLA_UNMOOVABLE_CLEARNCE 0.02
#define ABS_MOOVABLE_CLEARNCE 0.02
#define ABS_UNMOOVABLE_CLEARNCE 0.01

enum CubeFaceType {
    Top,
    Bottom,
    Right,
    Left,
    Fron,
    Back
};

struct VectorPoint
{
    Ptr<Vector3D> vector;
    Ptr<Point3D> point;
};

Ptr<Point3D> GetCenterPoint();
Ptr<Point3D> GetCirclePoint(double radius, double angel);
Ptr<Point3D> GetCirclePoint(Ptr<Point3D> circleCenter, double radius, double angel, bool saveZ = false);

Ptr<BoundingBox3D> CreateBound(double top, double bottom, double left, double right);
Ptr<BoundingBox3D> CutShell(Ptr<BoundingBox3D> box, double shellThickness);

bool Equal(double a, double b, double delta = 0.001);
bool Equal(Ptr<Point3D> p1, Ptr<Point3D> p2, double delta = 0.001);
bool Equal(Ptr<SketchLine> line, Ptr<BRepEdge> edge, double delta = 0.001);
bool EqualAny(Ptr<BRepEdge> edge, Ptr<SketchLines> lines, double delta = 0.001);
bool isPointOnFace(Ptr<BRepFace> face, Ptr<Point3D> point, double tolerance = 0.01);
bool isPointOnEdge(Ptr<BRepEdge> edge, Ptr<Point3D> point, double tolerance = 0.01);
Ptr<Point3D> getPointOnSphere(Ptr<Point3D> center, double radius, Ptr<Vector3D> vector);
double getTriangleSideLength(double side1, double side2, double angleInRadians);
double getTriangleLeg(double legB, double angleBInRadians);

Ptr<BRepFace> getBodyFace(Ptr<BRepBody> body, CubeFaceType faceType);
Ptr<BRepFace> getBodyFace(Ptr<BRepBody> body, Ptr<Point3D> pointOnFace, double tolerance);
Ptr<BRepEdge> getJoinedEdge(Ptr<BRepFace> face1, Ptr<BRepFace> face2);

DialogResults MessageBox(std::string message, std::string title = "", MessageBoxButtonTypes buttonType = OKButtonType);

void addToObjectCollection(Ptr<ObjectCollection> objectCollection, std::initializer_list<Ptr<Base>> items);
Ptr<ObjectCollection> createObjectCollection(std::initializer_list<Ptr<Base>> items);

Ptr<Sketch> CreateSketch(Ptr<Component> component, Ptr<ConstructionPlane> plane, std::string name);

void Rotate(Ptr<Sketch> sketch, double angel, Ptr<Point3D> point, Ptr<ObjectCollection> items);
void Rotate(Ptr<Sketch> sketch, double angel, Ptr<Point3D> point, std::initializer_list<Ptr<Base>> items);
void Rotate(Ptr<Sketch> sketch, double angel, Ptr<Point3D> point);

void Rotate(Ptr<SectionAnalysis> analysis, double angel, Ptr<Vector3D> axis, Ptr<Point3D> originPointOfAxis);
void Rotate(Ptr<SectionAnalysis> analysis, double angel, Ptr<ConstructionAxis> axis);

Ptr<ConstructionPoint> AddConstructionPoint(Ptr<Component> component, Ptr<Base> point);
Ptr<ConstructionAxis> AddConstructionAxis(Ptr<Component> component, Ptr<Point3D> point, Ptr<Vector3D> vector);
Ptr<ConstructionAxis> AddConstructionAxis(Ptr<Component> component, Ptr<Vector3D> vector);

Ptr<SketchCircle> AddCircle(Ptr<Sketch> sketch, Ptr<Point3D> circleCentr, double radius);
Ptr<SketchArc> AddArc(Ptr<Sketch> sketch, Ptr<Point3D> circleCentr, double radius, double length, double pivotAngelInRadian, bool pivotAngelIsCenterOfArc = true);
Ptr<SketchArc> AddArc(Ptr<Sketch> sketch, Ptr<Point3D> circleCentr, Ptr<Point3D> startPoint, Ptr<Point3D> endPoint);
Ptr<SketchLine> AddLine(Ptr<Sketch> sketch, Ptr<Base> startPoint, Ptr<Base> endPoint);
Ptr<SketchLine> AddLine(Ptr<Sketch> sketch, double startPointX, double startPointY, double startPointZ, double endPointX, double endPointY, double endPointZ);

Ptr<RevolveFeature> Revolve(Ptr<Component> component, Ptr<Profile> profile, Ptr<ConstructionAxis> axis, double angelRad);
Ptr<RevolveFeature> Revolve(Ptr<Component> component, Ptr<Sketch> sketch, Ptr<ConstructionAxis> axis, double angelRad);
Ptr<ExtrudeFeature> Extrude(Ptr<Component> component, Ptr<ObjectCollection> collection, double distance, bool isSymetric = false);
Ptr<ExtrudeFeature> Extrude(Ptr<Component> component, Ptr<Profile> profile, double distance, bool isSymetric = false);
Ptr<ExtrudeFeature> Extrude(Ptr<Component> component, Ptr<Sketch> sketch, double distance, bool isSymetric = false);
Ptr<ObjectCollection> ExtrudeAll(Ptr<Component> component, Ptr<Sketch> sketch, double distance, bool isSymetric = false);

Ptr<BRepBody> Move(Ptr<Component> component, Ptr<BRepBody> body, Ptr<ConstructionAxis> axis, double distance, bool createCopy = false);
Ptr<BRepBody> Rotate(Ptr<Component> component, Ptr<BRepBody> body, Ptr<ConstructionAxis> axis, double angel, bool createCopy = false);
Ptr<BRepBody> Combine(Ptr<Component> component, FeatureOperations operation, Ptr<BRepBody> body1, Ptr<BRepBody> body2);
Ptr<CombineFeature> CreateCombineFeature(Ptr<Component> component, FeatureOperations operation, Ptr<BRepBody> body1, Ptr<BRepBody> body2);
Ptr<FilletFeature> Fillet(Ptr<Component> component, Ptr<ObjectCollection> edges, double val);

bool HasCommonEdge(Ptr<BRepFace> face1, Ptr<BRepFace> face2);

Ptr<ObjectCollection> GetEdges(Ptr<BRepBody> body, std::function <bool(Ptr<BRepEdge>)> isGoodEdge);
Ptr<ObjectCollection> GetEdges(Ptr<ObjectCollection> edges, std::function <bool(Ptr<BRepEdge>)> isGoodEdge);
Ptr<ObjectCollection> GetEdges(std::vector<Ptr<BRepFace>> joinedFaces, std::vector<Ptr<BRepFace>> unjoinedFaces);
double GetMin(Ptr<ObjectCollection> items, std::function <double(Ptr<Base>)> getValue);

Ptr<ObjectCollection> GetProfiles(Ptr<Sketch> sketch, std::function <bool(Ptr<Profile>)> isGoodProfile);

Ptr<Vector3D> ConstructionAxisToVector3D(Ptr<ConstructionAxis> axis);
VectorPoint ConstructionAxisToVectorPoint(Ptr<ConstructionAxis> axis);

Ptr<BRepBody> CreateSphere(Ptr<Component> component, Ptr<Point3D> center, double radius);
Ptr<BRepBody> CreateCylinder(Ptr<Component> component, Ptr<Point3D> center, double radius, double height);

Ptr<BRepBody> CreateBox(Ptr<Component> component, Ptr<Point3D> point1, Ptr<Point3D> point2, double height);
Ptr<BRepBody> CreateBox(Ptr<Component> component, Ptr<Point3D> point1, Ptr<Point3D> point2, double height, double verticalCornerFilletRadius);
Ptr<BRepBody> CreateBox(Ptr<Component> component, Ptr<Point3D> point1, Ptr<Point3D> point2, double height, double verticalCornerFilletRadius, double wallThicknness);

bool EdgeIsHorizontal(Ptr<BRepEdge> edge);
bool EdgeIsVerticalLine(Ptr<BRepEdge> edge);

bool BodyContainPoint(Ptr<BRepBody> body, Ptr<Point3D> point);

Ptr<SectionAnalysis> AddSectionAnalysis(Ptr<Component> component, Ptr<Base> plane, double distance);

void SaveAsStl(Ptr<BRepBody> body, std::string filepath);

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
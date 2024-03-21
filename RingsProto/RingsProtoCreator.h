#pragma once

#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>
//#include <CAM/CAMAll.h>

using namespace adsk::core;
using namespace adsk::fusion;
//using namespace adsk::cam;

#define RAD_90 M_PI / 2.0
#define RAD_180 M_PI
#define RAD_360 M_PI * 2.0
#define RAD_144 M_PI * 0.8

class RingsProtoCreator
{
    class BaseCuttingParams
    {
    public:
        double outerRadius;
        double outerLength;
        double innerRadius;
        double innerLength;
        double middleRadius;
        double miidleLength;
        double miidleLegLength;
    };
private:
    int volfCount;
    double outerRadius;
	double innerRadius;
	double wallThickness;
	double floorTopThickness;
	double floorBottomThickness;
	double volfLegRate;
	double clearanceMovable = 0.04;
	double clearanceUnmovable = 0.02;
	double clearanceBetweenBaseWallAndVolfLeg = 0.03;
	double baseExternalCornerFilletRadius = 0.4; // base external corner - is corner at cross of base
	double baseInternalCornerFilletRadius = 0.5; // base internal corner - is corner inside triangle of 3 base body
    double baseArcEdgesFilletRate = 0.7;  // arc edges - long edges of base; rate about wallThickness
    double baseToothThickness = 0.09;
    double baseToothSideCrossSideFilletRate = 0.3;
    double baseToothSideCrossFloorFilletRate = 0.45;
    double volfHeigntOverBaseInCenter = 0.3;

    BaseCuttingParams baseCuttingParams;

    Ptr<ConstructionAxis> xy45Axis;
    Ptr<ConstructionAxis> xy135Axis;
    Ptr<ConstructionAxis> yz135Axis;

    void Initialize();
	static Ptr<Sketch> createSketch(Ptr<Component> component, Ptr<ConstructionPlane> plane, std::string name);
	Ptr<Sketch> createSketchBase(Ptr<Component> component);
    Ptr<Sketch> createSketchCutting(Ptr<Component> component);
	Ptr<Sketch> createSketchCuttingFinal(Ptr<Component> component);
    Ptr<Sketch> createSketchSquare(Ptr<Component> component, double size);
    static Ptr<BRepBody> createArcBody(Ptr<Component> component, double radius, double thickness, double size, double sideCrossSideFilletRate = 0, double sideCrossFoolrFilletRate = 0);
    Ptr<BRepBody> joinFloorToothToBase(Ptr<Component> component, Ptr<BRepBody> baseBody, double radius, double thickness, double size, double rotateAngel, bool inverse = false);
	bool isBaseExternalCornerEdge(Ptr<BRepEdge> edge);
	bool isBaseIntearnalCornerEdge(Ptr<BRepEdge> edge);
    /// O######   ######O
    /// #######   #######
    /// ###           ###
    /// ###           ###
    /// #################
    /// O###############O
    bool isBaseWallXFloorsOuterEdge(Ptr<BRepEdge> edge);
public:
	RingsProtoCreator(
		double outerRadius,
		double innerRadius,
		int volfCount,
		double wallThickness,
		double floorTopThickness,
		double floorBottomThickness,
		double volfLegRate
	);
    bool createBodies(Ptr<Component> component);
	bool createBaseBody(Ptr<Component> component);
    bool createVolfBody(Ptr<Component> component);
	double getVolfAngel();
	double getVolfLegAngel();
	double getBaseOuterLength();
	double getBaseInnerLength();
};
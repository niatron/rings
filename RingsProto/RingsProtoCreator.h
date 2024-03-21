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
	double outerRadius;
	double innerRadius;
	double wallThickness;
	double floorTopThickness;
	double floorBottomThickness;
	double volfLegRate;
	int volfCount;
	double volfAngel;
	double clearanceMovable;
	double clearanceUnmovable;
	double clearanceBetweenVolfsRate = 0.00;
	double clearanceBetweenBaseWallAndVolfLeg = 0.04;
	double baseExternalCornerFilletRadius = 0.4; // base external corner - is corner at cross of base
	double baseInternalCornerFilletRadius = 0.5; // base internal corner - is corner inside triangle of 3 base body
    double baseArcEdgesFilletRate = 0.7;  // arc edges - long edges of base; rate about wallThickness
    double baseToothThickness = 0.09;
    double baseToothCornerFilletRate = 0.3;

    Ptr<ConstructionAxis> xy45Axis;
    Ptr<ConstructionAxis> xy135Axis;
    Ptr<ConstructionAxis> yz135Axis;

	Ptr<Sketch> createSketch(Ptr<Component> component, Ptr<ConstructionPlane> plane, std::string name);
	Ptr<Sketch> createSketchBase(Ptr<Component> component);
    Ptr<Sketch> createSketchCutting(Ptr<Component> component, BaseCuttingParams& params);
	Ptr<Sketch> createSketchCuttingFinal(Ptr<Component> component);
    Ptr<BRepBody> createFloorTooth(Ptr<Component> component, double radius, double thickness, double size);
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
		double volfLegRate,
		double clearanceMovable,
		double clearanceUnmovable
	);
	bool createBody(Ptr<Component> component);
	double getVolfAngel();
	double getVolfAngelWithClearance();
	double getVolfLegOuterLength();
	double getBaseOuterLength();
	double getBaseInnerLength();
};
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

class RingsProtoCreator
{
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
	double baseExternalCornerFilletRadius = 0.4;
	double baseInternalCornerFilletRadius = 0.5;

	Ptr<Sketch> createSketch(Ptr<Component> component, Ptr<ConstructionPlane> plane, std::string name);
	Ptr<Sketch> createSketchBase(Ptr<Component> component);
	Ptr<Sketch> createSketchCutting(Ptr<Component> component);
	Ptr<Sketch> createSketchCuttingFinal(Ptr<Component> component);
	bool isBaseExternalCornerEdge(Ptr<BRepEdge> edge);
	bool isBaseIntearnalCornerEdge(Ptr<BRepEdge> edge);
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
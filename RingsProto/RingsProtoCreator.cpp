#include "RingsProtoCreator.h"
#include "ComponentUtils.h"

#define _USE_MATH_DEFINES
#include <math.h>

using namespace adsk::core;
using namespace adsk::fusion;
//using namespace adsk::cam;

RingsProtoCreator::RingsProtoCreator(
	double outerRadius,
	double innerRadius,
	int volfCount,
	double wallThickness,
	double floorTopThickness,
	double floorBottomThickness,
	double volfLegRate,
	double clearanceMovable,
	double clearanceUnmovable
) :
	outerRadius(outerRadius),
	innerRadius(innerRadius),
	volfCount(volfCount),
	wallThickness(wallThickness),
	floorTopThickness(floorTopThickness),
	floorBottomThickness(floorBottomThickness),
	volfLegRate(volfLegRate),
	clearanceMovable(clearanceMovable),
	clearanceUnmovable(clearanceUnmovable)
{ }

bool RingsProtoCreator::createBody(Ptr<Component> component)
{
	auto baseSketch = createSketchBase(component);
	auto baseRevolve = RevolveSketch(component, baseSketch, component->xConstructionAxis(), RAD_180);
	auto baseBody = Rotate(component, baseRevolve->bodies()->item(0), component->zConstructionAxis(), RAD_90, true);
	baseBody = Combine(component, FeatureOperations::JoinFeatureOperation, baseRevolve->bodies()->item(0), baseBody);

	auto cuttingSketch = createSketchCutting(component);
	auto cuttingRevolve = RevolveSketch(component, cuttingSketch, component->xConstructionAxis(), RAD_180);
	auto cuttingBody = Rotate(component, cuttingRevolve->bodies()->item(0), component->zConstructionAxis(), RAD_90, true);
	cuttingBody = Combine(component, FeatureOperations::JoinFeatureOperation, cuttingRevolve->bodies()->item(0), cuttingBody);

	baseBody = Combine(component, FeatureOperations::CutFeatureOperation, baseBody, cuttingBody);

	auto edges = GetEdges(baseBody, [=](Ptr<BRepEdge> edge) {return isEdgeOnTopFloorCorner(edge); });

	Fillet(component, edges, 0.5);

	auto cuttingFinalSketch = createSketchCuttingFinal(component);
	auto cuttingFinalExtrude = ExtrudeSketch(component, cuttingFinalSketch, outerRadius);

	baseBody = Combine(component, FeatureOperations::CutFeatureOperation, baseBody, cuttingFinalExtrude->bodies()->item(0));

	return true;
}

Ptr<Sketch> RingsProtoCreator::createSketch(Ptr<Component> component, Ptr<ConstructionPlane> plane, std::string name)
{
	Ptr<Sketch> sketch = component->sketches()->add(plane);
	sketch->name(name);
	return sketch;
}

Ptr<Sketch> RingsProtoCreator::createSketchBase(Ptr<Component> component)
{
	Ptr<Sketch> baseSketch = createSketch(component, component->xYConstructionPlane(), "BaseSketch");
	Ptr<SketchPoint> centerPoint = baseSketch->sketchPoints()->add(GetCenterPoint());

	Ptr<SketchArc> arc1 = baseSketch->sketchCurves()->sketchArcs()->addByCenterStartSweep(GetCenterPoint(), GetCirclePoint(outerRadius, RAD_90 - getVolfAngel() / 2.0), getVolfAngel());
	Ptr<SketchArc> arc2 = baseSketch->sketchCurves()->sketchArcs()->addByCenterStartSweep(GetCenterPoint(), GetCirclePoint(innerRadius, RAD_90 - getVolfAngel() / 2.0), getVolfAngel());

	Ptr<SketchLine> line1 = baseSketch->sketchCurves()->sketchLines()->addByTwoPoints(arc1->startSketchPoint(), arc2->startSketchPoint());
	Ptr<SketchLine> line2 = baseSketch->sketchCurves()->sketchLines()->addByTwoPoints(arc1->endSketchPoint(), arc2->endSketchPoint());
	
	return baseSketch;
}

bool RingsProtoCreator::isEdgeOnTopFloorCorner(Ptr<BRepEdge> edge)
{
	return abs(edge->length() - floorTopThickness) < 0.000001 && edge->startVertex()->geometry()->z() > innerRadius;
}

Ptr<Sketch> RingsProtoCreator::createSketchCutting(Ptr<Component> component)
{
	Ptr<Sketch> sketch = createSketch(component, component->xYConstructionPlane(), "CuttingSketch");

	auto cuttingTopRadius = outerRadius;
	auto cuttingTopAngel = getVolfLegWithClearanceAngel();
	auto cuttingBottomRadius = innerRadius + floorBottomThickness;
	auto cuttingBottomAngel = getVolfAngel() - 2.0 * wallThickness / cuttingBottomRadius;
	auto cuttingMiddleRadius = outerRadius - floorTopThickness;
	auto cuttingMiddleAngel = (getVolfAngel() - getVolfLegWithClearanceAngel()) / 2.0 - wallThickness / cuttingMiddleRadius;

	Ptr<SketchArc> arc1 = sketch->sketchCurves()->sketchArcs()->addByCenterStartSweep(GetCenterPoint(), GetCirclePoint(cuttingTopRadius, RAD_90 - cuttingTopAngel / 2.0), cuttingTopAngel);
	Ptr<SketchArc> arc2 = sketch->sketchCurves()->sketchArcs()->addByCenterStartSweep(GetCenterPoint(), GetCirclePoint(cuttingBottomRadius, RAD_90 - cuttingBottomAngel / 2.0), cuttingBottomAngel);

	Ptr<SketchArc> arcML = sketch->sketchCurves()->sketchArcs()->addByCenterStartSweep(GetCenterPoint(), GetCirclePoint(cuttingMiddleRadius, RAD_90 + cuttingTopAngel / 2.0), cuttingMiddleAngel);
	Ptr<SketchArc> arcMR = sketch->sketchCurves()->sketchArcs()->addByCenterStartSweep(GetCenterPoint(), GetCirclePoint(cuttingMiddleRadius, RAD_90 - cuttingTopAngel / 2.0), -cuttingMiddleAngel);

	Ptr<SketchLine> line1 = sketch->sketchCurves()->sketchLines()->addByTwoPoints(arc1->startSketchPoint(), arcMR->endSketchPoint());
	Ptr<SketchLine> line2 = sketch->sketchCurves()->sketchLines()->addByTwoPoints(arc1->endSketchPoint(), arcML->startSketchPoint());
	Ptr<SketchLine> line3 = sketch->sketchCurves()->sketchLines()->addByTwoPoints(arcMR->startSketchPoint(), arc2->startSketchPoint());
	Ptr<SketchLine> line4 = sketch->sketchCurves()->sketchLines()->addByTwoPoints(arcML->endSketchPoint(), arc2->endSketchPoint());

	return sketch;
}

Ptr<Sketch> RingsProtoCreator::createSketchCuttingFinal(Ptr<Component> component)
{
	Ptr<Sketch> sketch = createSketch(component, component->xYConstructionPlane(), "CuttingFinalSketch");

	Ptr<SketchLine> line1 = sketch->sketchCurves()->sketchLines()->addByTwoPoints(GetCenterPoint(), Point3D::create(outerRadius, -outerRadius, 0));
	Ptr<SketchLine> line2 = sketch->sketchCurves()->sketchLines()->addByTwoPoints(line1->endSketchPoint(), Point3D::create(outerRadius, outerRadius, 0));
	Ptr<SketchLine> line3 = sketch->sketchCurves()->sketchLines()->addByTwoPoints(line2->endSketchPoint(), Point3D::create(-outerRadius, outerRadius, 0));
	Ptr<SketchLine> line4 = sketch->sketchCurves()->sketchLines()->addByTwoPoints(line3->endSketchPoint(), Point3D::create(-outerRadius, -outerRadius, 0));
	Ptr<SketchLine> line5 = sketch->sketchCurves()->sketchLines()->addByTwoPoints(line4->endSketchPoint(), line1->startSketchPoint());

	return sketch;
}



double RingsProtoCreator::getVolfAngel()
{
	return RAD_360 / volfCount;
}

double RingsProtoCreator::getVolfLegAngel()
{
	return getVolfAngel() * volfLegRate;
}

double RingsProtoCreator::getVolfLegWithClearanceAngel()
{
	return getVolfLegAngel() + 2.0 * clearanceMovable / outerRadius;
}

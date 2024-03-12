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

	auto baseRevolve = Revolve(component, baseSketch, component->xConstructionAxis(), RAD_180);
	auto baseBody = Rotate(component, baseRevolve->bodies()->item(0), component->zConstructionAxis(), RAD_90, true);
	baseBody = Combine(component, FeatureOperations::JoinFeatureOperation, baseRevolve->bodies()->item(0), baseBody);

	auto cuttingSketch = createSketchCutting(component);
	auto cuttingRevolve = Revolve(component, cuttingSketch, component->xConstructionAxis(), RAD_180);
	auto cuttingBody = Rotate(component, cuttingRevolve->bodies()->item(0), component->zConstructionAxis(), RAD_90, true);
	cuttingBody = Combine(component, FeatureOperations::JoinFeatureOperation, cuttingRevolve->bodies()->item(0), cuttingBody);

	baseBody = Combine(component, FeatureOperations::CutFeatureOperation, baseBody, cuttingBody);

	auto internalEdges = GetEdges(baseBody, [=](Ptr<BRepEdge> edge) {return isBaseIntearnalCornerEdge(edge); });
	auto externalEdges = GetEdges(baseBody, [=](Ptr<BRepEdge> edge) {return isBaseExternalCornerEdge(edge); });

	//Application::get()->userInterface()->messageBox("IEdges Count = " + std::to_string(internalEdges->count()) + " EEdges Count = " + std::to_string(externalEdges->count()));

	Fillet(component, internalEdges, baseInternalCornerFilletRadius);
	Fillet(component, externalEdges, baseExternalCornerFilletRadius);

	auto cuttingFinalSketch = createSketchCuttingFinal(component);
	auto cuttingFinalExtrude = Extrude(component, cuttingFinalSketch, outerRadius);

	baseBody = Combine(component, FeatureOperations::CutFeatureOperation, baseBody, cuttingFinalExtrude->bodies()->item(0));

    auto baseToothRadius = outerRadius - floorTopThickness / 2.0;
    auto baseToothSize = (getBaseOuterLength() - getVolfLegOuterLength()) / 2.0 - wallThickness;
    auto baseToothBody = createFloorTooth(component, baseToothRadius, baseToothThickness, baseToothSize);
    
    auto centrSketchPoint = baseSketch->sketchPoints()->add(GetCenterPoint());
    auto xy45SketchPoint = baseSketch->sketchPoints()->add(Point3D::create(1, 1, 0));
    auto xy45AxisInput = component->constructionAxes()->createInput();
    xy45AxisInput->setByTwoPoints(centrSketchPoint, xy45SketchPoint);
    auto xy45Axis = component->constructionAxes()->add(xy45AxisInput);

    auto filletShift = baseInternalCornerFilletRadius * (sqrt(2.0) - 1.0);
    Rotate(component, baseToothBody, xy45Axis, ((((getBaseOuterLength() - getVolfLegOuterLength()) / 2.0 + getVolfLegOuterLength()) / 2.0 + clearanceBetweenBaseWallAndVolfLeg) * sqrt(2.0) + filletShift) / outerRadius);

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
	Ptr<Sketch> sketch = createSketch(component, component->xYConstructionPlane(), "BaseSketch");

	auto outerLength = getBaseOuterLength();
	auto innerLength = getBaseInnerLength();

	auto arc1 = AddArc(sketch, GetCenterPoint(), outerRadius, outerLength, RAD_90);
	auto arc2 = AddArc(sketch, GetCenterPoint(), innerRadius, innerLength, RAD_90);

	Ptr<SketchLine> line1 = sketch->sketchCurves()->sketchLines()->addByTwoPoints(arc1->startSketchPoint(), arc2->startSketchPoint());
	Ptr<SketchLine> line2 = sketch->sketchCurves()->sketchLines()->addByTwoPoints(arc1->endSketchPoint(), arc2->endSketchPoint());

	return sketch;
}

bool RingsProtoCreator::isBaseExternalCornerEdge(Ptr<BRepEdge> edge)
{
	return (edge->startVertex()->geometry()->z() > innerRadius) && (edge->endVertex()->geometry()->z() > innerRadius);
}

bool RingsProtoCreator::isBaseIntearnalCornerEdge(Ptr<BRepEdge> edge)
{
	return  (abs(edge->length() - outerRadius + innerRadius) < 0.02) && ((edge->startVertex()->geometry()->z() > innerRadius) || (edge->endVertex()->geometry()->z() > innerRadius));
}

Ptr<Sketch> RingsProtoCreator::createSketchCutting(Ptr<Component> component)
{
	Ptr<Sketch> sketch = createSketch(component, component->xYConstructionPlane(), "CuttingSketch");

	auto outerLength = getVolfLegOuterLength() + 2.0 * clearanceBetweenBaseWallAndVolfLeg;
	auto cutIinnerRadius = this->innerRadius + floorBottomThickness;
	auto innerLength = getVolfAngelWithClearance() * cutIinnerRadius + 2.0 * clearanceBetweenBaseWallAndVolfLeg;
	auto middleRadius = outerRadius - floorTopThickness;
	auto miidleLegLength = getVolfAngel() * volfLegRate * middleRadius + 2.0 * clearanceBetweenBaseWallAndVolfLeg;
	auto miidleLength = (getVolfAngelWithClearance() * middleRadius - miidleLegLength + 2.0 * clearanceBetweenBaseWallAndVolfLeg) / 2.0;

	auto arc1 = AddArc(sketch, GetCenterPoint(), outerRadius, outerLength, RAD_90);
	auto arc2 = AddArc(sketch, GetCenterPoint(), cutIinnerRadius, innerLength, RAD_90);

	auto arcML = AddArc(sketch, GetCenterPoint(), middleRadius, miidleLength, RAD_90 + miidleLegLength / 2.0 / middleRadius, false);
	auto arcMR = AddArc(sketch, GetCenterPoint(), middleRadius, -miidleLength, RAD_90 - miidleLegLength / 2.0 / middleRadius, false);

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

Ptr<BRepBody> RingsProtoCreator::createFloorTooth(Ptr<Component> component, double radius, double thickness, double size)
{
    double halfSize = size / 2.0;
    double outerRadius = radius + thickness / 2.0;
    double innerRadius = radius - thickness / 2.0;
    double outerLength = size * outerRadius / radius;
    double innerLength = size * innerRadius / radius;

    Ptr<Sketch> sketch = createSketch(component, component->xYConstructionPlane(), "BaseFloorToothSketch");

    auto arc1 = AddArc(sketch, GetCenterPoint(), outerRadius, outerLength, RAD_90);
    auto arc2 = AddArc(sketch, GetCenterPoint(), innerRadius, innerLength, RAD_90);

    auto line1 = AddLine(sketch, arc1->startSketchPoint(), arc2->startSketchPoint());
    auto line2 = AddLine(sketch, arc1->endSketchPoint(), arc2->endSketchPoint());

    /*auto lineSquare1 = AddLine(sketch, -halfSize, halfSize, 0, halfSize, halfSize, 0);
    auto lineSquare2 = AddLine(sketch, lineSquare1->endSketchPoint(), Point3D::create(halfSize, -halfSize, 0));
    auto lineSquare3 = AddLine(sketch, lineSquare2->endSketchPoint(), Point3D::create(-halfSize, -halfSize, 0));
    auto lineSquare4 = AddLine(sketch, lineSquare3->endSketchPoint(), lineSquare1->startSketchPoint());
    */
    auto profArc = sketch->profiles()->item(0);
    //auto profSquare = sketch->profiles()->item(1);
    
    auto revolveFeature = Revolve(component, profArc, component->xConstructionAxis(), RAD_180);
    //auto extrudeFeature = Extrude(component, profSquare, outerRadius);

    auto body = Rotate(component, revolveFeature->bodies()->item(0), component->zConstructionAxis(), RAD_90, true);

    body = Combine(component, FeatureOperations::IntersectFeatureOperation, revolveFeature->bodies()->item(0), body);

    auto cornerEdges = GetEdges(body, [=](Ptr<BRepEdge> edge) {return Equal(edge->length(), thickness); });
    Fillet(component, cornerEdges, size * baseToothCornerFilletRate);

    auto allEdges = GetEdges(body, [=](Ptr<BRepEdge> edge) {return edge->tangentiallyConnectedEdges()->count() == 8; });
    Fillet(component, allEdges, thickness * 0.9 / 2.0);

    return body;
}


double RingsProtoCreator::getVolfAngel()
{
	return RAD_360 / volfCount;
}

double RingsProtoCreator::getVolfAngelWithClearance()
{
	return getVolfAngel() + getVolfAngel() * clearanceBetweenVolfsRate * 2.0;
}

double RingsProtoCreator::getVolfLegOuterLength()
{
	return getVolfAngel() * volfLegRate * outerRadius;
}

double RingsProtoCreator::getBaseOuterLength()
{
	return getVolfAngelWithClearance() * outerRadius + 2.0 * clearanceBetweenBaseWallAndVolfLeg + 2.0 * wallThickness;
}

double RingsProtoCreator::getBaseInnerLength()
{
	return getVolfAngelWithClearance() * innerRadius + 2.0 * clearanceBetweenBaseWallAndVolfLeg + 2.0 * wallThickness;
}


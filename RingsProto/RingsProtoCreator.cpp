#include "RingsProtoCreator.h"
#include "FusionEnvironment.h"

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
	double volfLegRate
) :
	outerRadius(outerRadius),
	innerRadius(innerRadius),
	volfCount(volfCount),
	wallThickness(wallThickness),
	floorTopThickness(floorTopThickness),
	floorBottomThickness(floorBottomThickness),
	volfLegRate(volfLegRate)
{
    Initialize();
}

void RingsProtoCreator::Initialize()
{
    baseCuttingParams.outerRadius = outerRadius;
    baseCuttingParams.outerLength = getVolfLegAngel() * baseCuttingParams.outerRadius + 2.0 * clearanceMovable;
    baseCuttingParams.innerRadius = innerRadius + floorBottomThickness;
    baseCuttingParams.innerLength = getVolfAngel() * baseCuttingParams.innerRadius + 2.0 * clearanceBetweenBaseWallAndVolfLeg;
    baseCuttingParams.middleRadius = outerRadius - floorTopThickness;
    baseCuttingParams.miidleLegLength = getVolfLegAngel() * baseCuttingParams.middleRadius + 2.0 * clearanceBetweenBaseWallAndVolfLeg;
    baseCuttingParams.miidleLength = (getVolfAngel() * baseCuttingParams.middleRadius - baseCuttingParams.miidleLegLength + 2.0 * clearanceBetweenBaseWallAndVolfLeg) / 2.0;
}

bool RingsProtoCreator::createBodies(Ptr<Component> component)
{
    createBaseBody(component);
    createVolfBody(component);
    return true;
}

bool RingsProtoCreator::createBaseBody(Ptr<Component> component)
{
    xy45Axis = AddConstructionAxis(component, Vector3D::create(1, 1, 0));
    xy135Axis = AddConstructionAxis(component, Vector3D::create(-1, 1, 0));
    yz135Axis = AddConstructionAxis(component, Vector3D::create(0, -1, 1));

	auto baseSketch = createSketchBase(component);
    	
    auto baseRevolve = Revolve(component, baseSketch, component->xConstructionAxis(), RAD_144);
    
    auto baseLeftFace = getBodyFace(baseRevolve->bodies()->item(0), Left);
    auto baseRightFace = getBodyFace(baseRevolve->bodies()->item(0), Right);
    auto baseTopFace = getBodyFace(baseRevolve->bodies()->item(0), Top);
    auto baseBottomFace = getBodyFace(baseRevolve->bodies()->item(0), Bottom);
    auto baseTopLeftEdge = getJoinedEdge(baseTopFace, baseLeftFace);
    auto baseTopRightEdge = getJoinedEdge(baseTopFace, baseRightFace);
    auto baseBottomLeftEdge = getJoinedEdge(baseBottomFace, baseLeftFace);
    auto baseBottomRightEdge = getJoinedEdge(baseBottomFace, baseRightFace);

	auto baseBody = Rotate(component, baseRevolve->bodies()->item(0), component->zConstructionAxis(), RAD_90, true);
	baseBody = Combine(component, FeatureOperations::JoinFeatureOperation, baseRevolve->bodies()->item(0), baseBody);
    
	auto cuttingSketch = createSketchCutting(component);
	auto cuttingRevolve = Revolve(component, cuttingSketch, component->xConstructionAxis(), RAD_180);
	auto cuttingBody = Rotate(component, cuttingRevolve->bodies()->item(0), component->zConstructionAxis(), RAD_90, true);
	cuttingBody = Combine(component, FeatureOperations::JoinFeatureOperation, cuttingRevolve->bodies()->item(0), cuttingBody);

	baseBody = Combine(component, FeatureOperations::CutFeatureOperation, baseBody, cuttingBody);

	auto internalEdges = GetEdges(baseBody, [=](Ptr<BRepEdge> edge) {return isBaseIntearnalCornerEdge(edge); });
	auto externalEdges = GetEdges(baseBody, [=](Ptr<BRepEdge> edge) {return isBaseExternalCornerEdge(edge); });

	Fillet(component, internalEdges, baseInternalCornerFilletRadius);
	Fillet(component, externalEdges, baseExternalCornerFilletRadius);

    auto cuttingFinalSketch = createSketchCuttingFinal(component);
	auto cuttingFinalExtrude = Extrude(component, cuttingFinalSketch, outerRadius);

	baseBody = Combine(component, FeatureOperations::CutFeatureOperation, baseBody, cuttingFinalExtrude->bodies()->item(0));

    auto baseArcEdges = GetEdges(baseBody, [=](Ptr<BRepEdge> edge)
    {
        if (edge->tangentiallyConnectedEdges()->count() <= 1)
            return false;
        auto tedges = edge->tangentiallyConnectedEdges();
        for (int i = 0; i < tedges->count(); i++)
        {
            auto e = tedges->item(i)->cast<BRepEdge>();
            for (int j = 0; j < e->faces()->count(); j++)
            {
                auto f = e->faces()->item(j);
                if (f == baseLeftFace || f == baseRightFace)
                    return true;
            }
        }
        return false;
    });

    auto baseArcInsideEdges = GetEdges(baseBody, [=](Ptr<BRepEdge> edge)
    {
        if (edge->tangentiallyConnectedEdges()->count() <= 1)
            return false;
        auto tedges = edge->tangentiallyConnectedEdges();
        for (int i = 0; i < tedges->count(); i++)
        {
            auto e = tedges->item(i)->cast<BRepEdge>();
            for (int j = 0; j < e->faces()->count(); j++)
            {
                auto f = e->faces()->item(j);
                if (f == baseLeftFace || f == baseRightFace)
                    return false;
            }
        }
        return true;
    });
    
    Fillet(component, baseArcEdges, floorTopThickness * baseArcEdgesFilletRate);
    Fillet(component, baseArcInsideEdges, wallThickness * baseArcEdgesFilletRate);

    auto baseFilletShift = baseInternalCornerFilletRadius * (sqrt(2.0) - 1.0);
    auto baseTopFloorToothRadius = outerRadius - floorTopThickness / 2.0;
    auto baseTopFloorToothSize = (getBaseOuterLength() - getVolfLegAngel() * outerRadius) / 2.0 - wallThickness;
    auto baseTopFloorToothRotateAngel = ((((getBaseOuterLength() - getVolfLegAngel() * outerRadius) / 2.0 + getVolfLegAngel() * outerRadius) / 2.0 + clearanceBetweenBaseWallAndVolfLeg) * sqrt(2.0) + baseFilletShift) / outerRadius;
    
    auto baseBottomFloorToothRadius = innerRadius + floorBottomThickness / 2.0;
    auto baseBottomFloorToothSize = getBaseInnerLength() / 2.0 / innerRadius * baseBottomFloorToothRadius;
    auto baseBottomFloorToothRotateAngel = (getBaseInnerLength() / 4.0 * sqrt(2.0) + baseFilletShift / 2.0) / innerRadius;

    baseBody = joinFloorToothToBase(component, baseBody, baseTopFloorToothRadius, baseToothThickness, baseTopFloorToothSize, baseTopFloorToothRotateAngel);
    baseBody = joinFloorToothToBase(component, baseBody, baseBottomFloorToothRadius, baseToothThickness, baseBottomFloorToothSize, baseBottomFloorToothRotateAngel, true);
    
    auto baseReflectionBody = Rotate(component, baseBody, yz135Axis, RAD_180, true);

    baseBody = Combine(component, FeatureOperations::JoinFeatureOperation, baseBody, baseReflectionBody);

    Rotate(component, baseBody, component->zConstructionAxis(), RAD_90, true);

	return true;
}

bool RingsProtoCreator::createVolfBody(Ptr<Component> component)
{
    auto volfTop = outerRadius + volfHeigntOverBaseInCenter;
    auto volfHeadSize = 2.0 * getTriangleLeg(volfTop, getVolfAngel() / 2.0);
    auto squareScketch = createSketchSquare(component, volfHeadSize);

    auto body = Extrude(component, squareScketch, volfTop)->bodies()->item(0);
    auto cuttingBody = Extrude(component, squareScketch, volfTop / cos(getVolfAngel()))->bodies()->item(0);
    
    Move(component, cuttingBody, component->yConstructionAxis(), volfHeadSize / 2.0);
    Rotate(component, cuttingBody, component->xConstructionAxis(), -getVolfAngel() / 2.0);

    auto cuttingBodyCopy = Rotate(component, cuttingBody, component->zConstructionAxis(), RAD_90, true);
    cuttingBody = Combine(component, FeatureOperations::JoinFeatureOperation, cuttingBodyCopy, cuttingBody);
    cuttingBodyCopy = Rotate(component, cuttingBody, component->zConstructionAxis(), RAD_180, true);
    cuttingBody = Combine(component, FeatureOperations::JoinFeatureOperation, cuttingBodyCopy, cuttingBody);
    body = Combine(component, FeatureOperations::CutFeatureOperation, body, cuttingBody);

    Move(component, body, component->zConstructionAxis(), clearanceMovable);
    
    auto intersectBody = createArcBody(component, baseCuttingParams.innerRadius + clearanceMovable + volfTop / 2.0, volfTop, 2.0 * volfHeadSize);
    body = Combine(component, FeatureOperations::IntersectFeatureOperation, body, intersectBody);

    auto legThickness = floorTopThickness + 2.0 * clearanceMovable;
    auto legRadius = outerRadius - legThickness / 2.0 + clearanceMovable;
    auto legCuttingBody = createArcBody(component, legRadius, legThickness, 2.0 * volfHeadSize);
    auto legBody = createArcBody(component, legRadius, legThickness, legRadius * getVolfLegAngel(), 0.3);
    legCuttingBody = Combine(component, FeatureOperations::CutFeatureOperation, legCuttingBody, legBody);
    body = Combine(component, FeatureOperations::CutFeatureOperation, body, legCuttingBody);
    
    Rotate(component, body, component->xConstructionAxis(), RAD_90 / 2.0, true);

    return body;
}

Ptr<Sketch> RingsProtoCreator::createSketchBase(Ptr<Component> component)
{
	Ptr<Sketch> sketch = CreateSketch(component, component->xYConstructionPlane(), "BaseSketch");

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

bool RingsProtoCreator::isBaseWallXFloorsOuterEdge(Ptr<BRepEdge> edge)
{
    return  abs(edge->startVertex()->geometry()->x()) + 0.01 > getBaseInnerLength() / 2.0 && abs(edge->endVertex()->geometry()->x()) + 0.01 > getBaseInnerLength() / 2.0;
}

Ptr<Sketch> RingsProtoCreator::createSketchCutting(Ptr<Component> component)
{
    auto params = baseCuttingParams;
    auto sketch = CreateSketch(component, component->xYConstructionPlane(), "CuttingSketch");

    auto arc1 = AddArc(sketch, GetCenterPoint(), params.outerRadius, params.outerLength, RAD_90);
    auto arc2 = AddArc(sketch, GetCenterPoint(), params.innerRadius, params.innerLength, RAD_90);

    auto arcML = AddArc(sketch, GetCenterPoint(), params.middleRadius, params.miidleLength, RAD_90 + params.miidleLegLength / 2.0 / params.middleRadius, false);
    auto arcMR = AddArc(sketch, GetCenterPoint(), params.middleRadius, -params.miidleLength, RAD_90 - params.miidleLegLength / 2.0 / params.middleRadius, false);
    
    Ptr<SketchLine> line1 = sketch->sketchCurves()->sketchLines()->addByTwoPoints(arc1->startSketchPoint(), arcMR->endSketchPoint());
    Ptr<SketchLine> line2 = sketch->sketchCurves()->sketchLines()->addByTwoPoints(arc1->endSketchPoint(), arcML->startSketchPoint());
    Ptr<SketchLine> line3 = sketch->sketchCurves()->sketchLines()->addByTwoPoints(arcMR->startSketchPoint(), arc2->startSketchPoint());
    Ptr<SketchLine> line4 = sketch->sketchCurves()->sketchLines()->addByTwoPoints(arcML->endSketchPoint(), arc2->endSketchPoint());

    return sketch;
}

Ptr<Sketch> RingsProtoCreator::createSketchCuttingFinal(Ptr<Component> component)
{
	auto sketch = CreateSketch(component, component->xYConstructionPlane(), "CuttingFinalSketch");

    auto clearance = clearanceUnmovable;
    auto line1 = sketch->sketchCurves()->sketchLines()->addByTwoPoints(Point3D::create(0, -clearance, 0), Point3D::create(outerRadius, -outerRadius - clearance, 0));
    auto line2 = sketch->sketchCurves()->sketchLines()->addByTwoPoints(line1->endSketchPoint(), Point3D::create(outerRadius, outerRadius, 0));
    auto line3 = sketch->sketchCurves()->sketchLines()->addByTwoPoints(line2->endSketchPoint(), Point3D::create(-outerRadius, outerRadius, 0));
    auto line4 = sketch->sketchCurves()->sketchLines()->addByTwoPoints(line3->endSketchPoint(), Point3D::create(-outerRadius, -outerRadius - clearance, 0));
    auto line5 = sketch->sketchCurves()->sketchLines()->addByTwoPoints(line4->endSketchPoint(), line1->startSketchPoint());

	return sketch;
}

Ptr<Sketch> RingsProtoCreator::createSketchSquare(Ptr<Component> component, double size)
{
    auto sketch = CreateSketch(component, component->xYConstructionPlane(), "Square");
    auto halfSize = size / 2;
    auto line1 = sketch->sketchCurves()->sketchLines()->addByTwoPoints(Point3D::create(halfSize, halfSize), Point3D::create(halfSize, -halfSize));
    auto line2 = sketch->sketchCurves()->sketchLines()->addByTwoPoints(line1->endSketchPoint(), Point3D::create(-halfSize, -halfSize));
    auto line3 = sketch->sketchCurves()->sketchLines()->addByTwoPoints(line2->endSketchPoint(), Point3D::create(-halfSize, halfSize));
    auto line4 = sketch->sketchCurves()->sketchLines()->addByTwoPoints(line3->endSketchPoint(), line1->startSketchPoint());

    return sketch;
}

Ptr<BRepBody> RingsProtoCreator::createArcBody(Ptr<Component> component, double radius, double thickness, double size, double sideCrossSideFilletRate, double sideCrossFoolrFilletRate)
{
    auto halfSize = size / 2.0;
    auto outerRadius = radius + thickness / 2.0;
    auto innerRadius = radius - thickness / 2.0;
    auto outerLength = size * outerRadius / radius;
    auto innerLength = size * innerRadius / radius;

    auto sketch = CreateSketch(component, component->xYConstructionPlane(), "Arc");

    auto arc1 = AddArc(sketch, GetCenterPoint(), outerRadius, outerLength, RAD_90);
    auto arc2 = AddArc(sketch, GetCenterPoint(), innerRadius, innerLength, RAD_90);

    auto line1 = AddLine(sketch, arc1->startSketchPoint(), arc2->startSketchPoint());
    auto line2 = AddLine(sketch, arc1->endSketchPoint(), arc2->endSketchPoint());

    auto revolveFeature = Revolve(component, sketch, component->xConstructionAxis(), RAD_180);
    auto body = Rotate(component, revolveFeature->bodies()->item(0), component->zConstructionAxis(), RAD_90, true);
    body = Combine(component, FeatureOperations::IntersectFeatureOperation, revolveFeature->bodies()->item(0), body);

    if (sideCrossSideFilletRate > 0)
    {
        auto cornerEdges = GetEdges(body, [=](Ptr<BRepEdge> edge) {return Equal(edge->length(), thickness); });
        Fillet(component, cornerEdges, size * sideCrossSideFilletRate);
    }

    if (sideCrossSideFilletRate > 0)
    {
        auto allEdges = GetEdges(body, [=](Ptr<BRepEdge> edge) {return edge->tangentiallyConnectedEdges()->count() == 8; });
        Fillet(component, allEdges, thickness * sideCrossFoolrFilletRate);
    }

    return body;
}

Ptr<BRepBody> RingsProtoCreator::joinFloorToothToBase(Ptr<Component> component, Ptr<BRepBody> baseBody, double radius, double thickness, double size, double rotateAngel, bool inverse)
{
    auto resultBody = baseBody;
    auto toothBody = createArcBody(component, radius, thickness, size, baseToothSideCrossSideFilletRate, baseToothSideCrossFloorFilletRate);
    auto toothCuttingBody = createArcBody(component, radius, thickness + 2.0 * clearanceUnmovable, size + 2.0 * clearanceMovable, baseToothSideCrossSideFilletRate, baseToothSideCrossFloorFilletRate);

    Rotate(component, !inverse ? toothBody : toothCuttingBody, xy45Axis, rotateAngel);
    Rotate(component, !inverse ? toothCuttingBody : toothBody, xy135Axis, -rotateAngel);

    resultBody = Combine(component, FeatureOperations::JoinFeatureOperation, resultBody, toothBody);
    resultBody = Combine(component, FeatureOperations::CutFeatureOperation, resultBody, toothCuttingBody);

    return resultBody;
}


double RingsProtoCreator::getVolfAngel()
{
	return RAD_360 / volfCount;
}

double RingsProtoCreator::getVolfLegAngel()
{
	return getVolfAngel() * volfLegRate;
}

double RingsProtoCreator::getBaseOuterLength()
{
	return getVolfAngel() * outerRadius + 2.0 * clearanceMovable + 2.0 * wallThickness;
}

double RingsProtoCreator::getBaseInnerLength()
{
	return getVolfAngel() * innerRadius + 2.0 * clearanceMovable + 2.0 * wallThickness;
}


#include "ComponentUtils.h"


Ptr<Point3D> GetCenterPoint()
{
	return Point3D::create(0, 0, 0);
}

Ptr<Point3D> GetCirclePoint(double radius, double angel)
{
	return Point3D::create(radius * cos(angel), radius * sin(angel), 0);
}

bool Equal(double a, double b, double delta)
{
    return abs(a - b) < delta;
}

void MessageBox(std::string message)
{
    Application::get()->userInterface()->messageBox(message);
}

Ptr<SketchArc> AddArc(Ptr<Sketch> sketch, Ptr<Point3D> circleCentr, double radius, double length, double angel, bool angelIsCenterOfArc)
{
	double lengthRad = length / radius;
	return sketch->sketchCurves()->sketchArcs()->addByCenterStartSweep(circleCentr, GetCirclePoint(radius, angelIsCenterOfArc ? angel - lengthRad / 2.0 : angel), lengthRad);
}

Ptr<SketchLine> AddLine(Ptr<Sketch> sketch, Ptr<Base> startPoint, Ptr<Base> endPoint)
{
    return sketch->sketchCurves()->sketchLines()->addByTwoPoints(startPoint, endPoint);
}

Ptr<SketchLine> AddLine(Ptr<Sketch> sketch, double startPointX, double startPointY, double startPointZ, double endPointX, double endPointY, double endPointZ)
{
    return AddLine(sketch, Point3D::create(startPointX, startPointY, startPointZ), Point3D::create(endPointX, endPointY, endPointZ));
}


Ptr<RevolveFeature> Revolve(Ptr<Component> component, Ptr<Profile> profile, Ptr<ConstructionAxis> axis, double angelRad)
{
    auto revInput = component->features()->revolveFeatures()->createInput(profile, axis, FeatureOperations::NewBodyFeatureOperation);
    revInput->setAngleExtent(false, ValueInput::createByReal(angelRad));
    auto feature = component->features()->revolveFeatures()->add(revInput);
    return feature;
}

Ptr<RevolveFeature> Revolve(Ptr<Component> component, Ptr<Sketch> sketch, Ptr<ConstructionAxis> axis, double angelRad)
{
    return Revolve(component, sketch->profiles()->item(0), axis, angelRad);
}

Ptr<ExtrudeFeature> Extrude(Ptr<Component> component, Ptr<Profile> profile, double distance, bool isSymetric)
{
    auto input = component->features()->extrudeFeatures()->createInput(profile, FeatureOperations::NewBodyFeatureOperation);
    input->setDistanceExtent(isSymetric, ValueInput::createByReal(distance));
    auto feature = component->features()->extrudeFeatures()->add(input);
    return feature;
}

Ptr<ExtrudeFeature> Extrude(Ptr<Component> component, Ptr<Sketch> sketch, double distance, bool isSymetric)
{
    return Extrude(component, sketch->profiles()->item(0), distance, isSymetric);
}

Ptr<BRepBody> Rotate(Ptr<Component> component, Ptr<BRepBody> body, Ptr<ConstructionAxis> axis, double angel, bool createCopy)
{
	Ptr<BRepBody> moveBody = createCopy ? body->copyToComponent(component) : body;

	auto matrix3D = Matrix3D::create();
	auto vector3D = Vector3D::create();
	auto point3D = Point3D::create();
	axis->geometry()->getData(point3D, vector3D);
	matrix3D->setToRotation(angel, vector3D, point3D);
	auto moveFeatures = component->features()->moveFeatures();

	Ptr<ObjectCollection> collection = ObjectCollection::create();
	collection->add(moveBody);

	moveFeatures->add(moveFeatures->createInput(collection, matrix3D));
	return moveBody;
}

Ptr<BRepBody> Combine(Ptr<Component> component, FeatureOperations operation, Ptr<BRepBody> body1, Ptr<BRepBody> body2)
{
	auto combineFeatures = component->features()->combineFeatures();

	Ptr<ObjectCollection> collection = ObjectCollection::create();
	collection->add(body2);

	auto joinInput = combineFeatures->createInput(body1, collection);
	joinInput->operation(operation);
	return combineFeatures->add(joinInput)->bodies()->item(0);
}

Ptr<FilletFeature> Fillet(Ptr<Component> component, Ptr<ObjectCollection> edges, double val)
{
	auto filletFeatures = component->features()->filletFeatures();
	auto filletInput = filletFeatures->createInput();
	filletInput->addConstantRadiusEdgeSet(edges, ValueInput::createByReal(val), false);
	return filletFeatures->add(filletInput);
}

Ptr<ObjectCollection> GetEdges(Ptr<BRepBody> body, std::function <bool(Ptr<BRepEdge>)> isGoodEdge)
{
	auto collection = ObjectCollection::create();
	auto edges = body->edges();
	for (int i = 0; i < edges->count(); i++)
	{
		auto edge = edges->item(i);
		//if (abs(edge->length() - length) < 0.000001)
		if (isGoodEdge(edge))
			collection->add(edge);
	}
	return collection;
}

Ptr<Vector3D> ConstructionAxisToVector3D(Ptr<ConstructionAxis> axis)
{
	auto vector = Vector3D::create();
	auto point = Point3D::create();
	axis->geometry()->getData(point, vector);
	return vector;
}
#include "ComponentUtils.h"


Ptr<Point3D> GetCenterPoint()
{
	return Point3D::create(0, 0, 0);
}

Ptr<Point3D> GetCirclePoint(double radius, double angel)
{
	return Point3D::create(radius * cos(angel), radius * sin(angel), 0);
}

Ptr<Point3D> GetCirclePoint(Ptr<Point3D> circleCenter, double radius, double angel)
{
    return Point3D::create(circleCenter->x() + radius * cos(angel), circleCenter->y() + radius * sin(angel), 0);
}

bool Equal(double a, double b, double delta)
{
    return abs(a - b) < delta;
}

bool Equal(Ptr<Point3D> p1, Ptr<Point3D> p2, double delta)
{
    return Equal(p1->x(), p2->x(), delta) && Equal(p1->y(), p2->y(), delta) && Equal(p1->z(), p2->z(), delta);
}

bool Equal(Ptr<SketchLine> line, Ptr<BRepEdge> edge, double delta)
{
    auto startPoint1 = line->geometry()->startPoint();
    auto endPoint1 = line->geometry()->endPoint();
    auto startPoint2 = edge->startVertex()->geometry();
    auto endPoint2 = edge->endVertex()->geometry();

    return
        (Equal(startPoint1, startPoint2, delta) && Equal(endPoint1, endPoint2, delta)) ||
        (Equal(startPoint1, endPoint2, delta) && Equal(endPoint1, startPoint2, delta));
}

bool EqualAny(Ptr<BRepEdge> edge, Ptr<SketchLines> lines, double delta)
{
    for (int i = 0; i < lines->count(); i++)
        if (Equal(lines->item(i), edge, delta))
            return true;
    return false;
}

bool isPointOnFace(Ptr<BRepFace> face, Ptr<Point3D> point, double tolerance)
{
    Ptr<Point2D> parameter;
    Ptr<Point3D> newPoint;
    face->evaluator()->getParameterAtPoint(point, parameter);
    face->evaluator()->getPointAtParameter(parameter, newPoint);

    return point->isEqualToByTolerance(newPoint, tolerance);
}

bool isPointOnEdge(Ptr<BRepEdge> edge, Ptr<Point3D> point, double tolerance)
{
    double parameter;
    Ptr<Point3D> newPoint;
    edge->evaluator()->getParameterAtPoint(point, parameter);
    edge->evaluator()->getPointAtParameter(parameter, newPoint);

    return point->isEqualToByTolerance(newPoint, tolerance);
}

Ptr<Point3D> getPointOnSphere(Ptr<Point3D> center, double radius, Ptr<Vector3D> vector)
{
    vector->normalize();
    return Point3D::create(center->x() + radius * vector->x(), center->y() + radius * vector->y(), center->z() + radius * vector->z());
}

double getTriangleSideLength(double side1, double side2, double angleInRadians)
{
    return sqrt(pow(side1, 2) + pow(side2, 2) - 2 * side1 * side2 * cos(angleInRadians));
}

double getTriangleLeg(double legB, double angleBInRadians)
{
    return legB * tan(angleBInRadians);
}

Ptr<BRepFace> getBodyFace(Ptr<BRepBody> body, CubeFaceType faceType)
{
    Ptr<Vector3D> normalVector = Vector3D::create(
        faceType == Right ? 1 : (faceType == Left ? -1 : 0),
        faceType == Fron ? 1 : (faceType == Back ? -1 : 0),
        faceType == Top ? 1 : (faceType == Bottom ? -1 : 0));

    auto faces = body->faces();
    
    for (int i = 0; i < faces->count(); i++)
    {
        auto face = faces->item(i);
        auto point = face->pointOnFace();
        Ptr<Vector3D> normal;
        face->evaluator()->getNormalAtPoint(point, normal);
        
        if (normal->angleTo(normalVector) < RAD_90)
            return face;
    }
    return NULL;
}

Ptr<BRepFace> getBodyFace(Ptr<BRepBody> body, Ptr<Point3D> pointOnFace, double tolerance)
{
    auto faces = body->faces();

    for (int i = 0; i < faces->count(); i++)
    {
        auto face = faces->item(i);
        
        if (isPointOnFace(face, pointOnFace, tolerance))
            return face;
    }
    return NULL;
}

Ptr<BRepEdge> getJoinedEdge(Ptr<BRepFace> face1, Ptr<BRepFace> face2)
{
    auto edges1 = face1->edges();
    auto edges2 = face2->edges();
    for (int i = 0; i < edges1->count(); i++)
    {
        auto edge1 = edges1->item(i);
        for (int j = 0; j < edges2->count(); j++)
            if (edge1 == edges2->item(j))
                return edge1;
    }
    return nullptr;
}

void MessageBox(std::string message)
{
    Application::get()->userInterface()->messageBox(message);
}

void addToObjectCollection(Ptr<ObjectCollection> objectCollection, std::initializer_list<Ptr<Base>> items)
{
    for (auto item : items)
        objectCollection->add(item);
}

Ptr<ObjectCollection> createObjectCollection(std::initializer_list<Ptr<Base>> items)
{
    auto collection = ObjectCollection::create();
    addToObjectCollection(collection, items);
    return collection;
}


Ptr<Sketch> CreateSketch(Ptr<Component> component, Ptr<ConstructionPlane> plane, std::string name)
{
    auto sketch = component->sketches()->add(plane);
    sketch->name(name);
    return sketch;
}


void Rotate(Ptr<Sketch> sketch, double angel, Ptr<Point3D> point, Ptr<ObjectCollection> items)
{
    auto normal = sketch->xDirection()->crossProduct(sketch->yDirection());
    auto matrix = Matrix3D::create();
    matrix->setToRotation(angel, normal, point);
    sketch->move(items, matrix);
}

void Rotate(Ptr<Sketch> sketch, double angel, Ptr<Point3D> point, std::initializer_list<Ptr<Base>> items)
{
    Rotate(sketch, angel, point, createObjectCollection(items));
}

void Rotate(Ptr<Sketch> sketch, double angel, Ptr<Point3D> point)
{
    auto items = ObjectCollection::create();
    auto curves = sketch->sketchCurves();
    for (int i = 0; i < curves->count(); i++)
        items->add(curves->item(i));
    Rotate(sketch, angel, point, items);
}


Ptr<ConstructionPoint> AddConstructionPoint(Ptr<Component> component, Ptr<Base> point)
{
    auto input = component->constructionPoints()->createInput();
    input->setByPoint(point);
    return component->constructionPoints()->add(input);
}

Ptr<ConstructionAxis> AddConstructionAxis(Ptr<Component> component, Ptr<Point3D> point, Ptr<Vector3D> vector)
{
    auto designType = component->parentDesign()->designType();
    component->parentDesign()->designType(DesignTypes::DirectDesignType);
    auto input = component->constructionAxes()->createInput();
    auto line = InfiniteLine3D::create(point, vector);
    input->setByLine(line);
    auto axis = component->constructionAxes()->add(input);
    component->parentDesign()->designType(designType);
    return axis;
}

Ptr<ConstructionAxis> AddConstructionAxis(Ptr<Component> component, Ptr<Vector3D> vector)
{
    return AddConstructionAxis(component, component->originConstructionPoint()->geometry(), vector);
}

Ptr<SketchCircle> AddCircle(Ptr<Sketch> sketch, Ptr<Point3D> circleCentr, double radius)
{
    return sketch->sketchCurves()->sketchCircles()->addByCenterRadius(circleCentr, radius);
}

Ptr<SketchArc> AddArc(Ptr<Sketch> sketch, Ptr<Point3D> circleCentr, double radius, double length, double pivotAngelInRadian, bool pivotAngelIsCenterOfArc)
{
	double lengthRad = length / radius;
	return sketch->sketchCurves()->sketchArcs()->addByCenterStartSweep(circleCentr, GetCirclePoint(radius, pivotAngelIsCenterOfArc ? pivotAngelInRadian - lengthRad / 2.0 : pivotAngelInRadian), lengthRad);
}

Ptr<SketchArc> AddArc(Ptr<Sketch> sketch, Ptr<Point3D> circleCentr, Ptr<Point3D> startPoint, Ptr<Point3D> endPoint)
{
    return sketch->sketchCurves()->sketchArcs()->addByCenterStartEnd(circleCentr, startPoint, endPoint);
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

Ptr<ObjectCollection> ExtrudeAll(Ptr<Component> component, Ptr<Sketch> sketch, double distance, bool isSymetric)
{
    auto collection = ObjectCollection::create();
    for(int i =0; i< sketch->profiles()->count();i++)
        collection->add(Extrude(component, sketch->profiles()->item(i), distance, isSymetric));
    
    return collection;
}

Ptr<BRepBody> Move(Ptr<Component> component, Ptr<BRepBody> body, Ptr<ConstructionAxis> axis, double distance, bool createCopy)
{
    auto moveBody = createCopy ? body->copyToComponent(component) : body;
    auto moveFeatures = component->features()->moveFeatures();

    auto collection = ObjectCollection::create();
    collection->add(moveBody);

    auto input = moveFeatures->createInput2(collection);
    auto res = input->defineAsTranslateAlongEntity(axis, ValueInput::createByReal(distance));
    moveFeatures->add(input);
    
    return moveBody;
}

Ptr<BRepBody> Rotate(Ptr<Component> component, Ptr<BRepBody> body, Ptr<ConstructionAxis> axis, double angel, bool createCopy)
{
	auto moveBody = createCopy ? body->copyToComponent(component) : body;
	auto moveFeatures = component->features()->moveFeatures();

	auto collection = ObjectCollection::create();
	collection->add(moveBody);
    
    auto input = moveFeatures->createInput2(collection);
    input->defineAsRotate(axis, ValueInput::createByReal(angel));
	moveFeatures->add(input);
	
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
		
		if (isGoodEdge(edge))
			collection->add(edge);
	}
	return collection;
}

Ptr<ObjectCollection> GetEdges(std::vector<Ptr<BRepFace>> joinedFaces, std::vector<Ptr<BRepFace>> unjoinedFaces)
{
    auto collection = ObjectCollection::create();
    auto unjoinedEdges = ObjectCollection::create();
    
    for (int i = 0; i < unjoinedFaces.size(); i++)
    {
        auto face = unjoinedFaces[i];
        auto edges = face->edges();
        for (int j = 0; j < edges->count(); j++)
            unjoinedEdges->add(edges->item(j));
    }

    for (int i = 0; i < joinedFaces.size(); i++)
    {
        auto face = joinedFaces[i];
        auto edges = face->edges();
        for (int j = 0; j < edges->count(); j++)
        {
            auto edge = edges->item(j);
            if (unjoinedEdges->find(edge) < 0 && collection->find(edge) < 0)
                collection->add(edge);
        }
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


Ptr<BRepBody> CreateCylinder(Ptr<Component> component, Ptr<Point3D> center, double radius, double height)
{
    auto sketch = CreateSketch(component, component->xYConstructionPlane(), "CilinderSketch");
    AddCircle(sketch, center, radius);
    return Extrude(component, sketch->profiles()->item(0), height)->bodies()->item(0);
}

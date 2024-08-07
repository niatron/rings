#include "FusionEnvironment.h"


Ptr<Point3D> GetCenterPoint()
{
	return Point3D::create(0, 0, 0);
}

Ptr<Point3D> GetCirclePoint(double radius, double angel)
{
	return Point3D::create(radius * cos(angel), radius * sin(angel), 0);
}

Ptr<Point3D> GetCirclePoint(Ptr<Point3D> circleCenter, double radius, double angel, bool saveZ)
{
    return Point3D::create(circleCenter->x() + radius * cos(angel), circleCenter->y() + radius * sin(angel), saveZ ? circleCenter->z() : 0);
}


Ptr<BoundingBox3D> CreateBound(double top, double bottom, double left, double right)
{
    return BoundingBox3D::create(Point3D::create(left, bottom), Point3D::create(right, top));
}

Ptr<BoundingBox3D> CutShell(Ptr<BoundingBox3D> box, double shellThickness)
{
    auto minPoint = box->minPoint();
    auto maxPoint = box->maxPoint();
    auto newMinPoint = Point3D::create(minPoint->x() + shellThickness, minPoint->y() + shellThickness);
    auto newMaxPoint = Point3D::create(maxPoint->x() - shellThickness, maxPoint->y() - shellThickness);
    return BoundingBox3D::create(newMinPoint, newMaxPoint);
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

DialogResults MessageBox(std::string message, std::string title, MessageBoxButtonTypes buttonType)
{
    return Application::get()->userInterface()->messageBox(message, title, buttonType);
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


void Rotate(Ptr<SectionAnalysis> analysis, double angel, Ptr<Vector3D> axis, Ptr<Point3D> originPointOfAxis)
{
    Ptr<Plane> plane = analysis->cutPlane();
    auto matrix = Matrix3D::create();
    matrix->setToRotation(angel, axis, originPointOfAxis);
    analysis->transform(matrix);
}

void Rotate(Ptr<SectionAnalysis> analysis, double angel, Ptr<ConstructionAxis> axis)
{
    auto vectorPoint = ConstructionAxisToVectorPoint(axis);
    Rotate(analysis, angel, vectorPoint.vector, vectorPoint.point);
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

Ptr<ObjectCollection> AddRectangle(Ptr<Sketch> sketch, Ptr<Point3D> center, double width, double height, double cornerRadius)
{
    auto horizontalLineLength = width - 2.0 * cornerRadius;
    auto verticalLineLength = height - 2.0 * cornerRadius;

    auto p1 = Point3D::create(center->x() - horizontalLineLength / 2.0, center->y() + height / 2.0);
    auto p2 = Point3D::create(p1->x() + horizontalLineLength, p1->y());
    auto p3 = Point3D::create(p2->x() + cornerRadius, p2->y() - cornerRadius);
    auto p4 = Point3D::create(p3->x(), p3->y() - verticalLineLength);
    auto p5 = Point3D::create(p2->x(), p2->y() - height);
    auto p6 = Point3D::create(p1->x(), p1->y() - height);
    auto p7 = Point3D::create(p4->x() - width, p4->y());
    auto p8 = Point3D::create(p3->x() - width, p3->y());

    auto curves = createObjectCollection({
        AddLine(sketch, p1, p2),
        AddArc(sketch, Point3D::create(p2->x(), p3->y()), p3, p2),
        AddLine(sketch, p3, p4),
        AddArc(sketch, Point3D::create(p5->x(), p4->y()), p5, p4),
        AddLine(sketch, p5, p6),
        AddArc(sketch, Point3D::create(p6->x(), p7->y()), p7, p6),
        AddLine(sketch, p7, p8),
        AddArc(sketch, Point3D::create(p1->x(), p8->y()), p1, p8)
        });
    return curves;
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


Ptr<ExtrudeFeature> Extrude(Ptr<Component> component, Ptr<ObjectCollection> collection, double distance, bool isSymetric)
{
    auto input = component->features()->extrudeFeatures()->createInput(collection, FeatureOperations::NewBodyFeatureOperation);
    input->setDistanceExtent(isSymetric, ValueInput::createByReal(distance));
    auto feature = component->features()->extrudeFeatures()->add(input);
    return feature;
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
    return CreateCombineFeature(component, operation, body1, body2)->bodies()->item(0);
}

Ptr<CombineFeature> CreateCombineFeature(Ptr<Component> component, FeatureOperations operation, Ptr<BRepBody> body1, Ptr<BRepBody> body2)
{
    auto combineFeatures = component->features()->combineFeatures();

    Ptr<ObjectCollection> collection = ObjectCollection::create();
    collection->add(body2);

    auto joinInput = combineFeatures->createInput(body1, collection);
    joinInput->operation(operation);
    return combineFeatures->add(joinInput);
}

Ptr<FilletFeature> Fillet(Ptr<Component> component, Ptr<ObjectCollection> edges, double val)
{
	auto filletFeatures = component->features()->filletFeatures();
	auto filletInput = filletFeatures->createInput();
	filletInput->addConstantRadiusEdgeSet(edges, ValueInput::createByReal(val), false);
	return filletFeatures->add(filletInput);
}


bool HasCommonEdge(Ptr<BRepFace> face1, Ptr<BRepFace> face2)
{
    for (auto edge1 : face1->edges())
        for (auto edge2 : face2->edges())
            if (edge1->entityToken() == edge2->entityToken() )
                return true;
    return false;
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

Ptr<ObjectCollection> GetEdges(Ptr<ObjectCollection> edges, std::function <bool(Ptr<BRepEdge>)> isGoodEdge)
{
    auto collection = ObjectCollection::create();
    
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

double GetMin(Ptr<ObjectCollection> items, std::function <double(Ptr<Base>)> getValue)
{
    double result = INT32_MAX;
    for (int i = 0; i < items->count(); i++)
    {
        auto value = getValue(items->item(i));

        if (result > value)
            result = value;
    }
    return result;
}


Ptr<ObjectCollection> GetProfiles(Ptr<Sketch> sketch, std::function <bool(Ptr<Profile>)> isGoodProfile)
{
    {
        auto collection = ObjectCollection::create();
        auto profiles = sketch->profiles();
        for (int i = 0; i < profiles->count(); i++)
        {
            auto profile = profiles->item(i);

            if (isGoodProfile(profile))
                collection->add(profile);
        }
        return collection;
    }
}


Ptr<Vector3D> ConstructionAxisToVector3D(Ptr<ConstructionAxis> axis)
{
	auto vector = Vector3D::create();
	auto point = Point3D::create();
	axis->geometry()->getData(point, vector);
	return vector;
}

VectorPoint ConstructionAxisToVectorPoint(Ptr<ConstructionAxis> axis)
{
    VectorPoint vectorPoint;
    axis->geometry()->getData(vectorPoint.point, vectorPoint.vector);
    return vectorPoint;
}

Ptr<BRepBody> CreateSphere(Ptr<Component> component, Ptr<Point3D> center, double radius)
{
    auto sketch = CreateSketch(component, component->xYConstructionPlane(), "SphereSketch");
    auto startPoint = Point3D::create(-radius);
    auto endPoint = Point3D::create(radius);
    AddArc(sketch, Point3D::create(), startPoint, endPoint);
    AddLine(sketch, startPoint, endPoint);
    auto body = Revolve(component, sketch, component->xConstructionAxis(), RAD_360)->bodies()->item(0);
    body = Move(component, body, component->xConstructionAxis(), center->x());
    body = Move(component, body, component->yConstructionAxis(), center->y());
    body = Move(component, body, component->zConstructionAxis(), center->z());
    return body;
}

Ptr<BRepBody> CreateCylinder(Ptr<Component> component, Ptr<Point3D> center, double radius, double height)
{
    auto sketch = CreateSketch(component, component->xYConstructionPlane(), "CilinderSketch");
    AddCircle(sketch, center, radius);
    return Extrude(component, sketch->profiles()->item(0), height)->bodies()->item(0);
}

Ptr<BRepBody> CreateBox(Ptr<Component> component, Ptr<Point3D> point1, Ptr<Point3D> point2, double height)
{
    auto sketch = CreateSketch(component, component->xYConstructionPlane(), "BoxSketch");
    auto line1 = AddLine(sketch, Point3D::create(point1->x(), point1->y()), Point3D::create(point2->x(), point1->y()));
    auto line2 = AddLine(sketch, line1->endSketchPoint()->geometry(), Point3D::create(point2->x(), point2->y()));
    auto line3 = AddLine(sketch, line2->endSketchPoint()->geometry(), Point3D::create(point1->x(), point2->y()));
    auto line4 = AddLine(sketch, line3->endSketchPoint()->geometry(), line1->startSketchPoint()->geometry());
    return Extrude(component, sketch->profiles()->item(0), height)->bodies()->item(0);
}

Ptr<BRepBody> CreateBox(Ptr<Component> component, Ptr<Point3D> point1, Ptr<Point3D> point2, double height, double verticalCornerFilletRadius)
{
    auto body = CreateBox(component, point1, point2, height);
    if (verticalCornerFilletRadius > 0)
    {
        auto edges = GetEdges(body, EdgeIsVerticalLine);
        Fillet(component, edges, verticalCornerFilletRadius);
    }
    return body;
}

Ptr<BRepBody> CreateBox(Ptr<Component> component, Ptr<Point3D> point1, Ptr<Point3D> point2, double height, double verticalCornerFilletRadius, double wallThicknness)
{
    auto body = CreateBox(component, point1, point2, height, verticalCornerFilletRadius);
    if (wallThicknness <= 0)
        return body;
    auto xMax = fmax(point1->x(), point2->x()) - wallThicknness;
    auto xMin = fmin(point1->x(), point2->x()) + wallThicknness;
    auto yMax = fmax(point1->y(), point2->y()) - wallThicknness;
    auto yMin = fmin(point1->y(), point2->y()) + wallThicknness;
    auto cutBody = CreateBox(component, Point3D::create(xMax, yMax), Point3D::create(xMin, yMin), height, verticalCornerFilletRadius - wallThicknness);
    return Combine(component, CutFeatureOperation, body, cutBody);
}

bool EdgeIsHorizontal(Ptr<BRepEdge> edge)
{
    return Equal(abs(edge->startVertex()->geometry()->z() - edge->endVertex()->geometry()->z()), 0, 0.01) && !edge->isDegenerate();
}

bool EdgeIsVerticalLine(Ptr<BRepEdge> edge)
{
    return Equal(abs(edge->startVertex()->geometry()->z() - edge->endVertex()->geometry()->z()), edge->length(), 0.01) && !edge->isDegenerate();
}

bool BodyContainPoint(Ptr<BRepBody> body, Ptr<Point3D> point)
{
    auto pointContainment = body->pointContainment(point);
    return pointContainment == PointInsidePointContainment || pointContainment == PointOnPointContainment;
}

Ptr<SectionAnalysis> AddSectionAnalysis(Ptr<Component> component, Ptr<Base> plane, double distance)
{
    auto input = component->parentDesign()->analyses()->sectionAnalyses()->createInput(plane, distance);
    return component->parentDesign()->analyses()->sectionAnalyses()->add(input);
}

void SaveAsStl(Ptr<BRepBody> body, std::string filepath)
{
    auto exportManager = body->parentComponent()->parentDesign()->exportManager();
    auto stlOptions = exportManager->createSTLExportOptions(body, filepath);
    stlOptions->meshRefinement(MeshRefinementSettings::MeshRefinementHigh);
    exportManager->execute(stlOptions);
}

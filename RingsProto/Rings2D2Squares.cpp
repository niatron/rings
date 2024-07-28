#include "Rings2D2Squares.h"
#include "Geometry.h"
#include "FusionEnvironment.h"

#define _USE_MATH_DEFINES
#include <math.h>

using namespace adsk::core;
using namespace adsk::fusion;

Rings2D2Squares::Rings2D2Squares()
{
}

double Rings2D2Squares::getVolfRadius()
{
    // innerCornerRadius + volfRadius = middleCornerRadius
    // volfDiametr = 2 * volfRadius;
    // volfDiametr^2 = middleCornerRadius^2 + middleCornerRadius^2 - 2 * middleCornerRadius * middleCornerRadius * cos(2*pi / 4*cornerVolfCount) //теорема косинусов
    // volfDiametr = middleCornerRadius * sqrt(2)*sqrt(1 - cos(2*pi / 4*cornerVolfCount))
    // rate = sqrt(2)*sqrt(1 - cos(2*pi / 4*cornerVolfCount))
    // volfDiametr = middleCornerRadius * rate
    // middleCornerRadius = (squareMiddleSize - lineVolfCount * volfDiametr)/2
    // volfDiametr = (squareMiddleSize - lineVolfCount * volfDiametr) * rate / 2
    // volfDiametr = (squareMiddleSize * rate / 2) / (1 + lineVolfCount * rate / 2)
    
    auto rate = sqrt(2.0) * sqrt(1 - cos(RAD_360 / (4.0 * cornerVolfCount)));
    auto volfDiametr = (squareMiddleSize * rate / 2.0) / (1.0 + lineVolfCount * rate / 2.0);
    return volfDiametr / 2.0;
}

double Rings2D2Squares::getLineLength()
{
    return lineVolfCount * getVolfRadius() * 2.0;
}

double Rings2D2Squares::getCornerOuterRadius()
{
    return (squareMiddleSize - getLineLength()) / 2.0 + getVolfRadius();
}

double Rings2D2Squares::getSquareShift()
{
    auto sizeByVolfCenter = squareMiddleSize;
    return sizeByVolfCenter / (2.0 * sqrt(2.0));
}

Ptr<Point3D> Rings2D2Squares::getLeftCenterPoint()
{
    return Point3D::create(-getSquareShift());
}

Ptr<Point3D> Rings2D2Squares::getRightCenterPoint()
{
    return Point3D::create(getSquareShift());
}

void Rings2D2Squares::SetParams(BasePart& basePart, RoofPart& roofPart, VolfUpPart& volfUpPart, VolfDownPart& volfDownPart)
{
    auto volfRadius = getVolfRadius() - moovableClearence / (cornerVolfCount * 4.0 + lineVolfCount * 4.0);

    basePart.floorThickness = 0.3;
    basePart.wallThickness = 0.16;
    basePart.circlesOnSquareRadius = 0.25;

    roofPart.floorThickness = 0.3;
    roofPart.wallThickness = 0.16;

    volfDownPart.radius = volfRadius;
    volfDownPart.height = 0.54;
    volfDownPart.holeDownRadius = 0.28;
    volfDownPart.holeDownHeight = 0.3;
    volfDownPart.holeRadius = 0.15;

    volfUpPart.height = 0.5;
    volfUpPart.middleRadius = 0.3;
    volfUpPart.holeRadius = 0.14;
    volfUpPart.form = VolfUpPart::convex;
    volfUpPart.concaveHeight = 0.15;

    volfDownPart.holeHeight = volfDownPart.height;
    volfDownPart.zMoveShift = basePart.floorThickness + moovableClearence;
    volfDownPart.filletRadius = horizontalEdgeFilletRadius;

    volfUpPart.radius = volfRadius;
    volfUpPart.middleHeight = roofPart.floorThickness;
    volfUpPart.holeHeight = volfUpPart.middleHeight + volfUpPart.height;
    volfUpPart.concaveRadius = volfRadius * 3.0;
    volfUpPart.convexRadius = volfRadius * 1.5;
    volfUpPart.filletRadius = horizontalEdgeFilletRadius;

    basePart.lineLength = getLineLength();
    basePart.cornerMiddleRadius = getCornerOuterRadius() - getVolfRadius();
    basePart.innerWidth = getVolfRadius() + basePart.wallThickness + moovableClearence * 1.5;
    basePart.outerWidth = getVolfRadius() + basePart.wallThickness + moovableClearence * 0.5;
    basePart.height = basePart.floorThickness + volfDownPart.height + 2.0 * moovableClearence;
    basePart.leftCenterPoint = getLeftCenterPoint();
    basePart.rightCenterPoint = getRightCenterPoint();
    basePart.circlesOnSquarePeriodRadius = getVolfRadius();
    basePart.topEdgeFilletRadius = horizontalEdgeFilletRadius / 2.0;
    basePart.verticalEdgeFilletRadius = verticalEdgeFilletRadius;
    basePart.otherEdgeFilletRadius = horizontalEdgeFilletRadius;

    roofPart.lineLength = getLineLength();
    roofPart.cornerMiddleRadius = getCornerOuterRadius() - getVolfRadius();
    roofPart.innerWidth = basePart.innerWidth + roofPart.wallThickness;
    roofPart.outerWidth = basePart.outerWidth + roofPart.wallThickness;
    roofPart.height = basePart.height + roofPart.floorThickness + unmoovableClearence;
    roofPart.leftCenterPoint = basePart.leftCenterPoint;
    roofPart.rightCenterPoint = basePart.rightCenterPoint;
    roofPart.separationInnerWidth = volfUpPart.middleRadius + moovableClearence * 1.5;
    roofPart.separationOuterWidth = volfUpPart.middleRadius + moovableClearence * 0.5;
    roofPart.downTrimmingThicknes = moovableClearence * 2.0;
    //roofPart.circlesOnSquarePeriodRadius = getVolfRadius();
    roofPart.topEdgeFilletRadius = horizontalEdgeFilletRadius;
    roofPart.verticalEdgeFilletRadius = verticalEdgeFilletRadius;
    roofPart.otherEdgeFilletRadius = horizontalEdgeFilletRadius / 2.0;

    volfUpPart.zMoveShift = basePart.height + unmoovableClearence + roofPart.floorThickness + moovableClearence;
}


void Rings2D2Squares::SetParams(RectangledBasePart& basePart, RectangledRoofPart& roofPart, VolfUpPart& volfUpPart, VolfDownPart& volfDownPart)
{
    auto volfRadius = getVolfRadius() - moovableClearence / (cornerVolfCount * 4.0 + lineVolfCount * 4.0);

    basePart.floorThickness = 0.3;
    basePart.wallThickness = 0.16;
    basePart.circlesOnSquareRadius = 0.25;

    roofPart.floorThickness = 0.3;
    roofPart.wallThickness = 0.16;
    roofPart.cornerFilletRadius = getVolfRadius() * 2.0;

    volfDownPart.radius = volfRadius;
    volfDownPart.height = 0.44;
    volfDownPart.holeDownRadius = 0.28;
    volfDownPart.holeDownHeight = 0.3;
    volfDownPart.holeRadius = 0.15;

    volfUpPart.height = 0.5;
    volfUpPart.middleRadius = 0.3;
    volfUpPart.holeRadius = 0.14;
    volfUpPart.form = VolfUpPart::convex;
    volfUpPart.concaveHeight = 0.15;

    volfDownPart.holeHeight = volfDownPart.height;
    volfDownPart.zMoveShift = basePart.floorThickness + moovableClearence;
    volfDownPart.filletRadius = horizontalEdgeFilletRadius;

    volfUpPart.radius = volfRadius;
    volfUpPart.middleHeight = roofPart.floorThickness;
    volfUpPart.holeHeight = volfUpPart.middleHeight + volfUpPart.height;
    volfUpPart.concaveRadius = volfRadius * 3.0;
    volfUpPart.convexRadius = volfRadius * 1.5;
    volfUpPart.filletRadius = horizontalEdgeFilletRadius;

    basePart.lineLength = getLineLength();
    basePart.cornerMiddleRadius = getCornerOuterRadius() - getVolfRadius();
    basePart.innerWidth = getVolfRadius() + basePart.wallThickness + roofPart.wallThickness + moovableClearence * 1.5;
    basePart.outerWidth = getVolfRadius() + basePart.wallThickness + roofPart.wallThickness + moovableClearence * 0.5;
    basePart.height = basePart.floorThickness + volfDownPart.height + 2.0 * moovableClearence;
    basePart.leftCenterPoint = getLeftCenterPoint();
    basePart.rightCenterPoint = getRightCenterPoint();
    basePart.circlesOnSquarePeriodRadius = getVolfRadius();
    basePart.topEdgeFilletRadius = horizontalEdgeFilletRadius / 2.0;
    basePart.verticalEdgeFilletRadius = verticalEdgeFilletRadius;
    basePart.otherEdgeFilletRadius = horizontalEdgeFilletRadius / 2.0;
    basePart.cuttingShellThickness = basePart.wallThickness + roofPart.wallThickness;
    basePart.cornerFilletRadius = roofPart.cornerFilletRadius - roofPart.wallThickness;
    basePart.centralLinkerRadius = 0.2;
    
    basePart.linkingPart.floorState = LinkingPart::FloorStates::Top;
    basePart.linkingPart.isReverse = false;
    basePart.linkingPart.floorHoleRadius = 0.15;
    basePart.linkingPart.floorThickness = 0.2;
    basePart.linkingPart.wallThickness = 0.16;
    basePart.linkingPart.radius = basePart.linkingPart.wallThickness + 0.34;
    basePart.linkingPart.height = basePart.linkingPart.floorThickness + 0.44;
    basePart.linkingPart.z = 0;

    roofPart.lineLength = getLineLength();
    roofPart.cornerMiddleRadius = getCornerOuterRadius() - getVolfRadius();
    roofPart.innerWidth = basePart.innerWidth - roofPart.wallThickness;
    roofPart.outerWidth = basePart.outerWidth - roofPart.wallThickness;
    roofPart.height = basePart.height + roofPart.floorThickness + unmoovableClearence;
    roofPart.leftCenterPoint = basePart.leftCenterPoint;
    roofPart.rightCenterPoint = basePart.rightCenterPoint;
    roofPart.separationInnerWidth = volfUpPart.middleRadius + moovableClearence * 1.5;
    roofPart.separationOuterWidth = volfUpPart.middleRadius + moovableClearence * 0.5;
    roofPart.downTrimmingThicknes = moovableClearence * 2.0;
    //roofPart.circlesOnSquarePeriodRadius = getVolfRadius();
    roofPart.topEdgeFilletRadius = horizontalEdgeFilletRadius;
    roofPart.verticalEdgeFilletRadius = verticalEdgeFilletRadius;
    roofPart.otherEdgeFilletRadius = horizontalEdgeFilletRadius / 2.0;
    roofPart.deepThickness = basePart.floorThickness;
    roofPart.centralLinkerRadius = basePart.centralLinkerRadius + unmoovableClearence;
    
    roofPart.linkingPart.floorState = LinkingPart::FloorStates::Top;
    roofPart.linkingPart.isReverse = true;
    roofPart.linkingPart.floorThickness = 0.2;
    roofPart.linkingPart.radius = basePart.linkingPart.radius + roofPart.wallThickness;
    roofPart.linkingPart.floorHoleRadius = basePart.linkingPart.radius + unmoovableClearence;
    roofPart.linkingPart.wallThickness = roofPart.linkingPart.radius - 0.14;
    roofPart.linkingPart.z = roofPart.height - roofPart.wallThickness;
    roofPart.linkingPart.height = roofPart.linkingPart.z - basePart.linkingPart.height + roofPart.linkingPart.floorThickness - unmoovableClearence;
    

    volfUpPart.zMoveShift = basePart.height + unmoovableClearence + roofPart.floorThickness + moovableClearence;
}

void Rings2D2Squares::createBodies(Ptr<Component> component)
{
    /*PariedSquaresWithOuterRectanglePart part;

    part.lineLength = getLineLength();
    part.cornerOuterRadius = getCornerOuterRadius();
    part.rotateAngel = RAD_45;
    part.height = 1;
    part.thickness = 0.1;
    part.leftCenterPoint = getLeftCenterPoint();
    part.rightCenterPoint = getRightCenterPoint();
    part.rectangleWidth = 2.0 * part.getRight();
    part.rectangleHeight = 2.0 * part.getTop();
    part.rectangleCornerRadius = getVolfRadius() * 2.0;
    part.initialize(component);
    
    part.createCenterBody();
    part.createInnerWallBody();
    part.createOuterWallBody();
    part.createLeftCenterBody();
    part.createRightCenterBody();
    part.createRectangleBody();

    return;*/

    SetParams(basePart, roofPart, volfUpPart, volfDownPart);

    if (leftAxis == nullptr)
        leftAxis = AddConstructionAxis(component, getLeftCenterPoint(), Vector3D::create(0, 0, 1));
    if (rightAxis == nullptr)
        rightAxis = AddConstructionAxis(component, getRightCenterPoint(), Vector3D::create(0, 0, 1));

    auto volfsSketch = basePart.createCirclesSketch(component, getVolfRadius(), 0);
    volfsSketch->isLightBulbOn(false);

    auto baseBody = basePart.createBody(component);
    auto roofBodies = roofPart.createBodies(component);
    
    Ptr<BRepBody> volfDownBody;
    Ptr<BRepBody> volfUpBody;
    int k = 2;
    for (int i = 0; i < volfsSketch->sketchCurves()->sketchCircles()->count() && k < 2; i++)
    {
        auto volfCenter = volfsSketch->sketchCurves()->sketchCircles()->item(i)->centerSketchPoint()->geometry();
        if (volfCenter->x() < getRightCenterPoint()->x() + 2.0 * getVolfRadius() || volfCenter->y() < 0)
            continue;
        k++;

        volfDownPart.centerPoint = volfCenter;
        volfDownBody = volfDownPart.createBody(component);

        //if (i > 0)
        //    continue;
        volfUpPart.centerPoint = volfCenter;
        volfUpBody = volfUpPart.createBody(component);
        //volfUpBody->isLightBulbOn(false);
    }
    
    auto analysis = AddSectionAnalysis(component, component->xZConstructionPlane(), 0);
    analysis->flip();
    Rotate(analysis, RAD_45, rightAxis);
    
    component->parentDesign()->namedViews()->homeNamedView()->apply();

    if (MessageBox("Save bodies as STL?", "", YesNoButtonType) == DialogNo)
        return;

    std::string modelsFolderPath = "D:\\ServerTechnology\\RingsModels\\2D2S12v3\\";
    
    SaveAsStl(baseBody, modelsFolderPath + "BaseBody.stl");
    for (int i = 0; i < roofBodies->count(); i++)
    {
        Ptr<BRepBody> roofBody = roofBodies->item(i);
        SaveAsStl(roofBody, modelsFolderPath + roofBody->name() + ".stl");
    }
    if (volfUpBody != nullptr && volfDownBody != nullptr)
    {
        SaveAsStl(volfUpBody, modelsFolderPath + "VolfUpBody.stl");
        SaveAsStl(volfDownBody, modelsFolderPath + "VolfDownBody.stl");
    }
    MessageBox("All Done :)");
}
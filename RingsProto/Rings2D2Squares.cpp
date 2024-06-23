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

void Rings2D2Squares::createBodies(Ptr<Component> component)
{
    auto volfRadius = getVolfRadius() - moovableClearence / (cornerVolfCount * 4.0 + lineVolfCount * 4.0);
    
    basePart.floorThickness = 0.3;
    basePart.wallThickness = 0.16;
    basePart.circlesOnSquareRadius = 0.25;

    roofPart.floorThickness = 0.3;
    roofPart.wallThickness = 0.16;

    volfDownPart.radius = volfRadius;
    volfDownPart.height = 0.46;
    volfDownPart.holeDownRadius = 0.32;
    volfDownPart.holeDownHeight = 0.28;
    volfDownPart.holeRadius = 0.15;

    volfUpPart.height = 0.4;
    volfUpPart.middleRadius = 0.3;
    volfUpPart.holeRadius = 0.14;
    volfUpPart.cuttedSphreCuttingHeight = 0.15;

    volfDownPart.holeHeight = volfDownPart.height;
    volfDownPart.zMoveShift = basePart.floorThickness + moovableClearence;
    volfDownPart.filletRadius = horizontalEdgeFilletRadius;

    volfUpPart.radius = volfRadius;
    volfUpPart.middleHeight = roofPart.floorThickness + moovableClearence * 3.0;
    volfUpPart.holeHeight = volfUpPart.middleHeight + volfUpPart.height;
    volfUpPart.cuttedSphreRadius = volfRadius * 3.0;
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
    for (int i = 0; i < volfsSketch->sketchCurves()->sketchCircles()->count() && i < 2; i++)
    {
        auto volfCenter = volfsSketch->sketchCurves()->sketchCircles()->item(i)->centerSketchPoint()->geometry();

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
    component->parentDesign()->namedViews()->homeNamedView()->apply();

    std::string modelsFolderPath = "D:\\ServerTechnology\\RingsModels\\2D2S12v3\\";
    
    SaveAsStl(baseBody, modelsFolderPath + "BaseBody.stl");
    for (int i = 0; i < roofBodies->count(); i++)
    {
        Ptr<BRepBody> roofBody = roofBodies->item(i);
        SaveAsStl(roofBody, modelsFolderPath + roofBody->name() + ".stl");
    }
    SaveAsStl(volfUpBody, modelsFolderPath + "VolfUpBody.stl");
    SaveAsStl(volfDownBody, modelsFolderPath + "VolfDownBody.stl");

}
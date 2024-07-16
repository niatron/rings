#include "Sketcher.h"

void Sketcher::AddSquareCurves(Ptr<Sketch> sketch, Ptr<Point3D> center, double size, double cornerOuterRadius, double rotateAngel)
{
    auto length = 2.0 * cornerOuterRadius + size;

    auto p1 = Point3D::create(center->x() - size / 2, center->y() + cornerOuterRadius + size / 2);
    auto p2 = Point3D::create(p1->x() + size, p1->y());
    auto p3 = Point3D::create(p2->x() + cornerOuterRadius, p2->y() - cornerOuterRadius);
    auto p4 = Point3D::create(p3->x(), p3->y() - size);
    auto p5 = Point3D::create(p2->x(), p2->y() - length);
    auto p6 = Point3D::create(p1->x(), p1->y() - length);
    auto p7 = Point3D::create(p4->x() - length, p4->y());
    auto p8 = Point3D::create(p3->x() - length, p3->y());

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

    Rotate(sketch, rotateAngel, center, curves);
}

void Sketcher::AddCirclesOnSquare(Ptr<Sketch> sketch, Ptr<Point3D> center, double lineLength, double cornerMiddleRadius, double circleRadius, double circlesOnSquarePeriodRadius, double rotateAngel)
{
    auto p1 = Point3D::create(center->x() - lineLength / 2, center->y() + cornerMiddleRadius + lineLength / 2);
    auto p2 = Point3D::create(p1->x() + lineLength, p1->y());
    auto p3 = Point3D::create(p2->x() + cornerMiddleRadius, p2->y() - cornerMiddleRadius);
    auto rotatePointRightUp = Point3D::create(p2->x(), p3->y());

    for (int i = 0; i < 4; i++)
    {
        auto onLineCircle = AddCircle(sketch, Point3D::create(p1->x() + circlesOnSquarePeriodRadius, p1->y()), circleRadius);
        auto onCornerCircle1 = AddCircle(sketch, p2, circleRadius);
        Rotate(sketch, -RAD_45, rotatePointRightUp, { onCornerCircle1 });
        auto onCornerCircle2 = AddCircle(sketch, p2, circleRadius);
        Rotate(sketch, -RAD_45 / 2.0, rotatePointRightUp, { onCornerCircle1,onCornerCircle2 });
        Rotate(sketch, -RAD_90, center);
    }

    Rotate(sketch, rotateAngel, center);
}

Ptr<BRepBody> Sketcher::CreateSquareBody(Ptr<Component> component, Ptr<Point3D> center, double size, double cornerOuterRadius, double rotateAngel, double thickness, double height)
{
    auto sketch = CreateSketch(component, component->xYConstructionPlane(), "SquareSketch");
    Sketcher::AddSquareCurves(sketch, center, size, cornerOuterRadius, rotateAngel);
    Sketcher::AddSquareCurves(sketch, center, size, cornerOuterRadius - thickness, rotateAngel);
    return Extrude(component, sketch, height)->bodies()->item(0);
}

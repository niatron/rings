#include "Geometry.h"

double DegreesToRadians(double degrees)
{
    return degrees * M_PI / 180.0;
}

double GetAngleOfRegularPolygon(int n)
{
    return (double)(n - 2) * 180 / n;
}

double GetTriangleSideLength(double side1, double side2, double angleInRadians)
{
    return sqrt(pow(side1, 2) + pow(side2, 2) - 2.0 * side1 * side2 * cos(angleInRadians));
}

double GetRightTriangleLeg(double otherLeg, double oppositeAngleInRadians)
{
    return otherLeg * tan(oppositeAngleInRadians);
}

double GetRightTriangleLegByHypotenuseAndAdjacentAngle(double hypotenuse, double adjacentAngleInRadians)
{
    return hypotenuse * cos(adjacentAngleInRadians);
}

double GetIsoscelesTriangleLeg(double baseSideLength, double legsAngelInRadians)
{
    return sqrt(pow(baseSideLength, 2) / (2.0 - 2.0 * cos(legsAngelInRadians)));
}
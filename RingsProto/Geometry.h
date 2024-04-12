#pragma once
#define _USE_MATH_DEFINES
#include <math.h>

constexpr auto FULL_CIRCLE_DEG = 360.0;
constexpr auto FULL_CIRCLE_RAD = 2 * M_PI;

double DegreesToRadians(double degrees);
double GetAngleOfRegularPolygon(int n);
double GetTriangleSideLength(double side1, double side2, double angleInRadians);
double GetRightTriangleLeg(double otherLeg, double oppositeAngleInRadians);
double GetRightTriangleLegByHypotenuseAndAdjacentAngle(double hypotenuse, double adjacentAngleInRadians);
double GetIsoscelesTriangleLeg(double baseSideLength, double legsAngelInRadians);
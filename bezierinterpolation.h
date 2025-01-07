#ifndef BEZIERINTERPOLATION_H
#define BEZIERINTERPOLATION_H

#include "vector3.h"
#include <vector>

class BezierInterpolation
{
public:
    // Oblicz współczynnik Newtona
    double static binomialCoefficient(int n, int k);

    // Funkcja bazowa Béziera
    double static bernstein(int i, int n, double t);

    // Punkt na powierzchni Béziera
    Vector3 static bezierSurfacePoint(const std::vector<Vector3>& controlPoints, int n, int m, double u, double v);

    // Obliczanie pochodnej po u
    Vector3 static derivativeU(const std::vector<Vector3>& controlPoints, int n, int m, float u, float v);

    // Obliczanie pochodnej po v
    Vector3 static derivativeV(const std::vector<Vector3>& controlPoints, int n, int m, float u, float v);

    // Obliczanie wektora normalnego
    Vector3 static normal(const std::vector<Vector3>& controlPoints, int n, int m, float u, float v);

};

#endif // BEZIERINTERPOLATION_H

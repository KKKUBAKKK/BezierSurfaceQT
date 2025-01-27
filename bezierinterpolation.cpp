#include "bezierinterpolation.h"
#include <math.h>

double BezierInterpolation::binomialCoefficient(int n, int k) {
    if (k == 0 || k == n) return 1;

    double result = 1;
    for (int i = 1; i <= k; ++i) {
        result *= (n - i + 1) / (double) i;
    }

    return result;
}

double BezierInterpolation::bernstein(int i, int n, double t) {
    return binomialCoefficient(n, i) * pow(t, i) * pow(1 - t, n - i);
}

Vector3 BezierInterpolation::bezierSurfacePoint(const std::vector<Vector3>& controlPoints, int n, int m, double u, double v) {
    Vector3 result = {0, 0, 0};

    for (int i = 0; i <= n; ++i) {
        for (int j = 0; j <= m; ++j) {
            double bu = bernstein(i, n, u);
            double bv = bernstein(j, m, v);
            result.x += controlPoints[i * (m + 1) + j].x * bu * bv;
            result.y += controlPoints[i * (m + 1) + j].y * bu * bv;
            result.z += controlPoints[i * (m + 1) + j].z * bu * bv;
        }
    }

    return result;
}

// Computes partial derivative with respect to u
Vector3 BezierInterpolation::derivativeU(const std::vector<Vector3>& controlPoints,
                                         int n, int m, float u, float v)
{
    Vector3 result(0, 0, 0);

    // For derivative wrt u, differences come from (i+1) - i along the i index
    // i goes up to n-1 because controlPoints for i+1 must exist
    for (int i = 0; i < n; i++) {
        for (int j = 0; j <= m; j++) {
            // Difference: control point (i+1, j) minus (i, j)
            Vector3 diff = controlPoints[(i + 1) * (m + 1) + j]
                           - controlPoints[i * (m + 1) + j];

            // Multiply by n * Bernstein_{n-1, i}(u) * Bernstein_{m, j}(v)
            result = result + diff * (n * bernstein(i, n - 1, u)
                                      * bernstein(j, m, v));
        }
    }
    // printf("Pu: %f, %f, %f\n", result.x, result.y, result.z);
    return result;
}

// Computes partial derivative with respect to v
Vector3 BezierInterpolation::derivativeV(const std::vector<Vector3>& controlPoints,
                                         int n, int m, float u, float v)
{
    Vector3 result(0, 0, 0);

    // For derivative wrt v, differences come from (j+1) - j along the j index
    // j goes up to m-1 because controlPoints for j+1 must exist
    for (int i = 0; i <= n; i++) {
        for (int j = 0; j < m; j++) {
            // Difference: control point (i, j+1) minus (i, j)
            Vector3 diff = controlPoints[i * (m + 1) + (j + 1)]
                           - controlPoints[i * (m + 1) + j];

            // Multiply by m * Bernstein_{n, i}(u) * Bernstein_{m-1, j}(v)
            result = result + diff * (m * bernstein(i, n, u)
                                      * bernstein(j, m - 1, v));
        }
    }
    // printf("Pv: %f, %f, %f\n", result.x, result.y, result.z);
    return result;
}

// Computes the normal vector by crossing the partial derivatives
Vector3 BezierInterpolation::normal(const std::vector<Vector3>& controlPoints,
                                    int n, int m, float u, float v)
{
    Vector3 du = derivativeU(controlPoints, n, m, u, v);
    Vector3 dv = derivativeV(controlPoints, n, m, u, v);

    // Cross product yields the normal
    Vector3 result = du.cross(dv);
    // printf("Normal: %f, %f, %f\n", result.x, result.y, result.z);
    result.normalize();
    return result;
}

#ifndef HELPERS_H
#define HELPERS_H

#include "triangle.h"

#include <array>

class Helpers
{
public:
    static void rotationMatrixZ(float alpha, float matrix[3][3]);
    static void rotationMatrixX(float beta, float matrix[3][3]);
    static float clamp01(float value);
    static float interpolateNormalAndZ(const Triangle &triangle,
                                         float w1, float w2, float w3,
                                       Vector3 &outNormal);
    static std::array<float, 3> getBarycentricCoordinates(
        const std::array<float, 2>& A,
        const std::array<float, 2>& B,
        const std::array<float, 2>& C,
        const std::array<float, 2>& P);
    static float interpolate(const Triangle &triangle, Vector3 &outNormal, float x, float y);
};

#endif // HELPERS_H

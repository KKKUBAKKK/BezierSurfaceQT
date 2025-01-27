#include "helpers.h"
#include "triangle.h"
#include <algorithm>
#include <array>
#include <stdlib.h>
#include <cmath>

// Funkcja do generowania macierzy obrotu wokół osi z
void Helpers::rotationMatrixZ(float alpha, float matrix[3][3]) {
    float rad = alpha * M_PI / 180.0f;
    matrix[0][0] = cos(rad);
    matrix[0][1] = -sin(rad);
    matrix[0][2] = 0;
    matrix[1][0] = sin(rad);
    matrix[1][1] = cos(rad);
    matrix[1][2] = 0;
    matrix[2][0] = 0;
    matrix[2][1] = 0;
    matrix[2][2] = 1;
}

// Funkcja do generowania macierzy obrotu wokół osi x
void Helpers::rotationMatrixX(float beta, float matrix[3][3]) {
    float rad = beta * M_PI / 180.0f;
    matrix[0][0] = 1;
    matrix[0][1] = 0;
    matrix[0][2] = 0;
    matrix[1][0] = 0;
    matrix[1][1] = cos(rad);
    matrix[1][2] = -sin(rad);
    matrix[2][0] = 0;
    matrix[2][1] = sin(rad);
    matrix[2][2] = cos(rad);
}

// Clamps a floating-point value to [0, 1]
float Helpers::clamp01(float value) {
    return std::max(0.0f, std::min(1.0f, value));
}

/**
 * @brief Interpolates the normal vector and z-coordinate for a pixel inside a triangle.
 *
 * @param triangle  An array of 3 Vertex instances defining the triangle.
 * @param w1        Barycentric weight for the first Vertex.
 * @param w2        Barycentric weight for the second Vertex.
 * @param w3        Barycentric weight for the third Vertex.
 * @param outNormal Reference to a Vector3 that will hold the interpolated normal.
 * @return          Interpolated z-coordinate (float).
 *
 * @note 1) outNormal is normalized before returning.
 *       2) Barycentric coordinates (w1, w2, w3) must sum to 1 for correct interpolation.
 */
float Helpers::interpolateNormalAndZ(const Triangle &triangle,
                            float w1, float w2, float w3,
                            Vector3 &outNormal)
{
    // Interpolate normal (N_after) using barycentric coordinates
    outNormal.x = (w1 * triangle.vertices[0].N_after.x) +
                  (w2 * triangle.vertices[1].N_after.x) +
                  (w3 * triangle.vertices[2].N_after.x);

    outNormal.y = (w1 * triangle.vertices[0].N_after.y) +
                  (w2 * triangle.vertices[1].N_after.y) +
                  (w3 * triangle.vertices[2].N_after.y);

    outNormal.z = (w1 * triangle.vertices[0].N_after.z) +
                  (w2 * triangle.vertices[1].N_after.z) +
                  (w3 * triangle.vertices[2].N_after.z);

    // Normalize the resulting normal
    outNormal.normalize();

    // Interpolate the z-coordinate from P_after
    float zInterpolated = (w1 * triangle.vertices[0].P_after.z) +
                          (w2 * triangle.vertices[1].P_after.z) +
                          (w3 * triangle.vertices[2].P_after.z);

    return zInterpolated;
}

/**
 * @brief Computes the barycentric coordinates of point P with respect to triangle ABC.
 *
 * If P is inside the triangle, all w1, w2, and w3 will be between 0 and 1
 * and their sum is always 1.
 *
 * @param A 2D coordinates of the first vertex of the triangle (Ax, Ay).
 * @param B 2D coordinates of the second vertex of the triangle (Bx, By).
 * @param C 2D coordinates of the third vertex of the triangle (Cx, Cy).
 * @param P 2D coordinates of the point for which we want barycentric coords (Px, Py).
 *
 * @return An array [w1, w2, w3] representing the barycentric coordinates of P.
 */
std::array<float, 3> Helpers::getBarycentricCoordinates(
    Triangle triangle, Vector3 P)
{
    float Ax = triangle.vertices[0].P_after.x;
    float Ay = triangle.vertices[0].P_after.y;
    float Bx = triangle.vertices[1].P_after.x;
    float By = triangle.vertices[1].P_after.y;
    float Cx = triangle.vertices[2].P_after.x;
    float Cy = triangle.vertices[2].P_after.y;

    float Px = P.x;
    float Py = P.y;

    // Compute vectors
    float v0x = Cx - Ax;
    float v0y = Cy - Ay; // Vector AC
    float v1x = Bx - Ax;
    float v1y = By - Ay; // Vector AB
    float v2x = Px - Ax;
    float v2y = Py - Ay; // Vector AP

    // Compute dot products
    float dot00 = v0x * v0x + v0y * v0y;
    float dot01 = v0x * v1x + v0y * v1y;
    float dot02 = v0x * v2x + v0y * v2y;
    float dot11 = v1x * v1x + v1y * v1y;
    float dot12 = v1x * v2x + v1y * v2y;

    // Compute denominator
    float denom = (dot00 * dot11) - (dot01 * dot01);

    // If denom is close to zero, triangle or area is degenerate
    if (denom == 0.0f) {
        // Return something indicative of failure or degenerate triangle
        // return { -1.0f, -1.0f, -1.0f };
        denom = 1.0f;
    }

    // Calculate barycentric coordinates
    float invDenom = 1.0f / denom;
    float w2 = (dot11 * dot02 - dot01 * dot12) * invDenom; // corresponds to AC
    float w3 = (dot00 * dot12 - dot01 * dot02) * invDenom; // corresponds to AB
    float w1 = 1.0f - w2 - w3;

    // w1, w2, and w3 now represent barycentric coordinates
    return { w1, w2, w3 };
}

std::array<float, 2> Helpers::interpolateUV(const Triangle &triangle, float x, float y)
{
    Vector3 P = {x, y, 0};
    auto bar = getBarycentricCoordinates(triangle, P);

    float u = bar[0] * triangle.vertices[0].u + bar[1] * triangle.vertices[1].u + bar[2] * triangle.vertices[2].u;
    float v = bar[0] * triangle.vertices[0].v + bar[1] * triangle.vertices[1].v + bar[2] * triangle.vertices[2].v;

    return {u, v};
}

float Helpers::interpolate(const Triangle &triangle, Vector3 &outNormal, float x, float y)
{
    // The point P where we want barycentric coords
    Vector3 P = { x, y, 0 };

    // Compute barycentric coordinates, returns [w1, w2, w3]
    auto bar = getBarycentricCoordinates(triangle, P);

    // Use bar[2] (not bar[3], which is out of bounds)
    float z = interpolateNormalAndZ(triangle, bar[0], bar[1], bar[2], outNormal);

    return z;
}

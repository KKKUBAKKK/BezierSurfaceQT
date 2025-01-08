#include "phonglighting.h"

#include "helpers.h"

// Calculate the Phong lighting for a fragment/pixel
// normal - the interpolated surface normal (e.g., via barycentric coords)
// objectColor - the base color of the object for this pixel
QColor PhongLighting::calculateColor(const Vector3 &normal, const Vector3 &point, const QColor &objectColor) const {
    // 1) Normalize the surface normal
    Vector3 N = normal;
    N.normalize();

    // 2) Compute L: vector from the surface point to the light
    Vector3 L = {lightPos.x - point.x, lightPos.y - point.y, lightPos.z - point.z};
    L.normalize();

    // 3) Define V = (0, 0, 1) as the view direction (user’s perspective)
    // Vector3 V = {0.0f - point.x, 0.0f - point.y, 1.0f - point.z};
    Vector3 V = {0.0f, 0.0f, 1.0f};
    V.normalize();

    // 4) cos(N,L) = dot(N, L)
    float NL = N.x * L.x + N.y * L.y + N.z * L.z;
    NL = std::max(0.0f, NL);

    // 5) Compute R = 2 <N,L> * N - L
    Vector3 R = {
        2.0f * NL * N.x - L.x,
        2.0f * NL * N.y - L.y,
        2.0f * NL * N.z - L.z
    };
    R.normalize();

    // 6) cos(V,R) = dot(V, R)
    float VR = V.x * R.x + V.y * R.y + V.z * R.z;
    VR = std::max(0.0f, VR);

    // 7) Convert the light color and object color to floating [0..1]
    //    For each channel, apply the Phong formula:
    //    I = (kd * IL * IO * cos(N,L)) + (ks * IL * IO * cos(V,R)^m)
    float lightR = lightColor.redF();
    float lightG = lightColor.greenF();
    float lightB = lightColor.blueF();

    float objR = objectColor.redF();
    float objG = objectColor.greenF();
    float objB = objectColor.blueF();

    // Apply formula per channel
    float finalR = kd * lightR * objR * NL + ks * lightR * objR * std::pow(VR, m);
    float finalG = kd * lightG * objG * NL + ks * lightG * objG * std::pow(VR, m);
    float finalB = kd * lightB * objB * NL + ks * lightB * objB * std::pow(VR, m);

    // Clamp each channel to [0..1]
    finalR = Helpers::clamp01(finalR);
    finalG = Helpers::clamp01(finalG);
    finalB = Helpers::clamp01(finalB);

    // Convert back to QColor in [0..255]
    int r255 = static_cast<int>(std::round(finalR * 255.0f));
    int g255 = static_cast<int>(std::round(finalG * 255.0f));
    int b255 = static_cast<int>(std::round(finalB * 255.0f));

    // Return the resulting QColor
    return QColor(r255, g255, b255);
}

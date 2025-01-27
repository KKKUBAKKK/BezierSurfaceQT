#include "phonglighting.h"

#include "helpers.h"

QColor PhongLighting::calculateColor(const Vector3 &normal,
                                     const Vector3 &point,
                                     const QColor &objectColor) const
{
    // Define the view direction based on your coordinate system
    // If the z-axis points away from the user, use (0, 0, -1)
    // If the z-axis points towards the user, use (0, 0, 1)
    Vector3 V = {0.0f, 0.0f, -1.0f}; // Adjusted for z-axis pointing away
    V.normalize();

    // Helper function for computing Phong shading from a single light
    auto phongFromLight = [&](const Vector3 &lp, const QColor &lc) -> Vector3
    {
        // 1) Normalize the surface normal
        Vector3 N = normal;
        N.normalize();

        // 2) Compute L = vector from surface point to light
        Vector3 L = {lp.x - point.x,
                     lp.y - point.y,
                     lp.z - point.z};
        L.normalize();

        // 4) cos(N, L) = dot(N, L)
        float NL = std::max(0.0f, N.dot(L));

        // 5) Compute R = 2(N ⋅ L)N − L
        Vector3 R = {
            2.0f * NL * N.x - L.x,
            2.0f * NL * N.y - L.y,
            2.0f * NL * N.z - L.z
        };
        R.normalize();

        // 6) cos(V, R) = dot(V, R)
        float VR = std::max(0.0f, V.dot(R));

        // Convert light & object color to [0..1]
        float lr = lc.redF();
        float lg = lc.greenF();
        float lb = lc.blueF();

        float or_ = objectColor.redF();
        float og = objectColor.greenF();
        float ob = objectColor.blueF();

        // Phong shading per channel
        float outR = kd * lr * or_ * NL + ks * lr * or_ * pow(VR, m);
        float outG = kd * lg * og * NL + ks * lg * og * pow(VR, m);
        float outB = kd * lb * ob * NL + ks * lb * ob * pow(VR, m);

        return Vector3(outR, outG, outB);
    };

    // If not a reflector, single standard light
    if (!reflectors)
    {
        Vector3 c = phongFromLight(lightPos, lightColor);
        c.x = Helpers::clamp01(c.x);
        c.y = Helpers::clamp01(c.y);
        c.z = Helpers::clamp01(c.z);

        int r255 = static_cast<int>(round(c.x * 255.0f));
        int g255 = static_cast<int>(round(c.y * 255.0f));
        int b255 = static_cast<int>(round(c.z * 255.0f));
        return QColor(r255, g255, b255);
    }
    else
    {
        // Reflector mode: two lights at opposite coordinates
        Vector3 reflectorPos1 = lightPos;
        Vector3 reflectorPos2 = {-lightPos.x, -lightPos.y, lightPos.z};

        // Calculate each light’s color contribution
        Vector3 c1 = phongFromLight(reflectorPos1, lightColor);
        Vector3 c2 = phongFromLight(reflectorPos2, lightColor);

        // Sum and clamp
        Vector3 c = {c1.x + c2.x, c1.y + c2.y, c1.z + c2.z};
        c.x = Helpers::clamp01(c.x);
        c.y = Helpers::clamp01(c.y);
        c.z = Helpers::clamp01(c.z);

        int r255 = static_cast<int>(round(c.x * 255.0f));
        int g255 = static_cast<int>(round(c.y * 255.0f));
        int b255 = static_cast<int>(round(c.z * 255.0f));

        return QColor(r255, g255, b255);
    }
}

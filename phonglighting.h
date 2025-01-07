#ifndef PHONGLIGHTING_H
#define PHONGLIGHTING_H

#include "vector3.h"
#include <QColor>
#include <QTimer>

class PhongLighting : public QObject{
    Q_OBJECT
public:
    // Phong model coefficients (0..1 for kd, ks; 1..100 for m)
    float kd;    // Diffuse coefficient
    float ks;    // Specular coefficient
    int m;     // Specular exponent
    QColor lightColor;  // Light color (IL), default is white (1,1,1)

    bool movingLight;   // Whether the light moves in a spiral
    Vector3 lightPos;   // Light position or direction
    float spiralAngle;  // Used if the light is moving in a spiral
    float spiralRadius; // Radius of the spiral
    bool reflectors;

    // Constructor with default values
    PhongLighting()
        : kd(0.5f), ks(0.5f), m(50),
        lightColor(Qt::white),  // default to white
        movingLight(false),
        lightPos{0.0f, 0.0f, 1.0f},
        spiralAngle(0.0f),
        spiralRadius(1.0f),
        reflectors(false) {}

    // Calculate the Phong lighting for a fragment/pixel
    // normal - the interpolated surface normal (e.g., via barycentric coords)
    // objectColor - the base color of the object for this pixel
    QColor calculateColor(const Vector3 &normal, const QColor &objectColor) const;
private:
};

#endif // PHONGLIGHTING_H

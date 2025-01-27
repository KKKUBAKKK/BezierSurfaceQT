#ifndef VECTOR3_H
#define VECTOR3_H

#include <cmath>

class Vector3 {
public:
    float x, y, z;

    Vector3() : x(0), y(0), z(0) {}
    Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

    // Mnożenie przez macierz
    Vector3 transformPoint(float matrix[3][3]) const;

    // Dodawanie wektorów
    Vector3 operator+(const Vector3& other) const;

    // Odejmowanie wektorów
    Vector3 operator-(const Vector3& other) const;

    // Mnożenie przez skalar
    Vector3 operator*(float scalar) const;

    // Iloczyn wektorowy
    Vector3 cross(const Vector3& other) const;

    float dot(const Vector3& other) const;

    // Normalizacja
    void normalize();
};

#endif // VECTOR3_H

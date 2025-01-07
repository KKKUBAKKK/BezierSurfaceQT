#include "vector3.h"

// Funkcja do przekształcania punktu za pomocą macierzy
Vector3 Vector3::transformPoint(float matrix[3][3]) const {
    Vector3 result;
    result.x = matrix[0][0] * this->x + matrix[0][1] * this->y + matrix[0][2] * this->z;
    result.y = matrix[1][0] * this->x + matrix[1][1] * this->y + matrix[1][2] * this->z;
    result.z = matrix[2][0] * this->x + matrix[2][1] * this->y + matrix[2][2] * this->z;
    return result;
}

// Dodawanie wektorów
Vector3 Vector3::operator+(const Vector3& other) const {
    return Vector3(x + other.x, y + other.y, z + other.z);
}

// Odejmowanie wektorów
Vector3 Vector3::operator-(const Vector3& other) const {
    return Vector3(x - other.x, y - other.y, z - other.z);
}

// Mnożenie przez skalar
Vector3 Vector3::operator*(float scalar) const {
    return Vector3(x * scalar, y * scalar, z * scalar);
}

// Iloczyn wektorowy
Vector3 Vector3::cross(const Vector3& other) const {
    return Vector3(
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x
        );
}

// Normalizacja
void Vector3::normalize() {
    float length = std::sqrt(x * x + y * y + z * z);
    if (length > 0.0f) {
        x /= length;
        y /= length;
        z /= length;
    }
}

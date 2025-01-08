#ifndef MESH_H
#define MESH_H

#include <QImage>
#include <vector>
#include "triangle.h"
#include "vector3.h"

#define N 3
#define M 3

class Mesh {
public:
    std::vector<Triangle> triangles;
    std::vector<Vector3> controlPoints;
    int resolution;
    float alpha;
    float beta;

    Mesh() {}

    void addTriangle(const Triangle &triangle);
    void generateMesh();
    void generateMesh(const int &resolution);
    void generateMesh(const std::vector<Vector3> &controlPoints, const int &resolution);
    void rotateMesh(float alpha, float beta);
    void applyNormalMap(const QImage &normalMap);

};

#endif // MESH_H

#include "mesh.h"
#include "helpers.h"
#include "bezierinterpolation.h"

void Mesh::addTriangle(const Triangle &triangle) {
    triangles.push_back(triangle);
}

// Funkcja generująca siatkę triangulowaną na podstawie powierzchni Béziera
void Mesh::generateMesh(const std::vector<Vector3> &controlPoints, const int &resolution) {
    if (controlPoints.size() != 16) {
        throw std::invalid_argument("Control points size must be 16.");
    }

    this->controlPoints = controlPoints;
    this->resolution = resolution;

    float step = 1.0f / (resolution - 1);

    for (int i = 0; i < resolution - 1; ++i) {
        for (int j = 0; j < resolution - 1; ++j) {
            float u1 = i * step;
            float v1 = j * step;
            float u2 = (i + 1) * step;
            float v2 = j * step;
            float u3 = i * step;
            float v3 = (j + 1) * step;
            float u4 = (i + 1) * step;
            float v4 = (j + 1) * step;

            Vector3 p1 = BezierInterpolation::bezierSurfacePoint(controlPoints, N, M, u1, v1);
            Vector3 p2 = BezierInterpolation::bezierSurfacePoint(controlPoints, N, M, u2, v2);
            Vector3 p3 = BezierInterpolation::bezierSurfacePoint(controlPoints, N, M, u3, v3);
            Vector3 p4 = BezierInterpolation::bezierSurfacePoint(controlPoints, N, M, u4, v4);

            Vertex vtx1, vtx2, vtx3, vtx4;

            vtx1.P_before = p1;
            vtx2.P_before = p2;
            vtx3.P_before = p3;
            vtx4.P_before = p4;

            vtx1.u = u1;
            vtx1.v = v1;

            vtx2.u = u2;
            vtx2.v = v2;

            vtx3.u = u3;
            vtx3.v = v3;

            vtx4.u = u4;
            vtx4.v = v4;

            vtx1.Pu_before = BezierInterpolation::derivativeU(controlPoints, N, M, vtx1.u, vtx1.v);
            vtx1.Pv_before = BezierInterpolation::derivativeV(controlPoints, N, M, vtx1.u, vtx1.v);
            vtx1.N_before = BezierInterpolation::normal(controlPoints, N, M, vtx1.u, vtx1.v);

            vtx2.Pu_before = BezierInterpolation::derivativeU(controlPoints, N, M, vtx2.u, vtx2.v);
            vtx2.Pv_before = BezierInterpolation::derivativeV(controlPoints, N, M, vtx2.u, vtx2.v);
            vtx2.N_before = BezierInterpolation::normal(controlPoints, N, M, vtx2.u, vtx2.v);

            vtx3.Pu_before = BezierInterpolation::derivativeU(controlPoints, N, M, vtx3.u, vtx3.v);
            vtx3.Pv_before = BezierInterpolation::derivativeV(controlPoints, N, M, vtx3.u, vtx3.v);
            vtx3.N_before = BezierInterpolation::normal(controlPoints, N, M, vtx3.u, vtx3.v);

            // Dodajemy dwa trójkąty na każdą "kwadratową" sekcję
            addTriangle(Triangle(vtx1, vtx2, vtx3));
            addTriangle(Triangle(vtx2, vtx4, vtx3));
        }
    }
}

void Mesh::generateMesh(const int &resolution) {
    this->resolution = resolution;
    this->generateMesh(this->controlPoints, this->resolution);
}

void Mesh::generateMesh() {
    this->generateMesh(this->controlPoints, this->resolution);
}

// Funkcja do obracania siatki wierzchołków
void Mesh::rotateMesh(float alpha, float beta) {
    this->alpha = alpha;
    this->beta = beta;

    float rotationZ[3][3];
    float rotationX[3][3];

    Helpers::rotationMatrixZ(alpha, rotationZ);
    Helpers::rotationMatrixX(beta, rotationX);

    for (auto &triangle : triangles) {
        for (auto &vertex : triangle.vertices) {
            // Obrót wokół osi z
            vertex.P_after = vertex.P_before.transformPoint(rotationZ);
            vertex.Pu_after = vertex.Pu_before.transformPoint(rotationZ);
            vertex.Pv_after = vertex.Pv_before.transformPoint(rotationZ);
            vertex.N_after = vertex.N_before.transformPoint(rotationZ);

            // Obrót wokół osi x
            vertex.P_after = vertex.P_after.transformPoint(rotationX);
            vertex.Pu_after = vertex.Pu_after.transformPoint(rotationX);
            vertex.Pv_after = vertex.Pv_after.transformPoint(rotationX);
            vertex.N_after = vertex.N_after.transformPoint(rotationX);
        }
    }
}

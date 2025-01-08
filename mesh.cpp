#include "mesh.h"
#include "helpers.h"
#include "bezierinterpolation.h"

#include <QImage>


void Mesh::addTriangle(const Triangle &triangle) {
    triangles.push_back(triangle);
}

void Mesh::generatePyramid()
{
    std::vector<Vector3> points = {
        Vector3(-1.0, -1.0, 0.0),
        Vector3(-1.0, 1.0, 0.0),
        Vector3(1.0, 1.0, 0.0),
        Vector3(1.0, -1.0, 0.0),
        Vector3(0.0, 0.0, 7.0)
    };

    for (int i = 0; i < points.size(); i++)
    {
        points[i] = points[i] * 90;
    }

    addTriangle(pyramidTriangle(points[0], points[1], points[2]));
    addTriangle(pyramidTriangle(points[2], points[3], points[0]));

    addTriangle(pyramidTriangle(points[0], points[1], points[4]));
    addTriangle(pyramidTriangle(points[1], points[2], points[4]));
    addTriangle(pyramidTriangle(points[2], points[3], points[4]));
    addTriangle(pyramidTriangle(points[3], points[0], points[4]));

    this->rotateMesh(15, 15);

    //Calculate normals and Pu and Pv
}

Triangle Mesh::pyramidTriangle(Vector3 p1, Vector3 p2, Vector3 p3)
{
    Vertex v1, v2, v3;

    v1.P_before = p1;
    v2.P_before = p2;
    v3.P_before = p3;

    v1.Pu_before = p2 - p1;
    v1.Pv_before = p3 - p1;

    v2.Pu_before = p3 - p2;
    v2.Pv_before = p1 - p2;

    v3.Pu_before = p1 - p3;
    v3.Pv_before = p2 - p3;

    v1.N_before = v1.Pu_before.cross(v1.Pv_before);
    v2.N_before = v2.Pu_before.cross(v2.Pv_before);
    v3.N_before = v3.Pu_before.cross(v3.Pv_before);

    return Triangle(v1, v2, v3);
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

// Example 3x3 matrix class or struct for convenience
// You can use your own matrix library or implement a similar structure
struct Matrix3 {
    float m[3][3];

    // Multiplies a Vector3 by this 3x3 matrix
    Vector3 multiply(const Vector3 &v) const {
        return Vector3(
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
            );
    }
};

// Helper function to build a 3x3 rotation/transform matrix
// given Pu, Pv, and the surface normal (Nsurface).
Matrix3 buildMatrix3(const Vector3 &Pu, const Vector3 &Pv, const Vector3 &Nsurface) {
    Matrix3 MM;
    // Each column is one of the vectors (assuming column-major):
    // MM = [ Pu  Pv  Nsurface ]
    // If you prefer row-major, store them accordingly.
    MM.m[0][0] = Pu.x;   MM.m[0][1] = Pv.x;   MM.m[0][2] = Nsurface.x;
    MM.m[1][0] = Pu.y;   MM.m[1][1] = Pv.y;   MM.m[1][2] = Nsurface.y;
    MM.m[2][0] = Pu.z;   MM.m[2][1] = Pv.z;   MM.m[2][2] = Nsurface.z;
    return MM;
}

void Mesh::applyNormalMap(const QImage &normalMap) {
    // For each triangle in this mesh, and each vertex, we sample
    // from the normalMap to get a 'normal texture vector' (Ntekstury).
    // Then we combine it with the surface-based matrix M
    // to get a modified normal.

    for (auto &triangle : triangles) {
        // For each vertex in the triangle:
        for (auto &vertex : triangle.vertices) {
            // Compute the surface normal (Nsurface) once per triangle or per vertex,
            // depending on your existing data. This is just a simple example
            // using cross product for demonstration:
            Vector3 surfaceNormal = vertex.N_before;
            surfaceNormal.normalize();

            // You may also have tangents (Pu) and bitangents (Pv) from your geometry pipeline.
            // Here we assume you have them stored or can compute them.
            // For illustration, let's just pick some arbitrary tangents:
            Vector3 Pu = vertex.Pu_before;
            Vector3 Pv = vertex.Pv_before;
            Pu.normalize();
            Pv.normalize();

            // Build the matrix M = [Pu, Pv, Nsurface]
            Matrix3 MM = buildMatrix3(Pu, Pv, surfaceNormal);

            // (1) Extract or compute texture coordinates for this vertex.
            //     This is just an example. Actual (u,v) depend on your model.
            float u = vertex.u; // Must be in [0..1] range typically
            float v = vertex.v; // Must be in [0..1] range typically

            // Convert (u,v) to pixel coordinates in the normal map
            int texX = static_cast<int>(u * (normalMap.width()  - 1));
            int texY = static_cast<int>((1.0f - v) * (normalMap.height() - 1));
            // The (1.0f - v) flips y if your texture coordinates have origin at the bottom

            // Clamp to valid range
            if (texX < 0) texX = 0;
            if (texX >= normalMap.width()) texX = normalMap.width() - 1;
            if (texY < 0) texY = 0;
            if (texY >= normalMap.height()) texY = normalMap.height() - 1;

            // (2) Get color from the normal map
            QColor color = normalMap.pixelColor(texX, texY);

            // (3) Convert color [0..255] to [-1..1] range
            float Nx = (color.redF()   * 2.0f) - 1.0f;  // redF() in [0..1]
            float Ny = (color.greenF() * 2.0f) - 1.0f;
            float Nz = (color.blueF()  * 2.0f) - 1.0f;
            // Ensure Nz is positive if you want outward-facing
            // According to the note, Blue is 128..255 => Nz is in [0..1].
            // If it becomes negative, you can clamp or invert:
            if (Nz < 0.0f) {
                Nz = 0.0f;
            }

            // (4) Build the normal vector from the normal map
            Vector3 Ntexture(Nx, Ny, Nz);
            Ntexture.normalize();

            // (5) Multiply by M to transform this normal into the surface basis
            Vector3 modifiedNormal = MM.multiply(Ntexture);
            modifiedNormal.normalize();

            // (6) Store the modified normal back in the vertex or use it in shading
            vertex.N_before = modifiedNormal;
        }
    }

    rotateMesh(alpha, beta);
}

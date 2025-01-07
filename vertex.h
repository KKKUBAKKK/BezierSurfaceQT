#ifndef VERTEX_H
#define VERTEX_H

#include "vector3.h"

class Vertex {
public:
    Vector3 P_before;  // Punkt przed obrotem
    Vector3 Pu_before; // Wektor styczny Pu przed obrotem
    Vector3 Pv_before; // Wektor styczny Pv przed obrotem
    Vector3 N_before;  // Wektor normalny N przed obrotem

    Vector3 P_after;   // Punkt po obrocie
    Vector3 Pu_after;  // Wektor styczny Pu po obrocie
    Vector3 Pv_after;  // Wektor styczny Pv po obrocie
    Vector3 N_after;   // Wektor normalny N po obrocie

    float u, v;        // Parametry u, v

    Vertex() : u(0), v(0) {}
};

#endif // VERTEX_H

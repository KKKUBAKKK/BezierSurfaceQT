#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "vertex.h"

class Triangle {
public:
    Vertex vertices[3];

    Triangle() {}
    Triangle(const Vertex &v1, const Vertex &v2, const Vertex &v3) {
        vertices[0] = v1;
        vertices[1] = v2;
        vertices[2] = v3;
    }

    // Dodaj tutaj inne przydatne metody, takie jak funkcje pomocnicze
};

#endif // TRIANGLE_H

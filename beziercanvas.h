#ifndef BEZIERCANVAS_H
#define BEZIERCANVAS_H

#include <QWidget>
#include "mesh.h"
#include "phonglighting.h"

#define TIME_INTERVAL 5

class BezierCanvas : public QWidget {
    Q_OBJECT

public:
    bool isVisible = true;
    bool isFilled = false;
    QColor fillColor = Qt::magenta;
    Mesh mesh;
    Mesh pyramid;
    PhongLighting phongLighting;
    QTimer* timer = new QTimer();
    QImage normalMap;
    bool loadedMap = false;
    std::vector<std::vector<float>> zBuffer = std::vector<std::vector<float>>(size().width(), std::vector<float>(size().height(), std::numeric_limits<float>::min()));
    std::mutex zBufferMutex;

    BezierCanvas(QWidget *parent = nullptr);

    struct Edge {
        float x;        // Current x
        float dx;       // Slope inverse
        int yMin;
        int yMax;       // Maximum y for this edge
    };

public slots:
    // Call each frame or render iteration to optionally update the light position
    void updateLightPosition();

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    void drawMesh(QPainter &painter);
    void fillMesh(QImage &image);
    void fillMeshParallel(QImage &image);
    void drawPixel(int x, int y, QColor color, QImage &image);
    void bucketSortEdges(std::vector<Edge> &edges);
    void scanLineFillPolygon(QImage &image, const std::vector<Vertex> &vertices, QColor fillColor);
    std::vector<BezierCanvas::Edge> createEdgesFromVertices(const std::vector<Vertex> &vertices);
    void drawInterpolatedNormals(QPainter& painter,
                                               const std::vector<Triangle> &triangles,
                                               float normalLength = 20.0f);
};

#endif // BEZIERCANVAS_H

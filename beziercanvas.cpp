#include "beziercanvas.h"

#include "helpers.h"

#include <QPainter>
#include <QtConcurrent/qtconcurrentmap.h>
#include <vector>
#include <algorithm>
#include <QImage>
#include <QColor>
#include <QFuture>
#include <QMutex>
#include <QMutexLocker>

BezierCanvas::BezierCanvas(QWidget *parent) : QWidget(parent) {
    setMinimumSize(600, 600);  // Ustaw minimalny rozmiar dla powierzchni Béziera

    // Connect the timer's timeout signal to the myMethod slot
    connect(timer, &QTimer::timeout, this, &BezierCanvas::updateLightPosition);
    timer->start(TIME_INTERVAL);
}

void BezierCanvas::paintEvent(QPaintEvent *event) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // Translate the origin to the center of the widget
    painter.translate(width() / 2, height() / 2);

    // Get widget size
    QSize size = this->size();

    // Create a QImage with the same dimensions
    QImage image(size, QImage::Format_ARGB32);
    image.fill(Qt::darkGray);

    if (isFilled) {
        // fillMesh(image);
        fillMeshParallel(image);
    }

    painter.drawImage(-size.width()/2, -size.height()/2, image);

    if (isVisible) {
        drawMesh(painter);
    }

    // drawInterpolatedNormals(painter, mesh.triangles);
}

void BezierCanvas::drawMesh(QPainter &painter) {
    // Ustawienia pędzla dla punktów
    painter.setPen(Qt::white);
    painter.setBrush(Qt::white);

    for (const auto &triangle : mesh.triangles) {
        // Rysujemy wierchołki trójkątów
        // for (const auto &vertex : triangle.vertices) {
        //     QPointF point(vertex.P_after.x, vertex.P_after.y);
        //     painter.drawEllipse(point, 3, 3);  // Rysujemy punkt jako małe kółko
        // }

        // Rysujemy linie między wierzchołkami trójkąta
        QPointF p1(triangle.vertices[0].P_after.x, triangle.vertices[0].P_after.y);
        QPointF p2(triangle.vertices[1].P_after.x, triangle.vertices[1].P_after.y);
        QPointF p3(triangle.vertices[2].P_after.x, triangle.vertices[2].P_after.y);

        painter.drawLine(p1, p2);
        painter.drawLine(p2, p3);
        painter.drawLine(p3, p1);
    }
}

// Mutex to protect shared QImage
static std::mutex g_imageMutex;

void BezierCanvas::fillMeshParallel(QImage &image) {
    // Number of hardware threads available
    unsigned int numThreads = std::thread::hardware_concurrency();
    if (numThreads == 0) {
        numThreads = 2; // fallback
    }

    // Split the triangle list into sub-ranges
    std::vector<std::future<void>> futures;
    size_t totalTriangles = mesh.triangles.size();
    size_t chunkSize = (totalTriangles + numThreads - 1) / numThreads;

    for (unsigned int threadIndex = 0; threadIndex < numThreads; ++threadIndex) {
        size_t startIdx = threadIndex * chunkSize;
        if (startIdx >= totalTriangles) break;

        size_t endIdx = std::min(startIdx + chunkSize, totalTriangles);

        // Each async call processes a subrange of triangles
        futures.push_back(std::async(std::launch::async, [this, &image, startIdx, endIdx]() {
            for (size_t i = startIdx; i < endIdx; ++i) {
                // Obtain vertices for this triangle
                const auto &triangle = mesh.triangles[i];
                std::vector<Vertex> vertices = {
                    triangle.vertices[0],
                    triangle.vertices[1],
                    triangle.vertices[2]
                };

                // Lock image access only during fill
                // An alternative is to use a local QImage and merge later
                {
                    // std::lock_guard<std::mutex> lock(g_imageMutex);
                    scanLineFillPolygon(image, vertices, fillColor);
                }
            }
        }));
    }

    // Wait for all tasks to finish
    for (auto &f : futures) {
        f.wait();
    }
}

void BezierCanvas::fillMesh(QImage &image) {
    for (int i = 0; i < mesh.triangles.size(); ++i) {
        std::vector<Vertex> vertices = {
            mesh.triangles[i].vertices[0],
            mesh.triangles[i].vertices[1],
            mesh.triangles[i].vertices[2]
        };
        scanLineFillPolygon(image, vertices, fillColor);
    }
}

// Revised bucket sort function for edges (handles negative and positive x values)
void BezierCanvas::bucketSortEdges(std::vector<Edge> &edges) {
    if (edges.empty()) return;

    // Find the minimum and maximum x values
    float minX = edges[0].x;
    float maxX = edges[0].x;
    for (const auto &edge : edges) {
        if (edge.x < minX) minX = edge.x;
        if (edge.x > maxX) maxX = edge.x;
    }

    // If all edges have the same x, sorting won't change anything
    float range = maxX - minX;
    if (range == 0.0f) {
        return;
    }

    int numBuckets = size().width();
    // Create empty buckets
    std::vector<std::vector<Edge>> buckets(numBuckets);

    // Distribute edges into buckets based on normalized x
    // Normalizing ensures negative x values are placed in correct buckets
    for (const auto &edge : edges) {
        float normalized = (edge.x - minX) / range;
        int bucketIndex = static_cast<int>(normalized * (numBuckets - 1));
        bucketIndex = std::min(std::max(bucketIndex, 0), numBuckets - 1);
        buckets[bucketIndex].push_back(edge);
    }

    // Merge buckets back into edges
    edges.clear();
    for (auto &bucket : buckets) {
        edges.insert(edges.end(), bucket.begin(), bucket.end());
    }
}

std::vector<BezierCanvas::Edge> BezierCanvas::createEdgesFromVertices(const std::vector<Vertex> &vertices) {
    std::vector<Edge> edges;
    if (vertices.size() < 2) {
        return edges;
    }

    // Loop through vertices and create edges
    for (size_t i = 0; i < vertices.size(); ++i) {
        const auto &start = vertices[i];
        const auto &end = vertices[(i + 1) % vertices.size()]; // Next vertex (wraps around)

        float x1 = start.P_after.x;
        float y1 = start.P_after.y;
        float x2 = end.P_after.x;
        float y2 = end.P_after.y;

        // Skip horizontal edges
        if (std::round(y1) == std::round(y2)) {
            continue;
        }

        // Ensure y1 < y2 for consistency
        if (y1 > y2) {
            std::swap(x1, x2);
            std::swap(y1, y2);
        }

        float dx = (x2 - x1) / (y2 - y1);
        int yMin = static_cast<int>(std::round(y1));
        int yMax = static_cast<int>(std::round(y2));

        edges.push_back({x1, dx, yMin, yMax});
    }
    return edges;
}

void BezierCanvas::scanLineFillPolygon(QImage &image, const std::vector<Vertex> &vertices, QColor fillColor) {
    std::vector<Edge> polygonEdges = createEdgesFromVertices(vertices);
    if (polygonEdges.empty()) {
        return;
    }

    int height = image.height();
    int width = image.width();

    // Find overall minY to shift edges into image coordinate space
    int minY = polygonEdges[0].yMin;
    int maxY = polygonEdges[0].yMax;
    for (int i = 1; i < polygonEdges.size(); ++i) {
        if (minY > polygonEdges[i].yMin) minY = polygonEdges[i].yMin;
        if (maxY < polygonEdges[i].yMax) maxY = polygonEdges[i].yMax;
    }

    // Offset to ensure we start from 0-based index when building edge table
    int offsetY = -minY;

    // Build an edge table of size (adjusted) for the polygon’s Y range
    // Ensure we do not exceed the image height
    int tableSize = std::min(height, maxY - minY + 1);
    if (tableSize <= 0) {
        return;
    }
    std::vector<std::vector<Edge>> edgeTable(tableSize);

    // Populate the edge table
    for (auto &edge : polygonEdges) {
        int yIndex = edge.yMin + offsetY;
        if (yIndex >= 0 && yIndex < tableSize) {
            edgeTable[yIndex].push_back(edge);
        }
    }

    // Active Edge Table (AET)
    std::vector<Edge> activeEdgeTable;

    // Process each scan line within the table size
    for (int row = 0; row < tableSize; ++row) {
        // Add edges from ET to AET for the current scanline
        for (const auto &edge : edgeTable[row]) {
            activeEdgeTable.push_back(edge);
        }

        // Remove edges where the current row is >= edge's yMax
        activeEdgeTable.erase(
            std::remove_if(
                activeEdgeTable.begin(),
                activeEdgeTable.end(),
                [row, offsetY](const Edge &e) {
                    return (row - offsetY) >= e.yMax;
                }
                ),
            activeEdgeTable.end()
            );

        // Sort AET by x
        bucketSortEdges(activeEdgeTable);

        Triangle triangle(vertices[0], vertices[1], vertices[2]);

        // Fill pixels between pairs of x intersections in the AET
        for (size_t i = 0; i + 1 < activeEdgeTable.size(); i += 2) {
            int xStart = static_cast<int>(std::round(activeEdgeTable[i].x));
            int xEnd   = static_cast<int>(std::round(activeEdgeTable[i + 1].x));

            // Draw the spans if within image horizontal range
            for (int x = xStart; x <= xEnd; ++x) {

                int drawX = x + (width / 2);
                int drawY = row - offsetY + (height / 2);

                // Convert these back to “local” coords that match P_after
                float localX = drawX - (width / 2);
                float localY = drawY - (height / 2);

                if (drawX >= 0 && drawX < width && drawY >= 0 && drawY < height) {
                    Vector3 interpolatedNormal;

                    // Now call interpolate() using localX, localY if your
                    // triangle P_after.x, P_after.y are in a (0,0) = center system
                    float z = Helpers::interpolate(triangle, interpolatedNormal, localX, localY);
                    // interpolatedNormal = interpolatedNormal * -1;

                    Vector3 interpolatedPoint(localX, localY, z);
                    QColor color = phongLighting.calculateColor(interpolatedNormal, interpolatedPoint, fillColor);
                    image.setPixelColor(drawX, drawY, color);
                }
            }
        }

        // Update x values for all edges in the AET
        for (auto &edge : activeEdgeTable) {
            edge.x += edge.dx;
        }
    }
}

void BezierCanvas::drawInterpolatedNormals(QPainter& painter,
                             const std::vector<Triangle> &triangles,
                             float normalLength)
{
    painter.setPen(Qt::red);

    for (const auto &tri : triangles) {
        Vector3 avgNormal;

        // Compute the 2D center of the triangle (average of the vertices' positions)
        float cx = (tri.vertices[0].P_after.x + tri.vertices[1].P_after.x + tri.vertices[2].P_after.x) / 3.0f;
        float cy = (tri.vertices[0].P_after.y + tri.vertices[1].P_after.y + tri.vertices[2].P_after.y) / 3.0f;

        // Interpolate normal
        Helpers::interpolate(tri, avgNormal, cx, cy);

        // Normalize the average normal
        avgNormal.normalize();

        // End point of the line in 2D (ignoring z for visualization)
        float ex = cx + avgNormal.x * normalLength;
        float ey = cy + avgNormal.y * normalLength;
        // (Minus sign on y if you want a typical top-left origin coordinate system)

        // Draw the line representing the normal
        painter.drawLine(QPointF(cx, cy), QPointF(ex, ey));
    }

    painter.setPen(Qt::yellow);
    painter.drawEllipse(phongLighting.lightPos.x, phongLighting.lightPos.y, 10, 10);
    painter.setPen(Qt::red);
}

// Call each frame or render iteration to optionally update the light position
void BezierCanvas::updateLightPosition() {
    if (phongLighting.movingLight) {
        // Example spiral around the origin on x-y plane
        phongLighting.spiralAngle += 0.05f; // adjust rotation speed as needed
        phongLighting.lightPos.x = phongLighting.spiralRadius * std::cos(phongLighting.spiralAngle);
        phongLighting.lightPos.y = phongLighting.spiralRadius * std::sin(phongLighting.spiralAngle);
        // phongLighting.lightPos.z = 1.0f;  // keep it slightly above, for example
        this->update();
    }
    timer->start(TIME_INTERVAL);
}

#ifndef BEZIERSURFACE_H
#define BEZIERSURFACE_H

#include <QMainWindow>
#include <QSlider>
#include <QCheckBox>
#include <QPushButton>
#include "beziercanvas.h"

#define SCALE 30

QT_BEGIN_NAMESPACE
namespace Ui {
class BezierSurface;
}
QT_END_NAMESPACE

class BezierSurface : public QMainWindow
{
    Q_OBJECT

public:
    BezierSurface(QWidget *parent = nullptr);
    void loadFile();
    void loadMap();
    ~BezierSurface();
    void updateAlpha(int value);
    void updateBeta(int value);
    void drawMode(int state);
    void fillMode(int state);
    void chooseLightColor();
    void chooseFillColor();
    void updateResolution(int value);
    void setDefaultSurface(std::string filename);
    void updateKd(int value);
    void updateKs(int value);
    void updateM(int value);
    void updateLightSpiralMode(int state);
    void updateReflectorsMode(int state);
    void updateZ(int value);

private:
    Ui::BezierSurface *ui;

    float alpha = 0;
    float beta = 0;
    BezierCanvas *bezierCanvas;
    QSlider *triangulationSlider;
    QSlider *alphaSlider;
    QSlider *betaSlider;
    QSlider *kdSlider;
    QSlider *ksSlider;
    QSlider *mSlider;
    QSlider *zSlider;
    QCheckBox *fillSurfaceCheckBox;
    QCheckBox *drawModeCheckbox;
    QCheckBox *lightSpiralCheckbox;
    QCheckBox *reflectorsCheckbox;
    QPushButton *surfaceColorBtn;
    QPushButton *lightColorBtn;
    QPushButton *loadFileButton;
    QPushButton *loadMapButton;
};
#endif // BEZIERSURFACE_H

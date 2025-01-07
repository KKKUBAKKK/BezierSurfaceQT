#include "beziersurface.h"
#include "./ui_beziersurface.h"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QColorDialog>
#include <QLabel>
#include <iostream>
#include <fstream>
#include <vector>

BezierSurface::BezierSurface(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::BezierSurface)
{
    ui->setupUi(this);

    this->setStyleSheet("background-color: darkGray;");

    QWidget *centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);

    QHBoxLayout *mainLayout = new QHBoxLayout(centralWidget);

    // Lewa strona - wyświetlanie powierzchni Béziera
    bezierCanvas = new BezierCanvas(this);
    mainLayout->addWidget(bezierCanvas);

    // Prawa strona - kontrolki
    QWidget *controlPanel = new QWidget(this);
    QVBoxLayout *controlLayout = new QVBoxLayout(controlPanel);

    // Suwak do dokładności triangulacji
    triangulationSlider = new QSlider(Qt::Horizontal, this);
    triangulationSlider->setRange(4, 50);
    triangulationSlider->setValue(4);  // Domyślna wartość
    controlLayout->addWidget(new QLabel("Dokładność triangulacji", this));
    controlLayout->addWidget(triangulationSlider);

    // Suwak do kąta alfa
    alphaSlider = new QSlider(Qt::Horizontal, this);
    alphaSlider->setRange(-90, 90);
    alphaSlider->setValue(0);  // Domyślna wartość
    controlLayout->addWidget(new QLabel("Kąt alfa", this));
    controlLayout->addWidget(alphaSlider);

    // Suwak do kąta beta
    betaSlider = new QSlider(Qt::Horizontal, this);
    betaSlider->setRange(-90, 90);
    betaSlider->setValue(0);  // Domyślna wartość
    controlLayout->addWidget(new QLabel("Kąt beta", this));
    controlLayout->addWidget(betaSlider);

    kdSlider = new QSlider(Qt::Horizontal, this);
    kdSlider->setRange(0, 100);
    kdSlider->setValue(50);
    controlLayout->addWidget(new QLabel("kd", this));
    controlLayout->addWidget(kdSlider);

    ksSlider = new QSlider(Qt::Horizontal, this);
    ksSlider->setRange(0, 100);
    ksSlider->setValue(50);
    controlLayout->addWidget(new QLabel("ks", this));
    controlLayout->addWidget(ksSlider);

    mSlider = new QSlider(Qt::Horizontal, this);
    mSlider->setRange(1, 100);
    mSlider->setValue(50);
    controlLayout->addWidget(new QLabel("m", this));
    controlLayout->addWidget(mSlider);

    fillSurfaceCheckBox = new QCheckBox("Wypełnij siatkę", this);
    fillSurfaceCheckBox->setChecked(false); // default to unchecked
    controlLayout->addWidget(fillSurfaceCheckBox);

    // Checkbox do wyboru rysowania siatki lub wypełnienia
    drawModeCheckbox = new QCheckBox("Rysuj siatkę", this);
    drawModeCheckbox->setChecked(true);
    controlLayout->addWidget(drawModeCheckbox);

    lightSpiralCheckbox = new QCheckBox("Światło w spirali", this);
    lightSpiralCheckbox->setChecked(false);
    controlLayout->addWidget(lightSpiralCheckbox);

    reflectorsCheckbox = new QCheckBox("Reflektory", this);
    reflectorsCheckbox->setChecked(false);
    controlLayout->addWidget(reflectorsCheckbox);

    surfaceColorBtn = new QPushButton("Pick Surface Color", this);
    controlLayout->addWidget(surfaceColorBtn);

    lightColorBtn   = new QPushButton("Pick Light Color", this);
    controlLayout->addWidget(lightColorBtn);

    // Przycisk do wczytywania pliku z danymi powierzchni Béziera
    loadFileButton = new QPushButton("Wczytaj plik", this);
    connect(loadFileButton, &QPushButton::clicked, this, &BezierSurface::loadFile);
    controlLayout->addWidget(loadFileButton);

    mainLayout->addWidget(controlPanel);

    // Connect the alpha slider to the updateAlpha slot
    connect(alphaSlider, &QSlider::valueChanged, this, &BezierSurface::updateAlpha);

    // Connect the beta slider to the updateBeta slot
    connect(betaSlider, &QSlider::valueChanged, this, &BezierSurface::updateBeta);

    // Connect the checkbox signal to the drawMode slot
    connect(drawModeCheckbox, &QCheckBox::checkStateChanged, this, &BezierSurface::drawMode);

    // Connect the triangulartion slider to the updateResolution slot
    connect(triangulationSlider, &QSlider::valueChanged, this, &BezierSurface::updateResolution);

    // Connect the fill checkbox to the fillMode slot
    connect(fillSurfaceCheckBox, &QCheckBox::checkStateChanged, this, &BezierSurface::fillMode);

    // Connect the fill color button to the chooseFillColor slot
    connect(surfaceColorBtn, &QPushButton::clicked, this, &BezierSurface::chooseFillColor);

    // Connect the fill color button to the chooseFillColor slot
    connect(lightColorBtn, &QPushButton::clicked, this, &BezierSurface::chooseLightColor);

    // Connect the triangulartion slider to the updateKd slot
    connect(kdSlider, &QSlider::valueChanged, this, &BezierSurface::updateKd);

    // Connect the triangulartion slider to the updateKs slot
    connect(ksSlider, &QSlider::valueChanged, this, &BezierSurface::updateKs);

    // Connect the triangulartion slider to the updateM slot
    connect(mSlider, &QSlider::valueChanged, this, &BezierSurface::updateM);

    // Connect the fill checkbox to the updateLigthSpiral slot
    connect(lightSpiralCheckbox, &QCheckBox::checkStateChanged, this, &BezierSurface::updateLightSpiralMode);

    // Connect the fill checkbox to the fillMode slot
    connect(reflectorsCheckbox, &QCheckBox::checkStateChanged, this, &BezierSurface::updateReflectorsMode);

    // Set default surface
    setDefaultSurface("/Users/jakubkindracki/QtProjects/BezierSurface/mesh_coordinates_0.txt");
}

// This slot updates the variables that control spiral mode
void BezierSurface::updateLightSpiralMode(int state)
{
    bezierCanvas->phongLighting.movingLight = (state == Qt::Checked);
    this->bezierCanvas->update();
}

// This slot updates the variables that control drawing mode
void BezierSurface::updateReflectorsMode(int state)
{
    bezierCanvas->phongLighting.reflectors = (state == Qt::Checked);
    this->bezierCanvas->update();
}

// This slot updates the kd
void BezierSurface::updateKd(int value)
{
    bezierCanvas->phongLighting.kd = (float) value / 100;
    bezierCanvas->update();
}

// This slot updates the kd
void BezierSurface::updateKs(int value)
{
    bezierCanvas->phongLighting.ks = (float) value / 100;
    bezierCanvas->update();
}

// This slot updates the kd
void BezierSurface::updateM(int value)
{
    bezierCanvas->phongLighting.m = (int) value;
    bezierCanvas->update();
}

// This slot updates the color of the surface filling
void BezierSurface::chooseFillColor() {
    // Otwórz dialog wyboru koloru
    QColor selectedColor = QColorDialog::getColor(Qt::white, this, "Wybierz kolor wypełnienia");

    // Sprawdź, czy użytkownik zatwierdził wybór koloru
    if (selectedColor.isValid()) {
        bezierCanvas->fillColor = selectedColor;
    }
}

// This slot updates the color of the light
void BezierSurface::chooseLightColor() {
    // Otwórz dialog wyboru koloru
    QColor selectedColor = QColorDialog::getColor(Qt::white, this, "Wybierz kolor światła");

    // Sprawdź, czy użytkownik zatwierdził wybór koloru
    if (selectedColor.isValid()) {
        bezierCanvas->phongLighting.lightColor = selectedColor;
    }
}

// This slot updates the variables that control filling mode
void BezierSurface::fillMode(int state)
{
    bezierCanvas->isFilled = (state == Qt::Checked);
    this->bezierCanvas->update();
}

// This slot updates the resolution of the mesh
void BezierSurface::updateResolution(int value)
{
    Mesh mesh;
    std::vector<Vector3> controlPoints = bezierCanvas->mesh.controlPoints;
    mesh.generateMesh(controlPoints, value);
    mesh.rotateMesh(alpha, beta);
    this->bezierCanvas->mesh = mesh;
    bezierCanvas->update();
}

// This slot updates the variables that control drawing mode
void BezierSurface::drawMode(int state)
{
    bezierCanvas->isVisible = (state == Qt::Checked);
    this->bezierCanvas->update();
}

// This slot updates alpha, rotates the mesh, and triggers a repaint
void BezierSurface::updateAlpha(int value)
{
    // Convert slider integer to a float angle if necessary
    alpha = static_cast<float>(value);


    // Regenerate or just rotate your mesh:
    // If your mesh depends on alpha/beta for rotation only:
    this->bezierCanvas->mesh.rotateMesh(alpha, beta);

    this->bezierCanvas->update();
}

// This slot updates beta, rotates the mesh, and triggers a repaint
void BezierSurface::updateBeta(int value)
{
    // Convert slider integer to a float angle if necessary
    beta = static_cast<float>(value);


    // Regenerate or just rotate your mesh:
    // If your mesh depends on alpha/beta for rotation only:
    this->bezierCanvas->mesh.rotateMesh(alpha, beta);

    this->bezierCanvas->update();
}

void BezierSurface::loadFile() {
    QString QfileName = QFileDialog::getOpenFileName(this, "Wybierz plik z danymi powierzchni Béziera", "", "Text Files (*.txt);;All Files (*)");
    std::string filename = QfileName.toStdString();
    if (!QfileName.isEmpty()) {
        std::vector<Vector3> controlPoints;
        std::ifstream file(filename);

        if (!file.is_open()) {
            std::cerr << "Nie można otworzyć pliku: " << filename << std::endl;
            return;
        }

        float x, y, z;
        while (file >> x >> y >> z) {
            controlPoints.emplace_back(x, y, z);
        }

        for (int i = 0; i < controlPoints.size(); ++i) {
            controlPoints[i] = controlPoints[i] * SCALE;
        }

        file.close();

        Mesh mesh;
        mesh.generateMesh(controlPoints, triangulationSlider->value());
        mesh.rotateMesh(alpha, beta);
        this->bezierCanvas->mesh = mesh;
    }
}

void BezierSurface::setDefaultSurface(std::string filename) {
    std::vector<Vector3> controlPoints;
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Nie można otworzyć pliku: " << filename << std::endl;
        return;
    }

    float x, y, z;
    while (file >> x >> y >> z) {
        controlPoints.emplace_back(x, y, z);
    }

    for (int i = 0; i < controlPoints.size(); ++i) {
        controlPoints[i] = controlPoints[i] * SCALE;
    }

    file.close();

    Mesh mesh;
    mesh.generateMesh(controlPoints, triangulationSlider->value());
    mesh.rotateMesh(alpha, beta);
    this->bezierCanvas->mesh = mesh;
}

BezierSurface::~BezierSurface()
{
    delete ui;
}

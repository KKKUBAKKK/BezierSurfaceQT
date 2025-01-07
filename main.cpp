#include "beziersurface.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    BezierSurface w;
    w.show();
    return a.exec();
}

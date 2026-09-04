#include <QApplication>
#include <QLocale>
#include <QTranslator>

#include "render/robotsui.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QApplication::setApplicationName("PIN-2026 Robots");

    RobotsUI w;
    w.show();

    return QApplication::exec();
}

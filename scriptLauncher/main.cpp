#include <QtGui/QGuiApplication>
#include "qtquick2applicationviewer.h"
#include <QQmlContext>
#include "scriptlauncher.h"

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);

    ScriptLauncher launcher;

    QtQuick2ApplicationViewer viewer;
    viewer.rootContext()->setContextProperty("scriptLauncher", &launcher);
    viewer.setMainQmlFile(QStringLiteral("qml/scriptLauncher/main.qml"));
    viewer.showExpanded();

    return app.exec();
}

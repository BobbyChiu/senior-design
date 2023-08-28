#include "mainwindow.h"

#include <QApplication>
#include <QProcess>
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    QProcess process;
    process.start("cmd", QStringList() << "/c" << "echo hello world");
    process.waitForFinished();
    QString output = process.readAllStandardOutput();
    QString error = process.readAllStandardError();

    qDebug() << "Command output" << output;
    qDebug() << "Error output: " << error;
    return a.exec();
}

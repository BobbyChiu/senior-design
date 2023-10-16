#include "mainwindow.h"

#include <QApplication>
#include <QTcpServer>
#include <QTcpSocket>

void runServer() {
    QTcpServer server;
    if (!server.listen(QHostAddress::LocalHost, 12369)) {
        qDebug() << "Server could not start. Error: " << server.errorString();
        return;
    }
    qDebug() << "Server is listening on port 12345.";

    QObject::connect(&server, &QTcpServer::newConnection, [&]() {
        QTcpSocket* client = server.nextPendingConnection();
        QObject::connect(client, &QTcpSocket::readyRead, [&]() {
            QByteArray data = client->readAll();
            qDebug() << "Received data from client: " << data;
        });
    });
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    runServer();
    return a.exec();
}

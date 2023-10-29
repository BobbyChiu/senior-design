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
    qDebug() << "Server is listening on port 12369.";

    server.waitForNewConnection(-1);
    QTcpSocket *client = server.nextPendingConnection();
    qDebug() << "Connected.";

    while (true)
    {
        client->waitForReadyRead();
        char data[16];
        auto len = client->read(data, 16);
        data[len] = '\0';
        qDebug() << "Received data from client: " << data << len;
        if (strcmp(data, "done") == 0)
        {
            break;
        }
    }

    /*
    QObject::connect(&server, &QTcpServer::newConnection, [&]() {
        QTcpSocket* client = server.nextPendingConnection();
        QObject::connect(client, &QTcpSocket::readyRead, [&]() {
            QByteArray data = client->readAll();
            qDebug() << "Received data from client: " << data;
        });
    });
    */
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    //runServer();
    return a.exec();
}

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QProcess>
#include <QTcpServer>
#include <QTcpSocket>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void startServer();

public slots:
    void executeStartCommand();

private slots:
    void on_horizontalSlider_valueChanged(int value);

    void on_openNotepadButton_clicked();

    void on_calibrateButton_clicked();

    void on_calibrationDurationSlider_valueChanged(int value);

    void on_actionOpen_Scan_from_Device_triggered();

    void on_scanButton_clicked();

    void on_checkBox_stateChanged(int arg1);

    void on_generateButton_clicked();

private:
    void sendData(const QString &str);

    QString receiveData();

private:
    Ui::MainWindow *ui;

    QTcpServer server;
    QProcess process;
    QTcpSocket *client = nullptr;
};
#endif // MAINWINDOW_H

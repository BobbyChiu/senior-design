#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QProcess>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    // Connect the button's clicked() signal to a custom slot
    QObject::connect(ui->startButton, &QPushButton::clicked, this, &MainWindow::executeStartCommand);
}

MainWindow::~MainWindow()
{
    delete ui;
}



// TODO: Implement support for Linux/Unix Systems?
//void MainWindow::executeStartCommand()
//{
//    // Create a QProcess instance
//    QProcess process;

//    // Set the command to run (e.g., ls)
//    process.start("ls");

//    // Wait for the process to finish
//    process.waitForFinished();

//    // Read the output from the process
//    QString output = process.readAllStandardOutput();

//    // Print the output or display it in a text widget, etc.
//    qDebug() << "Command Output:" << output;
//}

// Function for windows specific OS
void MainWindow::executeStartCommand()
{
    // Create a QProcess instance
    QProcess process;

    // Set the Windows command to run (e.g., dir)
    process.start("cmd", QStringList() << "/c" << "dir");

    // Wait for the process to finish
    if (process.waitForFinished() == true) {
        // Read the output from the process
        QString output = process.readAllStandardOutput();

        if (output.isEmpty()) {
            qDebug() << "No output from the command.";
        } else {
            qDebug() << "Command Output:\n" << output;
        }
    } else {
        // Handle command execution error
        qDebug() << "Error executing the command:" << process.errorString();
    }
}





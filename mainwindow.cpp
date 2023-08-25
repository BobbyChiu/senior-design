#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QProcess>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Connect a button click to a slot
    connect(ui->runCommandButton, &QPushButton::clicked, this, &MainWindow::runCommand);
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::runCommand()
{
    QProcess process;
    process.start("cmd", QStringList() << "/c" << "dir"); // Modify this command as needed

    process.waitForFinished();

    QString output = process.readAllStandardOutput();
    QString error = process.readAllStandardError();

    // Display output and error in a QTextEdit widget
    ui->outputTextEdit->clear();
    ui->outputTextEdit->append("Command output:\n" + output);
    ui->outputTextEdit->append("\nError output:\n" + error);
}


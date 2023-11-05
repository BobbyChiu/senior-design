#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QProcess>
#include <QDebug>
#include <QTcpServer>
#include <QTcpSocket>
#include <QMenuBar>
#include <QFileDialog>

void runServer() {
    QTcpServer server;
    QProcess process;
    QString pythonScript = ".\\..\\client-driver\\client-driver.py";
    process.startDetached("python", QStringList() << pythonScript);
    //process.startDetached(".\\..\\client-driver\\client-driver.py");
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
        if (strcmp(data, "doneXXXXXXXXXXXX") == 0)
        {
            break;
        }
    }
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    // Connect the button's clicked() signal to a custom slot
    QObject::connect(ui->startServerClient, &QPushButton::clicked, this, &MainWindow::executeStartCommand);

    connect(ui->scanDurationSlider, &QSlider::valueChanged, this, &MainWindow::on_horizontalSlider_valueChanged);

    // Connect the button's clicked() signal to the openNotepad() slot
    connect(ui->openNotepadButton, &QPushButton::clicked, this, &MainWindow::on_openNotepadButton_clicked);


}

MainWindow::~MainWindow()
{
    delete ui;
}


// Function for windows specific OS`
void MainWindow::executeStartCommand()
{
    // Create a QProcess instance
    QProcess process;

    // Set the Windows command to run (e.g., dir)
    runServer();
    process.start("cmd", QStringList() << "/c" << "dir");

    // Wait for the process to finish
    if (process.waitForFinished() == true) {
        // Read the output from the process
        QString output = process.readAllStandardOutput();

        if (output.isEmpty()) {
            qDebug() << "No output from the command.";
        } else {
            qDebug() << "Command Output:\n" << output;
            ui->sysCallOutput->setText(QString("System Call Output (Will be replaced once output is determined): %1").arg(output));
        }
    } else {
        // Handle command execution error
        qDebug() << "Error executing the command:" << process.errorString();
    }
}





void MainWindow::on_horizontalSlider_valueChanged(int value)
{
    //qDebug() << "Slider Value Changed:" << value;
    ui->label->setText(QString("Scan Duration (seconds): %1").arg(value));
}



void MainWindow::on_openNotepadButton_clicked()
{
    // Create a QProcess instance
    QProcess process;

// Set the command to run (Notepad on Windows)
#ifdef Q_OS_WIN
    process.startDetached("notepad.exe");
    //process.startDetached(".\\..\\client-driver\\client-driver.py");
    //process.startDetached(".\\..\\v2-gui-interface\\lidar2pc.exe"); // TODO: May need to change path. The mainWindow application is located at \senior-design\src\gui\build-v2-gui-interface-Replacement_for_Desktop_Qt_6_5_2_MinGW_64_bit-Debug
    //process.start("explorer.exe", { "/select,", QDir::toNativeSeparators(QDir::homePath()) });
#endif

    // You can add platform-specific commands for other operating systems here
    // For example, on Linux/Unix systems, you can use "gedit" or "nano" instead of "notepad.exe"

    // Wait for the process to finish (optional) Adding two waitForFinished prevent two instances from launching
    process.waitForFinished();

}


void MainWindow::on_calibrateButton_clicked()
{
    // TODO - integrate calibrate functionality
    QString command = QString("calibrate --calibration-duration %1").arg(ui->calibrationDurationSlider->value());
    qDebug() << command;

}
void MainWindow::on_calibrationDurationSlider_valueChanged(int value)
{
    ui->calibrationDurationLabel->setText(QString("Calibration Duration (seconds): %1").arg(value));
}


void MainWindow::on_actionOpen_Scan_from_Device_triggered()
{
    QString filePath = QFileDialog::getOpenFileName(this, "Open File", "", "Text Files (*.txt);;All Files (*)");

    if (!filePath.isEmpty()) {
        // You can now work with the selected file using filePath
        // For example, you can read and display its contents.
        qDebug() << "Filepath not empty";
        QString command = QString("open --open-xyz %1").arg(filePath);
        qDebug() << command;
    }
    else{
        qDebug() << "Filepath empty";
    }
}


void MainWindow::on_scanButton_clicked()
{
    QString command = QString("scan --scan-duration %1").arg(ui->scanDurationSlider->value());
    qDebug() << command;
}


void MainWindow::on_checkBox_stateChanged(int arg1)
{

    QString removeBackgroundState = QString("state changed: %1").arg(arg1);
    qDebug() << removeBackgroundState;

}


void MainWindow::on_generateButton_clicked()
{
    QString command("generate");

    if (ui->checkBox->isChecked())
    {
        command += " --remove-background";
    }

    command += " --k-nn-thresholding " + ui->knnThresholdingInputs->text();

    // TODO: Find a way to make this optional?
    command += " --gaussian " + ui->meanInput->text() + " " + ui->stdDevInput->text();


    command += " --scaling " + ui->scalingInput->text();

    command += " --save-as-stl " + ui->stlSaveInput->text();
    command += " --save-as-xyz " + ui->xyzSaveInput->text();

    qDebug() << command;
}


#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QDebug>
#include <QMenuBar>
#include <QFileDialog>
#include <QDir>

void MainWindow::startServer() {
    //QString pythonScript = ".\\..\\netstring-client\\netstring-client.py";
    QDir::setCurrent(".\\..\\..\\scanner");
    QString run_scanner = "run_scanner.bat";
    process.startDetached(run_scanner);
    //process.startDetached("python", QStringList() << pythonScript);
    //process.startDetached(".\\..\\client-driver\\client-driver.py");
    if (!server.listen(QHostAddress::LocalHost, 12369)) {
        qDebug() << "Server could not start. Error: " << server.errorString();
        return;
    }
    qDebug() << "Server is listening on port 12369.";

    server.waitForNewConnection(-1);
    client = server.nextPendingConnection();
    qDebug() << "Connected.";
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    connect(ui->scanDurationSlider, &QSlider::valueChanged, this, &MainWindow::on_horizontalSlider_valueChanged);


}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_horizontalSlider_valueChanged(int value)
{
    //qDebug() << "Slider Value Changed:" << value;
    ui->label->setText(QString("Scan Duration (sec): %1").arg(value));
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
    sendData(command);
    qDebug() << receiveData();

}
void MainWindow::on_calibrationDurationSlider_valueChanged(int value)
{
    ui->calibrationDurationLabel->setText(QString("Calibration Duration (sec): %1").arg(value));
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
        sendData(command);
        qDebug() << receiveData();
        ui->centralwidget->findChild<QStackedWidget *>("stackedWidget")->setCurrentIndex(2);
    }
    else{
        qDebug() << "Filepath empty";
    }
}


void MainWindow::on_scanButton_clicked()
{
    QString command = QString("scan --scan-duration %1").arg(ui->scanDurationSlider->value());
    qDebug() << command;
    sendData(command);
    qDebug() << receiveData();
}


void MainWindow::on_checkBox_stateChanged(int arg1)
{

    QString removeBackgroundState = QString("state changed: %1").arg(arg1);
    qDebug() << removeBackgroundState;

}


void MainWindow::on_generateButton_clicked()
{
    QString command("generate");

    command += " --scaling " + ui->scalingInput->text();

    command += " --save-as-stl " + ui->stlSaveInput->text();
    command += " --save-as-xyz " + ui->xyzSaveInput->text();

    qDebug() << command;
    sendData(command);
    qDebug() << receiveData();
}

void MainWindow::sendData(const QString &str)
{
    QString netstring = QString::number(str.size()) + ":" + str + ",";
    auto ba = netstring.toLocal8Bit();
    client->write(ba.data());
}

QString MainWindow::receiveData()
{
    client->waitForReadyRead();

    char c[1];
    size_t length = 0;

    while (true)
    {
        client->read(c, 1);
        if (*c == ':')
        {
            break;
        }
        else
        {
            length = length * 10 + (*c - '0');
        }
    }

    char data[1024];
    int readLen = client->read(data, length);
    data[readLen] = '\0';

    // Remove comma
    client->read(c, 1);

    QString sdata(data);

    qDebug() << "Received data from client: " << sdata;
    return sdata;
}


void MainWindow::on_page2_clicked()
{
    ui->centralwidget->findChild<QStackedWidget *>("stackedWidget")->setCurrentIndex(1);

}


void MainWindow::on_page3_clicked()
{
    ui->centralwidget->findChild<QStackedWidget *>("stackedWidget")->setCurrentIndex(2);
}


void MainWindow::on_page4_clicked()
{
    ui->centralwidget->findChild<QStackedWidget *>("stackedWidget")->setCurrentIndex(3);
}


void MainWindow::on_prev1_clicked()
{
    ui->centralwidget->findChild<QStackedWidget *>("stackedWidget")->setCurrentIndex(0);
}


void MainWindow::on_prev2_clicked()
{
    ui->centralwidget->findChild<QStackedWidget *>("stackedWidget")->setCurrentIndex(1);
}


void MainWindow::on_prev3_clicked()
{
    ui->centralwidget->findChild<QStackedWidget *>("stackedWidget")->setCurrentIndex(2);
}


void MainWindow::on_processScanButton_clicked()
{
    QString command("process");

    if (ui->checkBox->isChecked())
    {
        command += " --remove-background";
    }

    command += " --k-nn-thresholding " + ui->knnThresholdingInputs->text();

    // TODO: Find a way to make this optional?
    command += " --gaussian " + ui->meanInput->text() + " " + ui->stdDevInput->text();

    qDebug() << command;
    sendData(command);
    qDebug() << receiveData();
}


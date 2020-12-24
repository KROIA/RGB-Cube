#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtSerialPort>
#include <QSerialPortInfo>
#include <QDebug>
#include <vector>
#include <time.h>

#include <QTimer>
#include <QImage>

using std::vector;

//#define DYNAMIC_AXIS_SCALE

struct Color
{
   uint8_t red;
   uint8_t green;
   uint8_t blue;
};
const uint8_t cubeSize = 4;

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
        Q_OBJECT

    public:
        explicit MainWindow(QWidget *parent = 0);
        ~MainWindow();


    private slots:
        void onSerialDataAvailable();
        void on_connect_button_clicked();
        void on_disconnect_button_clicked();

        void on_timerFinished();

        void on_sendCubeData_pushButton_clicked();

        void on_clear_pushButton_clicked();

        void on_light_pushButton_clicked();

    private:
        void getAvalilableSerialDevices();
        void serialRead();
        void serialWrite(QString message);
        void serialWrite(char *message);

        void sendImage(const QString &filename);
        void sendImageSequence(const QString &filename,unsigned int count,unsigned int interval);



        Ui::MainWindow *ui;

        std::string endOfMessage;
        qint32 baudrate;
        QSerialPort *usbDevice;
        std::vector<QSerialPortInfo> serialComPortList; //A list of the available ports for the dropdownmenue in the GUI

        QString deviceDescription;
        std::string serialBuffer;

        bool serialDeviceIsConnected;


        QTimer *timer;

        std::chrono::high_resolution_clock::time_point t1;
        std::chrono::high_resolution_clock::time_point t2;
        std::chrono::duration<double> time_span;
        double averageTimeSpan;
        unsigned long messageCounter;


        vector<vector<vector<Color> >   > m_cubeData;
        vector<QString> m_imageSequence;
        unsigned int m_currentImageSequenceIndex;
};

#endif // MAINWINDOW_H

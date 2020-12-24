#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    usbDevice = new QSerialPort(this);
    connect(usbDevice,SIGNAL(readyRead()),this,SLOT(onSerialDataAvailable()));

    endOfMessage = "\n";
    //baudrate = QSerialPort::Baud115200;
    baudrate = 1000000;

    serialDeviceIsConnected = false;
    getAvalilableSerialDevices();
    messageCounter = 0;


    timer = new QTimer();
    connect(timer,SIGNAL(timeout()),this,SLOT(on_timerFinished()));
    m_currentImageSequenceIndex = 0;

    averageTimeSpan = 0;


    m_cubeData.reserve(cubeSize);
    for(uint8_t z=0; z<cubeSize; z++)
    {
        m_cubeData.push_back(vector<vector<Color> >());
        m_cubeData[z].reserve(cubeSize);
        for(uint8_t x=0; x<cubeSize; x++)
        {
            m_cubeData[z].push_back(vector<Color>());
            m_cubeData[z][x].reserve(cubeSize);
            for(uint8_t y=0; y<cubeSize; y++)
            {
                Color color;
                color.red = 100;
                color.green = 50;
                color.blue = 200;
                m_cubeData[z][x].push_back(color);
            }
        }
    }
}

MainWindow::~MainWindow()
{
    delete ui;
    delete usbDevice;
}
void MainWindow::getAvalilableSerialDevices()
{
    qDebug() << "Number of available ports: " << QSerialPortInfo::availablePorts().length();
    serialComPortList.clear();
    ui->serialPortSelect_comboBox->clear();
    foreach(const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts())
    {
        QString dbgStr = "Vendor ID: ";


	   if(serialPortInfo.hasVendorIdentifier())
	   {
		  dbgStr+= serialPortInfo.vendorIdentifier();
	   }
	   else
	   {
		  dbgStr+= " - ";
	   }
	   dbgStr+= "  Product ID: ";
	   if(serialPortInfo.hasProductIdentifier())
	   {
		  dbgStr+= serialPortInfo.hasProductIdentifier();
	   }
	   else
	   {
		  dbgStr+= " - ";
	   }
	   dbgStr+= " Name: " + serialPortInfo.portName();
	   dbgStr+= " Description: "+serialPortInfo.description();
	  qDebug()<<dbgStr;
	  serialComPortList.push_back(serialPortInfo);
	  ui->serialPortSelect_comboBox->addItem(serialPortInfo.portName() +" "+serialPortInfo.description());
    }
}
void MainWindow::serialWrite(QString message)
{
    if(serialDeviceIsConnected == true)
    {
        usbDevice->write(message.toUtf8()); // Send the message to the device
   //     qDebug() << "Message to device: "<<message;
    }
}
void MainWindow::serialWrite(char *message)
{
    if(serialDeviceIsConnected == true)
    {
        usbDevice->write(message); // Send the message to the device
  //      qDebug() << "Message to device: "<<message;
    }
}
void MainWindow::serialRead()
{
    if(serialDeviceIsConnected == true)
    {
        serialBuffer.reserve(serialBuffer.size()*2);
        serialBuffer += usbDevice->readAll().toStdString(); // Read the available data
    }
}
void MainWindow::onSerialDataAvailable()
{
    if(serialDeviceIsConnected == true)
    {
        serialRead(); // Read a chunk of the Message
        //To solve that problem I send a end char "]" in My case. That helped my to know when a message is complete

        t1 = std::chrono::high_resolution_clock::now();
        while(serialBuffer.find(endOfMessage) != std::string::npos)
        {
            std::string message = serialBuffer.substr(0,serialBuffer.find(endOfMessage));
            //if(serialBuffer.indexOf("\n") != -1) //Message complete
            {
                qDebug() << "Message from device: "<<message.c_str();
                //serialWrite("echoFromGui");

                //Do something with de message here

                messageCounter++;
            }
            serialBuffer = serialBuffer.substr(serialBuffer.find(endOfMessage)+endOfMessage.size());
        }
        t2 = std::chrono::high_resolution_clock::now();
        time_span = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
        averageTimeSpan = averageTimeSpan*0.9 + 0.1*time_span.count();
    }
}


void MainWindow::on_connect_button_clicked()
{
    if(serialDeviceIsConnected == false)
    {
        usbDevice->setPortName(serialComPortList[ui->serialPortSelect_comboBox->currentIndex()].portName());
        deviceDescription = serialComPortList[ui->serialPortSelect_comboBox->currentIndex()].description();
        qDebug() << "connecting to: "<<usbDevice->portName();
        if(usbDevice->open(QIODevice::ReadWrite))
        {
            //Now the serial port is open try to set configuration
            if(!usbDevice->setBaudRate(baudrate))        //Depends on your boud-rate on the Device
                qDebug()<<usbDevice->errorString();

            if(!usbDevice->setDataBits(QSerialPort::Data8))
                qDebug()<<usbDevice->errorString();

            if(!usbDevice->setParity(QSerialPort::NoParity))
                qDebug()<<usbDevice->errorString();

            if(!usbDevice->setStopBits(QSerialPort::OneStop))
                qDebug()<<usbDevice->errorString();

            if(!usbDevice->setFlowControl(QSerialPort::NoFlowControl))
                qDebug()<<usbDevice->errorString();

            //If any error was returned the serial il corrctly configured

            qDebug() << "Connection to: "<< usbDevice->portName() << " " << deviceDescription << " connected";
            serialDeviceIsConnected = true;


        }
        else
        {
            qDebug() << "Connection to: "<< usbDevice->portName() << " " << deviceDescription << " not connected";
            qDebug() <<"         Error: "<<usbDevice->errorString();
        }
    }
    else
    {
        qDebug() << "Can't connect, another device is connected";
    }
}

void MainWindow::on_disconnect_button_clicked()
{
    if(serialDeviceIsConnected)
    {
        usbDevice->close();
        serialDeviceIsConnected = false;

        qDebug() << "Connection to: "<< usbDevice->portName() << " " << deviceDescription << " closed";
    }
    else
    {
       qDebug() << "Can't disconnect, no device is connected";
    }
}
void MainWindow::on_timerFinished()
{
    if(m_currentImageSequenceIndex < m_imageSequence.size())
    {
        sendImage(m_imageSequence[m_currentImageSequenceIndex]);
        m_currentImageSequenceIndex++;
    }
    else
    {
        m_currentImageSequenceIndex = 0;
        m_imageSequence.clear();
        timer->stop();
        if(ui->sendCubeDataCycle_checkBox->isChecked())
             on_sendCubeData_pushButton_clicked();
    }

}


void MainWindow::on_sendCubeData_pushButton_clicked()
{
    sendImageSequence("sprite",17,10);
    //sendImage("cube.png");
}
void MainWindow::sendImage(const QString &filename)
{
    QImage *image = new QImage;
    if(!image->load(filename))
    {
        qDebug() << "can't open file: "<<filename;
        return;
    }

    //QString stringStream = "";
    size_t arraySize = cubeSize*2 + cubeSize*cubeSize*cubeSize*3 + 1;
    char *arr = new char[arraySize];
    //stringStream.reserve(1000);



    size_t iterator = 0;
    for(uint8_t z=0; z<cubeSize; z++)
    {
        //stringStream += "L"+QString(z);
        arr[iterator] = 'L';    iterator++;
        arr[iterator] = z+1;    iterator++;
        for(uint8_t x=0; x<cubeSize; x++)
        {
            for(uint8_t y=0; y<cubeSize; y++)
            {

                QColor pixelColor = image->pixel(x,y+((cubeSize*(cubeSize-1))-cubeSize*z));

                m_cubeData[z][x][y].red   = (uint8_t)((double)pixelColor.red()*(double)ui->saturation_slider->value()/255.f);
                m_cubeData[z][x][y].green = (uint8_t)((double)pixelColor.green()*(double)ui->saturation_slider->value()/255.f);
                m_cubeData[z][x][y].blue  = (uint8_t)((double)pixelColor.blue()*(double)ui->saturation_slider->value()/255.f);

                if(m_cubeData[z][x][y].red == 0)
                    m_cubeData[z][x][y].red = 1;
                if(m_cubeData[z][x][y].green == 0)
                    m_cubeData[z][x][y].green = 1;
                if(m_cubeData[z][x][y].blue == 0)
                    m_cubeData[z][x][y].blue = 1;

              //  qDebug() << "red: "<<m_cubeData[z][x][y].red  << "\tgreen: "<<m_cubeData[z][x][y].green << "\tblue: "<<m_cubeData[z][x][y].blue;

                arr[iterator] = m_cubeData[z][x][y].red;    iterator++;
                arr[iterator] = m_cubeData[z][x][y].green;  iterator++;
                arr[iterator] = m_cubeData[z][x][y].blue;   iterator++;

                //stringStream += m_cubeData[z][x][y].red;
                //stringStream += m_cubeData[z][x][y].green;
                //stringStream += m_cubeData[z][x][y].blue;

                //stringStream += QString((char)pixelColor.red());
                //stringStream += QString((char)pixelColor.green());
                //stringStream += QString((char)pixelColor.blue());
                //qDebug() << "blue: "<< QString((char)pixelColor.blue())<< " :endBlue";
            }
        }
    }
    arr[iterator] = '\0';
    serialWrite(arr);

    delete[] arr;
    delete image;
}
void MainWindow::sendImageSequence(const QString &filename,unsigned int count,unsigned int interval)
{
    const char separator = '_';

    for(unsigned int i=0; i<count; i++)
    {
        m_imageSequence.push_back(filename+separator+( i<10?"0":"" )+QString::number(i)+".png");
    }
    timer->start(interval);
}

void MainWindow::on_clear_pushButton_clicked()
{
    m_currentImageSequenceIndex = 0;
    m_imageSequence.clear();
    timer->stop();
    sendImage("clear.png");
}

void MainWindow::on_light_pushButton_clicked()
{
    m_currentImageSequenceIndex = 0;
    m_imageSequence.clear();
    timer->stop();
    sendImage("light.png");
}

#-------------------------------------------------
#
# Project created by QtCreator 2020-07-21T14:36:05
#
#-------------------------------------------------

QT       += core gui network
QT       += network
CONFIG   +=c++11

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += resources_big
TARGET = AKLS
TEMPLATE = app
QT     +=serialport

#PCL Header
INCLUDEPATH += /usr/include/pcl-1.8
LIBS  += /usr/lib/arm-linux-gnueabihf/libpcl_*.so

#opencv
INCLUDEPATH += /usr/local/opencv/include \
               /usr/local/opencv/include/opencv \
               /usr/local/opencv/include/opencv2

LIBS += /usr/local/opencv/lib/libopencv_highgui.so \
        /usr/local/opencv/lib/libopencv_core.so    \
        /usr/local/opencv/lib/libopencv_imgproc.so \
        /usr/local/opencv/lib/libopencv_imgcodecs.so

LIBS+=/usr/local/opencv/lib/libopencv_shape.so
LIBS+=/usr/local/opencv/lib/libopencv_videoio.so

LIBS  += /usr/lib/arm-linux-gnueabihf/libboost*

PKGCONFIG=opencv

INCLUDEPATH +=/home/ubuntu/UpdateFile/New/LS_EXE/Track
LIBS+=/home/ubuntu/UpdateFile/New/LS_EXE/Track/LidarTracking-0.5m.so
#Eigen
INCLUDEPATH += /usr/include/eigen3

RC_ICONS = leishen.ico
SOURCES += main.cpp\
        mainwindow.cpp \
    addlidar.cpp \
    Getlidardata.cpp \
    paintarea.cpp \
    filesystem.cpp \
    scanconfigue.cpp \
    tcpClient.cpp \
    udptarget.cpp \
    sethostdialog.cpp \
    handlethreadpool.cpp \
    handlerun.cpp \
    about.cpp \
    get16lidardata.cpp \
    Tcp_Server.cpp \
    Messagethread.cpp \
    SerialPort.cpp \
    upgradeprocess.cpp

HEADERS  += mainwindow.h \
    addlidar.h \
    Getlidardata.h \
    LidarData.h \
    paintarea.h \
    filesystem.h \
    scanconfigue.h \
    tcpClient.h \
    udptarget.h \
    sethostdialog.h \
    handlethreadpool.h \
    handlerun.h \
    about.h \
    get16lidardata.h \
    Tcp_Server.h \
    Messagethread.h \
    SerialPort.h \
    upgradeprocess.h \
    LidarTracking.h

FORMS    += mainwindow.ui \
    addlidar.ui \
    scanconfigue.ui \
    sethostdialog.ui \
    about.ui

RESOURCES += \
    res.qrc

DISTFILES += \
    leishen.ico



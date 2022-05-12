#ifndef FILESYSTEM_H
#define FILESYSTEM_H
#include "QTextStream"
#include "QSettings"
#include <QDebug>
#include <QObject>
#include <QString>
#include <QFile>
#include <QCoreApplication>
#include <QDir>
#include <QPointF>
#include <iostream>
#include <fstream>
#include <string>
#include "LidarData.h"

struct conf_Prama{
    int             lidarIndex;
    QString         lidarIp;
    QList<QPointF>  polygon[3];
    float           radius;
    float           distance;
    float           scanNum;

    int             MapDistance;
};
extern QMap<int, conf_Prama> conf_data;

struct update_scene{
    int initState;
    float coeRi;
};
extern update_scene updateScene;

extern QString LidarSerialport;
class FileSystem : public QObject
{
    Q_OBJECT
public:
    FileSystem();
    bool ReadConfigueFile(QString FilePath, QString &Host, int &HostPort, int &LocalPort, bool& autoStart, bool& outState);
    void WriteConfigueFile(QString FilePath, QString Host = "", int HostPort = 0, int LocalPort = 0, bool out = false, bool autoStart = true);

    void ReadMinDistance();
    void WriteMinDistance();
};

#endif // FILESYSTEM_H

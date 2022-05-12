#include "filesystem.h"

QMap<int, conf_Prama> conf_data;
update_scene updateScene;
QString LidarSerialport;
FileSystem::FileSystem()
{
    conf_data.clear();
}

bool FileSystem::ReadConfigueFile(QString FilePath, QString &Host, int &HostPort, int &LocalPort, bool &autoStart, bool &outState)
{
    QString path = "config.ini";
    FilePath = "/home/ubuntu/LS_Config";   //2022-3-8
    path = QString("%1/%2").arg(FilePath).arg(path);

    QFile* file = new QFile(path);
    if(!file->exists()){
        qDebug()<<"FileNoExist!!!!";
        Host.clear();
        return false;
    }

    QSettings *config = new QSettings(path, QSettings::IniFormat);
    //autoStart = config->value(QString("configue/auto")).toInt()==1?true:false;
    outState = config->value(QString("configue/out")).toInt()==1?true:false;
    bool ret = config->value(QString("hostInfo/canUse")).toString() == "true";
    if(ret){
        Host = config->value(QString("hostInfo/HostIp")).toString();
        HostPort = config->value(QString("hostInfo/HostPort")).toInt();
        LocalPort = config->value(QString("hostInfo/LocalPort")).toInt();
    }

    updateScene.initState = config->value(QString("updateScene/InitState")).toInt();
    updateScene.coeRi = config->value(QString("updateScene/coeRi")).toFloat();
    LidarSerialport = config->value(QString("LidarDataPort/SerialPort")).toString();

    int lidarNum = config->value(QString("lidarNum/Num")).toInt();
    for(int i = 0; i < lidarNum; i++){
        conf_Prama data;
        data.lidarIndex = config->value(QString("lidar%1/LidarIndex").arg(i)).toInt();
        data.radius = config->value(QString("lidar%1/Radius").arg(i)).toFloat();
        data.distance = config->value(QString("lidar%1/Distance").arg(i)).toFloat();
        data.scanNum = config->value(QString("lidar%1/ScanNum").arg(i)).toInt();
        data.lidarIp = config->value(QString("lidar%1/Ip").arg(i)).toString();
        data.MapDistance = config->value(QString("lidar%1/MiniDistance").arg(i)).toInt();
        QString str;
        for(int j = 0; j < 3; j++){
            str = config->value(QString("lidar%1/Polygon%2").arg(i).arg(j)).toString();
            if(!str.size()){
                data.polygon[j].clear();
                continue;
            }
            QStringList split = str.split(" ");
            QPointF tmp;
            for(int m = 0; m < split.size(); m++){
                str = split.at(m);
                QStringList str2 = str.split(",");
                QString str3, str4;
                str3 = str2[0];
                str4 = str2[1];;
                tmp.setX(str3.right(str3.length() - 1).toFloat());
                tmp.setY(-str4.left(str4.length() - 1).toFloat());
                data.polygon[j].push_back(tmp);
            }
        }
        conf_data.insert(data.lidarIndex, data);
    }
    delete config;
    return ret;
}

void FileSystem::WriteConfigueFile(QString FilePath, QString Host, int HostPort, int LocalPort, bool out, bool autoStart)
{
    QString path = "config.ini";
    FilePath = "/home/ubuntu/LS_Config";   //2022-3-8
    path = QString("%1/%2").arg(FilePath).arg(path);
    QSettings *config = new QSettings(path, QSettings::IniFormat);
  //  config->clear();

    config->beginGroup(QString("hostInfo"));
    config->setValue("HostIp",Host);
    config->setValue("HostPort",QString("%1").arg(HostPort));
    config->setValue("LocalPort",QString("%1").arg(LocalPort));
    config->setValue("canUse",Host.isEmpty()?"false":"true");

    config->endGroup();
    config->beginGroup(QString("configue"));
   // config->setValue("auto",QString::number(autoStart?1:0));
    config->setValue("out",QString("%1").arg(out?1:0));
    config->endGroup();
    config->beginGroup(QString("lidarNum"));
    config->setValue("Num", QString("%1").arg(conf_data.size()));
    config->endGroup();
    QList<int> keys = conf_data.keys();
    for(int i = 0; i < keys.size(); i++){
        config->beginGroup(QString("lidar%1").arg(i));
        config->setValue("LidarIndex", QString("%1").arg(keys[i]));
        config->setValue("Radius",QString("%1").arg(conf_data[keys[i]].radius));
        config->setValue("Ip",QString("%1").arg(conf_data[keys[i]].lidarIp));
        config->setValue("Distance",QString("%1").arg(conf_data[keys[i]].distance));
        config->setValue("ScanNum",QString("%1").arg(conf_data[keys[i]].scanNum));
        config->setValue("MiniDistance",QString("%1").arg(conf_data[keys[i]].MapDistance));
        qDebug()<<"SaveMAPDistance:"<<conf_data[keys[i]].MapDistance;
        for(int j = 0; j < 3; j++){
            QString datastr;
            datastr.clear();
            for(int n = 0; n < conf_data[keys[i]].polygon[j].size(); n++)
            {
                QPointF tmp = conf_data[keys[i]].polygon[j][n];
                QString str = QString("(%1,%2)").arg(tmp.rx()).arg(-tmp.ry());
                datastr.append(str);
                if(n!= conf_data[keys[i]].polygon[j].size()-1){
                    datastr.append(" ");
                }
            }
            config->setValue(QString("Polygon%1").arg(j), datastr);
        }
        config->endGroup();
    }
    delete config;
}

void FileSystem::ReadMinDistance()
{
    QString c_fileName = "/home/ubuntu/LS_Config/Dpolygon.txt" ;
    QFile file(c_fileName);
    if (file.exists())
    {
        std::string file = c_fileName.toStdString();  //fileName为文件名，QString型数据
        std::ifstream infile;
        infile.open(file.data());   //将文件流对象与文件连接起来
        //  assert(infile.is_open());   //若失败,则输出错误消息,并终止程序运行
        if(!infile)
        {
            qDebug()<<"open file failed";
        }
        std::string s;
        while(std::getline(infile,s))
        {
            QString ss,s1;
            ss=QString::fromStdString(s);
            s1=ss.section(" ",0,0);    // 用QString的section()函数提取三维坐标
            int Distance_1 = s1.toInt();
            polygonDistance.push_back(Distance_1);
        }
        infile.close();
    }
    qDebug()<<"filepolygon"<<polygonDistance.size();
}

void FileSystem::WriteMinDistance()
{
//    QDir dir;
//    if(!dir.exists("Dpolygon.txt"){
//        dir.mkdir("Dpolygon.txt");
//    }
    QString box_filename=QString("/home/ubuntu/LS_Config/Dpolygon.txt");
    std::string file = box_filename.toStdString();
    std::ofstream outfile;
    outfile.open(file.data()/*,std::ios::app*/);
    if(!outfile)
    {
        qDebug()<<"fail to write polygonDistance.txt";
    }
    for(int i=0;i<polygonDistance.size();i++)
    {
    outfile<<polygonDistance.at(i)<<""<<std::endl;
    }
    outfile.close();
}


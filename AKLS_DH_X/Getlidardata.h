#ifndef GETLIDARDATA_H
#define GETLIDARDATA_H

#define NOMINMAX
#include <iostream>
#include <QObject>
#include <QThread>
#include <sys/time.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <QMetaType>
#include "LidarData.h"
#include <QMutex>
#include <QMap>
#include <QPoint>
#include <filesystem.h>
#include "udptarget.h"
#include <QTimer>
#include <QTime>
#include <QDebug>
#include <QMetaType>
#include <handlethreadpool.h>
#include <handlerun.h>
#include "tcpClient.h"
#include "Tcp_Server.h"
#include "Messagethread.h"
#define PI 3.1415926835

extern unsigned short LidarPort;
extern QMap<int,conf_Prama> conf_data;

extern std::vector<int> sendID;
extern std::vector<float> sendWidth;
extern std::vector<float> sendLenth;
extern std::vector<float> sendSpeed;
extern Railway railway; //轨道界限
Q_DECLARE_METATYPE(LidarData)

class GetlidarData :public QThread
{
	Q_OBJECT
public:
	explicit GetlidarData(QObject *parent = nullptr);  
    int count = 0;
    bool contrl = false;

    void udp_init(int* sock_num, sockaddr_in* sockaddr, int server_port);
    void UnInitialize(int *sock_num);
    int m_sock;
    QMutex m_lock;
    void run();
    int sock1;
    struct sockaddr_in addr1;
    struct sockaddr_in clientAddr1;

    enum{
        NormalType = 1,
        PolyType,
        ClearType,
        SynType,
        polyClear,
        PolySyn
    };

    bool getStatus();
    bool addSock(lidar info);
    void setCurSock(int index);
    void deletePoint(int selectPoint,int pos, int index);
    void addPoint(QPointF p, int pos, int index, int pindex);
    void updataPoint(QPointF p, int selectPoint,int pos, int index);
    void setScanPower(bool flag);
    bool getScanPower();
    void setOutPutHandle(bool flag);
    bool getOutPutHandle();
    void getCurInfo(int &distance, int &scanNum, int index);
    void changeCurInfo(float distance, int scanNum, int index);
    void deleteSock(int index);
    QByteArray getPointData(int index,QPointF point);
    QByteArray getPointData(int index, int polyindex,QPointF point , int ID ,float sendlen,float sendwidth,QPointF draw_point, float speed);  //hct-add
    int check_sum(QByteArray ba,int len);

    void sendLeftCluster(QPointF leftcluster,int ID,float width,float len,float speed);
    void sendRightCluster(QPointF rightcluster,int ID,float width,float len,float speed);

    void WriteRailwayConfig();//2022

    int minWidth;
    int maxWidth;
    void get_device_config();
    void update_device_config();

    QMap<int,float> Angle_Filter;
    QMap<int,float> Angle_Filter_1;
    QMap<int,float> Distance_Filter;
private:
    void senddata(QByteArray &data, int Type);
    QByteArray datapower(int Type);
signals:
    void handledata(LidarC1Data, int);
    void SendData(point_data);
    void setPolyByRecv(int Index, lidar recvLidar);
    void updateTable(float proportion, int lidarRadius, QString ip, int incount, int outcount);

    void HandleDataSignal(lidar lidarinfo, LidarC1Data data);  //add
    void sendMessageToClient(LidarC1Data data);//202106

    void sendClusterMessage(QByteArray data);//07

    void HandleID(std::vector<int> ID);

    void sendPolygon(QList<QPointF> polygon); //传递到画布

    void sendPolygonToClient(QByteArray data); //传递到客户端

    void sendPolygonEnding(QByteArray data);

    void sendLeftTrain(QByteArray data);   //轨道列车
    void sendRightTrain(QByteArray data);
    void sendRailPolygon(QList<QPointF>,QList<QPointF>);

    void sendClearDraw();

public slots:
    void HandleDataSlot(LidarC1Data LidarData, int index);
    void Timeresponse();
    void SendPolydata();
    void ResultSlot(lidar li,point_data data);
    void RecvPolySet(QByteArray polyByte);

    void ReceivePolygon(QList<QPointF> polygon);   //接受区域点
    void sendPolygonToClient_1();  //触发信号
    void sendPolyEnding();

    void ReceiveRailPolygon(QList<QPointF> left,QList<QPointF> right);
    void ReceiveWidthFilter(int,int);
private:
    bool                bindStatus;
    int                 sockPos;
    int                 curSock;
    HandleThreadPool    *HandleWay;
    HandleRun           *handleRun;
    MessageThread       *messagethread;
public:
    unsigned int        LocalPort;
    qint64              utime;
    QMap<int, lidar>    lidarPack;
    QMap<int, int>      sockMap;
    int                 curPos;
    bool                scanState;
    bool                outPutHandle;
    UdpTarget           *target;
    tcpClient           *tcpTarget;
  //  Tcp_Server          *tcpserver;
    QString             HostAddress;
    unsigned int        HostPort;
    QTimer              *timer;
    QMap<QString, int>  IpKeys;
    QMap<int, LidarC1Data> lidarData;

    bool                InitState;
    double              Ri;
    int                 initID;

    QList<QPointF> LeftRailCluster;
    QList<QPointF> RightRailCluster;
    float msx;
    float msy;
};

#endif 

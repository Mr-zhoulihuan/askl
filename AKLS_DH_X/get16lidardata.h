#ifndef GET16LIDARDATA_H
#define GET16LIDARDATA_H

#include <QObject>
#include <QThread>
#include <sys/time.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <QMutex>
#include <iostream>
#include <QMetaType>
#include "LidarData.h"
extern unsigned short LidarPort;


class Get16LidarData : public QThread
{
    Q_OBJECT
public:
    explicit Get16LidarData(QObject *parent = 0);
//    void run();
//    int m_sock;

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
signals:
    void SendData(LidarData);

public slots:

};

#endif // GET16LIDARDATA_H

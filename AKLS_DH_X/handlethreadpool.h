#ifndef HANDLETHREADPOOL_H
#define HANDLETHREADPOOL_H

#include <QObject>
#include <QThread>
#include <QThreadPool>
#include <QQueue>
#include <QDebug>
#include <handlerun.h>
class HandleThreadPool : public QThread
{
    Q_OBJECT
public:
    explicit HandleThreadPool(QObject *parent = 0);
    void run();
signals:
    void recvCBack(lidar, point_data);
public slots:

public:
    QQueue<HandleRun*> waitQueue;
    QThreadPool     m_Threadpool;
};

#endif // HANDLETHREADPOOL_H

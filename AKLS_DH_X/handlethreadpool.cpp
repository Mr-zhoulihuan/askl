#include "handlethreadpool.h"

HandleThreadPool::HandleThreadPool(QObject *parent) : QThread(parent)
{
    m_Threadpool.setMaxThreadCount(4);
    waitQueue.clear();
    qRegisterMetaType<struct lidar>("struct lidar");
    qRegisterMetaType<struct point_data>("struct point_data");
}

void HandleThreadPool::run()
{
    while(waitQueue.size()>0){
//        qDebug()<<"waitQueueSize"<<waitQueue.size();
  //      HandleRun *runTh = waitQueue.dequeue();
 //       QObject::connect(runTh,SIGNAL(lidarThCBack(lidar,point_data)),this,SIGNAL(recvCBack(lidar,point_data)));
  //      m_Threadpool.start(runTh);
    }
}

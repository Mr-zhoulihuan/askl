#include "Getlidardata.h"
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;
unsigned short LidarPort =0;
bool updateLock = false;
Railway railway;
void GetlidarData::udp_init(int* sock_num, sockaddr_in* sockaddr, int server_port)
{
    UnInitialize(sock_num);
    //  unsigned int size = sizeof(sockaddr_in);
    //  int len=0;
    sockaddr->sin_family = AF_INET;
    sockaddr->sin_addr.s_addr = htonl(INADDR_ANY);
    sockaddr->sin_port = htons(server_port);
    *sock_num = socket(AF_INET, SOCK_DGRAM, 0);
    int i =bind(*sock_num, (struct sockaddr *)sockaddr, sizeof(*sockaddr));
    qDebug("i= %d",i);
}

void GetlidarData::UnInitialize(int* sock_num)
{
    shutdown(*sock_num,SHUT_RDWR);
}
GetlidarData::GetlidarData(QObject *parent) : QThread(parent)
{
    HostAddress = "";
    HostPort = 0;
#if 1
    LidarData lidardata;
    QVariant DataVar;
    DataVar.setValue(lidardata);
    qRegisterMetaType<QVariant>("QVariant");
    udp_init(&sock1,&clientAddr1,2368);
    utime = 0;
#endif
    lidarPack.clear();
    curPos = -1;
    scanState = true;
    outPutHandle = false;
    target = new UdpTarget();
    tcpTarget = new tcpClient();
    HostAddress.clear();
    connect(target, SIGNAL(dataRequire()),this,SLOT(SendPolydata()));
    connect(target,SIGNAL(polySet(QByteArray)),this,SLOT(RecvPolySet(QByteArray)));

//    QThread *thread_1 = new QThread(this);
//    thread_1->start();
//    timer = new QTimer(0);
//    timer->setInterval(100);
//    timer->moveToThread(thread_1);
//    connect(timer, SIGNAL(timeout()),this,SLOT(Timeresponse()));
//    connect(thread_1,SIGNAL(started()),timer,SLOT(start()));
    IpKeys.clear();

    qRegisterMetaType<struct lidar>("struct lidar");
    qRegisterMetaType<struct point_data>("struct point_data");
    qRegisterMetaType<struct LidarC1Data>("struct LidarC1Data");
    qRegisterMetaType<std::vector<int>>("std::vector<int>");

    handleRun = new HandleRun();
    messagethread = new MessageThread();

 //   connect(handleRun,SIGNAL(sendMessageToTcp()),this,SLOT(()));

    connect(this,SIGNAL(HandleDataSignal(lidar, LidarC1Data)),handleRun,SLOT(getHandleData(lidar, LidarC1Data)));  //发送到聚类处理线程

    connect(this,SIGNAL(sendMessageToClient(LidarC1Data)),messagethread,SLOT(ReceiveData(LidarC1Data)));

    connect(handleRun,SIGNAL(lidarThCBack(lidar,point_data)),this,SLOT(ResultSlot(lidar,point_data)));  //发送到二维画布绘画
}


void GetlidarData::run()
{
    unsigned int size = sizeof(sockaddr_in);
    clientAddr1 =addr1;
    //接受数据
    //
    QString RecvIp;
    char recvBuf[1206] = { 0 };
    int recvLen;
    int index;
    while (true)
    {
        //
        recvLen = recvfrom(sock1, recvBuf, sizeof(recvBuf), 0,(struct sockaddr*)&(clientAddr1), &size);
        RecvIp = QString(inet_ntoa(clientAddr1.sin_addr));

        //   qDebug()<<"RecIp"<<RecvIp;

        if(IpKeys.contains(RecvIp)){
            //    qDebug()<<"data1";

            if (recvLen > 0) //
            {
                index = IpKeys[RecvIp];
                u_char data[1206] = { 0 };
                memcpy(data, recvBuf, 1206);

                for(int i=0;i<12;i++)
                {
                    if (data[0] == 0xff && data[1] == 0xee)
                    {
                        float angle =(data[3+100*i]*256+data[2+100*i])/100.f;  //小端模式
                        for(int j=0;j<30;j++)
                        {

                            if((angle+0.18*j)<=360 && (angle+0.18*j)>=0 && j<15 /*&& 4*(data[5+100*i+3*j]*256 +data[4+100*i+3*j]) > 10*/)
                            {

                                lidarData[index].angle.push_back(angle+0.18*j);
                                //  lidarData[index].distance.push_back(4*(data[5+100*i+3*j]*256 +data[4+100*i+3*j]));
                                lidarData[index].distance.push_back((data[5+100*i+3*j]*256 +data[4+100*i+3*j]));     //add

                            }
                            if((angle+0.18*j)<=360 && (angle+0.18*j)>=0 && j>=15 /*&& 4*(data[8+100*i+3*j]*256 +data[7+100*i+3*j]) > 10*/)
                            {

                                lidarData[index].angle.push_back(angle+0.18*j);
                                //  lidarData[index].distance.push_back(4*(data[8+100*i+3*j]*256 +data[7+100*i+3*j]));
                                lidarData[index].distance.push_back((data[8+100*i+3*j]*256 +data[7+100*i+3*j]));         //add
                            }

                            if(lidarData[index].angle[lidarData[index].angle.size()-1]>=180 && lidarData[index].angle[lidarData[index].angle.size()-1]<181 && lidarData[index].angle.size()>=1000)
                            {

                                //   qDebug()<<"len "<<recvLen<<" lidardatasize "<< lidarData[index].angle.size();
                                emit handledata(lidarData[index], index);
                                lidarData[index].angle.clear();
                                lidarData[index].distance.clear();
                                lidarData[index].intensity.clear();
                            }
                        }
                    }
                }
            }
        }
    }
}

bool GetlidarData::getStatus()
{
    return bindStatus;
}

bool GetlidarData::addSock(lidar info)
{
    lidarPack.insert(info.lidar_Index, info);
    conf_Prama conf;
    for(int i = 0; i < 3; i++){
        conf.polygon[i] = info.Polygon[i];
    }
    conf.distance = info.distance;
    conf.radius = info.lidar_Radius;
    conf.lidarIndex = info.lidar_Index;
    conf.lidarIp = info.lidar_Ip;
    conf.scanNum = info.scanNum;
    conf.MapDistance = info.MapDistance;
    conf_data.insert(info.lidar_Index, conf);
    IpKeys.insert(info.lidar_Ip,info.lidar_Index);
    LidarC1Data tmp;
    tmp.angle.clear();
    tmp.distance.clear();
    tmp.intensity.clear();

    lidarData.insert(info.lidar_Index, tmp);
    return true;
}

void GetlidarData::setCurSock(int index)
{
    if(curPos == index)return;
    curPos = index;
}

void GetlidarData::deletePoint(int pos, int index, int selectPoint)
{
    lidarPack[pos].Polygon[index].removeAt(selectPoint);
    conf_data[pos].polygon[index] = lidarPack[pos].Polygon[index];
}

void GetlidarData::addPoint(QPointF p, int pos, int index,int pindex)
{
    lidarPack[pos].Polygon[index].insert(pindex, p);
    conf_data[pos].polygon[index] = lidarPack[pos].Polygon[index];
}

void GetlidarData::updataPoint(QPointF p, int selectPoint, int pos, int index)
{
    lidarPack[pos].Polygon[index][selectPoint] = p;
    conf_data[pos].polygon[index] = lidarPack[pos].Polygon[index];
}
//建图
void GetlidarData::ReceivePolygon(QList<QPointF> polygon)
{
    lidarPack[1].Polygon[0].clear();
    lidarPack[1].Polygon[0] = polygon;
    conf_data[1].polygon[0] = lidarPack[1].Polygon[0];
    emit sendPolygon(lidarPack[1].Polygon[0]);
    //  sendPolygonToClient_1();
    //   qDebug()<<"lidarPack"<<lidarPack[1].Polygon[0].size();
}

void GetlidarData::setScanPower(bool flag)
{
    scanState = flag;
}

bool GetlidarData::getScanPower()
{
    return scanState;
}

void GetlidarData::setOutPutHandle(bool flag)
{
    outPutHandle = flag;
    //    if(!outPutHandle){

    //        timer->stop();
    //        tcpTarget->flag = false;
    //        qDebug()<<"停止输出！";
    //    }
    //    else{

    //        timer->start(500);
    //        qDebug()<<"开启输出！";
    //    }
}

bool GetlidarData::getOutPutHandle()
{
    return outPutHandle;
}

void GetlidarData::getCurInfo(int &distance, int &scanNum, int index)
{
    distance = lidarPack[index].distance;   //聚类半径
    scanNum = lidarPack[index].scanNum;   //最小点数
}

void GetlidarData::changeCurInfo(float distance, int scanNum, int index)
{
    lidarPack[index].distance = distance;
    lidarPack[index].scanNum = scanNum;
    conf_data[index].distance = distance;
    conf_data[index].scanNum = scanNum;
}
void GetlidarData::deleteSock(int index)
{
    IpKeys.remove(lidarPack[index].lidar_Ip);
    lidarPack.remove(index);
    conf_data.remove(index);
}

QByteArray GetlidarData::getPointData(int index, QPointF point)
{
    QByteArray byte;
    int x = point.rx()+ 10000;  //？？？？/
    int y = point.ry()+ 10000;
    byte.clear();
    byte.append(index&0XFF);    //雷达编号
    byte.append((index>>8)&0XFF);
    byte.append(x&0XFF);     //中心点
    byte.append((x>>8)&0XFF);
    byte.append(y&0XFF);
    byte.append((y>>8)&0XFF);
    byte.append(6,0X00);
    return byte;
}

QByteArray GetlidarData::getPointData(int index, int polyindex, QPointF point , int ID ,float len,float width, QPointF draw_point,float speed)
{
    // qDebug()<<"Point_Data_SendID"<<ID;
    QByteArray byte;
    //    int x = point.rx()+ 10000;  //+10000/？？？
    //    int y = point.ry()+ 10000;
    float x = point.rx();
    float y = point.ry();
    float z = sqrt(x * x + y * y);
    float draw_x = draw_point.rx();
    float draw_y = draw_point.ry();
    float angle;

    if(x == 0)
    {
        if(y>0) angle = 90;
        if(y<0) angle = 270;
    }
    else
    {
        angle = atan2(y,x)/PI*180;
        if(angle >= 0)
        {
            angle = angle;
        }
        else
        {
            angle = 360+angle;
        }
    }

    Angle_Filter[ID] = angle;
    Distance_Filter[ID] = z;
    //   qDebug()<<"centerPoint"<<z<<angle<<speed<<ID;
    int send_x = draw_x * 100;
    int send_y = draw_y * 100;
    int send_z = z;
    int sendSpeed = speed * 100;
    int sendAngle = angle * 100;
    int sendLen = len * 100;
    int sendWid = width * 100;
    byte.clear();

    byte.append((ID>>8)&0XFF);  //ID
    byte.append(ID&0XFF);

    byte.append((send_x>>8)&0XFF);  //center_X
    byte.append(send_x&0XFF);

    byte.append((send_y>>8)&0XFF);  //center_y
    byte.append(send_y&0XFF);

    byte.append((sendLen>>8)&0xFF);  //len
    byte.append(sendLen&0xFF);

    byte.append((sendWid>>8)&0xFF); //width
    byte.append(sendWid&0xFF);

    byte.append((send_z>>8)&0XFF);  //距离
    byte.append(send_z&0XFF);

    byte.append((sendAngle>>8&0XFF));  //角度
    byte.append(sendAngle&0XFF);

    byte.append((sendSpeed>>8)&0XFF);  //速度
    byte.append(sendSpeed&0XFF);

    return byte;
}

void GetlidarData::senddata(QByteArray &byteData, int Type)
{
    int pointsize = byteData.size()/12;
    int moresize = (100 - pointsize)*12;
    byteData.prepend((pointsize>>8)&0XFF);
    byteData.prepend(pointsize&0XFF);
    if(Type == NormalType){
        byteData.prepend((char)0XEE);
        byteData.prepend((char)0XFF);
    }
    else{
        byteData.prepend((char)0XFF);
        byteData.prepend((char)0XEE);
    }
    byteData.append(moresize,0X00);
    byteData.append((char)0XBB);
    byteData.append((char)0XAA);
    if(target->isUseAble()){
        //senddata
        //        qDebug()<<"senddata";
        target->sendData(byteData,HostAddress,HostPort);
    }
}

QByteArray GetlidarData::datapower(int Type)
{
    QByteArray data;
    data.clear();
    data.append((char)0XFF);
    data.append((char)0XDD);
    if(Type == ClearType){
        data.append(0X01);
    }
    else if(Type == SynType){
        data.append(0X02);
    }
    else if(Type == polyClear){
        data.append(0X03);
    }
    else if(Type == PolySyn){
        data.append(0X04);
    }
    else{

    }
    data.append(10,0X00);
    data.append((char)0XBB);
    data.append((char)0XAA);
    return data;
}

void GetlidarData::HandleDataSlot(LidarC1Data LidarData, int index)
{
    lidarPack[index].Radius = Ri;
    lidarPack[index].updateScene = InitState;
    //  lidarPack[index].Init_ID = initID;
    emit HandleDataSignal(lidarPack[index],LidarData);
    handleRun->start();
    messagethread->UDP_IP = lidarPack[index].lidar_Ip;
    messagethread->UDP_PORT = HostPort;
    emit sendMessageToClient(LidarData);
    messagethread->start();
}
static int filterSize=0;
static int filterSize_1=0;
void GetlidarData::Timeresponse()          //timeout调用
{
#if 1
    if(HostAddress.isEmpty())return;
    QPointF tmp;
    QByteArray byteData,byte;
    int clusterSize = 0;
    lidar lidartemp;
    byteData.clear();

    Point2f pt;
    vector<Point2f> points[2];  //01-19
    QList<int> rmIndex;

    byteData.prepend(0xEE);
    byteData.prepend(0xFF);

    QMap<int,lidar> tmpLidar = lidarPack;
    QList<int> Keys = tmpLidar.keys();
    int count = 0;

    lidartemp = tmpLidar[Keys[0]];
    msx = 300 /(tmpLidar[Keys[0]].lidar_Radius*1000);
    msy = 300 /(tmpLidar[Keys[0]].lidar_Radius*1000);
    clusterSize = lidartemp.PolygonPoints[0].size(); //区域内聚类个数

    if(clusterSize>50)
        clusterSize=50;

    if(lidartemp.PolygonPoints[0].size()>0 && sendID.size()>0 && lidartemp.PolygonPoints[0].size() == sendID.size()){
        if(lidartemp.PolygonPoints[0].size()<50)   //限制障碍物数量
        {
            //  qDebug()<<"ALL_CLusterSize()"<<lidartemp.PolygonPoints[0].size()<<sendID.size();
            //******************************************轨道界限***********************************************
            vector<Point2f> tmpPoints_L,tmpPoints_R;             //2022-01-19
            QList<QPointF> rect_L = railway.Left_Polygon;   //Area_points
            tmpPoints_L.clear();
            for(int j = 0; j < rect_L.size(); j++){
                tmpPoints_L.push_back(Point2f(rect_L[j].x(), rect_L[j].y()));
            }
            points[0] = tmpPoints_L;

            QList<QPointF> rect_R = railway.Right_Polygon;
            tmpPoints_R.clear();
            for(int j = 0; j < rect_R.size(); j++){
                tmpPoints_R.push_back(Point2f(rect_R[j].x(), rect_R[j].y()));
            }
            points[1] = tmpPoints_R;

            for(int k=0;k<lidartemp.PolygonPoints[0].size();k++){
                if(lidartemp.PolygonPoints[0].size() != sendID.size())break;
                pt.x = lidartemp.PolygonPoints[0].at(k).x();
                pt.y = lidartemp.PolygonPoints[0].at(k).y();
                if(points[0].size()>2){
                    double dis_l = pointPolygonTest(points[0], pt, true);
                    if(dis_l>0){
                        sendLeftCluster(lidartemp.PolygonPoints[0].at(k),sendID[k],sendWidth[k],sendLenth[k],sendSpeed[k]);
                        rmIndex.push_back(k);
                    }
                }
                if(points[1].size()>2){
                    double dis_r = pointPolygonTest(points[1], pt, true);
                    if(dis_r>0){   //列车
                        sendRightCluster(lidartemp.PolygonPoints[0].at(k),sendID[k],sendWidth[k],sendLenth[k],sendSpeed[k]);
                        rmIndex.push_back(k);
                    }
                }
            }
            int size = rmIndex.size();
            if(rmIndex.size()>1){
                for(int g=size-1;g>-1;g--){
                    //      qDebug()<<"rmIndex:"<<rmIndex.at(g)<<g;
                    lidartemp.PolygonPoints[0].removeAt(rmIndex.at(g));
                    sendID.erase(sendID.begin()+rmIndex.at(g));
                    sendWidth.erase(sendWidth.begin()+rmIndex.at(g));
                    sendLenth.erase(sendLenth.begin()+rmIndex.at(g));
                    sendSpeed.erase(sendSpeed.begin()+rmIndex.at(g));
                }
            }
            else if(rmIndex.size()==1)
            {
                lidartemp.PolygonPoints[0].removeAt(rmIndex.at(0));
                sendID.erase(sendID.begin()+rmIndex.at(0));
                sendWidth.erase(sendWidth.begin()+rmIndex.at(0));
                sendLenth.erase(sendLenth.begin()+rmIndex.at(0));
                sendSpeed.erase(sendSpeed.begin()+rmIndex.at(0));
            }
            //   qDebug()<<"After_CLusterSize()"<<lidartemp.PolygonPoints[0].size()<<sendID.front()<<rmIndex.size()<<sendID.size();
            rmIndex.clear();
            //****************************************轨道界限**************************************************
            clusterSize = clusterSize-size;
            byteData.append((clusterSize>>8)&0XFF);
            byteData.append(clusterSize&0XFF);
            for(int j = 0; j < clusterSize; j++){
                    tmp = lidartemp.PolygonPoints[0][j];
                    byte = getPointData(Keys[0], 1, QPointF(tmp.rx()/msx*0.1,tmp.ry()/msy*(0.1)), sendID[j],sendLenth[j],sendWidth[j],QPointF(tmp.rx(),tmp.ry()),sendSpeed[j]); //转为cm **************hct-add
                    byteData.append(byte);
            }
        }

        int zero = 0;
        if(clusterSize<50 && clusterSize>0){
            for(int k=0;k<16*(50-clusterSize);k++)
            {
                byteData.append(zero);
            }
        }
    int sum = check_sum(byteData,byteData.size());
    byteData.append(sum & 0xFF);
   }
    byteData.append(0xAA);
    byteData.append(0xBB);
    //    qDebug()<<"byte.size"<<byteData.size();
    if(byteData.size()>7) emit sendClusterMessage(byteData);
    else{
        handleRun->idFlag = false;
        emit sendClearDraw();
       }
#endif
}

void GetlidarData::SendPolydata()
{
    //ff ee 01
    if(!target->isUseAble())return;
    QPointF tmp;
    QByteArray byteData,byte;
    lidar lidartemp;
    byteData.clear();
    QByteArray data = datapower(polyClear);
    target->sendData(data,HostAddress,HostPort);
    QMap<int,lidar> tmpLidar = lidarPack;
    QList<int> Keys = tmpLidar.keys();
    int count = 0;
    for(int m = 0; m < tmpLidar.size(); m++){
        lidartemp = tmpLidar[Keys[m]];
        msx = 300 /(tmpLidar[Keys[m]].lidar_Radius*1000);
        msy = 300 /(tmpLidar[Keys[m]].lidar_Radius*1000);
        for(int i = 0; i < 3; i++){
            for(int j = 0; j < lidartemp.Polygon[i].size(); j++){
                tmp = lidartemp.Polygon[i][j];   //区域边缘点
                //                qDebug()<<"x === "<<tmp.rx()/msx*0.1 <<" cm y=== "<<tmp.ry()/msy*0.1<< " cm";
                byte = getPointData(Keys[m], i+1, QPointF(tmp.rx()/msx*0.1,tmp.ry()/msy*0.1) , sendID[j],sendLenth[j],sendWidth[j],QPoint(tmp.rx(),tmp.ry()),sendSpeed[j]);  //cm
                byteData.append(byte);
                count++;
                if(count == 100){
                    senddata(byteData,PolyType);
                    count -= 100;
                    byteData.clear();
                }
            }
        }
    }
    senddata(byteData,PolyType);
    data = datapower(PolySyn);
    target->sendData(data,HostAddress,HostPort);

}

void GetlidarData::ResultSlot(lidar li, point_data data)
{
    //    qDebug()<<"resultSlot:"<<li.lidar_Index;
#if 1
    lidarPack[li.lidar_Index].PolygonPoints[0].clear();
    if(!lidarPack.contains(li.lidar_Index))return;
    for(int i = 0; i < 3; i++){
        lidarPack[li.lidar_Index].PolygonPoints[i] = li.PolygonPoints[i];
    }
    if(curPos == li.lidar_Index){
        float msx = 300 /(li.lidar_Radius*1000);
        //        float msy = 300 /(li.lidar_Radius*1000);
        int count = 0;
        for(int i = 0; i < 3; i++){
            if(scanState){
                data.in[i] = li.PolygonPoints[i];
            }
            count+= li.PolygonPoints[i].size();
        }
        Timeresponse();
        emit SendData(data);
        emit updateTable(msx*1000.0, li.lidar_Radius, li.lidar_Ip, count, data.out.size());
    }
#endif
}

void GetlidarData::RecvPolySet(QByteArray polyByte)      //获取区域设置的点
{
    //FF EE 02 INDEX1 INDEX2 NUM 200*8 BB AA    LEN 1610
    QByteArray polydata;
    int lidarIndex = (int)polyByte[3] + ((int)polyByte[4])*256;
    int polyIndex = (int)polyByte[3] + ((int)polyByte[4])*256;
    int num = (int)polyByte[7];
    QList<QPointF> polyPoints;
    int x,y;
    polyPoints.clear();
    if(lidarPack.contains(lidarIndex) && polyIndex <= 3 && polyIndex > 0){
        qDebug()<<"be lidar confige";
        float msx = 300 /(lidarPack[lidarIndex].lidar_Radius*1000);
        float msy = 300 /(lidarPack[lidarIndex].lidar_Radius*1000);
        for(int i = 0; i < num; i++){
            polydata = polyByte.mid(8+8*i,8);
            x = (int)polydata[0] + ((int)polydata[1])*256 - 10000; //-10000???/
            y = (int)polydata[2] + ((int)polydata[3])*256 - 10000;
            qDebug()<<x<<"    "<<y;
            QPointF tmp = QPointF(x*10*msx,y*10*msy);  //单位转换为m
            qDebug()<<tmp;
            polyPoints.push_back(tmp);
        }
        lidarPack[lidarIndex].Polygon[polyIndex-1] = polyPoints;
        conf_data[lidarIndex].polygon[polyIndex-1] = polyPoints;
        emit setPolyByRecv(lidarIndex,lidarPack[lidarIndex]);
    }
    else{
        qDebug()<<"no such lidar";
    }
}

void GetlidarData::sendPolygonToClient_1()
{
    QByteArray byte;
    float msxx = 300 /(lidarPack[1].lidar_Radius*1000);
    float msyy = 300 /(lidarPack[1].lidar_Radius*1000);
    int size = lidarPack[1].Polygon[0].size();
    qDebug()<<"up"<<size;
    int index = ceil(size/300.f);  //
    for(int j=0;j<index;j++)
    {
        byte.prepend(0xDD);
        byte.prepend(0xCC);
        byte.append((size>>8)&0XFF);
        byte.append(size&0XFF);
        for(int i=0;i<300;i++)
        {

            if((i + 300*j)>(size-1))
            {
                int zero = 0;
                byte.append((zero>>8)&0XFF);  //x
                byte.append(zero&0XFF);
                byte.append((zero>>8)&0XFF);  //y
                byte.append(zero&0XFF);
            }
            else{
                int polygon_x = lidarPack[1].Polygon[0].at(i + 300*j).x() / (10*msxx);  //cm
                int polygon_y = lidarPack[1].Polygon[0].at(i + 300*j).y() / (10*msyy);
                byte.append((polygon_x>>8)&0XFF);  //x
                byte.append(polygon_x&0XFF);
                byte.append((polygon_y>>8)&0XFF);  //y
                byte.append(polygon_y&0XFF);
            }
        }
        byte.append(0XA0);
        byte.append(0XB0);
        msleep(1);
        emit sendPolygonToClient(byte);
        byte.clear();
    }
}

void GetlidarData::sendPolyEnding()
{
    QByteArray byte;
    byte.prepend(0xD0);
    byte.prepend(0xC0);
    byte.append(0xE4);
    byte.append(0x74);
    byte.append(0xBB);
    byte.append(0xFF);
    emit sendPolygonEnding(byte);
    byte.clear();
}

int GetlidarData::check_sum(QByteArray ba, int len)
{
    int sum = 0;
    for(int i=0;i<len;i++)
    {
        int tmp = ba[i];
        if(tmp<0)
            tmp=256 + tmp;
        sum += tmp;
    }
    return sum;
}
//获取轨道界限
void GetlidarData::ReceiveRailPolygon(QList<QPointF> left, QList<QPointF> right)
{
    railway.Left_Polygon = left;
    railway.Right_Polygon = right;
    lidarPack[1].Polygon[1].clear();
    lidarPack[1].Polygon[2].clear();
    lidarPack[1].Polygon[1] = railway.Left_Polygon;
    lidarPack[1].Polygon[2] = railway.Right_Polygon;
    conf_data[1].polygon[1] = lidarPack[1].Polygon[1];
    conf_data[1].polygon[2] = lidarPack[1].Polygon[2];
    sendRailPolygon(railway.Left_Polygon,railway.Right_Polygon);
    WriteRailwayConfig();
}

//左边轨道列车
void GetlidarData::sendLeftCluster(QPointF leftcluster,int ID,float width,float len,float speed)
{
    QByteArray byte;
    float x,y,angle;
    int z;

    //  qDebug()<<"leftpoint"<<leftcluster.x()<<leftcluster.y();

    x=leftcluster.x()/msx*0.1;
    y=leftcluster.y()/msy*0.1;
    z=sqrt(x*x + y*y);

    if(x == 0)
    {
        if(y>0) angle = 90;
        if(y<0) angle = 270;
    }
    else
    {
        angle = atan2(y,x)/PI*180;
        if(angle >= 0)
        {
            angle = angle;
        }
        else
        {
            angle = 360+angle;
        }
    }

    int send_x = leftcluster.x() * 100;
    int send_y = leftcluster.y() * 100;
    int send_z = z;
    int sendSpeed = speed * 100;
    int sendAngle = angle * 100;
    int sendLen = len * 100;
    int sendWid = width * 100;
    byte.clear();

    byte.prepend(0xD7);
    byte.prepend(0xC7);
    byte.append(0xE7);
    byte.append(0xB0);

    byte.append((ID>>8)&0XFF);  //ID
    byte.append(ID&0XFF);

    byte.append((send_x>>8)&0XFF);  //center_X
    byte.append(send_x&0XFF);

    byte.append((send_y>>8)&0XFF);  //center_y
    byte.append(send_y&0XFF);

    byte.append((sendLen>>8)&0xFF);  //len
    byte.append(sendLen&0xFF);

    byte.append((sendWid>>8)&0xFF); //width
    byte.append(sendWid&0xFF);

    byte.append((send_z>>8)&0XFF);  //距离
    byte.append(send_z&0XFF);

    byte.append((sendAngle>>8&0XFF));  //角度
    byte.append(sendAngle&0XFF);

    byte.append((sendSpeed>>8)&0XFF);  //速度
    byte.append(sendSpeed&0XFF);

    int sum = check_sum(byte,byte.size());

    byte.append(sum & 0xFF);
    byte.append(0xB7);
    byte.append(0xF7);
    if(len>30 || width>30){
        //  qDebug()<<"Left_TrainPoint"<<x<<y<<z<<angle<<speed<<ID<<"尺寸:"<<len<<width;
        emit sendLeftTrain(byte);
    }
    byte.clear();
}
//右边轨道列车
void GetlidarData::sendRightCluster(QPointF rightcluster,int ID,float width,float len,float speed)
{
    // qDebug()<<"RightTrain";
    QByteArray byte;
    float x,y,angle;
    int z;

    x=rightcluster.x()/msx*0.1;
    y=rightcluster.y()/msy*0.1;
    z=sqrt(x*x + y*y);

    if(x == 0)
    {
        if(y>0) angle = 90;
        if(y<0) angle = 270;
    }
    else
    {
        angle = atan2(y,x)/PI*180;
        if(angle >= 0)
        {
            angle = angle;
        }
        else
        {
            angle = 360+angle;
        }
    }
    int send_x = rightcluster.x() * 100;
    int send_y = rightcluster.y() * 100;
    int send_z = z;
    int sendSpeed = speed * 100;
    int sendAngle = angle * 100;
    int sendLen = len * 100;
    int sendWid = width * 100;
    byte.clear();

    byte.prepend(0xD7);
    byte.prepend(0xC7);
    byte.append(0xE7);
    byte.append(0xB1);

    byte.append((ID>>8)&0XFF);  //ID
    byte.append(ID&0XFF);

    byte.append((send_x>>8)&0XFF);  //center_X
    byte.append(send_x&0XFF);

    byte.append((send_y>>8)&0XFF);  //center_y
    byte.append(send_y&0XFF);

    byte.append((sendLen>>8)&0xFF);  //len
    byte.append(sendLen&0xFF);

    byte.append((sendWid>>8)&0xFF); //width
    byte.append(sendWid&0xFF);

    byte.append((send_z>>8)&0XFF);  //距离
    byte.append(send_z&0XFF);

    byte.append((sendAngle>>8&0XFF));  //角度
    byte.append(sendAngle&0XFF);

    byte.append((sendSpeed>>8)&0XFF);  //速度
    byte.append(sendSpeed&0XFF);

    int sum = check_sum(byte,byte.size());

    byte.append(sum & 0xFF);
    byte.append(0xB7);
    byte.append(0xF7);
    if(len>30 || width>30){
        //  qDebug()<<"Right_TrainPoint"<<x<<y<<z<<angle<<speed<<ID<<"尺寸:"<<len<<width;
        emit sendRightTrain(byte);
    }
    byte.clear();
}
//保存轨道界限文件
void GetlidarData::WriteRailwayConfig()
{
    QSettings config("/home/ubuntu/LS_Config/config.ini", QSettings::IniFormat);
    config.setIniCodec("UTF-8");
    config.beginGroup("RailwayPolygon");
    QString datastr;
    datastr.clear();
    for(int n = 0; n < railway.Left_Polygon.size(); n++)
    {
        QPointF tmp = railway.Left_Polygon[n];
        QString str = QString("(%1,%2)").arg(tmp.rx()).arg(-tmp.ry());
        datastr.append(str);
        if(n!= railway.Left_Polygon.size()-1){
            datastr.append(" ");
        }
    }
    config.setValue(QString("Lelf_Polygon"), datastr);

    QString datastr_r;
    datastr.clear();
    for(int n = 0; n < railway.Right_Polygon.size(); n++)
    {
        QPointF tmp = railway.Right_Polygon[n];
        QString str = QString("(%1,%2)").arg(tmp.rx()).arg(-tmp.ry());
        datastr_r.append(str);
        if(n!= railway.Right_Polygon.size()-1){
            datastr_r.append(" ");
        }
    }
    config.setValue(QString("Right_Polygon"), datastr_r);
    config.endGroup();
}

void GetlidarData::get_device_config()
{
    QSettings config("VersionConfig.ini", QSettings::IniFormat);
    config.setIniCodec("UTF-8");
    config.beginGroup("WidthFilter");
    minWidth = config.value("minWidth","").toInt();
    maxWidth = config.value("maxWidth","").toInt();
    handleRun->minWidth = minWidth;
    handleRun->maxWidth = maxWidth;
    qDebug()<<"Widthfilter:"<<minWidth<<maxWidth;
    config.endGroup();
}

void GetlidarData::update_device_config()
{
    QSettings config("VersionConfig.ini", QSettings::IniFormat);
    config.setIniCodec("UTF-8");
    config.beginGroup("WidthFilter");
    config.setValue("minWidth",QString("%1").arg(minWidth));
    config.setValue("maxWidth",QString("%1").arg(maxWidth));
    config.endGroup();
}

void GetlidarData::ReceiveWidthFilter(int min, int max)
{
    minWidth = min;
    maxWidth = max;
    handleRun->minWidth = minWidth;
    handleRun->maxWidth = maxWidth;
    update_device_config();
}

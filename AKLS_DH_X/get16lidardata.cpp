#include "get16lidardata.h"

Get16LidarData::Get16LidarData(QObject *parent) : QThread(parent)
{
//    LidarData lidardata;
//    QVariant DataVar;
//    DataVar.setValue(lidardata);
//    qRegisterMetaType<QVariant>("QVariant");
    udp_init(&sock1,&clientAddr1,2368);
}

void Get16LidarData::udp_init(int* sock_num, sockaddr_in* sockaddr, int server_port)
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

void Get16LidarData::UnInitialize(int* sock_num)
{
  shutdown(*sock_num,SHUT_RDWR);
}


void Get16LidarData::run()
{
    unsigned int size = sizeof(sockaddr_in);
    clientAddr1 =addr1;
    //接受数据

    LidarData lidardata;
//        std::vector<std::vector<int> >distance;
    lidardata.distance.resize(16);
    lidardata.intensity.resize(16);
    lidardata.LaserID.resize(16);
    char recvBuf[1206] = { 0 };
    int recvLen;
     while(1){
     recvLen = recvfrom(sock1, recvBuf, sizeof(recvBuf), 0,(struct sockaddr*)&(clientAddr1), &size);
    //	qDebug("len = %d",recvLen);
        if (recvLen > 0) //接受到数据
        {
            u_char data[1206] = { 0 };
            memcpy(data, recvBuf, 1206);
            if (data[0] == 0xff && data[1] == 0xee)
            {
            //	int32_t mtimestamp = 16777216 * data[1203] + 65536 * data[1202] + 256 * data[1201] + data[1200];
                for (int i = 0; i < 12; i = i + 2)  //LS16线协议和Velodyne有区别
                {
                    if ((data[3 + 100 * i] * 256 + data[2 + 100 * i]) / 100.f >= 360.0)
                    {
                        lidardata.angle.push_back(0);
                    }
                    else
                    {
                        lidardata.angle.push_back((data[3 + 100 * i] * 256 + data[2 + 100 * i]) / 100.f);        //提取出一圈中的方位角度值
                    }
                    for (int j = 0; j < 16; j++)
                    {
                        lidardata.distance[j].push_back((data[5 + 3 * j + 100 * i] * 256 + data[4 + 3 * j + 100 * i]));
                        lidardata.intensity[j].push_back(data[6 + 3 * j + 100 * i]);
                        lidardata.LaserID[j].push_back(j);
                    //	lidardata.mtimestamp.push_back(mtimestamp + FIRING_TOFFSET*i + DSR_TOFFSET*j);
                    }
                    if (lidardata.angle.size() >= 2)
                    {
                        if (lidardata.angle[lidardata.angle.size() - 1] < 1.0 && lidardata.angle[lidardata.angle.size() - 2] > 359.0)
                        {
                            if (lidardata.angle[lidardata.angle.size() - 1] + lidardata.angle[lidardata.angle.size() - 2] > 360.0)
                            {
                                lidardata.angle.push_back(lidardata.angle[lidardata.angle.size() - 1] + lidardata.angle[lidardata.angle.size() - 2] - 360.0);
                            }
                            else
                            {
                                lidardata.angle.push_back(lidardata.angle[lidardata.angle.size() - 2] + 0.2);
                            }
                        }
                        else
                        {
                            lidardata.angle.push_back((lidardata.angle[lidardata.angle.size() - 1] + lidardata.angle[lidardata.angle.size() - 2]) / 2);  //插入后半部数据
                        }
                        for (int j = 0; j < 16; j++)
                        {
                            lidardata.distance[j].push_back((data[53 + 3 * j + 100 * i] * 256 + data[52 + 3 * j + 100 * i]));
                            lidardata.intensity[j].push_back(data[54 + 3 * j + 100 * i]);
                            lidardata.LaserID[j].push_back(j);
                        //	lidardata.mtimestamp.push_back(mtimestamp + FIRING_TOFFSET*(i + 1) + DSR_TOFFSET*j);
                        }
                    }
                //	qDebug("(%f   %f   %d  )", lidardata.angle[lidardata.angle.size() - 1], lidardata.angle[lidardata.angle.size() - 1], lidardata.angle.size());
                    if (lidardata.angle[lidardata.angle.size() - 1] >= 90 && lidardata.angle[lidardata.angle.size() - 1] <= 91 && lidardata.angle.size() > 1500 && lidardata.angle.size()<5000)
                    {
                        emit SendData(lidardata);
                        lidardata.angle.clear();
                        lidardata.LaserID.clear();
                        lidardata.LaserID.resize(16);
                        lidardata.distance.clear();
                        lidardata.distance.resize(16);
                        lidardata.intensity.clear();
                        lidardata.intensity.resize(16);
                    //	lidardata.mtimestamp.clear();
                    }
                    if (lidardata.angle.size() >= 5000){
                        lidardata.angle.clear();
                        lidardata.LaserID.clear();
                        lidardata.LaserID.resize(16);
                        lidardata.distance.clear();
                        lidardata.distance.resize(16);
                        lidardata.intensity.clear();
                        lidardata.intensity.resize(16);
                    }
                }
            }
            /*if (data[0] == 0xa5 && data[1] == 0xff)
            {
                m_ThDataPara.DataBuffer[0] = (u_char*)data;
                DecodeDIFOPData((const _Lidar_Parameter*)m_ThDataPara.DataBuffer[0], lidarstatus);
                emit SendLidarStatusToSettingUI(&lidarstatus);
            }*/
        }
        else
            sleep(1);
    }
}

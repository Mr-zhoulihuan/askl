#include "handlerun.h"
#include "QDebug"
#include <ctime>
#include <cstdlib>
//#include <opencv2/opencv.hpp>
#define PI 3.1415926835
using namespace cv;
std::vector<int> sendID;
std::vector<float> sendWidth;
std::vector<float> sendLenth;
std::vector<float> sendSpeed;

bool get_filelist_from_dir(std::string _path, std::vector<std::string>& _files)
{
    DIR* dir = opendir(_path.c_str());
    struct dirent* ptr;
    std::vector<std::string> file;
    while((ptr = readdir(dir)) != NULL)
    {
        if(ptr->d_name[0] == '.')	continue;
        file.push_back(ptr->d_name);
    }
    closedir(dir);
    sort(file.begin(), file.end());
    _files = file;
}
HandleRun::HandleRun(QObject *parent) :QThread(parent)
{
    trackinit(); //202104
    srand(time(0));//2022-4-22
    for(int i=0;i<m_idSize;i++)
    {
        m_idFrame.push_back(m_frameSz);
        m_isOut.push_back(true);
    }
}

HandleRun::~HandleRun()
{

}
/* lidar  区域内外点
   LidarC1data 雷达原始数据 distance angle*/
void HandleRun::getHandleData(lidar lidarinfo, LidarC1Data data)
{
    RunLi = lidarinfo;// 数据接收
    RunData = data;
}
static int id1=0;
void HandleRun::run()
{
    point_data data;  //point_data clusterPoints
    QPointF tmp;
    Point2f pt;
    vector<Point2f> points[3];  //three Area
    data.out.clear();  //out_points
    data.allpoints_In.clear();  //in_points
    data.lidarIndex = RunLi.lidar_Index;  //lidar_index
    msx = 300 /(RunLi.lidar_Radius*1000);         //??????
    float msy = 300 /(RunLi.lidar_Radius*1000);
#if 0    //opencv
    for(int i = 0; i < 3; i++){        //Area_index
        vector<Point2f> tmpPoints;
        QList<QPointF> rect = RunLi.Polygon[i];   //Area_points
        tmpPoints.clear();
        data.in[i].clear();
        for(int j = 0; j < rect.size(); j++){
            tmpPoints.push_back(Point2f(rect[j].x(), rect[j].y()));
        }
        points[i] = tmpPoints;  //Area_points
    }

    for(uint i = 0; i < RunData.angle.size();i++)
    {
        if(RunData.distance[i] < 10)continue;
        tmp.setX(RunData.distance[i] * sin(RunData.angle[i]*M_PI/180)*msx);
        tmp.setY(RunData.distance[i] * cos(RunData.angle[i]*M_PI/180)*msy*(-1.0));      //tmp-allPoints

        bool ins = false;
     //   for(int m = 0; m < 3; m++){
            if(points[0].size() > 2){   //three points constuct an Area
               // qDebug()<<"handlerun"<<points[m].size();
                pt.x = tmp.x();
                pt.y = tmp.y();
                double dis = pointPolygonTest(points[0], pt, true);  //OpenCv ---pointPolygonTest(InputArray contour, Point2f pt, bool measureDist)
                                                                    //当measureDist设置为true时，返回实际距离值。若返回值为正，表示点在多边形内部，返回值为负，表示在多边形外部，返回值为0，表示在多边形上。
                                                                   //当measureDist设置为false时，返回 -1、0、1三个固定值。若返回值为+1，表示点在多边形内部，返回值为-1，表示在多边形外部，返回值为0，表示在多边形上。
                if(dis > 0){      //In area
                    data.in[0].push_back(tmp);            //区域内所有的点
                    data.allpoints_In.push_back(tmp);
                    ins = true;
                }
            }
      //  }
        if(!ins)data.out.push_back(tmp);        //区域外所有的点
    }
#else
//    int size = RunData.angle.size();
//    if(polygonDistance.size()>0)
//    {
//        if(size>polygonDistance.size())
//            size = polygonDistance.size();
//    }
    if(polygonDistance.size()>0)
    {
        for(int i=0;i<RunData.angle.size();i++)
        {
            if(RunData.distance[i] < 500)continue;
            tmp.setX(RunData.distance[i] * sin(RunData.angle[i]*M_PI/180)*msx);
            tmp.setY(RunData.distance[i] * cos(RunData.angle[i]*M_PI/180)*msy*(-1.0));      //tmp-allPoints
            int index_m = (RunData.angle.at(i) * 100 +26)/18 -1;
            if( index_m < polygonDistance.size())
            {
#if 1
                if(RecoverFlag){
                    if(RunData.distance[i] < (polygonDistance[index_m] - 2 * conf_data[1].MapDistance)  || RunData.distance[i] > (polygonDistance[index_m] + 6 * conf_data[1].MapDistance))
                    {
                        //qDebug()<<"distance"<<RunData.angle[i]<<RunData.distance[i]<<polygonDistance[i];
                        data.in[0].push_back(tmp);
                        data.allpoints_In.push_back(tmp);
                    }
                    else{
                        data.out.push_back(tmp);
                    }
                }
                else{
                    if(RunData.distance[i] < polygonDistance[index_m])
                    {
                        //qDebug()<<"distance"<<RunData.angle[i]<<RunData.distance[i]<<polygonDistance[i];
                        data.in[0].push_back(tmp);
                        data.allpoints_In.push_back(tmp);
                    }
                    else{
                        data.out.push_back(tmp);
                    }
                }
#else
                if(RunData.distance[i] < polygonDistance[index_m])
                {
                    //qDebug()<<"distance"<<RunData.angle[i]<<RunData.distance[i]<<polygonDistance[i];
                    data.in[0].push_back(tmp);
                    data.allpoints_In.push_back(tmp);
                }
                else{
                    data.out.push_back(tmp);
                }
#endif
            }
            else{
                //   data.out.push_back(tmp);
            }
        }//雷达聚类算法

    }
    else{
        for(int i=0;i<RunData.angle.size();i++)
        {
            if(RunData.distance[i] < 500)continue;
            tmp.setX(RunData.distance[i] * sin(RunData.angle[i]*M_PI/180)*msx);
            tmp.setY(RunData.distance[i] * cos(RunData.angle[i]*M_PI/180)*msy*(-1.0));      //tmp-allPoints
            data.out.push_back(tmp);
        }
    }
#endif
  //  qDebug()<<"boolupdate2"<<RunLi.updateScene;
    if(RunLi.updateScene == true)
    {
     //  qDebug()<<"save points";
       SaveInitCenterpoints(RunLi.PolygonPoints[0]);
    }
    else
    {
        for(int i = 0; i<InitCenterPoints.size();i++)  //半径滤波
        {
           // qDebug()<<"Savepoints:"<<InitCenterPoints[i].x() <<InitCenterPoints[i].y()<<RunLi.Radius;
            data.in[0] = UpdateScene(InitCenterPoints[i],data.in[0],RunLi.Radius);
        }
    }

  //  RunLi.PolygonPoints[0] = InitDBtree(data.in[0],RunLi.distance,RunLi.scanNum);   //聚类的中心点

  //  void HandleRun::LidarClusterringRun(std::vector<pcl::PointXYZ> LidarPointFilter)

#if 1
    std::vector<pcl::PointXYZ> LidarPointFilter;
    for(int i=0;i<data.in[0].size();i++)
    {
        pcl::PointXYZ tem;
        tem.x =data.in[0].at(i).x()/msx/1000.0;
        tem.y =data.in[0].at(i).y()/msy/(-1000.0);

      //  qDebug()<<"i="<<i<<"x="<<tem.x<<"y="<<tem.y;
        LidarPointFilter.push_back(tem);
    }

    ObjectCenter.clear();
    ObjectWidth.clear();
    ObjectLength.clear();
    ObjectHeight.clear();
    LidarClusterringRun(LidarPointFilter);
    /************/
    std::vector<Cluster> clusters;
    std::vector<Object> All_Objects2;

  //  qDebug()<<"ObjectCenter.size();"<<ObjectCenter.size();
  //  static FILE* trackingFile = fopen("trackingResult.txt", "a");


    clusters.clear();
    for (uint i = 0; i < ObjectCenter.size(); i++)
    {
         Cluster cluster;
        cluster.type ="people";
        cluster.posX  =ObjectCenter[i].x;
        cluster.posY =ObjectCenter[i].y;
        cluster.posZ =ObjectCenter[i].z;

        cluster.width  =ObjectWidth[i];
        cluster.length =ObjectLength[i];
        cluster.height =ObjectHeight[i];
        cluster.direct =1;
        cluster.usedFlag =false;
        clusters.push_back(cluster);


    }


//    for(int i=0;i<ObjectCenter.size(); i++)
//    {
//        Object obj;
//       obj.ID = 0;
//      // obj.label = classifyTemp;
//       obj.speed_x= 0;
//       obj.speed_y =0;
//       obj.x =ObjectCenter[i].x;
//       obj.y =ObjectCenter[i].y;
//       obj.z = ObjectCenter[i].z;
//       obj.width =ObjectWidth[i];
//       obj.length =ObjectLength[i];
//       obj.height = ObjectHeight[i];
//       All_Objects2.push_back(obj);
//    }

    float dt =0.1f;
    std::vector<Tracking> trackingList;

    LidarTracking(clusters,trackingList,dt);
   // qDebug()<<"cluste"<<clusters.size()<<"      (int)trackingList.size() "<<(int)trackingList.size();
    for (int i = 0; i < (int)trackingList.size(); i++)
    {

        Object obj;
        obj.ID = trackingList[i].ID;
       // obj.label = classifyTemp;
        obj.speed_x= trackingList[i].X[2];
        obj.speed_y =trackingList[i].X[3];
        obj.x =trackingList[i].X[0];
        obj.y =trackingList[i].X[1];
        obj.z = trackingList[i].posZ;;
        obj.width =trackingList[i].width;
        obj.length =trackingList[i].length;
        obj.height = trackingList[i].height;
        All_Objects2.push_back(obj);
       // static FILE* trackingFile = fopen("trackingResult.txt", "a");

       // fprintf(trackingFile, "%d	%f	%f	%f	%f	%f	%f	%f\n", obj.ID, obj.x, obj.y, obj.z, obj.width, obj.length, obj.height);
    }

    sendID.clear();
    sendWidth.clear();
    sendLenth.clear();
    sendSpeed.clear();

vector<int>vec_idShow;
std::vector<Object> new_All_Objects2;
getShowId(All_Objects2,vec_idShow,new_All_Objects2);
//new_All_Objects2 =All_Objects2;


//qDebug()<<vec_idShow;
//vector<int>TestID;
QList<QPointF> mClusterPoint;
    for(int s = 0; s < (int)new_All_Objects2.size();s++)
    {
        //TestID.push_back(All_Objects2[s].ID);
        QPointF tmp;
        float trackSpeed;

       // qDebug()<<"All_Objects2[s].ID"<<All_Objects2[s].ID;

        tmp.setX(new_All_Objects2[s].x * 6);
        tmp.setY(-new_All_Objects2[s].y * 6) ;
        mClusterPoint.push_back(tmp);             //聚类x、y坐标，中心点
        trackSpeed = sqrt(new_All_Objects2[s].speed_x * new_All_Objects2[s].speed_x + new_All_Objects2[s].speed_y*new_All_Objects2[s].speed_y);
        sendSpeed.push_back(trackSpeed);   //m/s

        sendID.push_back(vec_idShow[s]);
      //  sendID.push_back(new_All_Objects2[s].ID);
        sendWidth.push_back(new_All_Objects2[s].width * 6);
        sendLenth.push_back(-new_All_Objects2[s].length * 6);
    }

    RunLi.PolygonPoints[0] =mClusterPoint;
#endif
     emit lidarThCBack(RunLi,data);      //数据传递，聚类代表点、雷达所有点以及ID

}

QList<QPointF> HandleRun::InitDBtree(QList<QPointF> m, float distance, int scanNum)
{
    QList<data_set> dataSets;
    if(m.size() ==0) return QList<QPointF>{};
    dataSets.clear();
    for(int i = 0; i< m.size(); i++)      //m为区域内的点
    {
        data_set tempdata;
        tempdata.dpId = i;
        tempdata.visited = false;
        tempdata.cluterID = -1;
        tempdata.point = m[i];
        tempdata.arrivalPointID.clear();
        dataSets.push_back(tempdata);     //点的id和所有点
    }
    for(int i = 0; i < m.size(); i++)
    {
        setArrivalPoint(dataSets, dataSets[i], distance, scanNum);   //筛选
    }
    return DoDBSCANRecursive(dataSets);
}


double HandleRun::GetDistance(data_set &dp1, data_set &dp2)         //两点之间的距离
{
    double d = 0;
    d = sqrt((dp1.point.x()-dp2.point.x())*(dp1.point.x()-dp2.point.x()) +(dp1.point.y()-dp2.point.y())*(dp1.point.y()-dp2.point.y()));
    return d;
}

void HandleRun::setArrivalPoint(QList<data_set> &dataSets, data_set &dp, float distance, int scanNum)
{
    for(int i = 0;  i < dataSets.size(); i++){   //所有点和单点比较，遍历dataSets.size()次
        double d = GetDistance(dataSets[i], dp);
        if(d <= distance && i != dp.dpId){   //距离半径
            dp.arrivalPointID.push_back(i);
        }
    }
    if(dp.arrivalPointID.size() >= scanNum){    //最小聚集点数
        dp.isKey = true;    //聚类key
        return;
    }
    dp.isKey = false;
}


QList<QPointF> HandleRun::DoDBSCANRecursive(QList<data_set> &dataSets)         //聚类
{
    std::vector<Object> All_Objects;
    All_Objects.clear();

    int clusterId = 0;
    for(int i = 0; i <dataSets.size(); i++){
        data_set& dp = dataSets[i];   //每个点
        if(!dp.visited && dp.isKey){
            dp.cluterID = clusterId;
            dp.visited = true;
            keyPointcluster(dataSets, i, clusterId);
            clusterId++;
        }
    }

    QList<QList<QPointF>> m_ListPoints;
    m_ListPoints.clear();
    for(int i = 0; i <clusterId; i++){
        QList<QPointF> tmp;
        tmp.clear();
        m_ListPoints.push_back(tmp);
    }

    int n;
    QList<QPointF>  mPoints;
    for(int i = 0; i < dataSets.size();i++){
        if(dataSets[i].cluterID != -1){
            mPoints.push_back(dataSets[i].point);
            n = dataSets[i].cluterID;
            m_ListPoints[n].push_back(dataSets[i].point);
        }
        else
        {
            //            qDebug()<<"孤立点"<<dataSets[i].point;
        }
    }

    QList<QPointF> mClusterPoint;
    mClusterPoint.clear();
    for(int i= 0; i <clusterId; i++){
        qreal x = 0;
        qreal y = 0;
        //***************************202104********************************
        float Minx = m_ListPoints[i].at(0).x();
        float Maxx = m_ListPoints[i].at(0).x();
        float Miny = m_ListPoints[i].at(0).y();
        float Maxy = m_ListPoints[i].at(0).y();
        //   qDebug()<<"Minmax:"<<Minx;

        float centerx = 0;
        float centery = 0;
        float centerz = 0;


        for(int k = 0; k < m_ListPoints[i].size(); k++){
            Minx = Minx < m_ListPoints[i].at(k).x() ? Minx : m_ListPoints[i].at(k).x();  //查找每个聚类的最大最小值
            Maxx = Maxx > m_ListPoints[i].at(k).x() ? Maxx : m_ListPoints[i].at(k).x();
            Miny = Miny < m_ListPoints[i].at(k).y() ? Miny : m_ListPoints[i].at(k).y();
            Maxy = Maxy > m_ListPoints[i].at(k).y() ? Maxy : m_ListPoints[i].at(k).y();

            x += m_ListPoints[i].at(k).x();     //总和
            y += m_ListPoints[i].at(k).y();
        }
#if 0
        //******************************test*******************
        centerx = x/m_ListPoints[i].size();
        centery = y/m_ListPoints[i].size();
        QPointF tmp;
        tmp.setX(centerx);
        tmp.setY(centery);
        mClusterPoint.push_back(tmp);
    }
#endif
    //***************************************************************

#if 1
     //   *********************202104*************************
        centerx = x/m_ListPoints[i].size();       //中心点
        centery = y/m_ListPoints[i].size();
        centerz = 0;

        Object object;
        object.ID =0;
        object.probability = 1.0;
        strcpy(object.label, "people");
        // float tempdist = sqrt(centerx * centerx + centery * centery);

      //  qDebug()<<"centerx="<<centerx / (msx *1000)<<"y="<<centery / (msx *1000);
        object.x = centerx / 6;      //object中心点
        object.y = -centery / 6;
        object.z = 0;
        //            //
        object.xmin = Minx / 6;    //object最大最小值
        object.xmax = Maxx / 6 ;
        object.ymin = -Miny / 6;
        object.ymax = -Maxy / 6;
        object.zmin = 0;
        object.zmax = 0;
        //
        object.height = 0;  //object长、宽、高
        object.width = (object.xmax - object.xmin);
        object.length = (object.ymax - object.ymin) ;
      //  qDebug()<<"width:"<<object.width;
      //  qDebug()<<"object:"<<object.width<<minWidth/100*6<<maxWidth/100*6*3;
        if( (minWidth/100 ) < object.width && object.width < (3*maxWidth / 100))
        All_Objects.push_back(object);   // std::vector<Object> All_Objects;
     //   qDebug()<<"object:"<<object.width<<object.length;
    }

 //qDebug()<<"All_object:"<<All_Objects.size();
    std::vector<Object> All_Objects2;

    if(All_Objects.size() > 0)
    {
         std::vector<Cluster> clusters;
        /******************************/
        std::vector<cv::Point3f> objectcenter;
        std::vector<float> objectwidth;
        std::vector<float> objectlength;
        std::vector<float> objectHeight;

        for (uint i = 0; i < All_Objects.size(); i++)
        {
            cv::Point3f cpt;
             Cluster cluster;
            cpt.x =All_Objects[i].x;
            cpt.y =All_Objects[i].y;
            cpt.z =All_Objects[i].z;
            objectcenter.push_back(cpt);  //中心

            objectwidth.push_back(All_Objects[i].width);  //长、宽、高
            objectlength.push_back(All_Objects[i].length);
            objectHeight.push_back(All_Objects[i].height);

            //   qDebug()<<"sixPoint:"<<cpt.x<<cpt.y<<cpt.z<<All_Objects[i].width<<All_Objects[i].length<<All_Objects[i].height;

            cluster.type ="people";
            cluster.posX  =All_Objects[i].x;
            cluster.posY =All_Objects[i].y;
            cluster.posZ =All_Objects[i].z;

            cluster.width  =All_Objects[i].width;
            cluster.length =All_Objects[i].length;
            cluster.height =All_Objects[i].height;
           // cluster.direct =1;
            cluster.usedFlag =false;
            clusters.push_back(cluster);
        }

        //  qDebug()<<"tracklist.size0"<<tracklist.size();


//        LidarTracking_2(objectcenter, objectwidth, objectlength,objectHeight);//轨迹追踪函数

//        // qDebug()<<"lidarlist.tracknum"<<lidarlist.back().tracknum;
//        //清空
//        objectcenter.clear();
//        objectHeight.clear();
//        objectlength.clear();
//        objectwidth.clear();

        float dt =0.1f;
        std::vector<Tracking> trackingList;

        LidarTracking(clusters,trackingList,dt);
 //qDebug()<<"cluste"<<clusters.size()<<"      (int)trackingList.size() "<<(int)trackingList.size();
        for (int i = 0; i < (int)trackingList.size(); i++)
        {

            Object obj;
            obj.ID = trackingList[i].ID;
           // obj.label = classifyTemp;
            obj.speed_x= trackingList[i].X[2];
            obj.speed_y =trackingList[i].X[3];
            obj.x =trackingList[i].X[0];
            obj.y =trackingList[i].X[1];
            obj.z = trackingList[i].posZ;;
            obj.width =trackingList[i].width;
            obj.length =trackingList[i].length;
            obj.height = trackingList[i].height;
            All_Objects2.push_back(obj);
        }


//        for(int i=0;i<All_Objects.size();i++)
//        {
//            All_Objects2.push_back(All_Objects.at(i));
//        }


    }


//        for (uint i = 0; i < lidarlist.back().tracknum; i++)
//        {
//            //  qDebug()<<"Innnnn";

//            trackobs temptrackobs;
//            temptrackobs.ID = lidarlist.back().track[i].ID;
//            temptrackobs.centerx = lidarlist.back().track[i].state(0, 0);
//            temptrackobs.speedx = lidarlist.back().track[i].state(0, 1);
//            temptrackobs.centery = lidarlist.back().track[i].state(0, 2);
//            temptrackobs.speedy = lidarlist.back().track[i].state(0, 3);
//            temptrackobs.width = lidarlist.back().track[i].width;
//            temptrackobs.length = lidarlist.back().track[i].length;
//            temptrackobs.height = lidarlist.back().track[i].height;

//            float speedtemp = sqrt(temptrackobs.speedx * temptrackobs.speedx + temptrackobs.speedy * temptrackobs.speedy);//求速度的平方根

//            //             对聚类目标大小进行的过滤
//            bool tempflag = false;
//            for (int j = 0; j < i; j++)
//            {
//                if (fabs(temptrackobs.centerx - lidarlist.back().track[j].state(0, 0)) < 0.15 && fabs(temptrackobs.centery - lidarlist.back().track[j].state(0, 2)) < 0.15 )
//                {
//                    tempflag = true;
//                }
//            }
//            if (tempflag)
//            {
//                continue;
//            }

//            float distance=(temptrackobs.length/2)*(temptrackobs.length/2)+(temptrackobs.width/2)*(temptrackobs.width/2);

//            int index=-i-1;

//            for (uint j = 0; j < All_Objects.size(); j++)
//            {
//                float distance_tmp = (All_Objects[j].x - lidarlist.back().track[i].state(0, 0))*(All_Objects[j].x - lidarlist.back().track[i].state(0, 0)) +
//                        (All_Objects[j].y - lidarlist.back().track[i].state(0, 2))*(All_Objects[j].y - lidarlist.back().track[i].state(0, 2));

//                if(distance > distance_tmp)//目的是分辨两个紧挨的聚类对象？
//                {
//                    distance=distance_tmp;
//                    index=j;
//                }vec_idShow
//            }

//            if(index>=0)
//            {
//                //    qDebug()<<"iiiinnn2";
//                Object object;
//                object.probability = All_Objects[index].probability;
//                strcpy(object.label, All_Objects[index].label);
//                object.ID = lidarlist.back().track[i].ID;
//                object.height = lidarlist.back().track[i].height;
//                object.width = lidarlist.back().track[i].width;
//                object.length = lidarlist.back().track[i].length;
//                object.y = lidarlist.back().track[i].state(0,2);
//                object.x = lidarlist.back().track[i].state(0,0);
//                object.z = object.height/2;
//                object.speed_y = lidarlist.back().track[i].state(0,3);
//                object.speed_x = lidarlist.back().track[i].state(0,1);
//                All_Objects2.push_back(object);
//            }
//        }

//        noobsframe = 0;
//    }
//    else
//    {
//        noobsframe++;
//        if(noobsframe >= 1)
//        {
//            trackinit();
//            tracklist.clear();
//            lidarlist.clear();
//            noobsframe = 0;
//        }
//    }

    sendID.clear();
    sendWidth.clear();
    sendLenth.clear();
    sendSpeed.clear();

    vector<int>vec_idShow;
    std::vector<Object> new_All_Objects2;
    getShowId(All_Objects2,vec_idShow,new_All_Objects2);
    for(int s = 0; s < (int)new_All_Objects2.size();s++)
    {
        QPointF tmp;
        float trackSpeed;
        tmp.setX(new_All_Objects2[s].x * 6);
        tmp.setY(-new_All_Objects2[s].y * 6) ;
        mClusterPoint.push_back(tmp);             //聚类x、y坐标，中心点
        trackSpeed = sqrt(new_All_Objects2[s].speed_x * new_All_Objects2[s].speed_x + new_All_Objects2[s].speed_y*new_All_Objects2[s].speed_y);
        sendSpeed.push_back(trackSpeed);   //m/s

        sendID.push_back(vec_idShow[s]);
       // sendID.push_back(All_Objects2[s].ID);
        sendWidth.push_back(new_All_Objects2[s].width * 6);
        sendLenth.push_back(-new_All_Objects2[s].length * 6);
    }

  //  qDebug()<<"size:"<<All_Objects.size()<<All_Objects2.size();

    for(int i = 0; i < new_All_Objects2.size() ;i++)
    {
        new_All_Objects2[i].xmin = All_Objects[i].xmin;
        new_All_Objects2[i].xmax = All_Objects[i].xmax;
        new_All_Objects2[i].ymin = All_Objects[i].ymin;
        new_All_Objects2[i].ymax = All_Objects[i].ymax;
        new_All_Objects2[i].zmin = All_Objects[i].zmin;
        new_All_Objects2[i].zmax = All_Objects[i].zmax;
    }
     //   *all_objects_out = All_Objects2;
#endif
    //***********************************202104**********************
    return mClusterPoint;
}

void HandleRun::keyPointcluster(QList<data_set> &dataSets, int dpID, int clusterID)  //关键点聚类
{
    data_set &srcdp = dataSets[dpID];  //每个点
    if(srcdp.isKey)     //true
    {
        QList<int>& arrvalPoint = srcdp.arrivalPointID;
        for(int n = 0; n < arrvalPoint.size(); n++)
        {
            data_set& desdp = dataSets[arrvalPoint[n]];
            if(!desdp.visited)
            {
                desdp.cluterID = clusterID;
                desdp.visited = true;
                if(desdp.isKey)
                {
                    keyPointcluster(dataSets, desdp.dpId,clusterID);
                }
            }
        }
    }
}

//****************************202104************************
void HandleRun::trackinit()         //初始化
{
      G = Eigen::MatrixXf::Zero(4, 2);
      G(0, 0) = T*T / 2;
      G(1, 0) = T;
      G(2, 1) = T*T / 2;
      G(3, 1) = T;
      A = Eigen::MatrixXf::Zero(4, 4);
      A(0, 0) = 1;
      A(0, 1) = T;
      A(1, 1) = 1;
      A(2, 2) = 1;
      A(2, 3) = T;
      A(3, 3) = 1;
      C = Eigen::MatrixXf::Zero(2, 4);
      C(0, 0) = 1;
      C(1, 2) = 1;
      V_Ob = Eigen::MatrixXf::Zero(2, 4);
      V_Ob(0, 1) = 1;
      V_Ob(1, 3) = 1;
      Q = Eigen::MatrixXf::Zero(2, 2);
      Q(0, 0) = 0.7 * qq;
      Q(1, 1) = 4 * qq;
      R = Eigen::MatrixXf::Zero(2, 2);
      R(0, 0) = 0.7 * rr*rr;
      R(1, 1) = 4 * rr*rr;

      P_predic = Eigen::MatrixXf::Zero(4, 4);
}

//目标追踪
void HandleRun::LidarTracking_2(std::vector<cv::Point3f> objectcenter, std::vector<float> objectwidth, std::vector<float> objectlength, std::vector<float> objectHeight)
{
    std::vector<trackld> trackframe;
    if (objectcenter.size() > 0)
    {
        for (int i = 0; i < objectcenter.size(); i++)
        {
            trackld trackldtemp;
            trackldtemp.centerx = objectcenter[i].x;
            trackldtemp.centery = objectcenter[i].y;           //中心点
            trackldtemp.width = objectwidth[i];              //长、宽、高
            trackldtemp.length = objectlength[i];
            trackldtemp.height = objectHeight[i];
            trackframe.push_back(trackldtemp);
        }

        tracklist0 tracklist1;
        tracklist1.tracknum = objectcenter.size();           //聚类数
        tracklist1.track = trackframe;        //聚类信息
        if (tracklist.size() >= 10)   //大于10帧就清除最开始一帧
        {
            tracklist.erase(tracklist.begin());
        }
        tracklist.push_back(tracklist1);  //帧数累加

        if (lidarlist.size() >= 10)
        {
            lidarlist.erase(lidarlist.begin());
            for (int k = 0; k < lidarlist.back().tracknum; k++)
            {
                idmax = idmax > lidarlist.back().track[k].ID ? idmax : lidarlist.back().track[k].ID;    //遍历lidarlist尾帧中的左右聚类对象找寻最大ID
            }
            if (idmax >= 99999999)  //重新遍历尾帧中左右聚类赋值
            {
                for (int j = 0; j < lidarlist.back().tracknum; j++)
                {
                    lidarlist.back().track[j].ID = j;
                }
                idmax = lidarlist.back().tracknum;
            }
        }

        int numset = pointset.size();
        Eigen::MatrixXf pointlabel = Eigen::MatrixXf::Zero(tracklist.back().tracknum, 1);
        Eigen::MatrixXf setlabel = Eigen::MatrixXf::Zero(numset, 1);
        if (tracklist.size() == 1)      //首帧
        {
            lidlist lidlisttemp;
            lidlisttemp.tracknum = 0;
            lidlisttemp.track.clear();
            lidarlist.push_back(lidlisttemp);
            for (int i = 0; i < tracklist.back().tracknum; i++)
            {
                trackld tracktemp;
                tracktemp.centerx = objectcenter[i].x;
                tracktemp.centery = objectcenter[i].y;
                tracktemp.width = objectwidth[i];
                tracktemp.length = objectlength[i];
                tracktemp.height = objectHeight[i];
                ptset ptsettemp;
                ptsettemp.track = tracktemp;
                ptsettemp.numframe = 1;
                ptsettemp.traframe = 1;
                pointset.push_back(ptsettemp);
            }
        }
        else                   //多帧
        {
            if (tracklist.back().tracknum == 0)     //尾帧没有聚类
            {
                if (!pointset.empty())
                {
                    for (int i = 0; i < numset; i++)
                    {
                        pointset[i].numframe = pointset[i].numframe + 1;
                    }
                }
                if (lidarlist.back().tracknum != 0)
                {
                    lidlist lidlisttemp;
                    lidlisttemp.tracknum = lidarlist.back().tracknum;
                    lidlisttemp.track.clear();
                    lidarlist.push_back(lidlisttemp);
                    std::vector<lidtrk> lidtrack;
                    for (int i = 0; i < lidarlist[lidarlist.size()-2].tracknum; i++)
                    {
                        lidtrk lidtrktemp;
                        lidtrktemp.state = A * lidarlist[lidarlist.size()-2].track[i].state.transpose(); //前一帧的state的转置
                        lidtrktemp.P = lidarlist[lidarlist.size()-2].track[i].P;
                        lidtrktemp.discount = lidarlist[lidarlist.size()-2].track[i].discount + 1;
                        lidtrktemp.ID = lidarlist[lidarlist.size()-2].track[i].ID; //Id等于前一帧的id
                        lidtrktemp.width = lidarlist[lidarlist.size()-2].track[i].width;
                        lidtrktemp.length = lidarlist[lidarlist.size()-2].track[i].length;
                        lidtrktemp.height = lidarlist[lidarlist.size()-2].track[i].height;
                        lidtrack.push_back(lidtrktemp);
                    }
                    lidarlist.back().track = lidtrack;

                    lidlisttemp.tracknum = lidarlist.back().track.size();
                }
            }
            else                       //尾帧有聚类
            {
                lidlist lidlisttemp;
                lidlisttemp.tracknum = lidarlist.back().tracknum;
                lidlisttemp.track.clear();
                lidarlist.push_back(lidlisttemp);

                std::vector<lidtrk> lidtrack;
             /****************************算法*****************************************/
              //新进来一帧的前一帧中的聚类数不为零
                if (lidarlist[lidarlist.size()-2].tracknum != 0)
                {
                    //定义变量，
                    //状态预测值 状态估计值 测量值 预测状态与真实状态的协防差矩阵 估计状态和真实状态的协防差矩阵
                    pointdistance = Eigen::MatrixXf::Zero(numset, tracklist.back().tracknum);
                    Eigen::MatrixXf D = Eigen::MatrixXf::Zero(lidarlist[lidarlist.size() - 2].tracknum, tracklist.back().tracknum);
                    Eigen::MatrixXf Pdt = Eigen::MatrixXf::Zero(lidarlist[lidarlist.size() - 2].tracknum, 1);
                    Eigen::MatrixXf W = Eigen::MatrixXf::Zero(lidarlist[lidarlist.size() - 2].tracknum, tracklist.back().tracknum);
                    Eigen::MatrixXf Wg = Eigen::MatrixXf::Zero(lidarlist[lidarlist.size() - 2].tracknum, tracklist.back().tracknum);
                    Eigen::MatrixXf Ps = Eigen::MatrixXf::Zero(lidarlist[lidarlist.size() - 2].tracknum, tracklist.back().tracknum);
                    Eigen::MatrixXf Ps0 = Eigen::MatrixXf::Zero(lidarlist[lidarlist.size() - 2].tracknum, 1);
                    Eigen::MatrixXf Pb = Eigen::MatrixXf::Zero(lidarlist[lidarlist.size() - 2].tracknum, tracklist.back().tracknum);
                    Eigen::MatrixXf Pb0 = Eigen::MatrixXf::Zero(lidarlist[lidarlist.size() - 2].tracknum, 1);
                    Eigen::MatrixXf Pr = Eigen::MatrixXf::Zero(lidarlist[lidarlist.size() - 2].tracknum, tracklist.back().tracknum);
                    Eigen::MatrixXf Pr0 = Eigen::MatrixXf::Zero(lidarlist[lidarlist.size() - 2].tracknum, tracklist.back().tracknum);
                    Eigen::MatrixXf Ag = Eigen::MatrixXf::Zero(lidarlist[lidarlist.size() - 2].tracknum, tracklist.back().tracknum);
                    float Pd = 0.99;        //假设雷达探测概率为0.99
                    float a = 1;
                    float N = 1;

                    //预测值
                    //估计值
                    //估计状态和真实状态的协防差矩阵

                    for (int j = 0; j < lidarlist[lidarlist.size() - 2].tracknum; j++)
                    {
                        Pdt(j,0) = lambda * (1.0 - Pd);       //lambda = 0.1
                        /*
                                        A|1 0.1 0  0  |
                                         |0  1  0  1  |
                                         |0  0  1 0.1 |
                                         |0  0  0  1  |
                        */

                       //P_predic 为4*4矩阵 A为4*4矩阵 T = 0.1 G 4*2矩阵 Q4*2矩阵
                        P_predic = A * lidarlist[lidarlist.size() - 2].track[j].P * A.transpose() + G * Q * G.transpose();

                        S = C * P_predic * C.transpose() + R;

                        K = P_predic * C.transpose() * S.inverse();//转置乘以逆矩阵

                        for (int i = 0; i < tracklist.back().tracknum; i++) //遍历新进的一帧
                        {
                            //预测值
                            //预测状态与真实状态的协防差矩阵
                            //测量值
                            //估计值
                            //估计状态与真实状态的协防差矩阵//****************************202104************************
                            Eigen::MatrixXf dtemp;
                            dtemp = C * (A * lidarlist[lidarlist.size() - 2].track[j].state.transpose());
                            Eigen::MatrixXf d = Eigen::MatrixXf::Zero(1, 2);
                            d(0,0) = fabs(tracklist.back().track[i].centerx - dtemp(0, 0));
                            d(0,1) = fabs(tracklist.back().track[i].centery - dtemp(1, 0));

                            Eigen::MatrixXf state;
                            state  = lidarlist[lidarlist.size() - 2].track[j].state;//前一帧的状态

                            Eigen::MatrixXf v = Eigen::MatrixXf::Zero(1, 2);
                            v(0, 0) = state(0, 1);
                            v(0, 1) = state(0, 3);

                            float nv = 0;
                            if (v.norm() != 0) //范数
                            {
                                nv = v.norm();
                                v(0, 0) = v(0 ,0) / nv;
                                v(0, 1) = v(0 ,1) / nv;
                            }

                            Eigen::MatrixXf d0 = d * S.inverse() * d.transpose();//逆矩阵乘以转置矩阵转置矩阵乘以逆矩阵

                            D(j, i) = d0.determinant();     //距离相关性
                            //N = (1 / (sqrt(2 * CV_PI * S.determinant()))) * exp(-0.5 * D(j ,i)* D(j ,i) / S.determinant());
                            N = (1 / (2 * CV_PI * sqrt(S.determinant()))) * exp(-0.5 * D(j ,i)* D(j ,i) / S.determinant());//平方根  指数  特征值
                            if (D(j ,i) > 0 && D(j ,i) < g_sigma)
                            {
                                W(j, i) = 1;
                            }

                            Pr0(j ,i) = W(j ,i) * N;       //量测j落入目标t的跟踪门内的概率密度

                            Eigen::MatrixXf V_reall = d / T;
                            Eigen::MatrixXf tmpv = V_Ob * lidarlist[lidarlist.size() - 2].track[j].state.transpose();
                            Eigen::MatrixXf dv = V_reall - tmpv.transpose();
                            Ag(j ,i) = (dv * S.inverse() * dv.transpose()).determinant();

                            float Ng =  (1 / (sqrt(2 * CV_PI * S.determinant()))) * exp(-0.5 * Ag(j ,i)*Ag(j ,i)/S.determinant());

                            if (Ag(j ,i) > 0 && Ag(j ,i) < 4)
                            {
                                Wg(j, i) = 1;
                            }
                        }
                    }

                    //对各目标跟踪门内的公共点迹的概率密度值进行衰减
                    for (int j = 0; j < lidarlist[lidarlist.size() - 2].tracknum; j++)
                    {
                        for (int i = 0; i < tracklist.back().tracknum; i++)
                        {
                            Pr(j, i) = Pr0(j ,i) * Pr0(j, i) / (Pr0.col(i).sum() + exp(-25.0));//求和乘以对-25取指数，exp指数函数
                        }
                    }

                    //计算互联概率
                    for (int j = 0; j < lidarlist[lidarlist.size() - 2].tracknum; j++)
                    {
                        for (int i = 0; i < tracklist.back().tracknum; i++)
                        {
                            Pb(j, i) = Pr(j ,i) / (Pr.row(j).sum() + exp(-25.0) + Pdt(j, 0));
                        }
                        Pb0(j, 0) = 1 - Pb.row(j).sum();
                    }

                    //对落入各跟踪波门内的公共点迹的互联概率进行衰减
                    Eigen::MatrixXf Pb_new = Eigen::MatrixXf::Zero(lidarlist[lidarlist.size() - 2].tracknum, tracklist.back().tracknum);
                    for (int j = 0; j < lidarlist[lidarlist.size() - 2].tracknum; j++)
                    {
                        for (int i = 0; i < tracklist.back().tracknum; i++)
                        {
                            Pb_new(j, i) = Pb(j ,i) * Pb(j ,i) / (Pb.col(i).sum() + exp(-25.0));
                        }
                    }

                    //计算新的互联概率
                    for (int j = 0; j < lidarlist[lidarlist.size() - 2].tracknum; j++)
                    {
                        for (int i = 0; i < tracklist.back().tracknum; i++)
                        {
                            Ps(j, i) = Pb_new(j ,i) / (Pb_new.row(j).sum() + exp(-25.0) + Pb0(j, 0));
                        }

                        Ps0(j, 0) = 1 - Ps.row(j).sum();
                    }

                    //根据概率更新数据
                    for (int j = 0; j < lidarlist[lidarlist.size() - 2].tracknum; j++)
                    {
                        P_predic = A * lidarlist[lidarlist.size() - 2].track[j].P * A.transpose() + G * Q * G.transpose();
                        S = C * P_predic * C.transpose() + R;
                        K = P_predic * C.transpose() * S.inverse();
                        //cout << "K" << endl << K <<endl;

                        Eigen::MatrixXf Gain = Eigen::MatrixXf::Zero(4, 4);
                        lidtrk lidtrktmp;
                        if (Ps0(j, 0) > 0.8)
                        {
                            Gain(0, 0) = 1;
                            Gain(0, 1) = T;
                            Gain(1, 1) = 1;
                            Gain(2, 2) = 1;
                            Gain(2, 3) = T;
                            Gain(3, 3) = 1;
                            lidtrktmp.state = (Gain * lidarlist[lidarlist.size() - 2].track[j].state.transpose()).transpose();
                            lidtrktmp.P = (Eigen::MatrixXf::Identity(4, 4) - K * C) * P_predic;
                            lidtrktmp.discount = lidarlist[lidarlist.size() - 2].track[j].discount + 1;
                            lidtrktmp.ID = lidarlist[lidarlist.size() - 2].track[j].ID;
                            lidtrktmp.width = lidarlist[lidarlist.size() - 2].track[j].width;
                            lidtrktmp.length = lidarlist[lidarlist.size() - 2].track[j].length;
                            lidtrktmp.height = lidarlist[lidarlist.size() - 2].track[j].height;
                        }
                        else
                        {
                            Eigen::MatrixXf tmp2 = Eigen::MatrixXf::Zero(4, 1);
                            Eigen::MatrixXf x_predic =	A * lidarlist[lidarlist.size() - 2].track[j].state.transpose();
                            Gain(0, 0) = 1;
                            Gain(1, 1) = 2;
                            Gain(2, 2) = 1;
                            Gain(3, 3) = 2;
                            Gain = 3 * Gain;

                            for (int i = 0; i < tracklist.back().tracknum; i++)
                            {
                                Eigen::MatrixXf tr_state(2, 1);
                                tr_state(0, 0) = tracklist.back().track[i].centerx;
                                tr_state(1, 0) = tracklist.back().track[i].centery;
                                tmp2 = tmp2 + Ps(j, i) * (x_predic + Gain * K * (tr_state - C * x_predic));
                            }

                            int tmmp = -1;
                            int ii, jj;
                            Pr0.row(j).maxCoeff(&ii,&jj);//求矩阵中最大的元素所在的行数与列数
                            tmmp = jj;
                            if (tmmp == -1)
                            {
                                tmmp = 0;
                            }

                            tmp2 = Ps0(j, 0) * x_predic + tmp2;

                            lidtrktmp.state = tmp2.transpose();
                            lidtrktmp.P = (Eigen::MatrixXf::Identity(4, 4) - K * C) * P_predic;
                            lidtrktmp.discount = 0;
                            lidtrktmp.ID = lidarlist[lidarlist.size() - 2].track[j].ID;
                            lidtrktmp.height = tracklist.back().track[tmmp].height;

                            if (tmmp != -1)
                            {
                                float temp1 = lidarlist[lidarlist.size() - 2].track[j].width * lidarlist[lidarlist.size() - 2].track[j].length;
                                float temp2 = tracklist.back().track[tmmp].width * tracklist.back().track[tmmp].length;

                                if (fabs(temp1 / temp2 - 1) < 0.3)
                                {
                                    lidtrktmp.width = tracklist.back().track[tmmp].width;
                                    lidtrktmp.length = tracklist.back().track[tmmp].length;
                                }
                                else
                                {
                                    lidtrktmp.width = 0.7 * lidarlist[lidarlist.size() - 2].track[j].width + 0.3 * tracklist.back().track[tmmp].width;
                                    lidtrktmp.length = 0.7 * lidarlist[lidarlist.size() - 2].track[j].length + 0.3 * tracklist.back().track[tmmp].length;
                                }
                            }
                            else
                            {
                                lidtrktmp.width = lidarlist[lidarlist.size() - 2].track[j].width;
                                lidtrktmp.length = lidarlist[lidarlist.size() - 2].track[j].length;
                            }
                        }
                        lidtrack.push_back(lidtrktmp);
                        lidarlist.back().tracknum = lidarlist[lidarlist.size() - 2].tracknum;
                        lidarlist.back().track = lidtrack;
                    }

                    for (int i = 0; i < tracklist.back().tracknum; i++)
                    {
                        if (W.col(i).sum() > 0)
                        {
                            pointlabel(i, 0) = 1;
                        }
                    }
                } //新进来的前一帧中的聚类数不为零

                if (pointset.empty())
                {
                    for (int i = 0; i < tracklist.back().tracknum; i++)
                    {
                        if (pointlabel(i, 0) == 0)
                        {
                            ptset ptsettemp;
                            ptsettemp.track = tracklist.back().track[i];
                            ptsettemp.numframe = 1;
                            ptsettemp.traframe = 1;
                            pointset.push_back(ptsettemp);
                            numset = numset + 1;
                        }
                    }
                }
                else
                {
                    pointdistance = Eigen::MatrixXf::Zero(numset, tracklist.back().tracknum);
                    for(int i = 0; i < tracklist.back().tracknum; i++)//遍历新进一帧
                    {
                        for (int j = 0; j < numset; j ++)
                        {
                            Eigen::MatrixXf dxy(1, 2);
                            float maxWidth = tracklist.back().track[i].width > 1.0 ? tracklist.back().track[i].width : 1.0;
                            float maxLength = tracklist.back().track[i].length > 1.0 ? tracklist.back().track[i].length : 1.0;
                            dxy(0, 0) = fabs((tracklist.back().track[i].centerx - pointset[j].track.centerx) / maxWidth);
                            dxy(0, 1) = fabs((tracklist.back().track[i].centery - pointset[j].track.centery) / maxLength);
                            pointdistance(j ,i) = sqrt(dxy(0, 0) * dxy(0, 0) + dxy(0, 1) * dxy(0, 1));//求平方根
                        }
                    }
                    for (int j = 0; j < numset; j ++)
                    {
                        int tv, tc;
                        pointdistance.row(j).minCoeff(&tv, &tc);//最小值所在行数列数最小行与列
                        if (tv < tvthd && pointlabel(tc, 0) == 0) //tvthd = 3 累积帧数小于3
                        {
                            pointset[j].track = tracklist.back().track[tc];
                            pointset[j].numframe = pointset[j].numframe + 1; //所在帧数
                            pointset[j].traframe = pointset[j].traframe + 1;  //一共有的帧数

                            pointlabel(tc, 0) = 1;
                            setlabel(j, 0) = 1;
                        }
                        else
                        {
                            pointset[j].numframe = pointset[j].numframe + 1; //所在帧数
                        }
                    }
                }

                Eigen::MatrixXf pointsetdet = Eigen::MatrixXf::Zero(1, 1);

                for (int i = 0; i < numset; i++)
                {
                    if (pointset[i].traframe > 2)
                    {
                        lidtrk lidtrktemp;
                        lidtrktemp.state = Eigen::MatrixXf::Zero(1,4);
                        lidtrktemp.state(0, 0) = pointset[i].track.centerx;
                        lidtrktemp.state(0, 1) = 0;
                        lidtrktemp.state(0, 2) = pointset[i].track.centery;
                        lidtrktemp.state(0, 3) = 0;
                        lidtrktemp.P = Eigen::MatrixXf::Zero(4,4);
                        lidtrktemp.P(0, 0) = 0.1422;
                        lidtrktemp.P(0, 1) = 0.1029;
                        lidtrktemp.P(1, 0) = 0.1029;
                        lidtrktemp.P(1, 1) = 0.1065;
                        lidtrktemp.P(2, 2) = 0.8128;
                        lidtrktemp.P(2, 3) = 0.5880;
                        lidtrktemp.P(3, 2) = 0.5880;
                        lidtrktemp.P(3, 3) = 0.9171;
                        lidtrktemp.discount = 0;

                       // idExist.clear();
                        for (int id1 = 0; id1 < lidarlist.back().tracknum; id1++)
                        {
                            idmax = idmax > lidarlist.back().track[id1].ID ? idmax : lidarlist.back().track[id1].ID;
                           // idExist.push_back(lidarlist.back().track[id1].ID);
                        //    idmin = idmin < lidarlist.back().track[id1].ID ? idmin : lidarlist.back().track[id1].ID;
                        }
#if 0
                        if(idmax>=63){
                            idFlag=true;
                        }
                        if(idFlag==false)
                        lidtrktemp.ID = idmax + 1;
                        else{
                            while (true) {
                            lidtrktemp.ID = idRand(0,63);
                            if(!idExist.contains(lidtrktemp.ID))
                                break;
                            }
                        }
#endif
                        lidtrktemp.ID = idmax + 1;
                        lidtrktemp.width = pointset[i].track.width;
                        lidtrktemp.length = pointset[i].track.length;
                        lidtrktemp.height = pointset[i].track.height;
                        lidarlist.back().track.push_back(lidtrktemp);
                        lidarlist.back().tracknum = lidarlist.back().track.size();
                    }
                }
            }

            std::vector<int> delindex;
            for(int j = 0; j < lidarlist.back().tracknum; j++)
            {
                if (lidarlist.back().track[j].discount > disthd)
                {
                    delindex.push_back(j);
                }
            }
            if (!delindex.empty())
            {
                for(int di = delindex.size() - 1; di >= 0; di--)
                {
                    lidarlist.back().track.erase(lidarlist.back().track.begin() + delindex[di]);
                }
                lidarlist.back().tracknum = lidarlist.back().tracknum - delindex.size();
            }

            std::vector<int> pointsetdel;
            for (int i = 0; i < numset; i++)
            {
                if (pointset[i].numframe > 3 || pointset[i].traframe > 2)
                {
                    pointsetdel.push_back(i);
                }
            }
            if (!pointsetdel.empty())
            {
                for (int pd = pointsetdel.size() - 1; pd >= 0; pd--)
                {
                    pointset.erase(pointset.begin()+pointsetdel[pd]);
                }
            }

        }
        noobsframe = 0;
    }
    else
    {
        noobsframe++;
        if(noobsframe >= 5)
        {
            trackinit();
            tracklist.clear();
            lidarlist.clear();
            noobsframe = 0;
        }
    }

}

//****************************202104************************
QList<QPointF> HandleRun::UpdateScene(QPointF centerpoint, QList<QPointF> in_data , double Ra)    //半径滤波
{
  //  qDebug()<<"in_data.size()"<<in_data.size();
    QList<QPointF> updatePoints;
    QList<QPointF> Points;
    updatePoints.clear();
    for(int j = 0; j<in_data.size();j++)
    {
        double d = sqrt(((centerpoint.y()-in_data[j].y()) * (centerpoint.y()-in_data[j].y())) + ((centerpoint.x() - in_data[j].x()) * (centerpoint.x() - in_data[j].x())));
        if(d > 2.0 * Ra)
        {
            updatePoints.push_back(in_data[j]);
        }
    }
#if 0
    Points = updatePoints;

    if((in_data.size()-Points.size()>20) && (in_data.size()-Points.size()<30))
    {
        updatePoints.clear();
        for(int j=0;j<Points.size();j++)
        {
            double d = sqrt(((centerpoint.y()-Points[j].y()) * (centerpoint.y()-Points[j].y())) + ((centerpoint.x() - Points[j].x()) * (centerpoint.x() - Points[j].x())));
            if(d > 4.5 * Ra)
            {
                updatePoints.push_back(Points[j]);
            }
        }
    }

    else if((in_data.size()-Points.size()>30))
    {
        updatePoints.clear();
        for(int j=0;j<Points.size();j++)
        {
            double d = sqrt(((centerpoint.y()-Points[j].y()) * (centerpoint.y()-Points[j].y())) + ((centerpoint.x() - Points[j].x()) * (centerpoint.x() - Points[j].x())));
            if(d > 5.5 * Ra)
            {
                updatePoints.push_back(Points[j]);
            }
        }
    }
#endif
  //  qDebug()<<"Size()"<<in_data.size()<<updatePoints.size()<<Ra;
    return updatePoints;
}

void HandleRun::SaveInitCenterpoints(QList<QPointF> centerPoints)
{
    InitCenterPoints = centerPoints;
  //  qDebug()<<"Save && Centerpoints.size()"<<InitCenterPoints.size();
}
//****************************202105***********************

int HandleRun::idRand(int min, int max)
{
    return ( rand() % (max - min + 1) ) + min ;
}

float HandleRun::calAngle(float x, float y)
{
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
    return angle;
}

static int sz=0;
void HandleRun::getShowId(std::vector<Object> All_Objects,vector<int>&keepIdShow,std::vector<Object> &keepRealShow)
{
    m_deleteObj.clear();
    /***************id****************/
    for(int n=0;n<m_idSize;n++)
    {
        if(m_idFrame[n] != m_frameSz && m_isOut[n]){
            m_idFrame[n] = m_idFrame[n]+1;
        }
    }
    vector<int>keepTem;
    vector<int>keepReal;
    for(int b=0;b<m_idSize;b++){
        m_isOut[b] = true;
    }

    for(int s = 0; s < (int)All_Objects.size();s++){
        int count;
        int countff;
        int *idShow = nullptr;
        bool isGet = false;
        for(int i =0;i<m_real_Objects.size();i++){
            if(find(m_real_Objects[i].begin(),m_real_Objects[i].end(),All_Objects[s].ID)!=m_real_Objects[i].end()){
                isGet = true;
                break;
            }
        }

      //  if(find(comArr.begin(),comArr.end(),All_Objects2[s].ID) != comArr.end()){
         //   isGet = false;
          //  qDebug()<<"comArr "<<comArr;
      //  }
        //comArr.push_back(All_Objects[s].ID);
        if(!isGet) //no
        {
            bool isSetVal = false;
            /***************id****************/
            for(int j=0;j<m_idSize;j++){
                if(m_idFrame[j] == m_frameSz && j>m_currMax){
                    bool flag = false;
                    for(int i =0;i<m_tem_Objects.size();i++){
                        if(find(m_tem_Objects[i].begin(),m_tem_Objects[i].end(),j)!=m_tem_Objects[i].end()){
                            flag = true;
                            break;
                        }
                    }
                    if(flag) continue;
                    idShow = &j;
                    m_idFrame[j]=0;
                    isSetVal = true;
                    break;
                }
            }
            if(!isSetVal){  //reset
                for(int j=0;j<m_idSize;j++){
                    if(m_idFrame[j] == m_frameSz){
                        bool flag = false;
                        for(int i =0;i<m_tem_Objects.size();i++){
                            if(find(m_tem_Objects[i].begin(),m_tem_Objects[i].end(),j)!=m_tem_Objects[i].end()){
                                flag = true;
                                break;
                            }
                        }
                        if(flag) continue;
                        idShow = &j;
                        m_idFrame[j]=0;
                        break;
                    }
                }
            }
            if(idShow != nullptr){
                m_currMax = *idShow;
            }

        }else{
            //qDebug()<<"runningHere";
            for(int f=0;f<m_real_Objects.size();f++)
            {
                bool state = false;
                for(int ff=0;ff<m_real_Objects[f].size();ff++){
                    if(m_real_Objects[f][ff] == All_Objects[s].ID){
                        count=f;
                        countff = ff;
                        state = true;
                        break;
                    }
                }
                if(state){
                    break;
                }
            }

            idShow = &m_tem_Objects[count][countff];
        }

        //int isPushed = false;
        if(idShow != nullptr){
            if(find(keepTem.begin(),keepTem.end(),*idShow) != keepTem.end()){ //如果已经存在，继续分配其他值
                qDebug()<<"isExistSame";
    //            for(int j=0;j<m_idSize;j++){
    //                if(m_idFrame[j] == m_frameSz){
    //                    idShow = j;
    //                    m_idFrame[j]=0;
    //                    break;
    //                }
    //            }
                m_deleteObj.push_back(true);
                //isPushed = true;
                continue;
            }
            m_deleteObj.push_back(false);
            m_isOut[*idShow] = false;
            keepTem.push_back(*idShow);
            keepIdShow.push_back(*idShow);
            keepReal.push_back(All_Objects[s].ID);
            keepRealShow.push_back(All_Objects[s]);
            //qDebug()<<"keepTem"<<keepTem;
        }
    }
    sz++;
    if(sz >m_keepFrameSz && m_real_Objects.size()>0 && m_tem_Objects.size()>0){
        m_real_Objects.erase(m_real_Objects.begin(),m_real_Objects.begin()+1);
        m_tem_Objects.erase(m_tem_Objects.begin(),m_tem_Objects.begin()+1);
        //qDebug()<<"sz:"<<sz<<m_real_Objects.size()<<m_tem_Objects.size();
        sz=m_keepFrameSz;
    }

    m_real_Objects.push_back(keepReal);
    m_tem_Objects.push_back(keepTem);
}

//雷达聚类算法
void HandleRun::LidarClusterringRun(std::vector<pcl::PointXYZ> LidarPointFilter)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0 (new pcl::PointCloud<pcl::PointXYZ>);
    if (!LidarPointFilter.empty())
    {
        //填充点云数据
        cloud0->width    = LidarPointFilter.size();				//设置点云宽度
        cloud0->height   = 1;									//设置点云高度
        cloud0->is_dense = false;								//非密集型
        cloud0->points.resize (cloud0->width * cloud0->height);	//变形，无序
        for (int i = 0; i < LidarPointFilter.size(); i++)
        {
            cloud0->points[i] = LidarPointFilter[i];		//PCL显示以m为单位
        }

        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//        vg.setInputCloud (cloud0);
//        vg.setLeafSize(0.3f, 0.3f, 0.3f);
//        vg.filter(*cloud_filtered);

        *cloud_filtered +=*cloud0;

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_filtered);
        tree->setInputCloud(cloud_filtered);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(RunLi.distance/10); //2cm
        ec.setMinClusterSize(RunLi.scanNum);
        ec.setMaxClusterSize(500);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_filtered);
        tree->setInputCloud(cloud_filtered);
        ec.extract(cluster_indices);
    //    qDebug()<<"cluster_indices.size()"<<cluster_indices.size();

        int areaCount = cluster_indices.size();
        std::vector< std::vector<cv::Point3f> > areaPoint(areaCount);
        int temp_j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
            {
                cv::Point3f temp_areapoint;
                temp_areapoint.x = cloud_filtered->points[*pit].x;
                temp_areapoint.y = cloud_filtered->points[*pit].y;
                temp_areapoint.z = cloud_filtered->points[*pit].z ;
                areaPoint[temp_j].push_back(temp_areapoint);
            }
            temp_j++;
        }

        std::vector<point4f> areapoints_centroid;
        for (int i = 0; i < areaCount; i++)
        {
            float Minx1 = areaPoint[i].front().x;
            float Maxx1 = areaPoint[i].front().x;
            float Miny1 = areaPoint[i].front().y;
            float Maxy1 = areaPoint[i].front().y;
            float Minz1 = areaPoint[i].front().z;
            float Maxz1 = areaPoint[i].front().z;

            for (int j = 0; j < areaPoint[i].size(); j++)
            {
                Minx1 = Minx1 < areaPoint[i][j].x ? Minx1 : areaPoint[i][j].x;
                Maxx1 = Maxx1 > areaPoint[i][j].x ? Maxx1 : areaPoint[i][j].x;
                Miny1 = Miny1 < areaPoint[i][j].y ? Miny1 : areaPoint[i][j].y;
                Maxy1 = Maxy1 > areaPoint[i][j].y ? Maxy1 : areaPoint[i][j].y;
                Minz1 = Minz1 < areaPoint[i][j].z ? Minz1 : areaPoint[i][j].z;
                Maxz1 = Maxz1 > areaPoint[i][j].z ? Maxz1 : areaPoint[i][j].z;
            }
            point4f temp_centoid;
            temp_centoid.x = (Maxx1 + Minx1) / 2;
            temp_centoid.y = (Maxy1 + Miny1) / 2;
            temp_centoid.z = (Maxz1 + Minz1) / 2;
            temp_centoid.state = -1;
            areapoints_centroid.push_back(temp_centoid);
        }

        int objectCnt = 0;
        float toternceX = 0.8;
        float toternceY = 1;
        float toternceZ = 2;

        for(int i = 0; i < areapoints_centroid.size(); i++)
        {
            if (areapoints_centroid[i].state == -1)
            {
                areapoints_centroid[i].state = objectCnt;
                for (int j = 0; j < areapoints_centroid.size(); j++)
                {
                    if (areapoints_centroid[j].state == -1 && abs(areapoints_centroid[i].y - areapoints_centroid[j].y) <= toternceY && abs(areapoints_centroid[i].x - areapoints_centroid[j].x) <= toternceX && abs(areapoints_centroid[i].z - areapoints_centroid[j].z) <= toternceZ)
                    {
                        areapoints_centroid[j].state = objectCnt;
                    }
                }
                objectCnt++;
            }
        }

        std::vector< std::vector<cv::Point3f> > objectpoints(objectCnt);
        for (int i = 0; i < areaPoint.size(); i++)
        {
            int tempState = areapoints_centroid[i].state;
            objectpoints[tempState].insert(objectpoints[tempState].end(), areaPoint[i].begin(), areaPoint[i].end());
        }

        for (int i = 0; i < objectpoints.size(); i++)
        {
            float Minx2 = objectpoints[i].front().x;
            float Maxx2 = objectpoints[i].front().x;
            float Miny2 = objectpoints[i].front().y;
            float Maxy2 = objectpoints[i].front().y;
            float Minz2 = objectpoints[i].front().z;
            float Maxz2 = objectpoints[i].front().z;
            for (int j = 0; j < objectpoints[i].size(); j++)
            {
                Minx2 = Minx2 < objectpoints[i][j].x ? Minx2 : objectpoints[i][j].x;
                Maxx2 = Maxx2 > objectpoints[i][j].x ? Maxx2 : objectpoints[i][j].x;
                Miny2 = Miny2 < objectpoints[i][j].y ? Miny2 : objectpoints[i][j].y;
                Maxy2 = Maxy2 > objectpoints[i][j].y ? Maxy2 : objectpoints[i][j].y;
                Minz2 = Minz2 < objectpoints[i][j].z ? Minz2 : objectpoints[i][j].z;
                Maxz2 = Maxz2 > objectpoints[i][j].z ? Maxz2 : objectpoints[i][j].z;
            }
            if ((Maxx2 - Minx2) > (minWidth/100) && (Maxx2 - Minx2) < (3 * maxWidth/100))
            {
               // std::cout << "Minz2:  " << Minz2 << std::endl;
                ObjectCenter.push_back(cv::Point3f((Minx2 + Maxx2) / 2.0, (Miny2 + Maxy2) / 2.0, (Minz2 + Maxz2) / 2.0));
                ObjectWidth.push_back(Maxx2 - Minx2);
                ObjectLength.push_back(Maxy2 - Miny2);
                ObjectHeight.push_back(Maxz2 - Minz2);
            }
        }
    }
}

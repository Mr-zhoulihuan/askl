#ifndef HANDLERUN_H
#define HANDLERUN_H

#include <QObject>
#include <QRunnable>
#include "LidarData.h"
#include <iostream>
#include <QtMath>
//************************20210421***********************
#include <Eigen/Dense>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <QThread>
#include "filesystem.h"
#include <LidarTracking.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#define CV_PI 3.1415926535897932384626433832795
extern std::vector<int> sendID;
extern std::vector<float> sendWidth;
extern std::vector<float> sendLenth;
extern std::vector<float> sendSpeed;
struct lidtrk
{
    Eigen::MatrixXf  state;
    Eigen::MatrixXf P;
    int discount;
    int ID;
    float width;
    float length;
    float height;
};

struct lidlist
{
    int tracknum;
    std::vector<lidtrk> track;
};

struct trackld
{
    float centerx;
    float centery;
    float width;
    float length;
    float height;
};

struct tracklist0
{
    int tracknum;
    std::vector<trackld> track;
};

struct ptset
{
    struct trackld track;
    int numframe;
    int traframe;
};

struct trackobs
{
    float ID;
    float centerx;
    float speedx;
    float centery;
    float speedy;
    float width;
    float length;
    float height;
};

typedef struct
{
    int label;
    double left_x, left_y, right_x, right_y;
    double probability;
}Original;

struct point4f {
    float x;
    float y;
    float z;
    int state;
};
//***************************202104*****************************


using namespace std;
//class HandleRun : public QObject, public QRunnable
class HandleRun : public QThread
{
    Q_OBJECT
public:
    explicit HandleRun(QObject *parent = 0);
    ~HandleRun();

    void run();
//*******************************202104**************************
    void trackinit();
    void LidarTracking_2(std::vector<cv::Point3f> objectcenter, std::vector<float> objectwidth, std::vector<float> objectlength, std::vector<float> objectHeight);


    void LidarClusterringRun(std::vector<pcl::PointXYZ> LidarPointFilter);

    std::vector<lidlist> lidarlist;//
    std::vector<tracklist0> tracklist;//
   // std::vector<int> test;

    double range = 0.0;			// We only filter point with local coordinates x, y in [-range, range] inmeters
    double cell_size = 0.0;		// Hight and width of grid in bitmap
    double extend_dist = 0.0;	// The distance extended away from the ROI boundary
    //   cv::Mat binary;

    int idmax = 0;
    int idmin = 0;
    int noobsframe = 0;

    float Lidar_Height = 0.0;
    int iterateNum = 40;
    int sampleNum = 4;
    float land_height_scale = 150.0;//地面高度差，单位mm

    const float disthd = 0.5;
    const float T = 0.1;
    Eigen::MatrixXf G;
    Eigen::MatrixXf A;
    Eigen::MatrixXf C;
    Eigen::MatrixXf V_Ob;
    float g_sigma = 4;
    float lambda = 0.1;
    float qq = 0.9;
    float rr = 0.9;
    Eigen::MatrixXf Q;
    Eigen::MatrixXf R;
    float tvthd = 3;
    Eigen::MatrixXf P_predic;
    Eigen::MatrixXf S;
    Eigen::MatrixXf K;
    Eigen::MatrixXf pointdistance;
    std::vector<ptset> pointset;

    uchar lidar_index = 0;

    double temp_detect[1][9];

    bool updateScene;   //202105

    bool idFlag = false;
    QList<int> idExist;
    int idRand(int,int);

    float minWidth = 0;
    float maxWidth = 0;

    float calAngle(float x,float y);
    QMap<int,float> LastAngle;
    QMap<int,float> LastAngle_1;
    float msx;

    std::vector<int> m_idFrame;  //ID 管理
    std::vector<vector<int> > m_tem_Objects;
    std::vector<vector<int> > m_real_Objects;
    std::vector<bool> m_isOut;
    const int m_frameSz = 100;
    const int m_keepFrameSz = 100;
    const int m_idSize = 64;
    int m_currMax = -1;
    vector<bool> m_deleteObj;


    std::vector<cv::Point3f> ObjectCenter;	//障碍物中心点坐标
    std::vector<float> ObjectWidth;			//障碍物宽度
    std::vector<float> ObjectLength;			//障碍物长度
    std::vector<float> ObjectHeight;			//障碍物高度


//**********************************202104*******************************

private:
    QList<QPointF> InitDBtree(QList<QPointF> m, float distance, int scanNum);
    double GetDistance(data_set &dp1, data_set &dp2);
    void setArrivalPoint(QList<data_set> &dataSets, data_set &dp, float distance, int scanNum);
    QList<QPointF> DoDBSCANRecursive(QList<data_set> &dateSet);
    void keyPointcluster(QList<data_set> &dataSets, int dpID, int clusterID);
    QList<QPointF> UpdateScene(QPointF centerpoint, QList<QPointF> in_data , double Ra); //202105
    QList<QPointF> InitCenterPoints;
    void getShowId(std::vector<Object> All_Objects,vector<int>&keepIdShow,std::vector<Object> &keepRealShow);
signals:
    void lidarThCBack(lidar,point_data);
    void sendMessageToTcp();

public slots:
    void getHandleData(lidar lidarinfo, LidarC1Data data);
    void SaveInitCenterpoints(QList<QPointF> centerPoint);//202105
private:
    lidar           RunLi;
    LidarC1Data     RunData;

};

#endif // HANDLERUN_H

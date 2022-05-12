#include "addlidar.h"
#include "ui_addlidar.h"
#include "QMetaType"
#include <QIntValidator>
#include <QMessageBox>
#include <QRegExpValidator>
#include <QRegExp>
AddLidar::AddLidar(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::AddLidar)
{
    ui->setupUi(this);
    setWindowTitle(QStringLiteral("添加雷达属性"));
    setWindowIcon(QIcon(":/image/leishen.ico"));
    setModal(true);
    qRegisterMetaType<struct lidar>("struct lidar");
    ui->ScanRadius->setValidator(new QIntValidator(0,300));
    ui->Index->setValidator(new QIntValidator(1,10000));
    QRegExp rx("^((2[0-4]\\d|25[0-5]|[01]?\\d\\d?)\\.){3}(2[0-4]\\d|25[0-5]|[01]?\\d\\d?)$");
    QRegExpValidator *vator = new QRegExpValidator(rx, this);
    ui->lidarIp->setValidator(vator);
    setFixedSize(size());
}

AddLidar::~AddLidar()
{
    delete ui;
}

void AddLidar::on_btn_sure_clicked()
{
    if(ui->ScanRadius->text().isEmpty() || ui->Index->text().isEmpty() ||ui->lidarIp->text().isEmpty())
    {
        QMessageBox message(QMessageBox::NoIcon, "提示", "不能为空",QMessageBox::Yes, NULL);
        message.exec();
        return;
    }
    lidar lidartmp;
    lidartmp.lidar_Radius = ui->ScanRadius->text().toInt();
    lidartmp.lidar_Index = ui->Index->text().toInt();
    lidartmp.lidar_Ip = ui->lidarIp->text();
    for(int i = 0; i < 3; i++){
        lidartmp.PolygonPoints[i].clear();
        lidartmp.Polygon[i].clear();
    }
    lidartmp.distance = 20;
    lidartmp.scanNum = 5;
    emit addlidar(lidartmp);
    this->close();
}

void AddLidar::on_btn_no_clicked()
{
    this->close();
}

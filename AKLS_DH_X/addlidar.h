#ifndef ADDLIDAR_H
#define ADDLIDAR_H

#include <QDialog>
#include "LidarData.h"

namespace Ui {
class AddLidar;
}

class AddLidar : public QDialog
{
    Q_OBJECT

public:
    explicit AddLidar(QWidget *parent = 0);
    ~AddLidar();

private slots:
    void on_btn_sure_clicked();

    void on_btn_no_clicked();
signals:
    void addlidar(lidar lidartmp);
private:
    Ui::AddLidar *ui;
};

#endif // ADDLIDAR_H

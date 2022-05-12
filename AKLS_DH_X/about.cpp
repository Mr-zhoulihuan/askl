#include "about.h"
#include "ui_about.h"
#include <QDebug>
#include <QUrl>
#include <QDesktopServices>
#include <QIcon>
About::About(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::About)
{
    ui->setupUi(this);
    setModal(true);
    ui->textEdit->setEnabled(false);
    setWindowIcon(QIcon(":/image/leishen.ico"));
    setFixedSize(size());
}

About::~About()
{
    delete ui;
}

void About::on_label_2_linkActivated(const QString &link)
{
    qDebug()<<link;
    QDesktopServices::openUrl(QUrl(link));
}

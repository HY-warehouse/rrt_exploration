#include "frmmain.h"
#include "ui_frmmain.h"
#include "qffmpeg.h"
#include "rtspthread.h"
#include <QDebug>
frmMain::frmMain(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::frmMain)
{
    ui->setupUi(this);    

    tempWidth=320;
    tempHeight=180;
    video1Max=false;
    video2Max=false;
    video3Max=false;
    video4Max=false;
//    all=false;
    ui->labVideo1->installEventFilter(this);
    ui->labVideo2->installEventFilter(this);
    ui->labVideo3->installEventFilter(this);
    ui->labVideo4->installEventFilter(this);
    ui->labVideo5->installEventFilter(this);
    ui->labImage->installEventFilter(this);
}

frmMain::~frmMain()
{
    delete ui;
}

//处理用户双击对应通道最大化处理
bool frmMain::eventFilter(QObject *obj, QEvent *event)
{    
    if (event->type()==QEvent::MouseButtonDblClick){
        if (obj==ui->labVideo1){
            if (video1Max){
                tempWidth=320;
                tempHeight=180;
                ui->labVideo2->setVisible(true);
                ui->labVideo3->setVisible(true);
                ui->labVideo4->setVisible(true);
                ui->labVideo5->setVisible(true);
                ui->labImage->setVisible(true);
            }else{
                tempWidth=645;
                tempHeight=370;
                ui->labVideo2->setVisible(false);
                ui->labVideo3->setVisible(false);
                ui->labVideo4->setVisible(true);
                ui->labVideo5->setVisible(true);
                ui->labImage->setVisible(false);
            }
            video1Max=!video1Max;
        }else if (obj==ui->labVideo2){
            if (video2Max){
                tempWidth=320;
                tempHeight=180;
                ui->labVideo1->setVisible(true);
                ui->labVideo3->setVisible(true);
                ui->labVideo4->setVisible(true);
                ui->labVideo5->setVisible(true);
                ui->labImage->setVisible(true);                
            }else{
                tempWidth=645;
                tempHeight=370;
                ui->labVideo1->setVisible(false);
                ui->labVideo3->setVisible(false);
                ui->labVideo4->setVisible(true);
                ui->labVideo5->setVisible(true);
                ui->labImage->setVisible(false);                
            }
            video2Max=!video2Max;
        }else if (obj==ui->labVideo3){
            if (video3Max){
                tempWidth=320;
                tempHeight=180;
                ui->labVideo1->setVisible(true);
                ui->labVideo2->setVisible(true);
                ui->labVideo4->setVisible(true);
                ui->labVideo5->setVisible(true);
                ui->labImage->setVisible(true);                
            }else{
                tempWidth=645;
                tempHeight=370;
                ui->labVideo1->setVisible(false);
                ui->labVideo2->setVisible(false);
                ui->labVideo4->setVisible(true);
                ui->labVideo5->setVisible(true);
                ui->labImage->setVisible(false);                
            }
            video3Max=!video3Max;
        }else if (obj==ui->labVideo4){
            if (video5Max){
                tempWidth=320;
                tempHeight=180;
                ui->labVideo1->setVisible(true);
                ui->labVideo2->setVisible(true);
                ui->labVideo3->setVisible(true);
                ui->labVideo5->setVisible(true);
                ui->labImage->setVisible(true);
            }else{
                tempWidth=645;
                tempHeight=370;
                ui->labVideo1->setVisible(false);
                ui->labVideo2->setVisible(false);
                ui->labVideo3->setVisible(true);
                ui->labVideo5->setVisible(true);
                ui->labImage->setVisible(false);
            }
            video5Max=!video5Max;
        }else if (obj==ui->labVideo5){
            if (video6Max){
                tempWidth=320;
                tempHeight=180;
                ui->labVideo1->setVisible(true);
                ui->labVideo2->setVisible(true);
                ui->labVideo4->setVisible(true);
                ui->labVideo3->setVisible(true);
                ui->labImage->setVisible(true);
            }else{
                tempWidth=645;
                tempHeight=370;
                ui->labVideo1->setVisible(false);
                ui->labVideo2->setVisible(false);
                ui->labVideo4->setVisible(true);
                ui->labVideo3->setVisible(true);
                ui->labImage->setVisible(false);
            }
            video6Max=!video6Max;
        }else if (obj==ui->labImage){
            if (video4Max){
                tempWidth=320;
                tempHeight=180;
                ui->labVideo1->setVisible(true);
                ui->labVideo2->setVisible(true);
                ui->labVideo3->setVisible(true);
                ui->labVideo4->setVisible(true);
                ui->labVideo5->setVisible(true);
            }else{
                tempWidth=645;
                tempHeight=370;
                ui->labVideo1->setVisible(false);
                ui->labVideo2->setVisible(false);
                ui->labVideo3->setVisible(false);
                ui->labVideo4->setVisible(true);
                ui->labVideo5->setVisible(true);
            }
            video4Max=!video4Max;
        }
    }
    return QObject::eventFilter(obj,event);
}

//void frmMain::on_btnOpen_clicked()//打开
//{
//    QFFmpeg *ffmpeg=new QFFmpeg(this);
//    connect(ffmpeg,SIGNAL(GetImage(QImage)),this,SLOT(SetImage(QImage)));
//    ffmpeg->SetUrl(ui->txtUrl->text());

//    if (ffmpeg->Init()){
//        RtspThread *rtsp=new RtspThread(this);
//        rtsp->setffmpeg(ffmpeg);
//        rtsp->start();
//    }
//}

//void frmMain::on_btnGetImage_clicked()//截图
//{
//    ui->labImage->clear();
//    int index=ui->cboxVideo->currentIndex();
//    if (index==0){
//        if (ui->labVideo1->pixmap()!=0x0)
//            ui->labImage->setPixmap(*ui->labVideo1->pixmap());
//    }else if (index==1){
//        if (ui->labVideo2->pixmap()!=0x0)
//            ui->labImage->setPixmap(*ui->labVideo2->pixmap());
//    }else if (index==2){
//        if (ui->labVideo3->pixmap()!=0x0)
//            ui->labImage->setPixmap(*ui->labVideo3->pixmap());
//    }
//}

//void frmMain::SetImage(const QImage &image)
//{
//    if (image.height()>0){
//        QPixmap pix = QPixmap::fromImage(image.scaled(tempWidth,tempHeight));
//        ui->labVideo1->setPixmap(pix);
//        if (all){//启用三通道同步
//            ui->labVideo2->setPixmap(pix);
//            ui->labVideo3->setPixmap(pix);
//        }
//    }
//}

void frmMain::SetImage1(const QImage &image)
{
    if (image.height()>0){
        QPixmap pix = QPixmap::fromImage(image.scaled(tempWidth,tempHeight));
        ui->labVideo1->setPixmap(pix);
    }
}
void frmMain::SetImage2(const QImage &image)
{
    if (image.height()>0){
        QPixmap pix = QPixmap::fromImage(image.scaled(tempWidth,tempHeight));
        ui->labVideo2->setPixmap(pix);
    }
}
void frmMain::SetImage3(const QImage &image)
{
    if (image.height()>0){
        QPixmap pix = QPixmap::fromImage(image.scaled(tempWidth,tempHeight));
        ui->labVideo3->setPixmap(pix);
    }
}
void frmMain::SetImage4(const QImage &image)
{
    if (image.height()>0){
        QPixmap pix = QPixmap::fromImage(image.scaled(tempWidth,tempHeight));
        ui->labVideo4->setPixmap(pix);
    }
}
void frmMain::SetImage5(const QImage &image)
{
    if (image.height()>0){
        QPixmap pix = QPixmap::fromImage(image.scaled(tempWidth,tempHeight));
        ui->labVideo5->setPixmap(pix);
    }
}
void frmMain::SetImage6(const QImage &image)
{
    if (image.height()>0){
        QPixmap pix = QPixmap::fromImage(image.scaled(tempWidth,tempHeight));
        ui->labImage->setPixmap(pix);
    }
}
//void frmMain::on_ckAll_stateChanged(int arg1)
//{
//    all=arg1!=0?true:false;
//}

void frmMain::on_btnOpen1_clicked()
{
    QFFmpeg *ffmpeg1=new QFFmpeg(this);
    connect(ffmpeg1,SIGNAL(GetImage(QImage)),this,SLOT(SetImage1(QImage)));
    ffmpeg1->SetUrl(ui->txtUrl1->text());

    if (ffmpeg1->Init()){
//        ffmpeg1->cam =1;
        RtspThread *rtsp1=new RtspThread(this);
        rtsp1->setffmpeg(ffmpeg1);
        rtsp1->start();
    }
}

void frmMain::on_btnOpen2_clicked()
{
    QFFmpeg *ffmpeg2=new QFFmpeg(this);
    connect(ffmpeg2,SIGNAL(GetImage(QImage)),this,SLOT(SetImage2(QImage)));
    ffmpeg2->SetUrl(ui->txtUrl2->text());

    if (ffmpeg2->Init()){
//        ffmpeg2->cam =2;
        RtspThread *rtsp2=new RtspThread(this);
        rtsp2->setffmpeg(ffmpeg2);
        rtsp2->start();
    }
}
void frmMain::on_btnOpen3_clicked()
{
    QFFmpeg *ffmpeg3=new QFFmpeg(this);
    connect(ffmpeg3,SIGNAL(GetImage(QImage)),this,SLOT(SetImage3(QImage)));
    ffmpeg3->SetUrl(ui->txtUrl3->text());

    if (ffmpeg3->Init()){
//        ffmpeg3->cam =3;
        RtspThread *rtsp3=new RtspThread(this);
        rtsp3->setffmpeg(ffmpeg3);
        rtsp3->start();
    }
}

void frmMain::on_btnOpen4_clicked()
{
    QFFmpeg *ffmpeg4=new QFFmpeg(this);
    connect(ffmpeg4,SIGNAL(GetImage(QImage)),this,SLOT(SetImage4(QImage)));
    ffmpeg4->SetUrl(ui->txtUrl4->text());

    if (ffmpeg4->Init()){
        RtspThread *rtsp4=new RtspThread(this);
        rtsp4->setffmpeg(ffmpeg4);
        rtsp4->start();
    }
}

void frmMain::on_btnOpen5_clicked()
{
    QFFmpeg *ffmpeg5=new QFFmpeg(this);
    connect(ffmpeg5,SIGNAL(GetImage(QImage)),this,SLOT(SetImage5(QImage)));
    ffmpeg5->SetUrl(ui->txtUrl5->text());

    if (ffmpeg5->Init()){
        RtspThread *rtsp5=new RtspThread(this);
        rtsp5->setffmpeg(ffmpeg5);
        rtsp5->start();
    }
}

void frmMain::on_btnOpen6_clicked()
{
    QFFmpeg *ffmpeg6=new QFFmpeg(this);
    connect(ffmpeg6,SIGNAL(GetImage(QImage)),this,SLOT(SetImage6(QImage)));
    ffmpeg6->SetUrl(ui->txtUrl6->text());

    if (ffmpeg6->Init()){
        RtspThread *rtsp6=new RtspThread(this);
        rtsp6->setffmpeg(ffmpeg6);
        rtsp6->start();
    }
}

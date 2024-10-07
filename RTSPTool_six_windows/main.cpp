#include "frmmain.h"
#include <QApplication>
#include <QFile>
#include <QTextCodec>
#include <QDesktopWidget>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);   

    //根据不同QT版本编译,在QT5以下版本中需要设置为UTF-8编码才能正常显示中文
#if (QT_VERSION <= QT_VERSION_CHECK(5,0,0))
    QTextCodec *codec=QTextCodec::codecForName("UTF-8");
    QTextCodec::setCodecForLocale(codec);
    QTextCodec::setCodecForCStrings(codec);
    QTextCodec::setCodecForTr(codec);
#endif

    //加载和应用黑灰色主题样式表
    QFile qssFile(":/style.qss");
    qssFile.open(QFile::ReadOnly);
    if(qssFile.isOpen()){
        qApp->setStyleSheet(QLatin1String(qssFile.readAll()));
        qssFile.close();
    }

    frmMain w;
    //设置窗体居中显示,并且不能改变大小
    QDesktopWidget desktop;
    int screenX=desktop.availableGeometry().width();
    int screenY=desktop.availableGeometry().height();
    int frmX=w.width();
    int frmY=w.height();
    QPoint movePoint(screenX/2-frmX/2,screenY/2-frmY/2);
    w.move(movePoint);
    //设置窗体不能调整大小
    w.setFixedSize(frmX,frmY);
    w.show();

    return a.exec();
}

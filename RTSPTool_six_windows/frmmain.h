#ifndef FRMMAIN_H
#define FRMMAIN_H

#include <QWidget>

namespace Ui {
class frmMain;
}

class frmMain : public QWidget
{
    Q_OBJECT

public:
    explicit frmMain(QWidget *parent = 0);
    ~frmMain();

private slots:
//    void SetImage(const QImage &image);
    void SetImage1(const QImage &image);
    void SetImage2(const QImage &image);
    void SetImage3(const QImage &image);
    void SetImage4(const QImage &image);
    void SetImage5(const QImage &image);
    void SetImage6(const QImage &image);

//    void on_btnOpen_clicked();
//    void on_btnGetImage_clicked();

//    void on_ckAll_stateChanged(int arg1);

    void on_btnOpen3_clicked();

    void on_btnOpen4_clicked();

    void on_btnOpen1_clicked();

    void on_btnOpen2_clicked();

    void on_btnOpen5_clicked();

    void on_btnOpen6_clicked();

protected:
    bool eventFilter(QObject *obj, QEvent *event);

private:
    Ui::frmMain *ui;

    int tempWidth;
    int tempHeight;
    bool video1Max;
    bool video2Max;
    bool video3Max;
    bool video4Max;
    bool video5Max;
    bool video6Max;
//    bool all;
};
#endif // FRMMAIN_H

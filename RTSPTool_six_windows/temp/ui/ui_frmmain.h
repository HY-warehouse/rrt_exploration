/********************************************************************************
** Form generated from reading UI file 'frmmain.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_FRMMAIN_H
#define UI_FRMMAIN_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_frmMain
{
public:
    QVBoxLayout *verticalLayout;
    QHBoxLayout *lay1;
    QLabel *labVideo1;
    QLabel *labVideo2;
    QLabel *labVideo3;
    QHBoxLayout *lay2;
    QLabel *labVideo4;
    QLabel *labVideo5;
    QLabel *labImage;
    QHBoxLayout *lay4;
    QLineEdit *txtUrl1;
    QPushButton *btnOpen1;
    QLineEdit *txtUrl2;
    QPushButton *btnOpen2;
    QHBoxLayout *lay5;
    QLineEdit *txtUrl3;
    QPushButton *btnOpen3;
    QLineEdit *txtUrl4;
    QPushButton *btnOpen4;
    QHBoxLayout *horizontalLayout_2;
    QLineEdit *txtUrl5;
    QPushButton *btnOpen5;
    QLineEdit *txtUrl6;
    QPushButton *btnOpen6;

    void setupUi(QWidget *frmMain)
    {
        if (frmMain->objectName().isEmpty())
            frmMain->setObjectName(QString::fromUtf8("frmMain"));
        frmMain->resize(873, 467);
        frmMain->setMinimumSize(QSize(0, 467));
        verticalLayout = new QVBoxLayout(frmMain);
        verticalLayout->setSpacing(5);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(5, 5, 5, 5);
        lay1 = new QHBoxLayout();
        lay1->setSpacing(5);
        lay1->setObjectName(QString::fromUtf8("lay1"));
        labVideo1 = new QLabel(frmMain);
        labVideo1->setObjectName(QString::fromUtf8("labVideo1"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(labVideo1->sizePolicy().hasHeightForWidth());
        labVideo1->setSizePolicy(sizePolicy);
        labVideo1->setFrameShape(QFrame::Box);
        labVideo1->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignTop);

        lay1->addWidget(labVideo1);

        labVideo2 = new QLabel(frmMain);
        labVideo2->setObjectName(QString::fromUtf8("labVideo2"));
        sizePolicy.setHeightForWidth(labVideo2->sizePolicy().hasHeightForWidth());
        labVideo2->setSizePolicy(sizePolicy);
        labVideo2->setFrameShape(QFrame::Box);

        lay1->addWidget(labVideo2);

        labVideo3 = new QLabel(frmMain);
        labVideo3->setObjectName(QString::fromUtf8("labVideo3"));
        sizePolicy.setHeightForWidth(labVideo3->sizePolicy().hasHeightForWidth());
        labVideo3->setSizePolicy(sizePolicy);
        labVideo3->setFrameShape(QFrame::Box);

        lay1->addWidget(labVideo3);


        verticalLayout->addLayout(lay1);

        lay2 = new QHBoxLayout();
        lay2->setSpacing(5);
        lay2->setObjectName(QString::fromUtf8("lay2"));
        labVideo4 = new QLabel(frmMain);
        labVideo4->setObjectName(QString::fromUtf8("labVideo4"));
        labVideo4->setFrameShape(QFrame::Box);

        lay2->addWidget(labVideo4);

        labVideo5 = new QLabel(frmMain);
        labVideo5->setObjectName(QString::fromUtf8("labVideo5"));
        sizePolicy.setHeightForWidth(labVideo5->sizePolicy().hasHeightForWidth());
        labVideo5->setSizePolicy(sizePolicy);
        labVideo5->setFrameShape(QFrame::Box);

        lay2->addWidget(labVideo5);

        labImage = new QLabel(frmMain);
        labImage->setObjectName(QString::fromUtf8("labImage"));
        sizePolicy.setHeightForWidth(labImage->sizePolicy().hasHeightForWidth());
        labImage->setSizePolicy(sizePolicy);
        labImage->setFrameShape(QFrame::Box);

        lay2->addWidget(labImage);


        verticalLayout->addLayout(lay2);

        lay4 = new QHBoxLayout();
        lay4->setSpacing(5);
        lay4->setObjectName(QString::fromUtf8("lay4"));
        lay4->setContentsMargins(0, 0, 0, 0);
        txtUrl1 = new QLineEdit(frmMain);
        txtUrl1->setObjectName(QString::fromUtf8("txtUrl1"));

        lay4->addWidget(txtUrl1);

        btnOpen1 = new QPushButton(frmMain);
        btnOpen1->setObjectName(QString::fromUtf8("btnOpen1"));
        btnOpen1->setCursor(QCursor(Qt::PointingHandCursor));

        lay4->addWidget(btnOpen1);

        txtUrl2 = new QLineEdit(frmMain);
        txtUrl2->setObjectName(QString::fromUtf8("txtUrl2"));

        lay4->addWidget(txtUrl2);

        btnOpen2 = new QPushButton(frmMain);
        btnOpen2->setObjectName(QString::fromUtf8("btnOpen2"));
        btnOpen2->setCursor(QCursor(Qt::PointingHandCursor));

        lay4->addWidget(btnOpen2);


        verticalLayout->addLayout(lay4);

        lay5 = new QHBoxLayout();
        lay5->setSpacing(5);
        lay5->setObjectName(QString::fromUtf8("lay5"));
        lay5->setContentsMargins(0, 0, 0, 0);
        txtUrl3 = new QLineEdit(frmMain);
        txtUrl3->setObjectName(QString::fromUtf8("txtUrl3"));

        lay5->addWidget(txtUrl3);

        btnOpen3 = new QPushButton(frmMain);
        btnOpen3->setObjectName(QString::fromUtf8("btnOpen3"));
        btnOpen3->setCursor(QCursor(Qt::PointingHandCursor));

        lay5->addWidget(btnOpen3);

        txtUrl4 = new QLineEdit(frmMain);
        txtUrl4->setObjectName(QString::fromUtf8("txtUrl4"));

        lay5->addWidget(txtUrl4);

        btnOpen4 = new QPushButton(frmMain);
        btnOpen4->setObjectName(QString::fromUtf8("btnOpen4"));
        btnOpen4->setCursor(QCursor(Qt::PointingHandCursor));

        lay5->addWidget(btnOpen4);


        verticalLayout->addLayout(lay5);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(5);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalLayout_2->setSizeConstraint(QLayout::SetFixedSize);
        horizontalLayout_2->setContentsMargins(-1, 0, -1, -1);
        txtUrl5 = new QLineEdit(frmMain);
        txtUrl5->setObjectName(QString::fromUtf8("txtUrl5"));

        horizontalLayout_2->addWidget(txtUrl5);

        btnOpen5 = new QPushButton(frmMain);
        btnOpen5->setObjectName(QString::fromUtf8("btnOpen5"));

        horizontalLayout_2->addWidget(btnOpen5);

        txtUrl6 = new QLineEdit(frmMain);
        txtUrl6->setObjectName(QString::fromUtf8("txtUrl6"));

        horizontalLayout_2->addWidget(txtUrl6);

        btnOpen6 = new QPushButton(frmMain);
        btnOpen6->setObjectName(QString::fromUtf8("btnOpen6"));

        horizontalLayout_2->addWidget(btnOpen6);


        verticalLayout->addLayout(horizontalLayout_2);


        retranslateUi(frmMain);

        QMetaObject::connectSlotsByName(frmMain);
    } // setupUi

    void retranslateUi(QWidget *frmMain)
    {
        frmMain->setWindowTitle(QCoreApplication::translate("frmMain", "\345\256\236\346\227\266\344\276\246\345\257\237\345\233\276\345\203\217\347\263\273\347\273\237", nullptr));
        labVideo1->setText(QString());
        labVideo2->setText(QString());
        labVideo3->setText(QString());
        labVideo4->setText(QString());
        labVideo5->setText(QString());
        labImage->setText(QString());
        txtUrl1->setText(QCoreApplication::translate("frmMain", "rtsp://0.0.0.0:8554/back", nullptr));
        btnOpen1->setText(QCoreApplication::translate("frmMain", "\346\211\223\345\274\200", nullptr));
        txtUrl2->setText(QCoreApplication::translate("frmMain", "rtsp://0.0.0.0:8554/back1", nullptr));
        btnOpen2->setText(QCoreApplication::translate("frmMain", "\346\211\223\345\274\200", nullptr));
        txtUrl3->setText(QCoreApplication::translate("frmMain", "rtsp://0.0.0.0:8554/back2", nullptr));
        btnOpen3->setText(QCoreApplication::translate("frmMain", "\346\211\223\345\274\200", nullptr));
        txtUrl4->setText(QCoreApplication::translate("frmMain", "rtsp://0.0.0.0:8554/back3", nullptr));
        btnOpen4->setText(QCoreApplication::translate("frmMain", "\346\211\223\345\274\200", nullptr));
        btnOpen5->setText(QCoreApplication::translate("frmMain", "\346\211\223\345\274\200", nullptr));
        txtUrl6->setText(QCoreApplication::translate("frmMain", "rtsp://0.0.0.0:8554/back4", nullptr));
        btnOpen6->setText(QCoreApplication::translate("frmMain", "\346\211\223\345\274\200", nullptr));
    } // retranslateUi

};

namespace Ui {
    class frmMain: public Ui_frmMain {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FRMMAIN_H

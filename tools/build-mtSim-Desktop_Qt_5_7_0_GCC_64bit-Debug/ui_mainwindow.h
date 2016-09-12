/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "renderview.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QGridLayout *gridLayout_3;
    QTabWidget *tabWidget;
    QWidget *tab;
    QGridLayout *gridLayout_2;
    QGridLayout *gridLayout;
    QHBoxLayout *horizontalLayout_3;
    QPushButton *pushButton;
    QPushButton *pushButton_2;
    QPushButton *pushButton_8;
    QFrame *line;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *pushButton_4;
    QPushButton *pushButton_5;
    QPushButton *pushButton_6;
    QPushButton *pushButton_7;
    QLabel *label;
    QLabel *label_2;
    QFrame *line_5;
    QFrame *line_6;
    QFrame *line_8;
    RenderView *graphicsView;
    QFrame *line_7;
    QHBoxLayout *horizontalLayout;
    QLabel *label_3;
    QLabel *lb_sim_state;
    QFrame *line_2;
    QLabel *label_5;
    QLabel *lb_sim_time;
    QFrame *line_3;
    QLabel *label_6;
    QLabel *lb_real_time;
    QFrame *line_4;
    QLabel *label_7;
    QLabel *lb_fps;
    QPushButton *pushButton_9;
    QWidget *tab_2;
    QGridLayout *gridLayout_6;
    QLabel *label_14;
    QGridLayout *gridLayout_5;
    QLabel *label_15;
    QLabel *label_16;
    QLabel *label_17;
    QLabel *label_18;
    QLabel *label_19;
    QLabel *label_20;
    QVBoxLayout *verticalLayout_7;
    QPushButton *put_rob_1;
    QPushButton *take_rob_1;
    QVBoxLayout *verticalLayout_8;
    QPushButton *put_rob_2;
    QPushButton *take_rob_2;
    QVBoxLayout *verticalLayout_9;
    QPushButton *put_rob_3;
    QPushButton *take_rob_3;
    QVBoxLayout *verticalLayout_10;
    QPushButton *put_rob_4;
    QPushButton *take_rob_4;
    QVBoxLayout *verticalLayout_11;
    QPushButton *put_rob_5;
    QPushButton *take_rob_5;
    QVBoxLayout *verticalLayout_12;
    QPushButton *put_rob_6;
    QPushButton *take_rob_6;
    QLabel *label_13;
    QGridLayout *gridLayout_4;
    QLabel *label_4;
    QLabel *label_11;
    QLabel *label_8;
    QLabel *label_10;
    QLabel *label_9;
    QLabel *label_12;
    QVBoxLayout *verticalLayout;
    QPushButton *tele_open_1;
    QPushButton *tele_close_1;
    QVBoxLayout *verticalLayout_2;
    QPushButton *tele_open_2;
    QPushButton *tele_close_2;
    QVBoxLayout *verticalLayout_3;
    QPushButton *tele_open_3;
    QPushButton *tele_close_3;
    QVBoxLayout *verticalLayout_4;
    QPushButton *tele_open_4;
    QPushButton *tele_close_4;
    QVBoxLayout *verticalLayout_5;
    QPushButton *tele_open_5;
    QPushButton *tele_close_5;
    QVBoxLayout *verticalLayout_6;
    QPushButton *tele_open_6;
    QPushButton *tele_close_6;
    QLabel *label_21;
    QTableWidget *tableWidget;
    QMenuBar *menuBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(755, 685);
        MainWindow->setMinimumSize(QSize(0, 0));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        gridLayout_3 = new QGridLayout(centralWidget);
        gridLayout_3->setSpacing(6);
        gridLayout_3->setContentsMargins(11, 11, 11, 11);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        tabWidget = new QTabWidget(centralWidget);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tab = new QWidget();
        tab->setObjectName(QStringLiteral("tab"));
        gridLayout_2 = new QGridLayout(tab);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        gridLayout = new QGridLayout();
        gridLayout->setSpacing(6);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        pushButton = new QPushButton(tab);
        pushButton->setObjectName(QStringLiteral("pushButton"));

        horizontalLayout_3->addWidget(pushButton);

        pushButton_2 = new QPushButton(tab);
        pushButton_2->setObjectName(QStringLiteral("pushButton_2"));

        horizontalLayout_3->addWidget(pushButton_2);

        pushButton_8 = new QPushButton(tab);
        pushButton_8->setObjectName(QStringLiteral("pushButton_8"));

        horizontalLayout_3->addWidget(pushButton_8);


        gridLayout->addLayout(horizontalLayout_3, 2, 0, 1, 1);

        line = new QFrame(tab);
        line->setObjectName(QStringLiteral("line"));
        line->setFrameShadow(QFrame::Plain);
        line->setLineWidth(2);
        line->setMidLineWidth(0);
        line->setFrameShape(QFrame::VLine);

        gridLayout->addWidget(line, 2, 1, 1, 1);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        pushButton_4 = new QPushButton(tab);
        pushButton_4->setObjectName(QStringLiteral("pushButton_4"));

        horizontalLayout_2->addWidget(pushButton_4);

        pushButton_5 = new QPushButton(tab);
        pushButton_5->setObjectName(QStringLiteral("pushButton_5"));

        horizontalLayout_2->addWidget(pushButton_5);

        pushButton_6 = new QPushButton(tab);
        pushButton_6->setObjectName(QStringLiteral("pushButton_6"));

        horizontalLayout_2->addWidget(pushButton_6);

        pushButton_7 = new QPushButton(tab);
        pushButton_7->setObjectName(QStringLiteral("pushButton_7"));

        horizontalLayout_2->addWidget(pushButton_7);


        gridLayout->addLayout(horizontalLayout_2, 2, 2, 1, 1);

        label = new QLabel(tab);
        label->setObjectName(QStringLiteral("label"));

        gridLayout->addWidget(label, 1, 0, 1, 1);

        label_2 = new QLabel(tab);
        label_2->setObjectName(QStringLiteral("label_2"));

        gridLayout->addWidget(label_2, 1, 2, 1, 1);

        line_5 = new QFrame(tab);
        line_5->setObjectName(QStringLiteral("line_5"));
        line_5->setFrameShadow(QFrame::Plain);
        line_5->setLineWidth(2);
        line_5->setFrameShape(QFrame::HLine);

        gridLayout->addWidget(line_5, 0, 0, 1, 1);

        line_6 = new QFrame(tab);
        line_6->setObjectName(QStringLiteral("line_6"));
        line_6->setFrameShadow(QFrame::Plain);
        line_6->setLineWidth(2);
        line_6->setFrameShape(QFrame::HLine);

        gridLayout->addWidget(line_6, 0, 2, 1, 1);

        line_8 = new QFrame(tab);
        line_8->setObjectName(QStringLiteral("line_8"));
        line_8->setFrameShadow(QFrame::Plain);
        line_8->setLineWidth(2);
        line_8->setMidLineWidth(0);
        line_8->setFrameShape(QFrame::VLine);

        gridLayout->addWidget(line_8, 1, 1, 1, 1);


        gridLayout_2->addLayout(gridLayout, 0, 0, 1, 1);

        graphicsView = new RenderView(tab);
        graphicsView->setObjectName(QStringLiteral("graphicsView"));

        gridLayout_2->addWidget(graphicsView, 1, 0, 1, 1);

        line_7 = new QFrame(tab);
        line_7->setObjectName(QStringLiteral("line_7"));
        line_7->setFrameShadow(QFrame::Plain);
        line_7->setLineWidth(2);
        line_7->setFrameShape(QFrame::HLine);

        gridLayout_2->addWidget(line_7, 2, 0, 1, 1);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setSizeConstraint(QLayout::SetDefaultConstraint);
        label_3 = new QLabel(tab);
        label_3->setObjectName(QStringLiteral("label_3"));
        QSizePolicy sizePolicy(QSizePolicy::Maximum, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(label_3->sizePolicy().hasHeightForWidth());
        label_3->setSizePolicy(sizePolicy);

        horizontalLayout->addWidget(label_3);

        lb_sim_state = new QLabel(tab);
        lb_sim_state->setObjectName(QStringLiteral("lb_sim_state"));

        horizontalLayout->addWidget(lb_sim_state);

        line_2 = new QFrame(tab);
        line_2->setObjectName(QStringLiteral("line_2"));
        line_2->setFrameShadow(QFrame::Plain);
        line_2->setLineWidth(2);
        line_2->setFrameShape(QFrame::VLine);

        horizontalLayout->addWidget(line_2);

        label_5 = new QLabel(tab);
        label_5->setObjectName(QStringLiteral("label_5"));
        sizePolicy.setHeightForWidth(label_5->sizePolicy().hasHeightForWidth());
        label_5->setSizePolicy(sizePolicy);

        horizontalLayout->addWidget(label_5);

        lb_sim_time = new QLabel(tab);
        lb_sim_time->setObjectName(QStringLiteral("lb_sim_time"));

        horizontalLayout->addWidget(lb_sim_time);

        line_3 = new QFrame(tab);
        line_3->setObjectName(QStringLiteral("line_3"));
        line_3->setFrameShadow(QFrame::Plain);
        line_3->setLineWidth(2);
        line_3->setFrameShape(QFrame::VLine);

        horizontalLayout->addWidget(line_3);

        label_6 = new QLabel(tab);
        label_6->setObjectName(QStringLiteral("label_6"));
        sizePolicy.setHeightForWidth(label_6->sizePolicy().hasHeightForWidth());
        label_6->setSizePolicy(sizePolicy);

        horizontalLayout->addWidget(label_6);

        lb_real_time = new QLabel(tab);
        lb_real_time->setObjectName(QStringLiteral("lb_real_time"));

        horizontalLayout->addWidget(lb_real_time);

        line_4 = new QFrame(tab);
        line_4->setObjectName(QStringLiteral("line_4"));
        line_4->setFrameShadow(QFrame::Plain);
        line_4->setLineWidth(2);
        line_4->setFrameShape(QFrame::VLine);

        horizontalLayout->addWidget(line_4);

        label_7 = new QLabel(tab);
        label_7->setObjectName(QStringLiteral("label_7"));
        sizePolicy.setHeightForWidth(label_7->sizePolicy().hasHeightForWidth());
        label_7->setSizePolicy(sizePolicy);

        horizontalLayout->addWidget(label_7);

        lb_fps = new QLabel(tab);
        lb_fps->setObjectName(QStringLiteral("lb_fps"));

        horizontalLayout->addWidget(lb_fps);

        pushButton_9 = new QPushButton(tab);
        pushButton_9->setObjectName(QStringLiteral("pushButton_9"));

        horizontalLayout->addWidget(pushButton_9);


        gridLayout_2->addLayout(horizontalLayout, 3, 0, 1, 1);

        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QStringLiteral("tab_2"));
        gridLayout_6 = new QGridLayout(tab_2);
        gridLayout_6->setSpacing(6);
        gridLayout_6->setContentsMargins(11, 11, 11, 11);
        gridLayout_6->setObjectName(QStringLiteral("gridLayout_6"));
        label_14 = new QLabel(tab_2);
        label_14->setObjectName(QStringLiteral("label_14"));

        gridLayout_6->addWidget(label_14, 0, 0, 1, 1);

        gridLayout_5 = new QGridLayout();
        gridLayout_5->setSpacing(6);
        gridLayout_5->setObjectName(QStringLiteral("gridLayout_5"));
        label_15 = new QLabel(tab_2);
        label_15->setObjectName(QStringLiteral("label_15"));
        QFont font;
        font.setPointSize(7);
        label_15->setFont(font);

        gridLayout_5->addWidget(label_15, 0, 0, 1, 1);

        label_16 = new QLabel(tab_2);
        label_16->setObjectName(QStringLiteral("label_16"));
        label_16->setFont(font);

        gridLayout_5->addWidget(label_16, 0, 4, 1, 1);

        label_17 = new QLabel(tab_2);
        label_17->setObjectName(QStringLiteral("label_17"));
        label_17->setFont(font);

        gridLayout_5->addWidget(label_17, 0, 1, 1, 1);

        label_18 = new QLabel(tab_2);
        label_18->setObjectName(QStringLiteral("label_18"));
        label_18->setFont(font);

        gridLayout_5->addWidget(label_18, 0, 3, 1, 1);

        label_19 = new QLabel(tab_2);
        label_19->setObjectName(QStringLiteral("label_19"));
        label_19->setFont(font);

        gridLayout_5->addWidget(label_19, 0, 2, 1, 1);

        label_20 = new QLabel(tab_2);
        label_20->setObjectName(QStringLiteral("label_20"));
        label_20->setFont(font);

        gridLayout_5->addWidget(label_20, 0, 5, 1, 1);

        verticalLayout_7 = new QVBoxLayout();
        verticalLayout_7->setSpacing(6);
        verticalLayout_7->setObjectName(QStringLiteral("verticalLayout_7"));
        put_rob_1 = new QPushButton(tab_2);
        put_rob_1->setObjectName(QStringLiteral("put_rob_1"));
        QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(put_rob_1->sizePolicy().hasHeightForWidth());
        put_rob_1->setSizePolicy(sizePolicy1);
        put_rob_1->setFont(font);

        verticalLayout_7->addWidget(put_rob_1);

        take_rob_1 = new QPushButton(tab_2);
        take_rob_1->setObjectName(QStringLiteral("take_rob_1"));
        sizePolicy1.setHeightForWidth(take_rob_1->sizePolicy().hasHeightForWidth());
        take_rob_1->setSizePolicy(sizePolicy1);
        take_rob_1->setFont(font);

        verticalLayout_7->addWidget(take_rob_1);


        gridLayout_5->addLayout(verticalLayout_7, 1, 0, 1, 1);

        verticalLayout_8 = new QVBoxLayout();
        verticalLayout_8->setSpacing(6);
        verticalLayout_8->setObjectName(QStringLiteral("verticalLayout_8"));
        put_rob_2 = new QPushButton(tab_2);
        put_rob_2->setObjectName(QStringLiteral("put_rob_2"));
        sizePolicy1.setHeightForWidth(put_rob_2->sizePolicy().hasHeightForWidth());
        put_rob_2->setSizePolicy(sizePolicy1);
        put_rob_2->setFont(font);

        verticalLayout_8->addWidget(put_rob_2);

        take_rob_2 = new QPushButton(tab_2);
        take_rob_2->setObjectName(QStringLiteral("take_rob_2"));
        sizePolicy1.setHeightForWidth(take_rob_2->sizePolicy().hasHeightForWidth());
        take_rob_2->setSizePolicy(sizePolicy1);
        take_rob_2->setFont(font);

        verticalLayout_8->addWidget(take_rob_2);


        gridLayout_5->addLayout(verticalLayout_8, 1, 1, 1, 1);

        verticalLayout_9 = new QVBoxLayout();
        verticalLayout_9->setSpacing(6);
        verticalLayout_9->setObjectName(QStringLiteral("verticalLayout_9"));
        put_rob_3 = new QPushButton(tab_2);
        put_rob_3->setObjectName(QStringLiteral("put_rob_3"));
        sizePolicy1.setHeightForWidth(put_rob_3->sizePolicy().hasHeightForWidth());
        put_rob_3->setSizePolicy(sizePolicy1);
        put_rob_3->setFont(font);

        verticalLayout_9->addWidget(put_rob_3);

        take_rob_3 = new QPushButton(tab_2);
        take_rob_3->setObjectName(QStringLiteral("take_rob_3"));
        sizePolicy1.setHeightForWidth(take_rob_3->sizePolicy().hasHeightForWidth());
        take_rob_3->setSizePolicy(sizePolicy1);
        take_rob_3->setFont(font);

        verticalLayout_9->addWidget(take_rob_3);


        gridLayout_5->addLayout(verticalLayout_9, 1, 2, 1, 1);

        verticalLayout_10 = new QVBoxLayout();
        verticalLayout_10->setSpacing(6);
        verticalLayout_10->setObjectName(QStringLiteral("verticalLayout_10"));
        put_rob_4 = new QPushButton(tab_2);
        put_rob_4->setObjectName(QStringLiteral("put_rob_4"));
        sizePolicy1.setHeightForWidth(put_rob_4->sizePolicy().hasHeightForWidth());
        put_rob_4->setSizePolicy(sizePolicy1);
        put_rob_4->setFont(font);

        verticalLayout_10->addWidget(put_rob_4);

        take_rob_4 = new QPushButton(tab_2);
        take_rob_4->setObjectName(QStringLiteral("take_rob_4"));
        sizePolicy1.setHeightForWidth(take_rob_4->sizePolicy().hasHeightForWidth());
        take_rob_4->setSizePolicy(sizePolicy1);
        take_rob_4->setFont(font);

        verticalLayout_10->addWidget(take_rob_4);


        gridLayout_5->addLayout(verticalLayout_10, 1, 3, 1, 1);

        verticalLayout_11 = new QVBoxLayout();
        verticalLayout_11->setSpacing(6);
        verticalLayout_11->setObjectName(QStringLiteral("verticalLayout_11"));
        put_rob_5 = new QPushButton(tab_2);
        put_rob_5->setObjectName(QStringLiteral("put_rob_5"));
        sizePolicy1.setHeightForWidth(put_rob_5->sizePolicy().hasHeightForWidth());
        put_rob_5->setSizePolicy(sizePolicy1);
        put_rob_5->setFont(font);

        verticalLayout_11->addWidget(put_rob_5);

        take_rob_5 = new QPushButton(tab_2);
        take_rob_5->setObjectName(QStringLiteral("take_rob_5"));
        sizePolicy1.setHeightForWidth(take_rob_5->sizePolicy().hasHeightForWidth());
        take_rob_5->setSizePolicy(sizePolicy1);
        take_rob_5->setFont(font);

        verticalLayout_11->addWidget(take_rob_5);


        gridLayout_5->addLayout(verticalLayout_11, 1, 4, 1, 1);

        verticalLayout_12 = new QVBoxLayout();
        verticalLayout_12->setSpacing(6);
        verticalLayout_12->setObjectName(QStringLiteral("verticalLayout_12"));
        put_rob_6 = new QPushButton(tab_2);
        put_rob_6->setObjectName(QStringLiteral("put_rob_6"));
        sizePolicy1.setHeightForWidth(put_rob_6->sizePolicy().hasHeightForWidth());
        put_rob_6->setSizePolicy(sizePolicy1);
        put_rob_6->setFont(font);

        verticalLayout_12->addWidget(put_rob_6);

        take_rob_6 = new QPushButton(tab_2);
        take_rob_6->setObjectName(QStringLiteral("take_rob_6"));
        sizePolicy1.setHeightForWidth(take_rob_6->sizePolicy().hasHeightForWidth());
        take_rob_6->setSizePolicy(sizePolicy1);
        take_rob_6->setFont(font);

        verticalLayout_12->addWidget(take_rob_6);


        gridLayout_5->addLayout(verticalLayout_12, 1, 5, 1, 1);


        gridLayout_6->addLayout(gridLayout_5, 1, 0, 1, 1);

        label_13 = new QLabel(tab_2);
        label_13->setObjectName(QStringLiteral("label_13"));

        gridLayout_6->addWidget(label_13, 2, 0, 1, 1);

        gridLayout_4 = new QGridLayout();
        gridLayout_4->setSpacing(6);
        gridLayout_4->setObjectName(QStringLiteral("gridLayout_4"));
        label_4 = new QLabel(tab_2);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setFont(font);

        gridLayout_4->addWidget(label_4, 0, 0, 1, 1);

        label_11 = new QLabel(tab_2);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setFont(font);

        gridLayout_4->addWidget(label_11, 0, 4, 1, 1);

        label_8 = new QLabel(tab_2);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setFont(font);

        gridLayout_4->addWidget(label_8, 0, 1, 1, 1);

        label_10 = new QLabel(tab_2);
        label_10->setObjectName(QStringLiteral("label_10"));
        label_10->setFont(font);

        gridLayout_4->addWidget(label_10, 0, 3, 1, 1);

        label_9 = new QLabel(tab_2);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setFont(font);

        gridLayout_4->addWidget(label_9, 0, 2, 1, 1);

        label_12 = new QLabel(tab_2);
        label_12->setObjectName(QStringLiteral("label_12"));
        label_12->setFont(font);

        gridLayout_4->addWidget(label_12, 0, 5, 1, 1);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        tele_open_1 = new QPushButton(tab_2);
        tele_open_1->setObjectName(QStringLiteral("tele_open_1"));
        sizePolicy1.setHeightForWidth(tele_open_1->sizePolicy().hasHeightForWidth());
        tele_open_1->setSizePolicy(sizePolicy1);
        tele_open_1->setFont(font);

        verticalLayout->addWidget(tele_open_1);

        tele_close_1 = new QPushButton(tab_2);
        tele_close_1->setObjectName(QStringLiteral("tele_close_1"));
        sizePolicy1.setHeightForWidth(tele_close_1->sizePolicy().hasHeightForWidth());
        tele_close_1->setSizePolicy(sizePolicy1);
        tele_close_1->setFont(font);

        verticalLayout->addWidget(tele_close_1);


        gridLayout_4->addLayout(verticalLayout, 1, 0, 1, 1);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        tele_open_2 = new QPushButton(tab_2);
        tele_open_2->setObjectName(QStringLiteral("tele_open_2"));
        sizePolicy1.setHeightForWidth(tele_open_2->sizePolicy().hasHeightForWidth());
        tele_open_2->setSizePolicy(sizePolicy1);
        tele_open_2->setFont(font);

        verticalLayout_2->addWidget(tele_open_2);

        tele_close_2 = new QPushButton(tab_2);
        tele_close_2->setObjectName(QStringLiteral("tele_close_2"));
        sizePolicy1.setHeightForWidth(tele_close_2->sizePolicy().hasHeightForWidth());
        tele_close_2->setSizePolicy(sizePolicy1);
        tele_close_2->setFont(font);

        verticalLayout_2->addWidget(tele_close_2);


        gridLayout_4->addLayout(verticalLayout_2, 1, 1, 1, 1);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        tele_open_3 = new QPushButton(tab_2);
        tele_open_3->setObjectName(QStringLiteral("tele_open_3"));
        sizePolicy1.setHeightForWidth(tele_open_3->sizePolicy().hasHeightForWidth());
        tele_open_3->setSizePolicy(sizePolicy1);
        tele_open_3->setFont(font);

        verticalLayout_3->addWidget(tele_open_3);

        tele_close_3 = new QPushButton(tab_2);
        tele_close_3->setObjectName(QStringLiteral("tele_close_3"));
        sizePolicy1.setHeightForWidth(tele_close_3->sizePolicy().hasHeightForWidth());
        tele_close_3->setSizePolicy(sizePolicy1);
        tele_close_3->setFont(font);

        verticalLayout_3->addWidget(tele_close_3);


        gridLayout_4->addLayout(verticalLayout_3, 1, 2, 1, 1);

        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        tele_open_4 = new QPushButton(tab_2);
        tele_open_4->setObjectName(QStringLiteral("tele_open_4"));
        sizePolicy1.setHeightForWidth(tele_open_4->sizePolicy().hasHeightForWidth());
        tele_open_4->setSizePolicy(sizePolicy1);
        tele_open_4->setFont(font);

        verticalLayout_4->addWidget(tele_open_4);

        tele_close_4 = new QPushButton(tab_2);
        tele_close_4->setObjectName(QStringLiteral("tele_close_4"));
        sizePolicy1.setHeightForWidth(tele_close_4->sizePolicy().hasHeightForWidth());
        tele_close_4->setSizePolicy(sizePolicy1);
        tele_close_4->setFont(font);

        verticalLayout_4->addWidget(tele_close_4);


        gridLayout_4->addLayout(verticalLayout_4, 1, 3, 1, 1);

        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        tele_open_5 = new QPushButton(tab_2);
        tele_open_5->setObjectName(QStringLiteral("tele_open_5"));
        sizePolicy1.setHeightForWidth(tele_open_5->sizePolicy().hasHeightForWidth());
        tele_open_5->setSizePolicy(sizePolicy1);
        tele_open_5->setFont(font);

        verticalLayout_5->addWidget(tele_open_5);

        tele_close_5 = new QPushButton(tab_2);
        tele_close_5->setObjectName(QStringLiteral("tele_close_5"));
        sizePolicy1.setHeightForWidth(tele_close_5->sizePolicy().hasHeightForWidth());
        tele_close_5->setSizePolicy(sizePolicy1);
        tele_close_5->setFont(font);

        verticalLayout_5->addWidget(tele_close_5);


        gridLayout_4->addLayout(verticalLayout_5, 1, 4, 1, 1);

        verticalLayout_6 = new QVBoxLayout();
        verticalLayout_6->setSpacing(6);
        verticalLayout_6->setObjectName(QStringLiteral("verticalLayout_6"));
        tele_open_6 = new QPushButton(tab_2);
        tele_open_6->setObjectName(QStringLiteral("tele_open_6"));
        sizePolicy1.setHeightForWidth(tele_open_6->sizePolicy().hasHeightForWidth());
        tele_open_6->setSizePolicy(sizePolicy1);
        tele_open_6->setFont(font);

        verticalLayout_6->addWidget(tele_open_6);

        tele_close_6 = new QPushButton(tab_2);
        tele_close_6->setObjectName(QStringLiteral("tele_close_6"));
        sizePolicy1.setHeightForWidth(tele_close_6->sizePolicy().hasHeightForWidth());
        tele_close_6->setSizePolicy(sizePolicy1);
        tele_close_6->setFont(font);

        verticalLayout_6->addWidget(tele_close_6);


        gridLayout_4->addLayout(verticalLayout_6, 1, 5, 1, 1);


        gridLayout_6->addLayout(gridLayout_4, 3, 0, 1, 1);

        label_21 = new QLabel(tab_2);
        label_21->setObjectName(QStringLiteral("label_21"));

        gridLayout_6->addWidget(label_21, 4, 0, 1, 1);

        tableWidget = new QTableWidget(tab_2);
        tableWidget->setObjectName(QStringLiteral("tableWidget"));

        gridLayout_6->addWidget(tableWidget, 5, 0, 1, 1);

        tabWidget->addTab(tab_2, QString());

        gridLayout_3->addWidget(tabWidget, 0, 0, 1, 1);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 755, 19));
        MainWindow->setMenuBar(menuBar);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MinhoTeam Simulation Tools", 0));
        pushButton->setText(QApplication::translate("MainWindow", "Run", 0));
        pushButton_2->setText(QApplication::translate("MainWindow", "Pause", 0));
        pushButton_8->setText(QApplication::translate("MainWindow", "World", 0));
        pushButton_4->setText(QApplication::translate("MainWindow", "View 1", 0));
        pushButton_5->setText(QApplication::translate("MainWindow", "View 2", 0));
        pushButton_6->setText(QApplication::translate("MainWindow", "View 3", 0));
        pushButton_7->setText(QApplication::translate("MainWindow", "View 4", 0));
        label->setText(QApplication::translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-weight:600;\">SIMULATION CONTROL</span></p></body></html>", 0));
        label_2->setText(QApplication::translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-weight:600;\">CAMERA VIEW CONTROL</span></p></body></html>", 0));
        label_3->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-weight:600;\">State:</span></p></body></html>", 0));
        lb_sim_state->setText(QString());
        label_5->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-weight:600;\">Simulation Time:</span></p></body></html>", 0));
        lb_sim_time->setText(QString());
        label_6->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-weight:600;\">Real Time:</span></p></body></html>", 0));
        lb_real_time->setText(QString());
        label_7->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-weight:600;\">FPS:</span></p></body></html>", 0));
        lb_fps->setText(QString());
        pushButton_9->setText(QApplication::translate("MainWindow", "Toggle Render", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("MainWindow", "Simulation", 0));
        label_14->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:10pt; font-weight:600; font-style:italic;\">PLAYER MANAGER</span></p></body></html>", 0));
        label_15->setText(QApplication::translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-weight:600;\">Robot 1</span></p></body></html>", 0));
        label_16->setText(QApplication::translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-weight:600;\">Robot 5</span></p></body></html>", 0));
        label_17->setText(QApplication::translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-weight:600;\">Robot 2</span></p></body></html>", 0));
        label_18->setText(QApplication::translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-weight:600;\">Robot 4</span></p></body></html>", 0));
        label_19->setText(QApplication::translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-weight:600;\">Robot 3</span></p></body></html>", 0));
        label_20->setText(QApplication::translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-weight:600;\">Robot 6</span></p></body></html>", 0));
        put_rob_1->setText(QApplication::translate("MainWindow", "Put in Field", 0));
        take_rob_1->setText(QApplication::translate("MainWindow", "Take from Field", 0));
        put_rob_2->setText(QApplication::translate("MainWindow", "Put in Field", 0));
        take_rob_2->setText(QApplication::translate("MainWindow", "Take from Field", 0));
        put_rob_3->setText(QApplication::translate("MainWindow", "Put in Field", 0));
        take_rob_3->setText(QApplication::translate("MainWindow", "Take from Field", 0));
        put_rob_4->setText(QApplication::translate("MainWindow", "Put in Field", 0));
        take_rob_4->setText(QApplication::translate("MainWindow", "Take from Field", 0));
        put_rob_5->setText(QApplication::translate("MainWindow", "Put in Field", 0));
        take_rob_5->setText(QApplication::translate("MainWindow", "Take from Field", 0));
        put_rob_6->setText(QApplication::translate("MainWindow", "Put in Field", 0));
        take_rob_6->setText(QApplication::translate("MainWindow", "Take from Field", 0));
        label_13->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:10pt; font-weight:600; font-style:italic;\">TELEOP LAUNCH</span></p></body></html>", 0));
        label_4->setText(QApplication::translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-weight:600;\">MinhoTeleop 1</span></p></body></html>", 0));
        label_11->setText(QApplication::translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-weight:600;\">MinhoTeleop 5</span></p></body></html>", 0));
        label_8->setText(QApplication::translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-weight:600;\">MinhoTeleop 2</span></p></body></html>", 0));
        label_10->setText(QApplication::translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-weight:600;\">MinhoTeleop 4</span></p></body></html>", 0));
        label_9->setText(QApplication::translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-weight:600;\">MinhoTeleop 3</span></p></body></html>", 0));
        label_12->setText(QApplication::translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-weight:600;\">MinhoTeleop 6</span></p></body></html>", 0));
        tele_open_1->setText(QApplication::translate("MainWindow", "Open", 0));
        tele_close_1->setText(QApplication::translate("MainWindow", "Close", 0));
        tele_open_2->setText(QApplication::translate("MainWindow", "Open", 0));
        tele_close_2->setText(QApplication::translate("MainWindow", "Close", 0));
        tele_open_3->setText(QApplication::translate("MainWindow", "Open", 0));
        tele_close_3->setText(QApplication::translate("MainWindow", "Close", 0));
        tele_open_4->setText(QApplication::translate("MainWindow", "Open", 0));
        tele_close_4->setText(QApplication::translate("MainWindow", "Close", 0));
        tele_open_5->setText(QApplication::translate("MainWindow", "Open", 0));
        tele_close_5->setText(QApplication::translate("MainWindow", "Close", 0));
        tele_open_6->setText(QApplication::translate("MainWindow", "Open", 0));
        tele_close_6->setText(QApplication::translate("MainWindow", "Close", 0));
        label_21->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:10pt; font-weight:600; font-style:italic;\">ROBOT INFORMATION</span></p></body></html>", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("MainWindow", "Model Manager", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H

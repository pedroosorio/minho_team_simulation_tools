#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSplashScreen>
#include <QIcon>
#include <QtXml>
#include <QFile>
#include "renderingcamera.h"
#include "worldmanager.h"
#include "teleopprocessmanager.h"

namespace Ui {
class MainWindow;

}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void start();
private slots:
    void initializeGUI();
    void readCameraConf();
    void readRosServiceConf();

    vector<_cameraview> parseCameraViewList(QDomNodeList list);
    _cameraview parseView(QDomElement view);
    vector<_rosservice> parseRosServiceList(QDomNodeList list);
    _rosservice parseService(QDomElement service);
    _rosservice getServiceCall(QString name);

    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *event);
    void closeEvent(QCloseEvent *event);

    void on_pushButton_4_clicked();
    void on_pushButton_5_clicked();
    void on_pushButton_6_clicked();
    void on_pushButton_7_clicked();
    void on_pushButton_clicked();
    void on_pushButton_2_clicked();
    void on_pushButton_8_clicked();

    void update_world_stats(QString pause,QString sim_time,QString real_time);
    void update_model_poses(std::vector<gazebo::msgs::Pose> poses);
    void on_pushButton_9_clicked();

    void on_tele_open_1_clicked();

    void on_tele_open_2_clicked();

    void on_tele_open_3_clicked();

    void on_tele_open_4_clicked();

    void on_tele_open_5_clicked();

    void on_tele_open_6_clicked();

    void on_tele_close_1_clicked();

    void on_tele_close_2_clicked();

    void on_tele_close_3_clicked();

    void on_tele_close_4_clicked();

    void on_tele_close_5_clicked();

    void on_tele_close_6_clicked();

    void on_put_rob_1_clicked();

    void on_put_rob_2_clicked();

    void on_put_rob_3_clicked();

    void on_put_rob_4_clicked();

    void on_put_rob_5_clicked();

    void on_put_rob_6_clicked();

    void on_take_rob_1_clicked();

    void on_take_rob_2_clicked();

    void on_take_rob_3_clicked();

    void on_take_rob_4_clicked();

    void on_take_rob_5_clicked();

    void on_take_rob_6_clicked();

private:
    Ui::MainWindow *ui;
    RenderingCamera *_gfx_sim_;
    WorldManager *_sim_control_;
    // Configurations
    vector<_cameraview> defaultviews_,customviews_;
    vector<_rosservice> rosservicecalls_;
    QGraphicsScene *scene;
    TeleopProcessManager *minho_manager;
    bool modCtrl_, modShift_;

};

#endif // MAINWINDOW_H

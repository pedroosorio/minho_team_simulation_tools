#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    qRegisterMetaType<std::vector<gazebo::msgs::Pose> >("std::vector<gazebo::msgs::Pose>");
    // Start Resources
    MainWindow w(false);
    QSplashScreen *screen = new QSplashScreen();
    screen->setPixmap(QString("://resources/images/splash.png"));
    screen->showNormal();
    //Do external resources load, while showing splash screen
    QTimer::singleShot(2000,Qt::CoarseTimer,screen,SLOT(close()));
    QTimer::singleShot(2000,Qt::CoarseTimer,&w,SLOT(show()));
    //Start app
    w.start();
    return a.exec();
}

///
/// TODO
///     Implement object selection/dragging (Ctrl+Mouse)
///     Improve rendering : Try using threading or using directly the gazebo API
///     Implement model manager to add or remove a model, using ROS or gazebo API
///

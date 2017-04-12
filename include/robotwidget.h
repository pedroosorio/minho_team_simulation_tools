

#ifndef ROBOTWIDGET_H
#define ROBOTWIDGET_H

#include <QWidget>
#include <QLabel>
#include <QString>
#include <minho_team_ros/hardwareInfo.h>
#include <QPixmap>
#include <QPainter>

using minho_team_ros::hardwareInfo;

namespace Ui {
class robotWidget;
}

class robotWidget : public QWidget
{
    Q_OBJECT

public:
    explicit robotWidget(QWidget *parent = 0);
    ~robotWidget();
    void setRobotId(unsigned int id);
    void setWidgetState(bool active);
    void updateInformation(hardwareInfo info,float freq);
    unsigned int getCurrentRole();
    void updateComsFrequency(float freq);
private:

private:
    Ui::robotWidget *ui;
    QPixmap online, offline;
};

#endif // ROBOTWIDGET_H

#include "robotwidget.h"
#include "ui_robotwidget.h"

///
/// SIGNAL newRobotInformationReceived is triggered when new information is received from RTDB
///

robotWidget::robotWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::robotWidget)
{
    ui->setupUi(this);
}

robotWidget::~robotWidget()
{
    delete ui;
}

void robotWidget::setRobotId(unsigned int id)
{
    if(id<0||id>=6) return;
    else ui->lb_rname->setText("● Robot "+QString::number(id));
}

void robotWidget::setWidgetState(bool active)
{
    setEnabled(active);
    if(!active) { ui->lb_rname->setStyleSheet("color:red;"); ui->lb_freewheel->setStyleSheet("color:green;"); }
    else ui->lb_rname->setStyleSheet("color:green;");
}

void robotWidget::updateInformation(minho_team_ros::hardwareInfo info, float freq)
{
    ui->lb_cambat->setText(QString::number(info.battery_camera,'f',2)+"V");
    ui->lb_pcbat->setText(QString::number(info.battery_pc,'f',2)+"V");
    ui->lb_mainbat->setText(QString::number(info.battery_main,'f',2)+"V");
    ui->lb_imu->setText(QString::number(info.imu_value,'f',2)+"º");
    if(info.free_wheel_activated) { ui->lb_freewheel->setText("Active"); ui->lb_freewheel->setStyleSheet("color:red;");}
    else { ui->lb_freewheel->setText("Inactive"); ui->lb_freewheel->setStyleSheet("color:green;");}
    ui->lb_ball_sens->setText(QString::number(info.ball_sensor));
    updateComsFrequency(freq);
}

unsigned int robotWidget::getCurrentRole()
{
    return ui->cb_role->currentIndex();
}

void robotWidget::updateComsFrequency(float freq)
{
    ui->lb_comfreq->setText(QString::number(freq,'f',2)+" Hz");
}

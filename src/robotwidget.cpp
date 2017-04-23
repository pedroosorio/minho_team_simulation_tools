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
    agent_id = id;
}

void robotWidget::setWidgetState(bool active)
{
    setEnabled(active);
    if(!active) { ui->lb_rname->setStyleSheet("color:red;"); ui->lb_freewheel->setStyleSheet("color:green;"); }
    else ui->lb_rname->setStyleSheet("color:green;");
}

void robotWidget::updateInformation(minho_team_ros::aiInfo ai, minho_team_ros::hardwareInfo info, float freq)
{
    QString strActions[11] = {"aSTOP","aAPPROACHPOSITION","aFASTMOVE","aAPPROACHBALL","aENGAGEBALL",
    "aSLOWENGAGEBALL","aRECEIVEBALL", "aPASSBALL", "aKICKBALL", "aDRIBBLEBALL", "aHOLDBALL"};

    ui->lb_cambat->setText(QString::number(info.battery_camera,'f',2)+"V");
    ui->lb_pcbat->setText(QString::number(info.battery_pc,'f',2)+"V");
    ui->lb_mainbat->setText(QString::number(info.battery_main,'f',2)+"V");
    ui->lb_imu->setText(QString::number(info.imu_value,'f',2)+"º");
    if(info.free_wheel_activated) { ui->lb_freewheel->setText("Active"); ui->lb_freewheel->setStyleSheet("color:red;");}
    else { ui->lb_freewheel->setText("Inactive"); ui->lb_freewheel->setStyleSheet("color:green;");}
    ui->lb_ball_sens->setText(QString::number(info.ball_sensor));
    ui->lb_action->setText(strActions[ai.action]);
    ui->lb_tpose->setText(QString("[ ")+QString::number(ai.target_pose.x,'f',2)+QString(" , ")
                          +QString::number(ai.target_pose.y,'f',2)+QString(" , ")
                          +QString::number(ai.target_pose.z,'f',2)+QString(" ]"));
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

void robotWidget::on_bt_reloc_clicked()
{
    emit relocRequested(agent_id);
}

void robotWidget::on_bt_resetimu_clicked()
{
    emit resetIMURequested(agent_id);
}

#include "replay_window.h"
#include "ui_replay_window.h"
#define BUTTON_ICON_SIZE 30

/// \brief class constructor
ReplayWindow::ReplayWindow(QString file,QDialog *parent) :
    QDialog(parent),
    ui(new Ui::Window)
{
   ui->setupUi(this);
   this->setupUI();
   replay_running = false;

   int ret = 0;
   replay = new Replay(&ret,file);
   if(ret==0){
       // failed to load replay file
       QMessageBox not_av;
       not_av.setText("Sorry, but we encountered errors when reading the SimReplay file you provided.");
       not_av.setWindowTitle("Minho Simulator: SimReplay file error");
       not_av.addButton(QMessageBox::Ok);
       not_av.setPalette(ui->bt_startpause->palette());
       not_av.exec();
       this->close();
   } else {
       ui->lb_simname->setText(replay->getName());
       ui->lb_simdate->setText(replay->getDate());
       total_seconds = " / ";
       float total_secs = replay->getTotalTime();
       int minutes = total_secs/60;
       float seconds = total_secs-(int)total_secs/60;
       int mseconds = (seconds-(int)seconds)*1000;
       total_seconds += QString("%1").arg(minutes, 2, 10, QChar('0'))+":"
                      + QString("%1").arg((int)seconds, 2, 10, QChar('0'))+":"
                      + QString("%1").arg(mseconds, 3, 10, QChar('0'));
       total_frames = replay->getTotalSamples();
       ui->hbar_time->setMaximum(total_frames);
       current_rendered_frame = 0;
       on_hbar_time_valueChanged(0);
       render_timer = new QTimer();
       connect(render_timer,SIGNAL(timeout()),this,SLOT(renderTrigger()));
   }
}

/// \brief class destructor
ReplayWindow::~ReplayWindow()
{    
    delete ui;
}

void ReplayWindow::setupUI()
{
    home = QString::fromStdString(getenv("HOME"));
    this->setWindowIcon(QIcon(home+"/.msim/resources/images/logo.png"));

    ui->bt_startpause->setIcon(QIcon(home+"/.msim/resources/images/start.png"));
    ui->bt_startpause->setIconSize(QSize(BUTTON_ICON_SIZE,BUTTON_ICON_SIZE));
    ui->bt_startpause->setToolButtonStyle(Qt::ToolButtonIconOnly);

    ui->bt_restart->setIcon(QIcon(home+"/.msim/resources/images/restart.png"));
    ui->bt_restart->setIconSize(QSize(BUTTON_ICON_SIZE,BUTTON_ICON_SIZE));
    ui->bt_restart->setToolButtonStyle(Qt::ToolButtonIconOnly);

    ui->bt_movback->setIcon(QIcon(home+"/.msim/resources/images/mov_back.png"));
    ui->bt_movback->setIconSize(QSize(BUTTON_ICON_SIZE,BUTTON_ICON_SIZE));
    ui->bt_movback->setToolButtonStyle(Qt::ToolButtonIconOnly);

    ui->bt_movforw->setIcon(QIcon(home+"/.msim/resources/images/mov_forw.png"));
    ui->bt_movforw->setIconSize(QSize(BUTTON_ICON_SIZE,BUTTON_ICON_SIZE));
    ui->bt_movforw->setToolButtonStyle(Qt::ToolButtonIconOnly);
}

void ReplayWindow::renderReplayFrame(int frame_id)
{
    qDebug() << "Rendering frame " << frame_id;
}

void ReplayWindow::on_hbar_time_valueChanged(int value)
{
    renderReplayFrame(value);
    float total_secs = (float)value*replay->getTimeStep();
    int minutes = total_secs/60;
    float seconds = total_secs-(int)total_secs/60;
    int mseconds = (seconds-(int)seconds)*1000;
    ui->lb_time->setText(QString("%1").arg(minutes, 2, 10, QChar('0'))+":"
                         + QString("%1").arg((int)seconds, 2, 10, QChar('0'))+":"
                         + QString("%1").arg(mseconds, 3, 10, QChar('0'))+total_seconds);
}


void ReplayWindow::on_bt_startpause_clicked()
{
    replay_running = !replay_running;
    if(replay_running) {
        ui->bt_startpause->setIcon(QIcon(home+"/.msim/resources/images/pause.png"));
        render_timer->start(1000*replay->getTimeStep());
    }else {
        render_timer->stop();
        ui->bt_startpause->setIcon(QIcon(home+"/.msim/resources/images/start.png"));
    }
}

void ReplayWindow::on_bt_restart_clicked()
{
    render_timer->stop();
    current_rendered_frame = 0;
    ui->hbar_time->setValue(0);
}

void ReplayWindow::on_bt_movback_clicked()
{
    render_timer->stop();
    current_rendered_frame--;
    if(current_rendered_frame<0) current_rendered_frame = 0;
    ui->hbar_time->setValue(current_rendered_frame);
}

void ReplayWindow::on_bt_movforw_clicked()
{
    render_timer->stop();
    current_rendered_frame++;
    if(current_rendered_frame>total_frames) current_rendered_frame = total_frames;
    ui->hbar_time->setValue(current_rendered_frame);
}

void ReplayWindow::renderTrigger()
{
    current_rendered_frame++;
    if(current_rendered_frame>total_frames) {
        on_bt_startpause_clicked();
        current_rendered_frame = 0;
    }
    ui->hbar_time->setValue(current_rendered_frame);
}

#include "action_dialog.h"
#include "ui_action_dialog.h"
#define BUTTON_ICON_SIZE 80

/// \brief class constructor
ActionDialog::ActionDialog(ACTION *act, QString *file,QDialog *parent) :
    QDialog(parent),
    ui(new Ui::Form)
{
   ui->setupUi(this);
   this->setupUI();
   filepath = file;
   action = act;
}

/// \brief class destructor
ActionDialog::~ActionDialog()
{    
    delete ui;
}

void ActionDialog::setupUI()
{
    home = QString::fromStdString(getenv("HOME"));

    this->setStyleSheet("QToolButton:hover { border-style: solid;\
                         border-width: 4px;\
                         border-radius: 10px;\
                         border-color: #d96f05;\
                         font-size:14px;\
                         font-weight: bold;\
                        }");

    this->setWindowIcon(QIcon(home+"/.msim/resources/images/logo.png"));
    ui->bt_newsim->setIcon(QIcon(home+"/.msim/resources/images/new.png"));
    ui->bt_newsim->setIconSize(QSize(BUTTON_ICON_SIZE,BUTTON_ICON_SIZE));
    ui->bt_newsim->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    ui->bt_newsim->setText("New\n Simulation");

    ui->bt_opensim->setIcon(QIcon(home+"/.msim/resources/images/open.png"));
    ui->bt_opensim->setIconSize(QSize(BUTTON_ICON_SIZE,BUTTON_ICON_SIZE));
    ui->bt_opensim->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    ui->bt_opensim->setText("Open\n Simulation");

    ui->bt_replay->setIcon(QIcon(home+"/.msim/resources/images/replay.png"));
    ui->bt_replay->setIconSize(QSize(BUTTON_ICON_SIZE,BUTTON_ICON_SIZE));
    ui->bt_replay->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    ui->bt_replay->setText("Replay\n Simulation");

    ui->bt_exit->setIcon(QIcon(home+"/.msim/resources/images/exit.png"));
    ui->bt_exit->setIconSize(QSize(BUTTON_ICON_SIZE,BUTTON_ICON_SIZE));
    ui->bt_exit->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    ui->bt_exit->setText("Exit");
}

void ActionDialog::on_bt_newsim_clicked()
{
    (*action) = INVALID; //not implemented
    (*filepath) = "NEW FILE";
    show_not_implemented();
    this->close();
}

void ActionDialog::on_bt_opensim_clicked()
{
    (*action) = INVALID; //not implemented
    (*filepath) = "OPEN FILE";
    show_not_implemented();
    this->close();
}

void ActionDialog::on_bt_replay_clicked()
{
    (*action) = INVALID;
    (*filepath) = QFileDialog::getOpenFileName(this, tr("Open Simulation Replay"),
                                               home+"/MinhoSimulator/Replays",
                                               tr("SimReplay Files (*.srp)"));
    if((*filepath)!="") { (*action) = REPLAY; this->close(); }
}

void ActionDialog::on_bt_exit_clicked()
{
    (*action) = EXIT;
    this->close();
}

void ActionDialog::show_not_implemented()
{
    QMessageBox not_av;
    not_av.setText("Sorry, but the feature you requested is not available at this time.");
    not_av.setWindowTitle("Minho Simulator: Feature not available");
    not_av.addButton(QMessageBox::Ok);
    not_av.setPalette(ui->bt_exit->palette());
    not_av.exec();
}


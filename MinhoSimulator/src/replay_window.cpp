#include "replay_window.h"
#include "ui_replay_window.h"
#define BUTTON_ICON_SIZE 80

/// \brief class constructor
ReplayWindow::ReplayWindow(QString *file,QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Window)
{
   ui->setupUi(this);
   this->setupUI();
}

/// \brief class destructor
ReplayWindow::~ReplayWindow()
{    
    delete ui;
}

void ReplayWindow::setupUI()
{
    home = QString::fromStdString(getenv("HOME"));
}

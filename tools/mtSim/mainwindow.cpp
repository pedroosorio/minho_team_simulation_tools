#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    initializeGUI();
    _gfx_sim_ = new RenderingCamera(ui->graphicsView);
    ui->graphicsView->setRenderingCamera(_gfx_sim_);
    readCameraConf();
    readRosServiceConf();
    _gfx_sim_->setCameraView(customviews_.at(0));
    _sim_control_ = new WorldManager();
    connect(_sim_control_,SIGNAL(new_world_stats(QString,QString,QString)),this,
                           SLOT(update_world_stats(QString,QString,QString)));
    scene = _gfx_sim_->getScene();
    minho_manager = new TeleopProcessManager(6);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::start()
{
    _gfx_sim_->startRendering();
}

void MainWindow::initializeGUI()
{
    modCtrl_ = modShift_ = false;
    ui->pushButton->setIcon(QIcon("://resources/images/play.png"));
    ui->pushButton_2->setIcon(QIcon("://resources/images/pause.png"));
    ui->pushButton_8->setIcon(QIcon("://resources/images/stop.png"));
    ui->pushButton_4->setIcon(QIcon("://resources/images/camera.png"));
    ui->pushButton_5->setIcon(QIcon("://resources/images/camera.png"));
    ui->pushButton_6->setIcon(QIcon("://resources/images/camera.png"));
    ui->pushButton_7->setIcon(QIcon("://resources/images/camera.png"));
}

void MainWindow::readCameraConf()
{
    QDomDocument conf_file;
    bool exit_error = false;
    QFile file("://resources/configs/camera_conf.xml");
    if(!file.open(QIODevice::ReadOnly)){
        exit_error = true;
    } else {
        if(!conf_file.setContent(&file)){
            exit_error = true;
        }
        file.close();
    }

    if(exit_error){
        QMessageBox asd;
        asd.setWindowTitle("Error opening camera_conf.xml");
        QString text = QString("Error opening camera_conf.xml. ")+
         QString("Check if the file was delete or is corrupted.");
        asd.setInformativeText(text);
        asd.setStandardButtons(QMessageBox::Ok);
        asd.setIconPixmap(QPixmap("://resources/images/critical.png"));
        asd.exec();
        exit(0);
    }

    //Parse XML
    QDomElement root = conf_file.firstChildElement();
    QDomNodeList _default = root.elementsByTagName("StandardCameraViews");
    QDomNodeList _custom = root.elementsByTagName("CustomCameraViews");
    QDomNodeList _defaultList = _default.at(0).toElement().elementsByTagName("view");
    QDomNodeList _customList = _custom.at(0).toElement().elementsByTagName("view");
    defaultviews_ = parseCameraViewList(_defaultList);
    customviews_ = parseCameraViewList(_customList);
}

vector<_cameraview> MainWindow::parseCameraViewList(QDomNodeList list)
{
    vector<_cameraview> _temp; _temp.clear();
    for(int i=0;i<list.count();i++) _temp.push_back(parseView(list.at(i).toElement()));
    return _temp;
}

_cameraview MainWindow::parseView(QDomElement view)
{
    _cameraview _temp;
    _temp.id = view.attribute("ID");
    _temp.g_cameraXPosition = view.attribute("positionX").toFloat();
    _temp.g_cameraYPosition = view.attribute("positionY").toFloat();
    _temp.g_cameraZPosition = view.attribute("positionZ").toFloat();
    _temp.g_cameraYawRotation = view.attribute("rotationYaw").toFloat();
    _temp.g_cameraPitchRotation = view.attribute("rotationPitch").toFloat();
    return _temp;
}

void MainWindow::readRosServiceConf()
{
    QDomDocument conf_file;
    bool exit_error = false;
    QFile file("://resources/configs/rosservice_conf.xml");
    if(!file.open(QIODevice::ReadOnly)){
        exit_error = true;
    } else {
        if(!conf_file.setContent(&file)){
            exit_error = true;
        }
        file.close();
    }

    if(exit_error){
        QMessageBox asd;
        asd.setWindowTitle("Error opening rosservice_conf.xml");
        QString text = QString("Error opening rosservice_conf.xml. ")+
         QString("Check if the file was delete or is corrupted.");
        asd.setInformativeText(text);
        asd.setStandardButtons(QMessageBox::Ok);
        asd.setIconPixmap(QPixmap("://resources/images/critical.png"));
        asd.exec();
        exit(0);
    }

    //Parse XML
    QDomElement root = conf_file.firstChildElement();
    QDomNodeList _services = root.elementsByTagName("Rosservice");
    QDomNodeList _servicesList = _services.at(0).toElement().elementsByTagName("service");
    rosservicecalls_ = parseRosServiceList(_servicesList);
}

vector<_rosservice> MainWindow::parseRosServiceList(QDomNodeList list)
{
    vector<_rosservice> _temp; _temp.clear();
    for(int i=0;i<list.count();i++) _temp.push_back(parseService(list.at(i).toElement()));
    return _temp;
}

_rosservice MainWindow::parseService(QDomElement service)
{
    _rosservice _temp;
    _temp.name = service.attribute("name");
    _temp.pre = service.attribute("pre");
    _temp.post = service.attribute("post");
    return _temp;
}

_rosservice MainWindow::getServiceCall(QString name)
{
    for(unsigned int i=0;i<rosservicecalls_.size();i++){
        if(name==rosservicecalls_[i].name){
            return rosservicecalls_[i];
        }
    }

    _rosservice _temp;
    memset(&_temp,0,sizeof(_rosservice));
    return _temp;
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    switch ( event->key() )
    {
        case Qt::Key_Control:{
            modCtrl_ = true;
        }
        case Qt::Key_Shift:{
            modShift_ = true;
        }
    }
}

void MainWindow::keyReleaseEvent(QKeyEvent *event)
{
    switch ( event->key() )
    {
        case Qt::Key_Control:{
            modCtrl_ = false;
        }
        case Qt::Key_Shift:{
            modShift_ = false;
        }
    }
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    _sim_control_->~WorldManager();
    _gfx_sim_->~RenderingCamera();
    event->accept();
}

void MainWindow::on_pushButton_4_clicked()
{
    if(!modShift_){
        _gfx_sim_->setCameraView(customviews_.at(0));
    } else _gfx_sim_->setCameraView(defaultviews_.at(0));
}

void MainWindow::on_pushButton_5_clicked()
{
    if(!modShift_){
        _gfx_sim_->setCameraView(customviews_.at(1));
    } else _gfx_sim_->setCameraView(defaultviews_.at(1));
}

void MainWindow::on_pushButton_6_clicked()
{
    if(!modShift_){
        _gfx_sim_->setCameraView(customviews_.at(2));
    } else _gfx_sim_->setCameraView(defaultviews_.at(2));
}

void MainWindow::on_pushButton_7_clicked()
{
    if(!modShift_){
        _gfx_sim_->setCameraView(customviews_.at(3));
    } else _gfx_sim_->setCameraView(defaultviews_.at(3));
}

void MainWindow::on_pushButton_clicked()
{
    _sim_control_->unpause();
}

void MainWindow::on_pushButton_2_clicked()
{
    _sim_control_->pause();
}

void MainWindow::on_pushButton_8_clicked()
{
    _sim_control_->resetSimulation();
}

void MainWindow::update_world_stats(QString pause, QString sim_time, QString real_time)
{
    ui->lb_sim_state->setText(pause);
    if(_sim_control_->isPaused())ui->lb_sim_state->setStyleSheet("QLabel { color : red; }");
    else ui->lb_sim_state->setStyleSheet("QLabel { color : green; }");
    ui->lb_sim_time->setText(sim_time);
    ui->lb_real_time->setText(real_time);
    ui->lb_fps->setText(QString::number(_gfx_sim_->getFPS()));
}

void MainWindow::on_pushButton_9_clicked()
{
    if(_gfx_sim_->isRendering()){
        _gfx_sim_->stopRendering();
    } else _gfx_sim_->startRender();
}

void MainWindow::on_tele_open_1_clicked()
{
    minho_manager->run_process(0);
}

void MainWindow::on_tele_open_2_clicked()
{
    minho_manager->run_process(1);
}

void MainWindow::on_tele_open_3_clicked()
{
    minho_manager->run_process(2);
}

void MainWindow::on_tele_open_4_clicked()
{
    minho_manager->run_process(3);
}

void MainWindow::on_tele_open_5_clicked()
{
    minho_manager->run_process(4);
}

void MainWindow::on_tele_open_6_clicked()
{
    minho_manager->run_process(5);
}

void MainWindow::on_tele_close_1_clicked()
{
    minho_manager->close_process(0);
}

void MainWindow::on_tele_close_2_clicked()
{
    minho_manager->close_process(1);
}

void MainWindow::on_tele_close_3_clicked()
{
    minho_manager->close_process(2);
}

void MainWindow::on_tele_close_4_clicked()
{
    minho_manager->close_process(3);
}

void MainWindow::on_tele_close_5_clicked()
{
    minho_manager->close_process(4);
}

void MainWindow::on_tele_close_6_clicked()
{
    minho_manager->close_process(5);
}

void MainWindow::on_put_rob_1_clicked()
{
    _sim_control_->addModel(1,"minho_robot_1");
}

void MainWindow::on_put_rob_2_clicked()
{
    _sim_control_->addModel(1,"minho_robot_2");
}

void MainWindow::on_put_rob_3_clicked()
{
    _sim_control_->addModel(1,"minho_robot_3");
}

void MainWindow::on_put_rob_4_clicked()
{
    _sim_control_->addModel(1,"minho_robot_4");
}

void MainWindow::on_put_rob_5_clicked()
{
    _sim_control_->addModel(1,"minho_robot_5");
}

void MainWindow::on_put_rob_6_clicked()
{
    _sim_control_->addModel(1,"minho_robot_6");
}

void MainWindow::on_take_rob_1_clicked()
{
    _sim_control_->removeModel("minho_robot_1");
}

void MainWindow::on_take_rob_2_clicked()
{
    _sim_control_->removeModel("minho_robot_2");
}

void MainWindow::on_take_rob_3_clicked()
{
    _sim_control_->removeModel("minho_robot_3");
}

void MainWindow::on_take_rob_4_clicked()
{
    _sim_control_->removeModel("minho_robot_4");
}

void MainWindow::on_take_rob_5_clicked()
{
    _sim_control_->removeModel("minho_robot_5");
}

void MainWindow::on_take_rob_6_clicked()
{
    _sim_control_->removeModel("minho_robot_6");
}

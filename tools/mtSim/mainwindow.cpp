#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(bool run, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    running = run;
    // INITALIZE BASE SYSTEM AND GAZEBO SERVER
    // ############################################
    if(run){
        if(isROSRunning()){qDebug() << "[Msg] ROSCORE already running ...";}
        else { killROS(); startROS(); }

        if(isGazeboRunning()){killGazebo(); qDebug() << "[Msg] GZSERVER already running. Killing running instances ...";  startGazebo();}
        else { killGazebo(); startGazebo(); }
    }
    // ############################################
    initializeGUI();
    _gfx_sim_ = NULL;
    _sim_control_ = NULL;
    minho_manager = NULL;
    if(run){
        _gfx_sim_ = new RenderingCamera(ui->graphicsView);
        ui->graphicsView->setRenderingCamera(_gfx_sim_);
        readCameraConf();
        readRosServiceConf();
        _gfx_sim_->setCameraView(customviews_.at(0));
        _sim_control_ = new WorldManager();
        connect(_sim_control_,SIGNAL(new_world_stats(QString,QString,QString)),this,
                               SLOT(update_world_stats(QString,QString,QString)));
        connect(_sim_control_,SIGNAL(new_poses(std::vector<gazebo::msgs::Pose>)),this,
                               SLOT(update_model_poses(std::vector<gazebo::msgs::Pose>)));
        scene = _gfx_sim_->getScene();
        minho_manager = new TeleopProcessManager(6);
    }
    ui->tabWidget->setCurrentIndex(0);
}

MainWindow::~MainWindow()
{
    delete ui;
}

bool MainWindow::isROSRunning()
{
    QProcess ros_finder;
    QString program = "pgrep";
    QStringList args;
    args.push_back("-c");
    args.push_back("ros");
    ros_finder.start(program,args);
    ros_finder.waitForFinished(1000);

    QString ret = ros_finder.readAll();
    if(ret.size()!=2) return false;

    ret.chop(1);
    int n_processes = ret.toInt();

    if(n_processes==3) return true;
    else return false;
}

bool MainWindow::isGazeboRunning()
{
    QProcess gazebo_finder;
    QString program = "pgrep";
    QStringList args;
    args.push_back("-c");
    args.push_back("gzserver");
    gazebo_finder.start(program,args);
    gazebo_finder.waitForFinished(1000);

    QString ret = gazebo_finder.readAll();
    if(ret.size()!=2) return false;

    ret.chop(1);
    int n_processes = ret.toInt();

    if(n_processes>=1) return true;
    else return false;
}

void MainWindow::killROS()
{
    QProcess ros_killer;
    QString program = "pkill";
    QStringList args;
    args.push_back("-f");
    args.push_back("ros");
    ros_killer.start(program,args);
    ros_killer.waitForFinished(1000);
}

void MainWindow::startROS()
{
    QString program = "roscore";
    QStringList args;
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    ros_runner.setProcessEnvironment(env);
    qDebug() << "[Msg] Starting ROSCORE ...";
    connect(&ros_runner,SIGNAL(finished(int,QProcess::ExitStatus)),this,SLOT(rosKilled(int,QProcess::ExitStatus)));
    ros_runner.start(program,args);
}

void MainWindow::rosKilled(int exitCode, QProcess::ExitStatus exitStatus)
{
    qDebug() << "[WARN] ROSCORE killed : " << exitStatus << "(" << exitCode << ")";
}

void MainWindow::killGazebo()
{
    QProcess gzserver_killer;
    QString program = "pkill";
    QStringList args;
    args.push_back("-f");
    args.push_back("gzserver");
    gzserver_killer.start(program,args);
    gzserver_killer.waitForFinished(1000);

    QProcess gzclient_killer;
    program = "pkill";
    args.clear();
    args.push_back("-f");
    args.push_back("gzclient");
    gzclient_killer.start(program,args);
    gzclient_killer.waitForFinished(1000);
}

void MainWindow::startGazebo()
{
    QString program = "rosrun";
    QStringList args;
    args.push_back("gazebo_ros");
    args.push_back("gzserver");
    args.push_back("msl.world"); //Default world
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    QString home = env.value("HOME");
    QString gz_rsc_path = ":/usr/share/gazebo-7";
    gz_rsc_path+=":"+home+"/catkin_ws/src/minho_team_simulation_tools/worlds";
    if(!env.contains("GAZEBO_RESOURCE_PATH")) env.insert("GAZEBO_RESOURCE_PATH",gz_rsc_path);
    gazebo_runner.setProcessEnvironment(env);
    qDebug() << "[Msg] Starting GZSERVER ...";
    connect(&gazebo_runner,SIGNAL(finished(int,QProcess::ExitStatus)),this,SLOT(gazeboKilled(int,QProcess::ExitStatus)));
    gazebo_runner.start(program,args);
    gazebo_runner.waitForStarted(10000);
}

void MainWindow::gazeboKilled(int exitCode, QProcess::ExitStatus exitStatus)
{
    qDebug() << "[WARN] GZSERVER killed : " << exitStatus << "(" << exitCode << ")";
}

void MainWindow::start()
{
    if(running)_gfx_sim_->startRendering();
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
    ui->tableWidget->setColumnCount(4);
    ui->tableWidget->setHorizontalHeaderLabels(QStringList() << tr("Model Name")
                                                                   << tr("Position X")
                                                                   << tr("Position Y")
                                                                   << tr("Orientation"));
    ui->tableWidget->verticalHeader()->setVisible(false);
    connect(ui->actionField_Builder,SIGNAL(triggered(bool)),this,SLOT(launch_field_builder()));
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
            break;
        }
        case Qt::Key_Shift:{
            modShift_ = true;
            break;
        }
        // Shortcuts for robot management
        case Qt::Key_F1:{
            if(event->modifiers()&&Qt::ControlModifier){ //Put robot in/out
                if(_sim_control_ && _sim_control_->isModelInWorld("minho_robot_1")){
                    on_take_rob_1_clicked();
                } else if(_sim_control_) on_put_rob_1_clicked();
            } else {
                if(minho_manager && minho_manager->isProcessRunning(0)){ //Teleop on/off
                    minho_manager->close_process(0);
                } else if(minho_manager) minho_manager->run_process(0);
            }
            break;
        }
        case Qt::Key_F2:{
            if(event->modifiers()&&Qt::ControlModifier){ //Put robot in/out
                if(_sim_control_ && _sim_control_->isModelInWorld("minho_robot_2")){
                    on_take_rob_2_clicked();
                } else on_put_rob_2_clicked();
            } else {
                if(minho_manager && minho_manager->isProcessRunning(1)){ //Teleop on/off
                    minho_manager->close_process(1);
                } else if(minho_manager) minho_manager->run_process(1);
            } break;
        }
        case Qt::Key_F3:{
            if(event->modifiers()&&Qt::ControlModifier){ //Put robot in/out
                if(_sim_control_ && _sim_control_->isModelInWorld("minho_robot_3")){
                    on_take_rob_3_clicked();
                } else on_put_rob_3_clicked();
            } else {
                if(minho_manager && minho_manager->isProcessRunning(2)){ //Teleop on/off
                    minho_manager->close_process(2);
                } else if(minho_manager) minho_manager->run_process(2);
            }
            break;
        }
        case Qt::Key_F4:{
            if(event->modifiers()&&Qt::ControlModifier){ //Put robot in/out
                if(_sim_control_ && _sim_control_->isModelInWorld("minho_robot_4")){
                    on_take_rob_4_clicked();
                } else on_put_rob_4_clicked();
            } else {
                if(minho_manager && minho_manager->isProcessRunning(3)){ //Teleop on/off
                    minho_manager->close_process(3);
                } else if(minho_manager) minho_manager->run_process(3);
            }
            break;
        }
        case Qt::Key_F5:{
            if(event->modifiers()&&Qt::ControlModifier){ //Put robot in/out
                if(_sim_control_ && _sim_control_->isModelInWorld("minho_robot_5")){
                    on_take_rob_5_clicked();
                } else on_put_rob_5_clicked();
            } else {
                if(minho_manager && minho_manager->isProcessRunning(4)){ //Teleop on/off
                    minho_manager->close_process(4);
                } else if(minho_manager) minho_manager->run_process(4);
            }
            break;
        }
        case Qt::Key_F6:{
            if(event->modifiers()&&Qt::ControlModifier){ //Put robot in/out
                if(_sim_control_ && _sim_control_->isModelInWorld("minho_robot_6")){
                    on_take_rob_6_clicked();
                } else on_put_rob_6_clicked();
            } else {
                if(minho_manager && minho_manager->isProcessRunning(5)){ //Teleop on/off
                    minho_manager->close_process(5);
                } else if(minho_manager) minho_manager->run_process(5);
            }
            break;
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
    disconnect(&ros_runner,SIGNAL(finished(int,QProcess::ExitStatus)),this,SLOT(rosKilled(int,QProcess::ExitStatus)));
    disconnect(&gazebo_runner,SIGNAL(finished(int,QProcess::ExitStatus)),this,SLOT(gazeboKilled(int,QProcess::ExitStatus)));
    ros_runner.terminate();
    gazebo_runner.terminate();
    while(ros_runner.state()!=QProcess::NotRunning && gazebo_runner.state()!=QProcess::NotRunning);
    if(_sim_control_)_sim_control_->~WorldManager();
    if(_gfx_sim_)_gfx_sim_->~RenderingCamera();
    event->accept();
}

void MainWindow::on_pushButton_4_clicked()
{
    if(_gfx_sim_){
        if(!modShift_){
            _gfx_sim_->setCameraView(customviews_.at(0));
        } else _gfx_sim_->setCameraView(defaultviews_.at(0));
    }
}

void MainWindow::on_pushButton_5_clicked()
{
    if(_gfx_sim_){
        if(!modShift_){
            _gfx_sim_->setCameraView(customviews_.at(1));
        } else _gfx_sim_->setCameraView(defaultviews_.at(1));
    }
}

void MainWindow::on_pushButton_6_clicked()
{
    if(_gfx_sim_){
        if(!modShift_){
            _gfx_sim_->setCameraView(customviews_.at(2));
        } else _gfx_sim_->setCameraView(defaultviews_.at(2));
    }
}

void MainWindow::on_pushButton_7_clicked()
{
    if(_gfx_sim_){
        if(!modShift_){
            _gfx_sim_->setCameraView(customviews_.at(3));
        } else _gfx_sim_->setCameraView(defaultviews_.at(3));
    }
}

void MainWindow::on_pushButton_clicked()
{
    if(_sim_control_) _sim_control_->unpause();
}

void MainWindow::on_pushButton_2_clicked()
{
    if(_sim_control_) _sim_control_->pause();
}

void MainWindow::on_pushButton_8_clicked()
{
   if(_sim_control_) _sim_control_->resetSimulation();
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

void MainWindow::update_model_poses(std::vector<gazebo::msgs::Pose> poses)
{
    //TODO: Fix orientation
    ui->tableWidget->clearContents();
    ui->tableWidget->setRowCount(poses.size());
    gazebo::msgs::Vector3d position;
    for(unsigned int i = 0; i<poses.size(); i++){
        position = poses[i].position();
        ui->tableWidget->setItem(i,0,new QTableWidgetItem(QString::fromStdString(poses[i].name())));
        ui->tableWidget->setItem(i,1,new QTableWidgetItem(QString::number(position.x(),'f',3)));
        ui->tableWidget->setItem(i,2,new QTableWidgetItem(QString::number(position.y(),'f',3)));
        ui->tableWidget->setItem(i,3,new QTableWidgetItem(QString::number(poses[i].orientation().z(),'f',3)));
    }
}

void MainWindow::on_pushButton_9_clicked()
{
    if(_gfx_sim_){
        if(_gfx_sim_->isRendering()){
            _gfx_sim_->stopRendering();
        } else _gfx_sim_->startRender();
    }
}

void MainWindow::on_tele_open_1_clicked()
{
    if(minho_manager) minho_manager->run_process(0);
}

void MainWindow::on_tele_open_2_clicked()
{
    if(minho_manager) minho_manager->run_process(1);
}

void MainWindow::on_tele_open_3_clicked()
{
    if(minho_manager) minho_manager->run_process(2);
}

void MainWindow::on_tele_open_4_clicked()
{
    if(minho_manager) minho_manager->run_process(3);
}

void MainWindow::on_tele_open_5_clicked()
{
    if(minho_manager) minho_manager->run_process(4);
}

void MainWindow::on_tele_open_6_clicked()
{
    if(minho_manager) minho_manager->run_process(5);
}

void MainWindow::on_tele_close_1_clicked()
{
    if(minho_manager) minho_manager->close_process(0);
}

void MainWindow::on_tele_close_2_clicked()
{
    if(minho_manager) minho_manager->close_process(1);
}

void MainWindow::on_tele_close_3_clicked()
{
    if(minho_manager) minho_manager->close_process(2);
}

void MainWindow::on_tele_close_4_clicked()
{
    if(minho_manager) minho_manager->close_process(3);
}

void MainWindow::on_tele_close_5_clicked()
{
    if(minho_manager) minho_manager->close_process(4);
}

void MainWindow::on_tele_close_6_clicked()
{
    if(minho_manager) minho_manager->close_process(5);
}

void MainWindow::on_put_rob_1_clicked()
{
    if(_sim_control_) _sim_control_->addModel(1,"minho_robot_1");
}

void MainWindow::on_put_rob_2_clicked()
{
    if(_sim_control_) _sim_control_->addModel(1,"minho_robot_2");
}

void MainWindow::on_put_rob_3_clicked()
{
    if(_sim_control_) _sim_control_->addModel(1,"minho_robot_3");
}

void MainWindow::on_put_rob_4_clicked()
{
    if(_sim_control_) _sim_control_->addModel(1,"minho_robot_4");
}

void MainWindow::on_put_rob_5_clicked()
{
    if(_sim_control_) _sim_control_->addModel(1,"minho_robot_5");
}

void MainWindow::on_put_rob_6_clicked()
{
    if(_sim_control_) _sim_control_->addModel(1,"minho_robot_6");
}

void MainWindow::on_take_rob_1_clicked()
{
    if(_sim_control_) _sim_control_->removeModel("minho_robot_1");
}

void MainWindow::on_take_rob_2_clicked()
{
    if(_sim_control_) _sim_control_->removeModel("minho_robot_2");
}

void MainWindow::on_take_rob_3_clicked()
{
   if(_sim_control_) _sim_control_->removeModel("minho_robot_3");
}

void MainWindow::on_take_rob_4_clicked()
{
   if(_sim_control_) _sim_control_->removeModel("minho_robot_4");
}

void MainWindow::on_take_rob_5_clicked()
{
    if(_sim_control_) _sim_control_->removeModel("minho_robot_5");
}

void MainWindow::on_take_rob_6_clicked()
{
    if(_sim_control_) _sim_control_->removeModel("minho_robot_6");
}

void MainWindow::launch_field_builder()
{
    field_builder = new FieldBuilderDialog();
    field_builder->show();
}

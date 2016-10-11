#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "customscene.h"
#include "desenharcampo.h"
#include "robo.h"
#include "ball.h"
#include "math.h"

#define SGN(x) ((x)>=0?1:-1)

#define official
//#define lar
#define RTDBREADTIME 17

#define OFFLINE

const int numberRobots = 5;

Ball *ball[numberRobots];
Robo *robo[numberRobots*2];
Ball *mainBall;

bool limitUp = false;
int roboPass = 1;

struct nodo
{
    float x[numberRobots];
    float y[numberRobots];
    float angle[numberRobots];
    char name[100];
};


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    nTaticsLoad = 0;

    minhoGoals=0; awayGoals=0,robotWithBall = 0;

    check_show_home_robots = true,check_show_away_robots = true,check_show_grid_cells = false
                ,check_show_pass_lines = false,check_show_shot_lines = false,check_show_obstacles = false
                ,check_game_direction = false,check_RTDB = false,check_main_loop=true;

    we_have_ball = false,goalKickMinhoSearch = false;

    KickMinhoRobotPasser=0,KickMinhoRobotReceiver=0;

    passerXpoint = 0,passerXpoint_old = 0,passerYpoint = 0,passerYpoint_old = 0;

    limitUpK = false;

    for(int i=0;i<5;i++){
        robo_state[i] = false;
        robo_state_old[i] = false;
    }

    int id = QFontDatabase::addApplicationFont(":display.ttf");
    QString family = QFontDatabase::applicationFontFamilies(id).at(0);
    QFont monospace(family);
    monospace.setPixelSize(80);
    ui->lb_home_score->setFont(monospace);
    ui->lb_away_score->setFont(monospace);

    ui->bt_show_away_robots->installEventFilter(this);
    ui->bt_show_home_robots->installEventFilter(this);
    ui->bt_show_grid_cells->installEventFilter(this);
    ui->bt_show_pass_lines->installEventFilter(this);
    ui->bt_show_shot_lines->installEventFilter(this);
    ui->bt_show_obstacles->installEventFilter(this);
    ui->bt_game_direction->installEventFilter(this);
    ui->bt_rtdb_state->installEventFilter(this);
    ui->bt_main_loop_state->installEventFilter(this);


    setRobotLable(ui->state_robo1, false, "1");
    setRobotLable(ui->state_robo2, false, "2");
    setRobotLable(ui->state_robo3, false, "3");
    setRobotLable(ui->state_robo4, false, "4");
    setRobotLable(ui->state_robo5, false, "5");


    // Default Definitions
    TOTAL_LENGTH=20000;
    TOTAL_WIDTH=14000;
    LENGTH=18000;
    WIDTH=12000;
    GOAL_WIDTH=2000;
    GOAL_LENGTH=500;
    LINE_WIDTH=120;
    CENTER_RADIUS=2000;
    SPOT_CENTER=150;
    SPOTS=100;
    AREA_LENGTH1=750;
    AREA_WIDTH1= GOAL_WIDTH + 1500;
    AREA_LENGTH2=2250;
    AREA_WIDTH2= GOAL_WIDTH + 4500;
    DISTANCE_PENALTY=3000;
    RADIUS_CORNER=750;
    ROBOT_DIAMETER=500;
    BALL_DIAMETER = 220;
    FREE_KICK_SPACE = 3000;
    FACTOR=20;// 1 pixel = 2 cm

    #ifdef lar
    readDimension("campo-lar.txt");
    #endif

    #ifdef official
    readDimension("campo-oficial.txt");
    #endif

    QGraphicsView *fieldView = ui->fieldView;
    fieldView->setFixedSize(TOTAL_LENGTH/FACTOR,TOTAL_WIDTH/FACTOR);
    fieldView->setRenderHint(QPainter::Antialiasing);

    scene = new CustomScene(this);

    scene->setSceneRect((-TOTAL_LENGTH/2)/FACTOR,(-TOTAL_WIDTH/2)/FACTOR,(TOTAL_LENGTH)/FACTOR,(TOTAL_WIDTH)/FACTOR);

    QBrush brush(QColor(0,150,0));
    QPen pen;
    pen.setColor(QColor (255,255,255));
    pen.setWidth(1);

    scene->addRect((-TOTAL_LENGTH/2)/FACTOR,(-TOTAL_WIDTH/2)/FACTOR,(TOTAL_LENGTH)/FACTOR,(TOTAL_WIDTH)/FACTOR,pen,brush);

    fieldView->setScene(scene);

    DesenharCampo *campo = new DesenharCampo();
    campo->desenharLinhas(scene,LENGTH,WIDTH,GOAL_WIDTH,GOAL_LENGTH,LINE_WIDTH,CENTER_RADIUS,SPOT_CENTER,SPOTS,AREA_LENGTH1,AREA_WIDTH1,AREA_LENGTH2,AREA_WIDTH2,DISTANCE_PENALTY,RADIUS_CORNER,FACTOR);

    //Pass Line;
    pen_line.setWidth(2);
    pen_line.setColor(QColor(0,255,0));
    pass_line = new QGraphicsLineItem();
    pass_line->setPen(pen_line);

    //Goal Circle
    pen_line.setColor(QColor(255,0,0));
    shot_circle = new QGraphicsEllipseItem();
    shot_circle->setPen(pen_line);
    static_goal_taget_x = ((LENGTH/2)/FACTOR);
    goal_target_y = -5;
    shot_circle->setRect(static_goal_taget_x,goal_target_y,10,10);

    //Goal Line;
    pen_line.setWidth(2);
    pen_line.setColor(QColor(0,255,0));
    shot_line = new QGraphicsLineItem();
    shot_line->setPen(pen_line);

    for(int i=5;i<10;i++)
    {
        robo[i] = new Robo(this,i+1);
        robo[i]->setNumber(QString::number(i-4));
        robo[i]->setData(0,i);
        #ifdef official
        robo[i]->setPosition(getx(1.5)+((i-4)*(ROBOT_DIAMETER*2/FACTOR)),gety(6.2));
        #endif
        #ifdef lar
        robo[i]->setPosition(getx(0.5)+((i-4)*(ROBOT_DIAMETER*1.5/FACTOR)),gety(4));
        #endif
        robotOldPositionsx[i] = robo[i]->pos().x();
        robotOldPositionsy[i] = robo[i]->pos().y();
        robo[i]->setAngle(0);
        robo[i]->setColor(0,0,100);
        scene->addItem(robo[i]);
    }

    for(int i=0;i<5;i++)
    {
        robo[i] = new Robo(this,i+1);
        robo[i]->setNumber(QString::number(i+1));
        robo[i]->setData(0,i);
        #ifdef official
        robo[i]->setPosition(getx(-7)+(i*(ROBOT_DIAMETER*2/FACTOR)),gety(6.2));
        #endif
        #ifdef lar
        robo[i]->setPosition(getx(-1.5)+((i-4)*(ROBOT_DIAMETER*1.5/FACTOR)),gety(4));
        #endif
        robotOldPositionsx[i] = robo[i]->pos().x();
        robotOldPositionsy[i] = robo[i]->pos().y();
        robo[i]->setAngle(0);
        robo[i]->setColor(120,10,15);
        scene->addItem(robo[i]);
    }


    for(int i=0;i<5;i++)
    {
        ball[i] = new Ball(this,i+1,BALL_DIAMETER);
        ball[i]->setNumber(QString::number(i+1));
        ball[i]->setData(0,i);
        ball[i]->setPos(getx(-7)+(i*(ROBOT_DIAMETER*2/FACTOR)),gety(6.2));
        scene->addItem(ball[i]);
    }

    mainBall = new Ball(this,6,BALL_DIAMETER);
    mainBall->setNumber("M");
    mainBall->setData(0,6);
    mainBall->setColor(255,0,0);
    mainBall->setPos(getx(-7)+(5*(ROBOT_DIAMETER*2/FACTOR)),gety(6.2));
    scene->addItem(mainBall);



    timerPassLine = new QTimer(this);
    connect(timerPassLine, SIGNAL(timeout()), this, SLOT(updatePassLine()));
    timerPassLine->start(10);

    LoadTatics();

    mysocket = new QTcpSocket();
    mysocket->connectToHost(QHostAddress("127.0.0.1"),28097);
    //mysocket->connectToHost(QHostAddress("172.16.1.2"),28097);
    connect(mysocket,SIGNAL(readyRead()),this,SLOT(readPendingDatagrams()));

    rtdb = new RTDBmainclass();

    timerRTDB = new QTimer(this);
    connect(timerRTDB, SIGNAL(timeout()), this, SLOT(updateRTDB()));
    //timerRTDB->start(RTDBREADTIME);

    timerRef = new QTimer(this);
    connect(timerRef,SIGNAL(timeout()),this,SLOT(senRef()));
    //1timerRef->start(100);//10fps

    timerMainLoop = new QTimer(this);
    connect(timerMainLoop, SIGNAL(timeout()), this, SLOT(updateMainLoop()));
    timerMainLoop->start(30);

    setRobotStop();

    rtdb->set_var_local(13,1,0); //Set Game Direction 0


}

MainWindow::~MainWindow()
{
    delete ui;
}

int MainWindow::getFactor()
{
    return FACTOR;
}

int MainWindow::getRoboTSize()
{
    return ROBOT_DIAMETER;
}

void MainWindow::readDimension(QString FileName)
{
    QFile file("../TestWidgetsLocal/data/"+FileName);

    if(!file.open(QIODevice::ReadOnly)) {
        QMessageBox::warning(0, "Error", file.errorString());
    }
    QTextStream in(&file);
    QString line;
    int lineN = 0;
    while(!in.atEnd()) {
        line = in.readLine();

        switch (lineN) {
        case 0:
            line = line.replace("TOTAL_LENGTH=",0);
            TOTAL_LENGTH = line.toInt();
        break;
        case 1:
            line = line.replace("TOTAL_WIDTH=",0);
            TOTAL_WIDTH = line.toInt();
        break;
        case 2:
            line = line.replace("LENGTH=",0);
            LENGTH = line.toInt();
        break;
        case 3:
            line = line.replace("WIDTH=",0);
            WIDTH = line.toInt();
        break;
        case 4:
            line = line.replace("GOAL_WIDTH=",0);
            GOAL_WIDTH = line.toInt();
        break;
        case 5:
            line = line.replace("GOAL_LENGTH=",0);
            GOAL_LENGTH = line.toInt();
        break;
        case 6:
            line = line.replace("LINE_WIDTH=",0);
            LINE_WIDTH = line.toInt();
        break;
        case 7:
            line = line.replace("CENTER_RADIUS=",0);
            CENTER_RADIUS = line.toInt();
        break;
        case 8:
            line = line.replace("SPOT_CENTER=",0);
            SPOT_CENTER = line.toInt();
        break;
        case 9:
            line = line.replace("SPOTS=",0);
            SPOTS = line.toInt();
        break;
        case 10:
            line = line.replace("AREA_LENGTH1=",0);
            AREA_LENGTH1 = line.toInt();
        break;
        case 11:
            line = line.replace("AREA_WIDTH1=",0);
            AREA_WIDTH1 = line.toInt();
        break;
        case 12:
            line = line.replace("AREA_LENGTH2=",0);
            AREA_LENGTH2 = line.toInt();
        break;
        case 13:
            line = line.replace("AREA_WIDTH2=",0);
            AREA_WIDTH2 = line.toInt();
        break;
        case 14:
            line = line.replace("DISTANCE_PENALTY=",0);
            DISTANCE_PENALTY = line.toInt();
        break;
        case 15:
            line = line.replace("RADIUS_CORNER=",0);
            RADIUS_CORNER = line.toInt();
        break;
        case 16:
            line = line.replace("ROBOT_DIAMETER=",0);
            ROBOT_DIAMETER = line.toInt();
        break;
        case 17:
            line = line.replace("BALL_DIAMETER=",0);
            BALL_DIAMETER = line.toInt();
        break;
        case 18:
            line = line.replace("FREE_KICK_SPACE=",0);
            FREE_KICK_SPACE = line.toInt();
            break;
        case 19:
            line = line.replace("FACTOR=",0);
            FACTOR = line.toInt();
        break;
        default:
            break;
        }
        lineN++;
    }

}

int MainWindow::getx(float meterX)
{
    return meterX*1000/FACTOR;
}

int MainWindow::gety(float meterY)
{
    return meterY*1000/FACTOR;
}

float MainWindow::getMeterx(int X)
{
    return (float)X*(float)FACTOR/1000.0;
}

float MainWindow::getMetery(int Y)
{
    return (float)Y*(float)FACTOR/1000.0;
}


bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
  if (event->type() == QEvent::HoverEnter) HoverEnter(obj->objectName());
  else if (event->type() == QEvent::HoverLeave) HoverLeave(obj->objectName());
  return false;
}

void MainWindow::HoverEnter(QString Name)
{
    QApplication::setOverrideCursor(Qt::PointingHandCursor);

    if(Name=="bt_show_home_robots"){
        SetOpacity(ui->bt_show_home_robots,0.5);
    }
    else if(Name=="bt_show_away_robots"){
        SetOpacity(ui->bt_show_away_robots,0.5);
    }
    else if(Name=="bt_show_grid_cells"){
        SetOpacity(ui->bt_show_grid_cells,0.5);
    }
    else if(Name=="bt_show_pass_lines"){
        SetOpacity(ui->bt_show_pass_lines,0.5);
    }
    else if(Name=="bt_show_shot_lines"){
        SetOpacity(ui->bt_show_shot_lines,0.5);
    }
    else if(Name=="bt_show_obstacles"){
        SetOpacity(ui->bt_show_obstacles,0.5);
    }
    else if(Name=="bt_game_direction"){
        SetOpacity(ui->bt_game_direction,0.5);
    }
    else if(Name=="bt_rtdb_state"){
        SetOpacity(ui->bt_rtdb_state,0.5);
    }
    else if(Name=="bt_main_loop_state"){
        SetOpacity(ui->bt_main_loop_state,0.5);
    }
}

void MainWindow::HoverLeave(QString Name)
{
    QApplication::restoreOverrideCursor();
    if(Name=="bt_show_home_robots"){
        SetOpacity(ui->bt_show_home_robots,1);
    }
    else if(Name=="bt_show_away_robots"){
        SetOpacity(ui->bt_show_away_robots,1);
    }
    else if(Name=="bt_show_grid_cells"){
        SetOpacity(ui->bt_show_grid_cells,1);
    }
    else if(Name=="bt_show_pass_lines"){
        SetOpacity(ui->bt_show_pass_lines,1);
    }
    else if(Name=="bt_show_shot_lines"){
        SetOpacity(ui->bt_show_shot_lines,1);
    }
    else if(Name=="bt_show_obstacles"){
        SetOpacity(ui->bt_show_obstacles,1);
    }
    else if(Name=="bt_game_direction"){
        SetOpacity(ui->bt_game_direction,1);
    }
    else if(Name=="bt_rtdb_state"){
        SetOpacity(ui->bt_rtdb_state,1);
    }
    else if(Name=="bt_main_loop_state"){
        SetOpacity(ui->bt_main_loop_state,1);
    }
}

void MainWindow::SetOpacity(QPushButton *button,float value)
{
    QGraphicsOpacityEffect * effect = new QGraphicsOpacityEffect(button);
    effect->setOpacity(value);
    button->setGraphicsEffect(effect);
}

void MainWindow::on_bt_show_home_robots_clicked()
{
    if(!check_show_home_robots)
    {
       ui->bt_show_home_robots->setStyleSheet("QPushButton{border-image: url(:/on.png) 0 0 0 0 stretch stretch;}");
       for(int i=0;i<5;i++) setOnField(i);
       check_show_home_robots = true;
    }
    else
    {
       ui->bt_show_home_robots->setStyleSheet("QPushButton{border-image: url(:/off.png) 0 0 0 0 stretch stretch;}");
       for(int i=0;i<5;i++) removeFromField(i);
       check_show_home_robots = false;
    }
}

void MainWindow::on_bt_show_away_robots_clicked()
{
    if(!check_show_away_robots)
    {
       ui->bt_show_away_robots->setStyleSheet("QPushButton{border-image: url(:/on.png) 0 0 0 0 stretch stretch;}");
       for(int i=5;i<10;i++) setOnField(i);
       check_show_away_robots = true;
    }
    else
    {
       ui->bt_show_away_robots->setStyleSheet("QPushButton{border-image: url(:/off.png) 0 0 0 0 stretch stretch;}");
       for(int i=5;i<10;i++) removeFromField(i);
       check_show_away_robots = false;
    }
}

void MainWindow::on_bt_show_grid_cells_clicked()
{
    if(!check_show_grid_cells)
    {
       ui->bt_show_grid_cells->setStyleSheet("QPushButton{border-image: url(:/on.png) 0 0 0 0 stretch stretch;}");
       check_show_grid_cells = true;
    }
    else
    {
       ui->bt_show_grid_cells->setStyleSheet("QPushButton{border-image: url(:/off.png) 0 0 0 0 stretch stretch;}");
       check_show_grid_cells = false;
    }
}

void MainWindow::on_bt_show_pass_lines_clicked()
{
    if(!check_show_pass_lines)
    {
       ui->bt_show_pass_lines->setStyleSheet("QPushButton{border-image: url(:/on.png) 0 0 0 0 stretch stretch;}");
       check_show_pass_lines = true;
    }
    else
    {
       ui->bt_show_pass_lines->setStyleSheet("QPushButton{border-image: url(:/off.png) 0 0 0 0 stretch stretch;}");
       scene->removeItem(pass_line);
       check_show_pass_lines = false;
    }

}

void MainWindow::on_bt_show_shot_lines_clicked()
{
    if(!check_show_shot_lines)
    {
       ui->bt_show_shot_lines->setStyleSheet("QPushButton{border-image: url(:/on.png) 0 0 0 0 stretch stretch;}");
       check_show_shot_lines = true;
    }
    else
    {
       ui->bt_show_shot_lines->setStyleSheet("QPushButton{border-image: url(:/off.png) 0 0 0 0 stretch stretch;}");
       scene->removeItem(shot_circle);
       scene->removeItem(shot_line);
       check_show_shot_lines = false;
    }
}

void MainWindow::on_bt_show_obstacles_clicked()
{
    if(!check_show_obstacles)
    {
       ui->bt_show_obstacles->setStyleSheet("QPushButton{border-image: url(:/on.png) 0 0 0 0 stretch stretch;}");
       check_show_obstacles = true;
    }
    else
    {
       ui->bt_show_obstacles->setStyleSheet("QPushButton{border-image: url(:/off.png) 0 0 0 0 stretch stretch;}");
       check_show_obstacles = false;
    }
}

void MainWindow::drawLine(int x1, int y1, int x2, int y2, QGraphicsLineItem *line)
{
    scene->removeItem(line);
    line->setLine(x1,y1,x2,y2);
    scene->addItem(line);
}


void MainWindow::removeFromField(int number)
{
    robotOldPositionsx[number] = robo[number]->pos().x();
    robotOldPositionsy[number] = robo[number]->pos().y();
    robo[number]->setPosition(50000,50000);
    scene->removeItem(robo[number]);
}

void MainWindow::setOnField(int number)
{
    robo[number]->setPosition(robotOldPositionsx[number],robotOldPositionsy[number]);
    scene->addItem(robo[number]);
}

void MainWindow::readPendingDatagrams()
{
    float ballx = getMeterx(mainBall->pos().x()+mainBall->width/2);

    float bally = getMetery(mainBall->pos().y()+mainBall->width/2);

    vector <int> robo_roleOn;


    int OnlineRobots = 0;

    #ifdef OFFLINE
    OnlineRobots = numberRobots;
    for(int i=0;i<OnlineRobots;i++){
            robo_roleOn.push_back(i);
    }

    #endif

    #ifndef OFFLINE
    for(int i=0;i<numberRobots;i++){
        if(robo_state[i]){
            robo_roleOn.push_back(i);
        }
    }
    OnlineRobots = robo_roleOn.size();
    #endif

    QString read = mysocket->readAll();


    if(read==(QString)"S"){
        rtdb->set_var_local(10,1,0); // Stop
        setRobotStop();
    }
    else if(read==(QString)"s"){
        rtdb->set_var_local(10,1,1); // start
        setRobotStart();
    }
    else if(read==(QString)"K"){
        rtdb->set_var_local(10,1,2); //MinhoKickOff

        QString fileLoad = "MinhoKickOff";
        if(check_game_direction)fileLoad+="R";
        else fileLoad+="L";
        for(int i=0;i<nTaticsLoad;i++){
            QString name = robotNode[i].name;
            if(name == fileLoad) setRobotPositions(i);
        }

        int positionNow = 0;
        for(int i=0;i<numberRobots;i++){
            if(robo_state[i]){
                switch (positionNow) {
                case 0:
                    setGuardaRedesRTDB(i);
                    break;
                case 1:
                    setPassadorRTDB(i);
                    break;
                case 2:
                    setRecetorRTDB(i);
                    break;
                case 3:
                    setDefensorRTDB(i);
                    break;
                case 4:
                    setDefensorRTDB(i);
                    break;
                default:
                    break;
                }
                positionNow++;
            }
        }
    }
    else if(read==(QString)"k"){
        rtdb->set_var_local(10,1,3);//AwayKickOff

        QString fileLoad = "AwayKickOff";
        if(check_game_direction)fileLoad+="R";
        else fileLoad+="L";
        for(int i=0;i<nTaticsLoad;i++){
            QString name = robotNode[i].name;
            if(name == fileLoad) setRobotPositions(i);
        }

        int positionNow = 0;
        for(int i=0;i<numberRobots;i++){
            if(robo_state[i]){
                switch (positionNow) {
                case 0:
                    setGuardaRedesRTDB(i);
                    break;
                case 1:
                    setDefensorRTDB(i);
                    break;
                case 2:
                    setDefensorRTDB(i);
                    break;
                case 3:
                    setDefensorRTDB(i);
                    break;
                case 4:
                    setDefensorRTDB(i);
                    break;
                default:
                    break;
                }
                positionNow++;
            }
        }
    }
    else if(read==(QString)"L"){
        rtdb->set_var_local(10,1,4); //Park

            QString fileLoad = "Park";
            if(check_game_direction)fileLoad+="R";
            else fileLoad+="L";
            for(int i=0;i<nTaticsLoad;i++){
                QString name = robotNode[i].name;
                if(name == fileLoad) setRobotPositions(i);
            }
    }
    else if(read==(QString)"A"){
        rtdb->set_var_local(10,1,5);//Goal Minho
            minhoGoals++;
            ui->lb_home_score->setText(QString::number(minhoGoals));
        }
    else if(read==(QString)"a"){
        rtdb->set_var_local(10,1,6);//No Goal Minho
        awayGoals++;
        ui->lb_away_score->setText(QString::number(awayGoals));
    }
    else if(read==(QString)"D"){
        rtdb->set_var_local(10,1,7);//Goal Away

            minhoGoals--;
            ui->lb_home_score->setText(QString::number(minhoGoals));
        }
    else if(read==(QString)"d"){
        rtdb->set_var_local(10,1,8);//No Goal Away

        awayGoals--;
        ui->lb_away_score->setText(QString::number(awayGoals));
    }
    else if(read==(QString)"F"){
        rtdb->set_var_local(10,1,9);// Free Kick Minho

        switch (OnlineRobots) {
        case 5:
        setGoalKeeper(robo_roleOn.at(0));
        FreeKick(ballx, bally, robo_roleOn.at(1), robo_roleOn.at(2),0.5,2.0);
        setDefense2(robo_roleOn.at(3),robo_roleOn.at(4));
        break;
        case 4:
        setGoalKeeper(robo_roleOn.at(0));
        FreeKick(ballx, bally, robo_roleOn.at(1), robo_roleOn.at(2),0.5,2.0);
        setDefense1(robo_roleOn.at(3));
        break;
        case 3:
        setGoalKeeper(robo_roleOn.at(0));
        FreeKick(ballx, bally, robo_roleOn.at(1), robo_roleOn.at(2),0.5,2.0);
        break;
        case 2:
        setGoalKeeper(robo_roleOn.at(0));
        Penalty(ballx,bally,robo_roleOn.at(1),0.5);
        break;
        case 1:
        setGoalKeeper(robo_roleOn.at(0));
        break;
        default:
            break;
        }


    }
    else if(read==(QString)"f"){
        rtdb->set_var_local(10,1,10);//Free Kick Away

        switch (OnlineRobots) {
        case 5:
        setGoalKeeper(robo_roleOn.at(0));
        setBlockerKick2(ballx, bally, robo_roleOn.at(1), robo_roleOn.at(2),3.0,3.0);
        setDefense2(robo_roleOn.at(3),robo_roleOn.at(4));
        break;
        case 4:
        setGoalKeeper(robo_roleOn.at(0));
        setBlockerKick2(ballx, bally, robo_roleOn.at(1), robo_roleOn.at(2),3.0,3.0);
        setDefense1(robo_roleOn.at(3));
        break;
        case 3:
        setGoalKeeper(robo_roleOn.at(0));
        setBlockerKick2(ballx, bally, robo_roleOn.at(1), robo_roleOn.at(2),3.0,3.0);
        break;
        case 2:
        setGoalKeeper(robo_roleOn.at(0));
        setBlockerKick1(ballx,bally,robo_roleOn.at(1),0.5);
        break;
        case 1:
        setGoalKeeper(robo_roleOn.at(0));
        break;
        default:
            break;
        }

    }
    else if(read==(QString)"G"){
        rtdb->set_var_local(10,1,11);//Goal Kick Minho
        switch (OnlineRobots) {
        case 5:
        setGoalKeeper(robo_roleOn.at(0));
        FreeKick(ballx, bally, robo_roleOn.at(1), robo_roleOn.at(2),0.5,2.0);
        setDefense2(robo_roleOn.at(3),robo_roleOn.at(4));
        break;
        case 4:
        setGoalKeeper(robo_roleOn.at(0));
        FreeKick(ballx, bally, robo_roleOn.at(1), robo_roleOn.at(2),0.5,2.0);
        setDefense1(robo_roleOn.at(3));
        break;
        case 3:
        setGoalKeeper(robo_roleOn.at(0));
        FreeKick(ballx, bally, robo_roleOn.at(1), robo_roleOn.at(2),0.5,2.0);
        break;
        case 2:
        setGoalKeeper(robo_roleOn.at(0));
        Penalty(ballx,bally,robo_roleOn.at(1),0.5);
        break;
        case 1:
        setGoalKeeper(robo_roleOn.at(0));
        break;
        default:
            break;
        }

    }
    else if(read==(QString)"g"){
        rtdb->set_var_local(10,1,12);// Goal Kick Away
        switch (OnlineRobots) {
        case 5:
        setGoalKeeper(robo_roleOn.at(0));
        setBlockerKick2(ballx, bally, robo_roleOn.at(1), robo_roleOn.at(2),3.0,3.0);
        setDefense2(robo_roleOn.at(3),robo_roleOn.at(4));
        break;
        case 4:
        setGoalKeeper(robo_roleOn.at(0));
        setBlockerKick2(ballx, bally, robo_roleOn.at(1), robo_roleOn.at(2),3.0,3.0);
        setDefense1(robo_roleOn.at(3));
        break;
        case 3:
        setGoalKeeper(robo_roleOn.at(0));
        setBlockerKick2(ballx, bally, robo_roleOn.at(1), robo_roleOn.at(2),3.0,3.0);
        break;
        case 2:
        setGoalKeeper(robo_roleOn.at(0));
        setBlockerKick1(ballx,bally,robo_roleOn.at(1),0.5);
        break;
        case 1:
        setGoalKeeper(robo_roleOn.at(0));
        break;
        default:
            break;
        }

    }
    else if(read==(QString)"T"){

        rtdb->set_var_local(10,1,13);// Throw In Minho

        switch (OnlineRobots) {
        case 5:
        setGoalKeeper(robo_roleOn.at(0));
        ThrowIn(ballx, bally, robo_roleOn.at(1), robo_roleOn.at(2),0.5,2.0);
        setDefense2(robo_roleOn.at(3),robo_roleOn.at(4));
        break;
        case 4:
        setGoalKeeper(robo_roleOn.at(0));
        ThrowIn(ballx, bally, robo_roleOn.at(1), robo_roleOn.at(2),0.5,2.0);
        setDefense1(robo_roleOn.at(3));
        break;
        case 3:
        setGoalKeeper(robo_roleOn.at(0));
        ThrowIn(ballx, bally, robo_roleOn.at(1), robo_roleOn.at(2),0.5,2.0);
        break;
        case 2:
        setGoalKeeper(robo_roleOn.at(0));
        Penalty(ballx,bally,robo_roleOn.at(1),0.5);
        break;
        case 1:
        setGoalKeeper(robo_roleOn.at(0));
        break;
        default:
            break;
        }

    }
    else if(read==(QString)"t"){
        rtdb->set_var_local(10,1,14);// Throw In Away

        switch (OnlineRobots) {
        case 5:
        setGoalKeeper(robo_roleOn.at(0));
        setBlockerKick2(ballx, bally, robo_roleOn.at(1), robo_roleOn.at(2),3.0,3.0);
        setDefense2(robo_roleOn.at(3),robo_roleOn.at(4));
        break;
        case 4:
        setGoalKeeper(robo_roleOn.at(0));
        setBlockerKick2(ballx, bally, robo_roleOn.at(1), robo_roleOn.at(2),3.0,3.0);
        setDefense1(robo_roleOn.at(3));
        break;
        case 3:
        setGoalKeeper(robo_roleOn.at(0));
        setBlockerKick2(ballx, bally, robo_roleOn.at(1), robo_roleOn.at(2),3.0,3.0);
        break;
        case 2:
        setGoalKeeper(robo_roleOn.at(0));
        setBlockerKick1(ballx,bally,robo_roleOn.at(1),0.5);
        break;
        case 1:
        setGoalKeeper(robo_roleOn.at(0));
        break;
        default:
            break;
        }


    }
    else if(read==(QString)"C"){
        rtdb->set_var_local(10,1,15);// Corner Minho

        switch (OnlineRobots) {
        case 5:
        setGoalKeeper(robo_roleOn.at(0));
        FreeKick(ballx, bally, robo_roleOn.at(1), robo_roleOn.at(2),0.5,2.0);
        setDefense2(robo_roleOn.at(3),robo_roleOn.at(4));
        break;
        case 4:
        setGoalKeeper(robo_roleOn.at(0));
        FreeKick(ballx, bally, robo_roleOn.at(1), robo_roleOn.at(2),0.5,2.0);
        setDefense1(robo_roleOn.at(3));
        break;
        case 3:
        setGoalKeeper(robo_roleOn.at(0));
        FreeKick(ballx, bally, robo_roleOn.at(1), robo_roleOn.at(2),0.5,2.0);
        break;
        case 2:
        setGoalKeeper(robo_roleOn.at(0));
        Penalty(ballx,bally,robo_roleOn.at(1),0.5);
        break;
        case 1:
        setGoalKeeper(robo_roleOn.at(0));
        break;
        default:
            break;
        }

    }
    else if(read==(QString)"c"){
        rtdb->set_var_local(10,1,16);// Corner Away

        switch (OnlineRobots) {
        case 5:
        setGoalKeeper(robo_roleOn.at(0));
        setBlockerKick2(ballx, bally, robo_roleOn.at(1), robo_roleOn.at(2),3.0,3.0);
        setDefense2(robo_roleOn.at(3),robo_roleOn.at(4));
        break;
        case 4:
        setGoalKeeper(robo_roleOn.at(0));
        setBlockerKick2(ballx, bally, robo_roleOn.at(1), robo_roleOn.at(2),3.0,3.0);
        setDefense1(robo_roleOn.at(3));
        break;
        case 3:
        setGoalKeeper(robo_roleOn.at(0));
        setBlockerKick2(ballx, bally, robo_roleOn.at(1), robo_roleOn.at(2),3.0,3.0);
        break;
        case 2:
        setGoalKeeper(robo_roleOn.at(0));
        setBlockerKick1(ballx,bally,robo_roleOn.at(1),0.5);
        break;
        case 1:
        setGoalKeeper(robo_roleOn.at(0));
        break;
        default:
            break;
        }

    }
    else if(read==(QString)"P"){
        rtdb->set_var_local(10,1,17);// Penalty Minho
        if(OnlineRobots>1){
            Penalty(ballx,bally,robo_roleOn.at(1),0.5);
        }

    }
    else if(read==(QString)"p"){
        rtdb->set_var_local(10,1,18);// Penalty Away
        QString fileLoad = "Penalty";
        if(check_game_direction)fileLoad+="R";
        else fileLoad+="L";
        for(int i=0;i<nTaticsLoad;i++){
            QString name = robotNode[i].name;
            if(name == fileLoad) setRobotPositions(i);
        }

    }
    else if(read==(QString)"O"){
        rtdb->set_var_local(10,1,19);// Repair Minho
    }
    else if(read==(QString)"o"){
        rtdb->set_var_local(10,1,20);// Repair Away
    }
    else if(read==(QString)"N"){
        rtdb->set_var_local(10,1,21);// Drop Ball
    }
    else if(read==(QString)"h"){
        rtdb->set_var_local(10,1,22);// Half Game
        if(check_game_direction){
            rtdb->set_var_local(13,1,0); //Set Game Direction 0
            static_goal_taget_x=((LENGTH/2)/FACTOR);
            ui->bt_game_direction->setStyleSheet("QPushButton{border-image: url(:/right.png) 0 0 0 0 stretch stretch;}");
            check_game_direction = false;
            //ui->bt_game_direction
        }
        else{
            rtdb->set_var_local(13,1,1); //Set Game Direction 1
            static_goal_taget_x=-((LENGTH/2)/FACTOR);
            ui->bt_game_direction->setStyleSheet("QPushButton{border-image: url(:/left.png) 0 0 0 0 stretch stretch;}");
            check_game_direction = true;
        }

    }

    qDebug() << read;
}

void MainWindow::addTatics(QString FileName)
{
    QString folderName = "";
    #ifdef official
    folderName = "startPosOfficial";
    #endif
    #ifdef lar
    folderName = "startPosLar";
    #endif

    QFile file("../TestWidgetsLocal/data/"+folderName+"/"+FileName);

    FileName = FileName.replace(".txt","");
    int i=0;
    for(i=0;i<FileName.length();i++){
        robotNode[nTaticsLoad].name[i] = FileName.at(i).toLatin1();
    }
    robotNode[nTaticsLoad].name[i] = '\0';


    if(!file.open(QIODevice::ReadOnly)) {
        QMessageBox::warning(0, "Error", file.errorString());
    }
    QTextStream in(&file);
    QString line;
    while(!in.atEnd()) {
        line = in.readLine();
             if(line.contains("Robo1x:")) robotNode[nTaticsLoad].x[0] = line.replace("Robo1x:","").toFloat();
        else if(line.contains("Robo1y:")) robotNode[nTaticsLoad].y[0] = line.replace("Robo1y:","").toFloat();
        else if(line.contains("Robo1a:")) robotNode[nTaticsLoad].angle[0] = line.replace("Robo1a:","").toFloat();
        else if(line.contains("Robo2x:")) robotNode[nTaticsLoad].x[1] = line.replace("Robo2x:","").toFloat();
        else if(line.contains("Robo2y:")) robotNode[nTaticsLoad].y[1] = line.replace("Robo2y:","").toFloat();
        else if(line.contains("Robo2a:")) robotNode[nTaticsLoad].angle[1] = line.replace("Robo2a:","").toFloat();
        else if(line.contains("Robo3x:")) robotNode[nTaticsLoad].x[2] = line.replace("Robo3x:","").toFloat();
        else if(line.contains("Robo3y:")) robotNode[nTaticsLoad].y[2] = line.replace("Robo3y:","").toFloat();
        else if(line.contains("Robo3a:")) robotNode[nTaticsLoad].angle[2] = line.replace("Robo3a:","").toFloat();
        else if(line.contains("Robo4x:")) robotNode[nTaticsLoad].x[3] = line.replace("Robo4x:","").toFloat();
        else if(line.contains("Robo4y:")) robotNode[nTaticsLoad].y[3] = line.replace("Robo4y:","").toFloat();
        else if(line.contains("Robo4a:")) robotNode[nTaticsLoad].angle[3] = line.replace("Robo4a:","").toFloat();
        else if(line.contains("Robo5x:")) robotNode[nTaticsLoad].x[4] = line.replace("Robo5x:","").toFloat();
        else if(line.contains("Robo5y:")) robotNode[nTaticsLoad].y[4] = line.replace("Robo5y:","").toFloat();
        else if(line.contains("Robo5a:")) robotNode[nTaticsLoad].angle[4] = line.replace("Robo5a:","").toFloat();
    }

    //qDebug() << "Added" << nTaticsLoad << " :" << robotNode[nTaticsLoad].name;

    nTaticsLoad++;
}

void MainWindow::on_bt_game_direction_clicked()
{
    if(check_game_direction)
    {
       rtdb->set_var_local(13,1,0); //Set Game Direction 0
       static_goal_taget_x=((LENGTH/2)/FACTOR);
       ui->bt_game_direction->setStyleSheet("QPushButton{border-image: url(:/right.png) 0 0 0 0 stretch stretch;}");
       check_game_direction = false;
    }
    else
    {
       rtdb->set_var_local(13,1,1); //Set Game Direction 1
       static_goal_taget_x=-((LENGTH/2)/FACTOR);
       ui->bt_game_direction->setStyleSheet("QPushButton{border-image: url(:/left.png) 0 0 0 0 stretch stretch;}");
       check_game_direction = true;
    }
}

void MainWindow::setRobotPositions(int TaticNumber)
{
    #ifdef OFFLINE
    //qDebug() << "Seting Position: " << robotNode[TaticNumber].name;
    for(int i=0;i<numberRobots;i++){

        robo[i]->setPosition(getx(robotNode[TaticNumber].x[i]),gety(robotNode[TaticNumber].y[i]));
        robo[i]->setAngle(robotNode[TaticNumber].angle[i]);
        //Set RTDB
        setRTDBRobotTarget(robotNode[TaticNumber].x[i], robotNode[TaticNumber].y[i], robotNode[TaticNumber].angle[i], i);
    }
    #endif
    #ifndef OFFLINE

    int positionNow = 0;
    for(int i=0;i<numberRobots;i++){
        if(robo_state[i]){
            robo[i]->setPosition(getx(robotNode[TaticNumber].x[positionNow]),gety(robotNode[TaticNumber].y[positionNow]));
            robo[i]->setAngle(robotNode[TaticNumber].angle[positionNow]);
            //Set RTDB
            setRTDBRobotTarget(robotNode[TaticNumber].x[positionNow], robotNode[TaticNumber].y[positionNow], robotNode[TaticNumber].angle[positionNow], positionNow);

            positionNow++;
        }
    }

    #endif
}

void MainWindow::on_bt_save_position_clicked()
{
    QString FileName = ui->cb_tactics->currentText();
    QString folderName = "";
    #ifdef official
    folderName = "startPosOfficial";
    #endif
    #ifdef lar
    folderName = "startPosLar";
    #endif

    QFile file("../TestWidgetsLocal/data/"+folderName+"/"+FileName);

    if(!file.open(QIODevice::WriteOnly)) {
        QMessageBox::warning(0, "Error", file.errorString());
    }
    QTextStream out(&file);
    for(int i=0;i<numberRobots;i++) {
    out << "Robo"+QString::number(i+1)+"x:" << getMeterx(robo[i]->pos().x()+ROBOT_DIAMETER/2/FACTOR) << "\n";
    out << "Robo"+QString::number(i+1)+"y:" << getMetery(robo[i]->pos().y()+ROBOT_DIAMETER/2/FACTOR) << "\n";
    out << "Robo"+QString::number(i+1)+"a:" << robo[i]->getAngle() << "\n";
    }
    file.close();

    LoadTatics();

    QMessageBox::information(this,"File Save","Saved: "+FileName);
}

void MainWindow::on_bt_load_position_clicked()
{
    for(int i=0;i<nTaticsLoad;i++){
        QString name = robotNode[i].name;
        if(name == ui->cb_tactics->currentText().replace(".txt","")) setRobotPositions(i);
    }

    int value = robo[0]->getAngle();
    ui->lb_robot1_angle->setText("Angle: "+QString::number(value)+"º");
    ui->sl_robot1_angle->setValue(value);

    value = robo[1]->getAngle();
    ui->lb_robot2_angle->setText("Angle: "+QString::number(value)+"º");
    ui->sl_robot2_angle->setValue(value);

    value = robo[2]->getAngle();
    ui->lb_robot3_angle->setText("Angle: "+QString::number(value)+"º");
    ui->sl_robot3_angle->setValue(value);

    value = robo[3]->getAngle();
    ui->lb_robot4_angle->setText("Angle: "+QString::number(value)+"º");
    ui->sl_robot4_angle->setValue(value);

    value = robo[4]->getAngle();
    ui->lb_robot5_angle->setText("Angle: "+QString::number(value)+"º");
    ui->sl_robot5_angle->setValue(value);
}

void MainWindow::LoadTatics()
{
    QString folderName = "";
    #ifdef official
    folderName = "startPosOfficial";
    #endif
    #ifdef lar
    folderName = "startPosLar";
    #endif

    QString path = "../TestWidgetsLocal/data/"+folderName; // assume it is some path

    QDir dir(path);

    dir.setFilter(QDir::AllEntries | QDir::NoDotAndDotDot);

    int limitTatics = dir.count();

    nTaticsLoad = 0;

    robotNode = (struct nodo*)calloc(limitTatics,sizeof(struct nodo));
    memset(robotNode,0,sizeof(nodo)*limitTatics);

    ui->cb_tactics->clear();

    QFileInfoList list = dir.entryInfoList();
        for (int i = 0; i < list.size(); ++i) {
            QFileInfo fileInfo = list.at(i);
            addTatics(fileInfo.fileName());
            ui->cb_tactics->addItem(fileInfo.fileName());
        }
}

void MainWindow::on_sl_robot1_angle_valueChanged(int value)
{
    ui->lb_robot1_angle->setText("Angle: "+QString::number(value)+"º");
    robo[0]->setAngle((float)value);
}

void MainWindow::on_sl_robot2_angle_valueChanged(int value)
{
    ui->lb_robot2_angle->setText("Angle: "+QString::number(value)+"º");
    robo[1]->setAngle((float)value);
}

void MainWindow::on_sl_robot3_angle_valueChanged(int value)
{
    ui->lb_robot3_angle->setText("Angle: "+QString::number(value)+"º");
    robo[2]->setAngle((float)value);
}

void MainWindow::on_sl_robot4_angle_valueChanged(int value)
{
    ui->lb_robot4_angle->setText("Angle: "+QString::number(value)+"º");
    robo[3]->setAngle((float)value);
}

void MainWindow::on_sl_robot5_angle_valueChanged(int value)
{
    ui->lb_robot5_angle->setText("Angle: "+QString::number(value)+"º");
    robo[4]->setAngle((float)value);
}

void MainWindow::setRobotLable(QLabel *lable, bool state, QString number)
{
    if(state){
        QMovie *movie = new QMovie(":/on_"+number+".gif");
        movie->setScaledSize(QSize(50,50));
        QLabel *processLabel = lable;
        processLabel->setMovie(movie);
        processLabel->setToolTip("Online");
        movie->start();
    }
    else{
        QMovie *movie = new QMovie(":/off_"+number+".gif");
        movie->setScaledSize(QSize(50,50));
        QLabel *processLabel = lable;
        processLabel->setToolTip("Offline");
        processLabel->setMovie(movie);
        movie->start();
    }

}

void MainWindow::updateRTDB()
{
    for(int i=0;i<numberRobots;i++){

        robo_state[i] = rtdb->get_var(61+i,i+1);

        if(robo_state[i]!=robo_state_old[i]){
        switch (i) {
        case 0:
            setRobotLable(ui->state_robo1, robo_state[i],QString::number(i+1));
            break;
        case 1:
            setRobotLable(ui->state_robo2, robo_state[i],QString::number(i+1));
            break;
        case 2:
            setRobotLable(ui->state_robo3, robo_state[i],QString::number(i+1));
            break;
        case 3:
            setRobotLable(ui->state_robo4, robo_state[i],QString::number(i+1));
            break;
        case 4:
            setRobotLable(ui->state_robo5, robo_state[i],QString::number(i+1));
            break;
            }
            robo_state_old[i] = robo_state[i];

            /*if(!robo_state[i]){
                QString fileLoad = "Park";
                if(check_game_direction)fileLoad+="R";
                else fileLoad+="L";
                for(int i=0;i<nTaticsLoad;i++){
                    QString name = robotNode[i].name;
                    if(name == fileLoad) setRobotPositions(i);
                }
            }*/
        }

        if(robo_state[i]){
        robo[i]->setPosition(getx((float)rtdb->get_var(1,i+1)/10.0),gety((float)rtdb->get_var(2,i+1)/10.0));
        int angle = getRTDBRobotAngle(rtdb->get_var(3,i+1),rtdb->get_var(4,i+1));
        robo[i]->setAngle(angle);

        ball[i]->setPos(getx((float)rtdb->get_var(5,i+1)/10.0)-BALL_DIAMETER/2/FACTOR,gety((float)rtdb->get_var(6,i+1)/10.0)-BALL_DIAMETER/2/FACTOR);

        switch (i) {
        case 0:
            ui->sl_robot1_angle->setValue(angle);
            ui->lb_robot1_angle->setText("Angle: "+QString::number(angle)+"º");
            ui->lb_hardware_battery_robo1->setText("Hardware Battery: "+QString::number((float)rtdb->get_var(9,i+1))+"V");
            ui->lb_camera_battery_robo1->setText("Camera Battery: "+QString::number((float)rtdb->get_var(126,i+1)/10.0)+"V");
            ui->lb_pc_battery_robo1->setText("PC Battery: "+QString::number((float)rtdb->get_var(125,i+1)/10.0)+"V");
            break;
        case 1:
            ui->sl_robot2_angle->setValue(angle);
            ui->lb_robot2_angle->setText("Angle: "+QString::number(angle)+"º");
            ui->lb_hardware_battery_robo2->setText("Hardware Battery: "+QString::number((float)rtdb->get_var(9,i+1))+"V");
            ui->lb_camera_battery_robo2->setText("Camera Battery: "+QString::number((float)rtdb->get_var(126,i+1)/10.0)+"V");
            ui->lb_pc_battery_robo2->setText("PC Battery: "+QString::number((float)rtdb->get_var(125,i+1)/10.0)+"V");
            break;
        case 2:
            ui->sl_robot3_angle->setValue(angle);
            ui->lb_robot3_angle->setText("Angle: "+QString::number(angle)+"º");
            ui->lb_hardware_battery_robo3->setText("Hardware Battery: "+QString::number((float)rtdb->get_var(9,i+1))+"V");
            ui->lb_camera_battery_robo3->setText("Camera Battery: "+QString::number((float)rtdb->get_var(126,i+1)/10.0)+"V");
            ui->lb_pc_battery_robo3->setText("PC Battery: "+QString::number((float)rtdb->get_var(125,i+1)/10.0)+"V");
            //qDebug() << rtdb->get_var(9,i+1);
            break;
        case 3:
            ui->sl_robot4_angle->setValue(angle);
            ui->lb_robot4_angle->setText("Angle: "+QString::number(angle)+"º");
            ui->lb_hardware_battery_robo4->setText("Hardware Battery: "+QString::number((float)rtdb->get_var(9,i+1))+"V");
            ui->lb_camera_battery_robo4->setText("Camera Battery: "+QString::number((float)rtdb->get_var(126,i+1)/10.0)+"V");
            ui->lb_pc_battery_robo4->setText("PC Battery: "+QString::number((float)rtdb->get_var(125,i+1)/10.0)+"V");
            break;
        case 4:
            ui->sl_robot5_angle->setValue(angle);
            ui->lb_robot5_angle->setText("Angle: "+QString::number(angle)+"º");
            ui->lb_hardware_battery_robo5->setText("Hardware Battery: "+QString::number((float)rtdb->get_var(9,i+1))+"V");
            ui->lb_camera_battery_robo5->setText("Camera Battery: "+QString::number((float)rtdb->get_var(126,i+1)/10.0)+"V");
            ui->lb_pc_battery_robo5->setText("PC Battery: "+QString::number((float)rtdb->get_var(125,i+1)/10.0)+"V");
            break;
            }
            robo_state_old[i] = robo_state[i];
        }
    }

}

void MainWindow::on_bt_rtdb_state_clicked()
{
    if(check_RTDB)
    {
       timerRTDB->stop();
       ui->bt_rtdb_state->setStyleSheet("QPushButton{border-image: url(:/off.png) 0 0 0 0 stretch stretch;}");
       check_RTDB = false;
    }
    else
    {
       timerRTDB->start(RTDBREADTIME);
       ui->bt_rtdb_state->setStyleSheet("QPushButton{border-image: url(:/on.png) 0 0 0 0 stretch stretch;}");
       check_RTDB = true;
    }
}

int MainWindow::getRTDBRobotAngle(int LSB, int MSB)
{
    return LSB+128+MSB;
}

void MainWindow::setRTDBRobotTarget(float x, float y, int angle, int robotNumber)
{
    //Set X
    rtdb->set_var_local(10+4*(robotNumber),1,(int)(x*10));
    //qDebug() << "VarNumber: " << 10+4*(robotNumber) << "Value: " << (int)(x*10);
    //Set Y
    rtdb->set_var_local(10+4*(robotNumber)+1,1,(int)(y*10));
    //qDebug() << "VarNumber: " << 10+4*(robotNumber)+1 << "Value: " << (int)(y*10);
    //Set Angle
    if(angle>255)
    {
        rtdb->set_var_local(12+4*(robotNumber),1,127);//selfPosition Angle;
        //qDebug() << "VarNumber: " << 12+4*(robotNumber) << "Value: " << 127;
        rtdb->set_var_local(12+4*(robotNumber)+1,1,angle-255);//selfPosition Angle;
        //qDebug() << "VarNumber: " << 12+4*(robotNumber)+1 << "Value: " << angle-255;
    } else {
        rtdb->set_var_local(12+4*(robotNumber),1,angle-128);//selfPosition Angle;
        //qDebug() << "VarNumber: " << 12+4*(robotNumber) << "Value: " << angle-128;
        rtdb->set_var_local(12+4*(robotNumber)+1,1,0);//selfPosition Angle;
        //qDebug() << "VarNumber: " << 12+4*(robotNumber)+1 << "Value: " << 0;
    }
}

QVariantList MainWindow::wrapJSONValues(float *val1, float *val2, float *val3)
{
    QVariantList list;
    if(val1!=NULL)list.append(QString::number(*val1,'f',3));
    if(val2!=NULL)list.append(QString::number(*val2,'f',3));
    if(val3!=NULL)list.append(QString::number(*val3,'f',3));
    return list;
}

void MainWindow::senRef()
{
    QVariantMap world,general;
    QVariantList list;

    world.insert("type","worldstate");
    world.insert("teamName","MIN");
    world.insert("intention","WinThisShit");
    float val = 0;

    //Robôs
    for(int i =0;i<5;i++){
        float valx = getMeterx(robo[i]->pos().x());
        float valy = getMetery(robo[i]->pos().y());
        float angle = robo[i]->getAngle()*(M_PI/180);

        general.insert("id",i);
        general.insert("pose",wrapJSONValues(&valx,&valy,&angle));
        general.insert("targetPose",wrapJSONValues(&val,&val,&val));
        general.insert("velocity",wrapJSONValues(&val,&val,&val));
        general.insert("intention","KILL");
        general.insert("batteryLevel",100.0);
        general.insert("ballEngaged",0);

        list.insert(i,general);
        general.clear();
    }
    world.insert("robots",QVariant(list));
    list.clear();

    //Obstáculos
    for(int i =0;i<3;i++){
        general.insert("position",wrapJSONValues(&val,&val,NULL));
        general.insert("velocity",wrapJSONValues(&val,&val,NULL));
        general.insert("radius",0.5);
        general.insert("confidence",1.00);

        list.insert(i,general);
        general.clear();
    }
    world.insert("obstacles",QVariant(list));
    list.clear();

    //Bolas
    for(int i =0;i<1;i++){
        float ballx = getMeterx(ball[i]->pos().x());
        float bally = getMetery(ball[i]->pos().y());
        general.insert("position",wrapJSONValues(&ballx,&bally,&val));
        general.insert("velocity",wrapJSONValues(&val,&val,&val));
        general.insert("confidence",1.00);

        list.insert(i,general);
        general.clear();
    }
    world.insert("balls",QVariant(list));
    list.clear();

    world.insert("age",20);
    QString sendRefBox;
    sendRefBox = QJsonDocument::fromVariant(world).toJson().simplified().replace(" ","");


    //qDebug() << sendRefBox;

    mysocket->write(sendRefBox.toLocal8Bit().append('\0'));
}

void MainWindow::on_bt_main_loop_state_clicked()
{
    if(!check_main_loop)
    {
       ui->bt_main_loop_state->setStyleSheet("QPushButton{border-image: url(:/on.png) 0 0 0 0 stretch stretch;}");
       timerMainLoop->start(5);
       check_main_loop = true;
    }
    else
    {
       ui->bt_main_loop_state->setStyleSheet("QPushButton{border-image: url(:/off.png) 0 0 0 0 stretch stretch;}");
       timerMainLoop->stop();
       check_main_loop = false;
    }
}

void MainWindow::updateMainLoop()
{
    /*if(we_have_ball){
        if(!ball_is_in_my_camp()){
            check_show_shot_lines = true;
            //1 - Rematar à baliza
        }
        else{
            check_show_pass_lines = true;

            //Temos de fazer um passe para la do meio campo

            //qDebug() << "ShowPassLine";
        }
    }
    else{

        //1 - guarda redes
        //1 - atras da bola
        // resto é defesas

        if(check_show_pass_lines) scene->removeItem(pass_line);
        if(check_show_shot_lines) scene->removeItem(shot_line);
        check_show_shot_lines = false;
        check_show_pass_lines = false;

    }*/

    if(check_show_pass_lines)
    {
        int xRobot1Center = robo[roboPass-1]->pos().x()+(ROBOT_DIAMETER/2)/FACTOR;
        int yRobot1Center = robo[roboPass-1]->pos().y()+(ROBOT_DIAMETER/2)/FACTOR;

        int xRobot2Center = robo[robotWithBall-1]->pos().x()+(ROBOT_DIAMETER/2)/FACTOR;
        int yRobot2Center = robo[robotWithBall-1]->pos().y()+(ROBOT_DIAMETER/2)/FACTOR;

        //qDebug() << "roboPass: " << roboPass << "robotShoot: " << robotShoot;

        drawLine(xRobot1Center,yRobot1Center,xRobot2Center,yRobot2Center,pass_line);

        bool colides = false;

        for(int i=0;i<10;i++)
        {
            if(i==roboPass-1 || i==robotWithBall-1);
            else if(pass_line->collidesWithItem(robo[i])){
                colides = true;
                i = 10;
            }
        }

        if(colides){
            pen_line.setColor(QColor(255,0,0));
            pass_line->setPen(pen_line);
        }
        else {
            pen_line.setColor(QColor(0,255,0));
            pass_line->setPen(pen_line);
        }

    }
    if(check_show_shot_lines)
    {
        //scene->removeItem(shot_circle);
        shot_circle->setPos(static_goal_taget_x,goal_target_y);
        //scene->addItem(shot_circle);

        int xRobot2Center = robo[robotWithBall-1]->pos().x()+(ROBOT_DIAMETER/2)/FACTOR;
        int yRobot2Center = robo[robotWithBall-1]->pos().y()+(ROBOT_DIAMETER/2)/FACTOR;


        drawLine(xRobot2Center,yRobot2Center,static_goal_taget_x,goal_target_y,shot_line);

        bool colides = false;

        for(int i=0;i<10;i++)
        {
            if(i==robotWithBall-1);
            else if(shot_line->collidesWithItem(robo[i])){
                colides = true;
                i = 10;
            }
        }
        if(colides){
            pen_line.setColor(QColor(255,0,0));
            shot_line->setPen(pen_line);

            if(goal_target_y >-(GOAL_WIDTH/2)/FACTOR && !limitUp)
            {
                goal_target_y -= (100/FACTOR);
            }
            else limitUp = true;
            if (goal_target_y<(GOAL_WIDTH/2)/FACTOR && limitUp)
            {
                goal_target_y += (100/FACTOR);
            }
            else limitUp = false;
        }
        else {
            pen_line.setColor(QColor(0,255,0));
            shot_line->setPen(pen_line);
        }
    }

    we_have_ball = false;
    //Check if main ball is on robot
    for(int i=0;i<numberRobots;i++){

        if(robo_state[i]){
            if(rtdb->get_var(8,i+1)) {
                robo[i]->setColor(255,0,255);
                robo[i]->update();
                we_have_ball = true;
                robotWithBall = i+1;
            }
            else{
                robo[i]->setColor(120,10,15);
                robo[i]->update();
            }
        }

        /*if(robo[i]->collidesWithItem(mainBall)){
            robo[i]->setColor(255,0,255);
            robo[i]->update();
            we_have_ball = true;
            robotWithBall = i+1;
        }
        else{
            robo[i]->setColor(120,10,15);
            robo[i]->update();
        }*/
    }


    float ballx = getMeterx(mainBall->pos().x()+mainBall->width/2);

    float bally = getMetery(mainBall->pos().y()+mainBall->width/2);

    //GoalKickMinho(ballx, bally, 0,1,0.5,0.5);

    //setBlockerKick2(ballx, bally, 0,1,3.0,3.0);

    //setBlockerKick1(ballx, bally, 0,3.0);

    //Penalty(ballx, bally, 0,0.5);

    //FreeKick(ballx, bally, 0, 1,0.5,2.0);

    //ThrowIn(ballx, bally, 0, 1,0.5,2.0);

    ui->lb_ball->setText("Ball x:"+QString::number(ballx)+" y: "+QString::number(bally));


    meshBalls();

}

bool MainWindow::ball_is_in_my_camp()
{
    if(check_game_direction){
        if(mainBall->pos().x()>=0) return true;
        else return false;
    }
    else {
        if(mainBall->pos().x()<=0) return true;
        else return false;
    }
}

void MainWindow::setRobotStart()
{
    rtdb->set_var_local(86,1,1);
    rtdb->set_var_local(87,1,1);
    rtdb->set_var_local(88,1,1);
    rtdb->set_var_local(89,1,1);
    rtdb->set_var_local(90,1,1);

    rtdb->set_var_local(96,1,1); // Start
}

void MainWindow::setRobotStop()
{
    rtdb->set_var_local(86,1,0);
    rtdb->set_var_local(87,1,0);
    rtdb->set_var_local(88,1,0);
    rtdb->set_var_local(89,1,0);
    rtdb->set_var_local(90,1,0);

    rtdb->set_var_local(96,1,0); // Stop
}

void MainWindow::meshBalls()
{
    vector <float> bx,by,rx,ry;

    float distancia[numberRobots];

    float xMain = 0.0,yMain = 0.0,distTotal = 0.0, mediaX = 0.0, mediaY = 0.0;

    int numNow = 0;

    for(int i=0;i<numberRobots;i++){

         //if(robo_state[i] && rtdb->get_var(7,i+1)){

         /*rx.push_back((float)rtdb->get_var(1,i+1)/10.0);
         ry.push_back((float)rtdb->get_var(1,i+1)/10.0);

         bx.push_back((float)rtdb->get_var(5,i+1)/10.0);
         by.push_back((float)rtdb->get_var(6,i+1)/10.0);*/

        rx.push_back(getMeterx(robo[i]->pos().x()));
        ry.push_back(getMetery(robo[i]->pos().y()));

        bx.push_back(getMeterx(ball[i]->pos().x()));
        by.push_back(getMetery(ball[i]->pos().y()));

         numNow++;

         //}
     }


    for(unsigned int i=0;i<rx.size();i++){

            float calcAux = (float)sqrt(((bx[i]-rx[i])*(bx[i]-rx[i])+(by[i]-ry[i])*(by[i]-ry[i])));
            distancia[i] = (calcAux);

            mediaX += bx[i];
            mediaY += by[i];
    }

    mediaX = mediaX / rx.size();
    mediaY = mediaY / rx.size();

    float distanciasVirtuais[rx.size()], distVirtMedia = 0.0;

    for(unsigned int i=0;i<rx.size();i++)
    {
        distanciasVirtuais[i] = sqrt(((bx[i]-mediaX)*(bx[i]-mediaX)+(by[i]-mediaY)*(by[i]-mediaY)));
        distVirtMedia+= distanciasVirtuais[i];
    }

    distVirtMedia = distVirtMedia/rx.size();

    int working = 0;

    for(unsigned int i=0;i<rx.size();i++)
    {
        if(distanciasVirtuais[i]<=distVirtMedia)
        {
            xMain += bx[i]*(1/distancia[i]);
            yMain += by[i]*(1/distancia[i]);
            distTotal+= (1/distancia[i]);
            working++;
        }
    }

    if(working>0){
        xMain = xMain/distTotal;
        yMain = yMain/distTotal;
        mainBall->setPosition(getx(xMain),gety(yMain));

        rtdb->set_var_local(11,1,(int)(xMain*10)); //Ballx
        rtdb->set_var_local(12,1,(int)(yMain*10)); //Bally
    }

    //qDebug() << "x:" << (int)(xMain*10) << "y:" << (int)(yMain*10);
    ui->lb_ball->setText("Ball x:"+QString::number(mainBall->pos().x())+" y:"+QString::number(mainBall->pos().y()));
}

void MainWindow::setRobotLableAngle(int angle, int robot)
{
    switch (robot) {
    case 1:
    ui->lb_robot1_angle->setText(QString::number(angle)+"º");
    ui->sl_robot1_angle->setValue(angle);
        break;
    case 2:
    ui->lb_robot2_angle->setText(QString::number(angle)+"º");
    ui->sl_robot2_angle->setValue(angle);
        break;
    case 3:
    ui->lb_robot3_angle->setText(QString::number(angle)+"º");
    ui->sl_robot3_angle->setValue(angle);
        break;
    case 4:
    ui->lb_robot4_angle->setText(QString::number(angle)+"º");
    ui->sl_robot4_angle->setValue(angle);
        break;
    case 5:
    ui->lb_robot5_angle->setText(QString::number(angle)+"º");
    ui->sl_robot5_angle->setValue(angle);
        break;
    default:
        break;
    }
}

void MainWindow::on_pushButton_clicked()
{
    robo[0]->setPosition(getx(-1.25),gety(1.25));
    ball[0]->setPosition(getx(-1.95),gety(2.45));

    robo[1]->setPosition(getx(-1.55),gety(1.65));
    ball[1]->setPosition(getx(-1.85),gety(2.45));

    robo[2]->setPosition(getx(1.25),gety(1.25));
    ball[2]->setPosition(getx(-1.55),gety(2.05));

    robo[3]->setPosition(getx(-2.25),gety(1.75));
    ball[3]->setPosition(getx(-1.75),gety(2.05));

    robo[4]->setPosition(getx(-6.25),gety(5.25));
    ball[4]->setPosition(getx(-6.45),gety(5.45));
}

void MainWindow::on_pushButton_2_clicked()
{
    float angle = ui->lineEdit->text().toFloat();

    robo[0]->setAngle(angle);
}

void MainWindow::FreeKick(float ballx, float bally, int passer, int receiver, float passerDistance, float receiverDistance)
{
    float targetX = LENGTH/2000;
    if(check_game_direction) targetX*=-1;
    float resultRad = atan2f(ballx-targetX,bally-0.0);
    float result = atan2f(ballx-targetX,bally-0.0)*180/M_PI;

    result *=-1;

    float robotPassNewX = ballx - passerDistance*sin(resultRad);
    float robotPassNewY = bally - passerDistance*cos(resultRad);

    robo[passer]->setPosition(getx(robotPassNewX),gety(robotPassNewY));

    robo[passer]->setAngle(result);

    float robotRecieveNewX = ballx + receiverDistance*sin(resultRad);
    float robotRecieveNewY = bally + receiverDistance*cos(resultRad);

    robo[receiver]->setPosition(getx(robotRecieveNewX),gety(robotRecieveNewY));

    robo[receiver]->setAngle(result+180);

    //float x, float y, int angle, int robotNumber
    setRTDBRobotTarget(robotPassNewX,robotPassNewY,result,passer);

    setRTDBRobotTarget(robotRecieveNewX,robotRecieveNewY,result+180,receiver);

    setPassadorRTDB(passer);
    setRecetorRTDB(receiver);
}

void MainWindow::ThrowIn(float ballx, float bally, int passer, int receiver, float passerDistance, float receiverDistance)
{
    float targetX = LENGTH/2000;
    if(check_game_direction) targetX*=-1;
    float resultRad = atan2f(ballx-targetX,bally-0.0);
    float result = atan2f(ballx-targetX,bally-0.0)*180/M_PI;

    result *=-1;

    float robotPassNewX = ballx + passerDistance*sin(resultRad);
    float robotPassNewY = bally + passerDistance*cos(resultRad);

    robo[passer]->setPosition(getx(robotPassNewX),gety(robotPassNewY));

    robo[passer]->setAngle(result+180);

    float robotRecieveNewX = ballx - receiverDistance*sin(resultRad);
    float robotRecieveNewY = bally - receiverDistance*cos(resultRad);

    robo[receiver]->setPosition(getx(robotRecieveNewX),gety(robotRecieveNewY));

    robo[receiver]->setAngle(result);

    //float x, float y, int angle, int robotNumber
    setRTDBRobotTarget(robotPassNewX,robotPassNewY,result,passer);

    setRTDBRobotTarget(robotRecieveNewX,robotRecieveNewY,result+180,receiver);

    setPassadorRTDB(passer);
    setRecetorRTDB(receiver);
}

void MainWindow::Penalty(float ballx, float bally, int kicker, float kickerDistance)
{
    float targetX = LENGTH/2000;
    if(check_game_direction) targetX*=-1;

    float resultRad = atan2f(ballx-targetX,bally-0.0);
    float result = atan2f(ballx-targetX,bally-0.0)*180/M_PI;
    result *=-1;

    float robotkickerNewX = ballx + kickerDistance*sin(resultRad);
    float robotkickerNewY = bally + kickerDistance*cos(resultRad);

    robo[kicker]->setPosition(getx(robotkickerNewX),gety(robotkickerNewY));

    robo[kicker]->setAngle(result+180);

    //float x, float y, int angle, int robotNumber
    setRTDBRobotTarget(robotkickerNewX,robotkickerNewY,result+180,kicker);

    setRematadorRTDB(kicker);
}

void MainWindow::GoalKickMinho(float ballx, float bally, int passer, int receiver, float passerDistance, float receiverDistance)
{
    float targetX = LENGTH/2000;
    if(check_game_direction) targetX*=-1;
    float resultRad = atan2f(ballx-targetX,bally-0.0);
    float result = atan2f(ballx-targetX,bally-0.0)*180/M_PI;

    result *=-1;

    float robotPassNewX = ballx + passerDistance*sin(resultRad);
    float robotPassNewY = bally + passerDistance*cos(resultRad);

    robo[passer]->setPosition(getx(robotPassNewX),gety(robotPassNewY));

    robo[passer]->setAngle(result+180);

    KickMinhoRobotPasser = passer;
    KickMinhoRobotReceiver = receiver;

    goalKickMinhoSearch = true;
}



void MainWindow::updatePassLine()
{
    if(goalKickMinhoSearch){
        //pass_line


        int xRobot1Center = robo[KickMinhoRobotPasser]->pos().x()+(ROBOT_DIAMETER/2)/FACTOR;
        int yRobot1Center = robo[KickMinhoRobotPasser]->pos().y()+(ROBOT_DIAMETER/2)/FACTOR;

        int targetX = 1;
        if(check_game_direction) targetX*=-1;

        passerXpoint = getx(1*targetX);

        robo[KickMinhoRobotReceiver]->setPosition(passerXpoint,passerYpoint);
        robo[KickMinhoRobotReceiver]->setAngle(atan2f(yRobot1Center,xRobot1Center)*180/M_PI-90);
        drawLine(xRobot1Center,yRobot1Center,passerXpoint,passerYpoint,pass_line);



        bool colides = false;

        for(int i=0;i<10;i++)
        {
            if(i==KickMinhoRobotReceiver || i==KickMinhoRobotPasser);
            else if(pass_line->collidesWithItem(robo[i])){
                colides = true;
                i = 10;
            }
        }

        if(colides){
            pen_line.setColor(QColor(255,0,0));
            pass_line->setPen(pen_line);

            if(passerYpoint >-(WIDTH/2)/FACTOR && !limitUpK)
            {
                passerYpoint -= (100/FACTOR);
            }
            else limitUpK = true;
            if (passerYpoint<(WIDTH/2)/FACTOR && limitUpK)
            {
                passerYpoint += (100/FACTOR);
            }
            else limitUpK = false;
        }
        else {
            pen_line.setColor(QColor(0,255,0));
            pass_line->setPen(pen_line);
        }

        if(passerXpoint!=passerXpoint_old || passerYpoint!=passerYpoint_old){

            //Escreve na RTDB novo ponto de pass
        }


        passerXpoint_old = passerXpoint;
        passerYpoint_old = passerYpoint;
    }
}

void MainWindow::setGoalKeeper(int idGoalKeeper)
{
    int targetX = 1;
    if(!check_game_direction) targetX*=-1;

    float robotGoalKeeperNewX = targetX*((LENGTH/2000)-0.5);
    float robotGoalKeeperNewY  = 0;

    robo[idGoalKeeper]->setPosition(getx(robotGoalKeeperNewX),robotGoalKeeperNewY);

    float result = 0;

    if(check_game_direction) result = 90;
    else result = 280;

    robo[idGoalKeeper]->setAngle(result);

    //float x, float y, int angle, int robotNumber
    setRTDBRobotTarget(robotGoalKeeperNewX,robotGoalKeeperNewY,result,idGoalKeeper);

    setGuardaRedesRTDB(idGoalKeeper);

}

void MainWindow::setDefense1(int Defenser)
{
    int targetX = 1;
    if(!check_game_direction) targetX*=-1;

    float robotDefenserNewX = targetX*((LENGTH/2-AREA_LENGTH2)/1000);
    float robotDefenserNewY  = 0;

    robo[Defenser]->setPosition(getx(robotDefenserNewX),robotDefenserNewY);

    float result = 0;
    if(check_game_direction) result = 90;
    else result = 280;

    robo[Defenser]->setAngle(result);

    //float x, float y, int angle, int robotNumber
    setRTDBRobotTarget(robotDefenserNewX,robotDefenserNewY,result,Defenser);

    setDefensorRTDB(Defenser);
}

void MainWindow::setDefense2(int Defenser1, int Defenser2)
{
    int targetX = 1;
    if(!check_game_direction) targetX*=-1;

    float robotDefenserNewX = targetX*((LENGTH/2-AREA_LENGTH2)/1000);
    float robotDefenserNewY  = AREA_LENGTH2/2000;

    robo[Defenser1]->setPosition(getx(robotDefenserNewX),gety(robotDefenserNewY));

    float result = 0;
    if(check_game_direction) result = 90;
    else result = 280;

    robo[Defenser1]->setAngle(result);

    float robotDefenserNewX2 = targetX*((LENGTH/2-AREA_LENGTH2)/1000);
    float robotDefenserNewY2  = -AREA_LENGTH2/2000;

    robo[Defenser2]->setPosition(getx(robotDefenserNewX2),gety(robotDefenserNewY2));

    robo[Defenser2]->setAngle(result);

    //float x, float y, int angle, int robotNumber
    setRTDBRobotTarget(robotDefenserNewX,robotDefenserNewY,result,Defenser1);
    //float x, float y, int angle, int robotNumber
    setRTDBRobotTarget(robotDefenserNewX2,robotDefenserNewY2,result,Defenser2);

    setDefensorRTDB(Defenser1);
    setDefensorRTDB(Defenser2);
}

void MainWindow::setBlockerKick2(float ballx, float bally, int blocker1, int blocker2, float blocker1Distance, float blocker2Distance)
{
    float targetX = LENGTH/2000;
    if(!check_game_direction) targetX*=-1;
    float resultRad = atan2f(ballx-targetX,bally-0.0);
    float result = atan2f(ballx-targetX,bally-0.0)*180/M_PI;

    result *=-1;

    float robotblocker1NewX = ballx - blocker1Distance*sin(resultRad-M_PI/12);
    float robotblocker1NewY = bally - blocker1Distance*cos(resultRad-M_PI/12);

    while(getInsideArea(robotblocker1NewX,robotblocker1NewY)){

        blocker1Distance -= 0.1;
        robotblocker1NewX = ballx - blocker1Distance*sin(resultRad-M_PI/12);
        robotblocker1NewY = bally - blocker1Distance*cos(resultRad-M_PI/12);

    }

    robo[blocker1]->setPosition(getx(robotblocker1NewX),gety(robotblocker1NewY));

    robo[blocker1]->setAngle(result);

    float robotblocker2NewX = ballx - blocker2Distance*sin(resultRad+M_PI/12);
    float robotblocker2NewY = bally - blocker2Distance*cos(resultRad+M_PI/12);

    while(getInsideArea(robotblocker2NewX,robotblocker2NewY)){

        blocker2Distance -= 0.1;
        robotblocker2NewX = ballx - blocker2Distance*sin(resultRad+M_PI/12);
        robotblocker2NewY = bally - blocker2Distance*cos(resultRad+M_PI/12);
    }

    robo[blocker2]->setPosition(getx(robotblocker2NewX),gety(robotblocker2NewY));

    robo[blocker2]->setAngle(result);

    //float x, float y, int angle, int robotNumber
    setRTDBRobotTarget(robotblocker1NewX,robotblocker1NewY,result,blocker1);
    //float x, float y, int angle, int robotNumber
    setRTDBRobotTarget(robotblocker2NewX,robotblocker2NewY,result,blocker2);

    setDefensorRTDB(blocker1);
    setDefensorRTDB(blocker2);
}

void MainWindow::setBlockerKick1(float ballx, float bally, int blocker1, float blocker1Distance)
{
    float targetX = LENGTH/2000;
    if(!check_game_direction) targetX*=-1;
    float resultRad = atan2f(ballx-targetX,bally-0.0);
    float result = atan2f(ballx-targetX,bally-0.0)*180/M_PI;

    result *=-1;

    float robotblocker1NewX = ballx - blocker1Distance*sin(resultRad);
    float robotblocker1NewY = bally - blocker1Distance*cos(resultRad);

    while(getInsideArea(robotblocker1NewX,robotblocker1NewY)){

        blocker1Distance -= 0.1;
        robotblocker1NewX = ballx - (blocker1Distance)*sin(resultRad);
        robotblocker1NewY = bally - (blocker1Distance)*cos(resultRad);


    }

    robo[blocker1]->setPosition(getx(robotblocker1NewX),gety(robotblocker1NewY));

    robo[blocker1]->setAngle(result);

    //float x, float y, int angle, int robotNumber
    setRTDBRobotTarget(robotblocker1NewX,robotblocker1NewY,result,blocker1);

    setDefensorRTDB(blocker1);

}

bool MainWindow::getInsideArea(float x, float y)
{
    if(fabs(x)>((LENGTH/2.0-AREA_LENGTH1)/1000.0)-0.2 && fabs(y)<(AREA_WIDTH1/2000.0)+0.2) return 1;
    else return 0;
}

void MainWindow::setGuardaRedesRTDB(int id)
{
    //0
    rtdb->set_var_local(91+id,1,0); //Guarda Redes
}

void MainWindow::setDefensorRTDB(int id)
{
    //1
    rtdb->set_var_local(91+id,1,1); //Defesa
}

void MainWindow::setPassadorRTDB(int id)
{
    //3
    rtdb->set_var_local(91+id,1,3); //Passador
}

void MainWindow::setRecetorRTDB(int id)
{
    //4
    rtdb->set_var_local(91+id,1,4); //Recetor
}

void MainWindow::setRematadorRTDB(int id)
{
    //5
    rtdb->set_var_local(91+id,1,5); //Rematador
}

void MainWindow::setPerseguidorAtacador(int id)
{
    //6
    rtdb->set_var_local(91+id,1,6); //PerseguidorAtacador
}

void MainWindow::closeEvent (QCloseEvent *event)
{
    qDebug() << "Closing BaseStation";
    event->accept();
}

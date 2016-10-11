#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include <QGraphicsLineItem>
#include <QTimer>
#include <QFontDatabase>
#include <QDebug>
#include <QMovie>
#include <QLabel>
#include <QGraphicsOpacityEffect>
#include <QPen>
#include <QTcpSocket>
#include <QHostAddress>
#include <QDir>
#include "RTDB_Module/rtdbmainclass.h"
#include "math.h"
#include <QLabel>
#include <vector>
#include <QCloseEvent>

using namespace std;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
public slots:

    int getFactor();

    int getRoboTSize();

    void setRobotLableAngle(int angle,int robot);

private slots:

    void readDimension(QString FileName);

    int getx(float meterX);

    int gety(float meterY);

    float getMeterx(int X);

    float getMetery(int Y);

    void on_bt_show_home_robots_clicked();

    void on_bt_show_away_robots_clicked();

    bool eventFilter(QObject *obj, QEvent *event);

    void HoverEnter(QString Name);

    void HoverLeave(QString Name);

    void SetOpacity(QPushButton *button, float value);

    void on_bt_show_grid_cells_clicked();

    void on_bt_show_pass_lines_clicked();

    void on_bt_show_shot_lines_clicked();

    void on_bt_show_obstacles_clicked();

    void drawLine(int x1, int y1, int x2, int y2, QGraphicsLineItem *line);

    void updatePassLine();

    void removeFromField(int number);

    void setOnField(int number);

    void readPendingDatagrams();

    void addTatics(QString FileName);

    void on_bt_game_direction_clicked();

    void setRobotPositions(int TaticNumber);

    void on_bt_save_position_clicked();

    void on_bt_load_position_clicked();

    void LoadTatics();

    void on_sl_robot1_angle_valueChanged(int value);

    void on_sl_robot2_angle_valueChanged(int value);

    void on_sl_robot3_angle_valueChanged(int value);

    void on_sl_robot4_angle_valueChanged(int value);

    void on_sl_robot5_angle_valueChanged(int value);

    void setRobotLable(QLabel *lable, bool state, QString number);

    void updateRTDB();

    void on_bt_rtdb_state_clicked();

    int getRTDBRobotAngle(int LSB,int MSB);

    void setRTDBRobotTarget(float x,float y,int angle, int robotNumber);

    QVariantList wrapJSONValues(float *val1, float *val2, float *val3);

    void senRef();

    void on_bt_main_loop_state_clicked();

    void updateMainLoop();

    bool ball_is_in_my_camp();

    void setRobotStart();

    void setRobotStop();

    void meshBalls();

    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void FreeKick(float ballx,float bally,int passer,int receiver,float passerDistance,float receiverDistance);

    void ThrowIn(float ballx,float bally,int passer,int receiver,float passerDistance,float receiverDistance);

    void Penalty(float ballx,float bally,int kicker,float kickerDistance);

    void GoalKickMinho(float ballx,float bally,int passer,int receiver,float passerDistance,float receiverDistance);

    void setGoalKeeper(int idGoalKeeper);

    void setDefense1(int Defenser);

    void setDefense2(int Defenser1,int Defenser2);

    void setBlockerKick2(float ballx,float bally,int blocker1,int blocker2,float blocker1Distance,float blocker2Distance);

    void setBlockerKick1(float ballx,float bally,int blocker1,float blocker1Distance);

    bool getInsideArea(float x,float y);

    void setGuardaRedesRTDB(int id);

    void setDefensorRTDB(int id);

    void setPassadorRTDB(int id);

    void setRecetorRTDB(int id);

    void setRematadorRTDB(int id);

    void setPerseguidorAtacador(int id);

    void closeEvent (QCloseEvent *event);

private:
    Ui::MainWindow *ui;

    QGraphicsScene *scene;

    QTimer *timerPassLine,*timerRTDB,*timerRef,*timerMainLoop;

    QPen pen_line;

    // Global Definitions
    int TOTAL_LENGTH;
    int TOTAL_WIDTH;
    int LENGTH;
    int WIDTH;
    int GOAL_WIDTH;
    int GOAL_LENGTH;
    int LINE_WIDTH;
    int CENTER_RADIUS;
    int SPOT_CENTER;
    int SPOTS;
    int AREA_LENGTH1;
    int AREA_WIDTH1;
    int AREA_LENGTH2;
    int AREA_WIDTH2;
    int DISTANCE_PENALTY;
    int RADIUS_CORNER;
    int ROBOT_DIAMETER;
    int BALL_DIAMETER;
    int FREE_KICK_SPACE;
    int FACTOR;// 1 pixel = 2 cm

    int robotOldPositionsx[10],robotOldPositionsy[10];

    bool check_show_home_robots,check_show_away_robots,check_show_grid_cells
            ,check_show_pass_lines,check_show_shot_lines,check_show_obstacles
            ,check_game_direction,check_RTDB,check_main_loop;

    int goal_target_y,static_goal_taget_x;

    QGraphicsLineItem *pass_line,*shot_line;
    QGraphicsEllipseItem *shot_circle;

    QTcpSocket *mysocket;

    struct nodo *robotNode;

    int nTaticsLoad;

    int minhoGoals,awayGoals;

    RTDBmainclass *rtdb;

    bool robo_state[5],robo_state_old[5],we_have_ball;

    int robotWithBall;

    bool goalKickMinhoSearch;

    int KickMinhoRobotPasser,KickMinhoRobotReceiver;

    int passerXpoint,passerXpoint_old,passerYpoint,passerYpoint_old;

    bool limitUpK;

};

#endif // MAINWINDOW_H

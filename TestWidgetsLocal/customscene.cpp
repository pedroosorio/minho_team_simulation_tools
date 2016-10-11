#include "customscene.h"
#include <QGraphicsSceneEvent>
#include <QMouseEvent>
#include "mainwindow.h"
#include <QDebug>

MainWindow *mainW;
int FACTOR;
CustomScene::CustomScene(MainWindow *mainWin)
{
    mainW = mainWin;
    FACTOR = mainWin->getFactor();
}

void CustomScene::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    if(event->button()==1)//Left Click
    {
    qDebug() << "x:"+QString::number(event->scenePos().rx()) + " y:"+QString::number(event->scenePos().ry());
    qDebug() << "xMeter:"+QString::number(event->scenePos().rx()*FACTOR/1000) + "m yMeter:"+QString::number(event->scenePos().ry()*FACTOR/1000)+"m";
    QGraphicsScene::mousePressEvent(event);
    }
    else if(event->button()==2)//Right Click
    {
        qDebug() << "Go to: xMeter:"+QString::number(event->scenePos().rx()*FACTOR/1000) + "m yMeter:"+QString::number(event->scenePos().ry()*FACTOR/1000)+"m";
    }
}


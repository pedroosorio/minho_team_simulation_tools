#ifndef ROBO1_H
#define ROBO1_H

#include <QPainter>
#include <QGraphicsItem>
#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QWheelEvent>
#include "mainwindow.h"

class Robo : public QGraphicsItem
{
public:
    Robo(MainWindow *mainw,int number);

    QRectF boundingRect() const;

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

    void setNumber(QString number);

    void setclicked();

    void setNotclicked();

    void setAngle(float angle);

    int  getAngle();

    void setColor(int red,int blue,int green);

    int width,height;

    void setPosition(int x,int y);

    void setBallCatch(bool state);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);

    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);

    QVariant itemChange(GraphicsItemChange change, const QVariant &value);

    void wheelEvent(QGraphicsSceneWheelEvent *event);

private:
    QString numberSet;

    bool pressed,has_ball;

    MainWindow *myMain;

    int x1,x2,y1,y2;

    QPen Pen;

    QFont font;

    float AngleDirection1,angleRaw;

    int myNumber;

    int diameter,factor,alfa,r,g,b;

    QPixmap pixmap3;
};

#endif // ROBO1_H

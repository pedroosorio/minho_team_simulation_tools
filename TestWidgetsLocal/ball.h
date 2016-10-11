#ifndef Ball_H
#define Ball_H

#include <QPainter>
#include <QGraphicsItem>
#include <QGraphicsScene>
#include "mainwindow.h"

class Ball : public QGraphicsItem
{
public:
    Ball(MainWindow *mainw, int number, int ball_size);
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    void setNumber(QString number);
    void setclicked();
    void setNotclicked();
    void setColor(int r,int g,int b);
    int width,height;
    void setPosition(int x, int y);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

private slots:
    QVariant itemChange(GraphicsItemChange change, const QVariant &value);
private:
    QString numberSet;

    bool pressed;

    MainWindow *myMain;

    int diameter,factor,myNumber,red,green,blue;
};

#endif // Ball_H

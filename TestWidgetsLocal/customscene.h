#ifndef CUSTOMSCENE_H
#define CUSTOMSCENE_H
#include "mainwindow.h"
#include <QGraphicsScene>

class CustomScene : public QGraphicsScene
{
public:
    CustomScene(MainWindow *mainw);
private slots:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
};

#endif // CUSTOMSCENE_H

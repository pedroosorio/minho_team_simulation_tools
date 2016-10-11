#ifndef DESENHARCAMPO_H
#define DESENHARCAMPO_H
#include <QGraphicsScene>

class DesenharCampo
{
public:
    DesenharCampo();

    void desenharLinhas(QGraphicsScene *scene, int LENGTH, int WIDTH, int GOAL_WIDTH, int GOAL_LENGTH, int LINE_WIDTH, int CENTER_RADIUS, int SPOT_CENTER, int SPOTS, int AREA_LENGTH1, int AREA_WIDTH1, int AREA_LENGTH2, int AREA_WIDTH2, int DISTANCE_PENALTY, int RADIUS_CORNER, int FACTOR);
};

#endif // DESENHARCAMPO_H

#include "desenharcampo.h"
#include <QGraphicsEllipseItem>
#include "math.h"
#include <QDebug>

DesenharCampo::DesenharCampo()
{
}

void DesenharCampo::desenharLinhas(QGraphicsScene *scene, int LENGTH, int WIDTH, int GOAL_WIDTH, int GOAL_LENGTH, int LINE_WIDTH, int CENTER_RADIUS, int SPOT_CENTER, int SPOTS, int AREA_LENGTH1, int AREA_WIDTH1, int AREA_LENGTH2, int AREA_WIDTH2, int DISTANCE_PENALTY, int RADIUS_CORNER, int FACTOR)
{
      int Xc=0;
      int Yc=0;

      QPen pen(QColor (255,255,255));
      pen.setWidth(LINE_WIDTH/FACTOR);

      // White Outside Lines
      scene->addRect((Xc-LENGTH/2+LINE_WIDTH/2)/FACTOR,(Yc-WIDTH/2+LINE_WIDTH/2)/FACTOR,(LENGTH-LINE_WIDTH)/FACTOR,(WIDTH-LINE_WIDTH)/FACTOR,pen);

      // White Central Circle
      scene->addEllipse(-(CENTER_RADIUS-LINE_WIDTH/2)/FACTOR, -(CENTER_RADIUS-LINE_WIDTH/2)/FACTOR, (CENTER_RADIUS-LINE_WIDTH/2)*2/FACTOR, (CENTER_RADIUS-LINE_WIDTH/2)*2/FACTOR,pen);

      // Center Line
      scene->addRect((Xc-LINE_WIDTH/2+LINE_WIDTH/2)/FACTOR,(Yc-WIDTH/2+LINE_WIDTH/2)/FACTOR,(1)/FACTOR,(WIDTH-LINE_WIDTH)/FACTOR,pen);

      // Goal Area 1 (smaller)
      scene->addRect((Xc-LENGTH/2+LINE_WIDTH/2)/FACTOR,             (Yc-AREA_WIDTH1/2+LINE_WIDTH/2)/FACTOR, (AREA_LENGTH1-LINE_WIDTH)/FACTOR,(AREA_WIDTH1-LINE_WIDTH)/FACTOR,pen);
      scene->addRect((Xc+LENGTH/2+LINE_WIDTH/2-AREA_LENGTH1)/FACTOR,(Yc-AREA_WIDTH1/2+LINE_WIDTH/2)/FACTOR, (AREA_LENGTH1-LINE_WIDTH)/FACTOR,(AREA_WIDTH1-LINE_WIDTH)/FACTOR,pen);

      // Goal Area 2 (larger)
      scene->addRect((Xc-LENGTH/2+LINE_WIDTH/2)/FACTOR,             (Yc-AREA_WIDTH2/2+LINE_WIDTH/2)/FACTOR, (AREA_LENGTH2-LINE_WIDTH)/FACTOR,(AREA_WIDTH2-LINE_WIDTH)/FACTOR,pen);
      scene->addRect((Xc+LENGTH/2+LINE_WIDTH/2-AREA_LENGTH2)/FACTOR,(Yc-AREA_WIDTH2/2+LINE_WIDTH/2)/FACTOR, (AREA_LENGTH2-LINE_WIDTH)/FACTOR,(AREA_WIDTH2-LINE_WIDTH)/FACTOR,pen);

      // White Dots
      QBrush b1(QColor(255,255,255));
      scene->addEllipse(-SPOT_CENTER/2/FACTOR,-SPOT_CENTER/2/FACTOR,SPOT_CENTER/FACTOR,SPOT_CENTER/FACTOR,pen,b1);
      scene->addEllipse(((Xc-LENGTH/2+DISTANCE_PENALTY)-SPOTS/2)/FACTOR,(Yc-SPOTS/2)/FACTOR,SPOTS/FACTOR,SPOTS/FACTOR,pen,b1);
      scene->addEllipse(((Xc+LENGTH/2-DISTANCE_PENALTY)-SPOTS/2)/FACTOR,(Yc-SPOTS/2)/FACTOR,SPOTS/FACTOR,SPOTS/FACTOR,pen,b1);

      // Goals
      b1.setColor(QColor(0,255,255));
      scene->addRect((Xc-LENGTH/2-GOAL_LENGTH+LINE_WIDTH/2)/FACTOR,(Yc-GOAL_WIDTH/2-LINE_WIDTH/2)/FACTOR,GOAL_LENGTH/FACTOR,GOAL_WIDTH/FACTOR,pen,b1);
      b1.setColor(QColor(255,255,0));
      scene->addRect((Xc+LENGTH/2-LINE_WIDTH/2)/FACTOR,            (Yc-GOAL_WIDTH/2-LINE_WIDTH/2)/FACTOR,GOAL_LENGTH/FACTOR,GOAL_WIDTH/FACTOR,pen,b1);

      // Black Spots
      QBrush b0(QColor(0,0,0));
      pen.setColor(QColor(0,0,0));
      scene->addEllipse(((Xc-LENGTH/2+DISTANCE_PENALTY)-SPOTS/2)/FACTOR,((Yc-DISTANCE_PENALTY)-SPOTS/2)/FACTOR,SPOTS/FACTOR,SPOTS/FACTOR,pen,b0);
      scene->addEllipse(((Xc+LENGTH/2-DISTANCE_PENALTY)-SPOTS/2)/FACTOR,((Yc-DISTANCE_PENALTY)-SPOTS/2)/FACTOR,SPOTS/FACTOR,SPOTS/FACTOR,pen,b0);
      scene->addEllipse(((Xc-LENGTH/2+DISTANCE_PENALTY)-SPOTS/2)/FACTOR,((Yc+DISTANCE_PENALTY)-SPOTS/2)/FACTOR,SPOTS/FACTOR,SPOTS/FACTOR,pen,b0);
      scene->addEllipse(((Xc+LENGTH/2-DISTANCE_PENALTY)-SPOTS/2)/FACTOR,((Yc+DISTANCE_PENALTY)-SPOTS/2)/FACTOR,SPOTS/FACTOR,SPOTS/FACTOR,pen,b0);

      pen.setColor(QColor(255,255,255));
      // Corner lines

      //arc((Xc-LENGTH/2)/FACTOR,(Yc-WIDTH/2)/FACTOR, (RADIUS_CORNER-LINE_WIDTH/2)*2/FACTOR, (RADIUS_CORNER-LINE_WIDTH/2)*2/FACTOR, 0+correcao, HALF_PI-correcao);

      QGraphicsEllipseItem *QEsqrC = new QGraphicsEllipseItem(((Xc-LENGTH/2)-(RADIUS_CORNER-LINE_WIDTH))/FACTOR,((Yc-WIDTH/2)-(RADIUS_CORNER-LINE_WIDTH))/FACTOR, (RADIUS_CORNER-LINE_WIDTH/2)*2/FACTOR, (RADIUS_CORNER-LINE_WIDTH/2)*2/FACTOR);
      int startAngle = 360*16;
      int endAngle = 270*16;
      QEsqrC->setStartAngle(startAngle);
      QEsqrC->setSpanAngle(endAngle - startAngle);
      QEsqrC->setPen(pen);
      scene->addItem(QEsqrC);

      //CantoRaio BaixoEsquerda
      QGraphicsEllipseItem *QEsqrB = new QGraphicsEllipseItem(((Xc-LENGTH/2)-(RADIUS_CORNER-LINE_WIDTH))/FACTOR,((Yc+WIDTH/2)-(RADIUS_CORNER))/FACTOR, (RADIUS_CORNER-LINE_WIDTH/2)*2/FACTOR, (RADIUS_CORNER-LINE_WIDTH/2)*2/FACTOR);
      startAngle = 0;
      endAngle = 90*16;
      QEsqrB->setStartAngle(startAngle);
      QEsqrB->setSpanAngle(endAngle - startAngle);
      QEsqrB->setPen(pen);
      scene->addItem(QEsqrB);

      //CantoRaio BaixoDir
      QGraphicsEllipseItem *QDirB = new QGraphicsEllipseItem(((Xc+LENGTH/2)-(RADIUS_CORNER))/FACTOR,((Yc+WIDTH/2)-(RADIUS_CORNER))/FACTOR, (RADIUS_CORNER-LINE_WIDTH/2)*2/FACTOR, (RADIUS_CORNER-LINE_WIDTH/2)*2/FACTOR);
      startAngle = 90*16;
      endAngle = 180*16;
      QDirB->setStartAngle(startAngle);
      QDirB->setSpanAngle(endAngle - startAngle);
      QDirB->setPen(pen);
      scene->addItem(QDirB);

      //CantoRaio CimaDireita
      QGraphicsEllipseItem *QDirC = new QGraphicsEllipseItem(((Xc+LENGTH/2)-(RADIUS_CORNER))/FACTOR,((Yc-WIDTH/2)-(RADIUS_CORNER-LINE_WIDTH))/FACTOR, (RADIUS_CORNER-LINE_WIDTH/2)*2/FACTOR, (RADIUS_CORNER-LINE_WIDTH/2)*2/FACTOR);
      startAngle = 180*16;
      endAngle = 270*16;
      QDirC->setStartAngle(startAngle);
      QDirC->setSpanAngle(endAngle - startAngle);
      QDirC->setPen(pen);
      scene->addItem(QDirC);

      QFont font;
      font.setFamily("Arial");
      font.setPointSize( 12 );


      scene->addText("Referee Box",font)->setPos(-60,(WIDTH/2+50)/FACTOR);


}

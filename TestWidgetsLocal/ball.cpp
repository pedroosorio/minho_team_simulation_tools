#include "ball.h"

Ball::Ball(MainWindow *mainw, int number,int ball_size)
{
    myNumber = number;
    diameter = ball_size;
    factor = mainw->getFactor();
    myMain = mainw;
    pressed = false;
    setFlag(ItemIsSelectable);
    setFlag(ItemIsMovable);
    setFlag(ItemSendsScenePositionChanges);
    width = diameter/factor;//23cm
    height = diameter/factor;//23cm
    red = 255;
    green = 102;
    blue = 0;
}

QRectF Ball::boundingRect() const
{
    return QRectF(0,0,width,height);
}

void Ball::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);

    QPainterPath paht;
    paht.addEllipse(boundingRect());

    QBrush brush(Qt::gray);
    QFont font("Arial");
    font.setPointSize(6);

    if(pressed) brush.setColor(Qt::gray);
    else brush.setColor(QColor(red,green,blue));

    painter->setBrush(brush);
    painter->drawPath(paht);

    if(numberSet!="")
    {
        painter->setFont(font);
        painter->setPen(QPen(QColor(0,0,0)));
        painter->drawText(width/3,height/1.2,numberSet);
    }
}

void Ball::setNumber(QString number)
{
    numberSet = number;
}

void Ball::setclicked()
{
    pressed = true;
}

void Ball::setNotclicked()
{
    pressed = false;
}

void Ball::setColor(int r, int g, int b)
{
    red = r;
    green = g;
    blue = b;
}

void Ball::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    pressed = true;
    update();
    QGraphicsItem::mousePressEvent(event);
}

void Ball::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    pressed = false;
    update();
    QGraphicsItem::mouseReleaseEvent(event);
}

QVariant Ball::itemChange(GraphicsItemChange change, const QVariant &value)
{
    if (change == ItemPositionChange && scene()) {
            // value is the new position.
            QPointF newPos = value.toPointF();
            QRectF rect = scene()->sceneRect();
            rect.setX(rect.x()-(width/2));
            rect.setWidth(rect.width()-(width/2));
            rect.setY(rect.y()-(height/2)+2);
            rect.setHeight(rect.height()-(height/2)-2);

            if (!rect.contains(newPos)) {
                // Keep the item inside the scene rect.
                newPos.setX(qMin(rect.right()-(diameter/2)/factor, qMax(newPos.x(), rect.left()+(diameter/2)/factor)));
                newPos.setY(qMin(rect.bottom()-(diameter/2)/factor, qMax(newPos.y(), rect.top()+(diameter/2)/factor)));
                return newPos;
            }
        }
    return QGraphicsItem::itemChange(change, value);
}

void Ball::setPosition(int x, int y)
{
    this->setPos(x-width/2,y-height/2);
}

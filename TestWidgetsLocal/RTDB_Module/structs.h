#ifndef STRUCTS_H
#define STRUCTS_H

#include <QObject>
#include <QTime>

struct var
{
    int id;							// id da 'variavel'
    int agent;                      // agente
    int size;						// sizeof da 'variavel'
    int period;						// refresh period for broadcast
    int offset;                    // indice apontador para data
    QTime timecreated;              // tempo em que foi
    QTime timereceved;              // tempo em que foi recevido
    var(){id=-1;agent=-1;size=-1;period=-1;offset=-1;}
};



#endif


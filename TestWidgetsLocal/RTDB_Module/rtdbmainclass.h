#ifndef RTDBMAINCLASS_H
#define RTDBMAINCLASS_H
#include "rtdbclient.h"
#include "filewriter.h"
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QUdpSocket>
#include <QDebug>
#include <QObject>
#include <QFile>
#include <QTimer>
#include <QByteArray>
#include <QMutex>
#include <QLinkedList>
#include <QThread>
#include "sender_bythread.h"
#include "cicle_iterator.h"
#include "externalrtdb.h"

class RTDBmainclass : public RTDBClient
{
    Q_OBJECT
public:
    explicit RTDBmainclass();
    void udp_cfg();

private slots:

private:
    quint16 my_port;
    quint16 bs_port;
    quint16 broadcast_port;
    QHostAddress broadcast_ip;
    QHostAddress bs_ip;
    QHostAddress my_ip;
    QUdpSocket *send_socket;
    QByteArray buffer;
    QString data_send[80];
    int data_iterator;
    int cicle_iterator_agents;
//CICLE
    cicle_iterator *cicle;
    QThread *thread_cicle;

//FILE

//SENDER
    sender_bythread *senderpointer;
    QThread *senderthread;

//EXTERNAL
public:
    externalRTDB *externalpointer; //publico para DEBUG
    QThread *externalThread;
};

#endif // RTDBMAINCLASS_H

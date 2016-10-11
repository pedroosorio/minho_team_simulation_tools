#ifndef CICLE_ITERATOR_H
#define CICLE_ITERATOR_H
#include <QObject>
#include <QLinkedList>
#include <QLinkedListIterator>
#include <QThread>
#include <QString>
#include <QTextStream>
#include <QTime>
#include <QDebug>
#include "structs.h"
#include "sender_bythread.h"
#include <QDebug>
#define TIMEOUT 10

class cicle_iterator : public QObject
{
    Q_OBJECT
public:
    explicit cicle_iterator(var *local_vars, int *cicle_iterator_agent , int * data_iterator, QString *data_send, int self_agent, int max_agent, QChar * local_memory_data, QLinkedList<int> *list , sender_bythread * pointer, QObject *parent = 0);
private:
    QString *data_send;
    int *data_iterator;
    int self_agent;
    int max_agent;
    QLinkedList<int> *list;
    int *cicle_iterator_agents;
    QThread *thread;
    QChar * local_memory_data;
    var * local_vars;
    sender_bythread *pointer;
    void send();
    void send_var(int );
    void send_end();
    QString get_end_data();
    QString get_QString_var(int id);

signals:

public slots:
    void doSetup(QThread *t);
private slots:
    void doWork();
};

#endif // CICLE_ITERATOR_H


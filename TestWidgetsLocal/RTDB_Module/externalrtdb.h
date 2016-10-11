#ifndef EXTERNALRTDB_H
#define EXTERNALRTDB_H
#include <QObject>
#include "structs.h"
#include <QString>
#include <QTextStream>
#include <QDebug>
#include <QUdpSocket>
#include "filewriter.h"
#include <QEventLoop>
#include <QTimer>


class externalRTDB : public QObject
{
    Q_OBJECT
private:
    void set_var_shared(QString a);
    void set_new_var_shared(var aux, QChar data);
    void set_update_var_shared(var aux, QChar data);
    bool checkvarshared(var a);

private:
    QChar * shared_memory_data;
    var** shared_vars;
    int *shared_memory_data_i;
    int *n_shared_recs;
    QUdpSocket *socket;
    quint16 my_port;
    QHostAddress broadcast_ip;
    int self_agent;
    QByteArray *buffer;
    int *cicle_iterator_agents;
    void set_var_on(int id);
    void set_var_off(int id);

public:
    explicit externalRTDB(int self_agent, int *cicle_iterator_agents, QByteArray *buffer, QHostAddress broadcast_ip, quint16 my_port, QChar * shared_memory_data, var ** shared_vars, int * shared_memory_data_i, int *n_shared_recs, QObject *parent = 0);

signals:
    void ready_signal();

private slots:
    void readyREAD();
    void doWork();
public slots:
    void doSetup(QThread *t);
};

#endif // EXTERNALRTDB_H

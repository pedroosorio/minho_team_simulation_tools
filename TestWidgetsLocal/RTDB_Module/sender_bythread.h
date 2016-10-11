#ifndef SENDER_BYTHREAD_H
#define SENDER_BYTHREAD_H
#include <QObject>
#define SEND_TIME 2
#include <QObject>
#include <QHostAddress>
#include <QUdpSocket>
#include <QMutex>
#include <QTimer>
#include <QThread>

class sender_bythread : public QObject
{
    Q_OBJECT
public:
    explicit sender_bythread(QString *buffer, QHostAddress*address, quint16*port, QUdpSocket *socket, int *data_iterator, QObject *parent = 0);
signals:

private:
    void send_var(int i);
    void write_socket_end();
    QHostAddress *senderadress;
    quint16 *senderport;
    QUdpSocket *socket;
    QString *buffer;
    int * data_iterator;
    bool send_flag;
    QThread *thread;
public:
    void set_send_flag(bool a);
    void write_socket();

public slots:
    void doSetup(QThread *t);
private slots:
    void doWork();

};

#endif // SENDER_BYTHREAD_H

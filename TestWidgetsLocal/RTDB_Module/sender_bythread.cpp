#include "sender_bythread.h"

sender_bythread::sender_bythread(QString *buffer, QHostAddress *address, quint16 *port, QUdpSocket *socket, int *data_iterator, QObject *parent) : QObject(parent)
{
    this->buffer=buffer;
    this->senderadress=address;
    this->senderport=port;
    this->socket=socket;
    this->data_iterator=data_iterator;
    send_flag= false;
}

void sender_bythread::set_send_flag(bool a)
{
    send_flag=a;
}



void sender_bythread::write_socket()
{

    if(!this->socket)
       {
           qDebug() << "RTDBmainclass::write_socket ERROR" << this;
       }
       else
       {
           QMutex a;
           a.lock();
           QByteArray byteArray(buffer[*data_iterator].toStdString().c_str(), buffer[*data_iterator].toStdString().length());
           this->socket->writeDatagram(byteArray,*senderadress,*senderport);

           if((*data_iterator)>-1)
           {
               (*data_iterator)--;
           }

           if((*data_iterator)==-1)
           {
                send_flag=false;
           }
           else
           {
               thread->msleep(SEND_TIME);
           }
           a.unlock();

    }
}
void sender_bythread::doSetup(QThread *t)
{
    connect(t,SIGNAL(started()),this,SLOT(doWork()));
    thread = t;
}

void sender_bythread::doWork()
{
    while(1){
        if(send_flag)
        {
        QMutex b;
        if(b.tryLock(30))
        {
            write_socket();
            b.unlock();
        }

        }
    }
}


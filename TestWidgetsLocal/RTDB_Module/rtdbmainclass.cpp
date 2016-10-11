#include "rtdbmainclass.h"

RTDBmainclass::RTDBmainclass()
{

    this->config_RTDBClient();
    this->udp_cfg();
}

void RTDBmainclass::udp_cfg()
{
    QFile file(pathconfigtcp);
    file.open(QIODevice::ReadOnly);

    if(file.isOpen()){
        QByteArray rawData = file.readAll();
        QJsonDocument Jfile(QJsonDocument::fromJson(rawData));
        QJsonObject json = Jfile.object();
        this->bs_port=json["BS_port"].toInt();
        this->bs_ip.setAddress(json["BS_ip"].toString());
        this->my_ip.setAddress(json["robot_ip"].toString());
        this->my_port=json["robot_port"].toInt();
        this->broadcast_ip.setAddress(json["broadcast_ip"].toString());
        this->broadcast_port=json["broadcast_port"].toInt();

    }
    else
    {
        qDebug() << "UCPConfig: File Not Open";
    }

    this->send_socket = new QUdpSocket(this);

    this->send_socket->connectToHost(broadcast_ip,broadcast_port);

    data_iterator=0;

    cicle_iterator_agents=0;
    senderpointer = new sender_bythread(data_send, &broadcast_ip, &broadcast_port, send_socket, &data_iterator);
    senderthread = new QThread();
    senderpointer->doSetup(senderthread);
    senderpointer->moveToThread(senderthread);
    senderthread->start();
    senderthread->setPriority(QThread::NormalPriority);

    cicle= new cicle_iterator(this->local_vars,&cicle_iterator_agents,&data_iterator,data_send,self_agent,MAX_AGENTS,local_memory_data,lista,senderpointer);
    thread_cicle = new QThread();
    cicle->doSetup(thread_cicle);
    cicle->moveToThread(thread_cicle);
    thread_cicle->start();
    thread_cicle->setPriority(QThread::NormalPriority);

    externalpointer = new externalRTDB(self_agent,&cicle_iterator_agents,&buffer,broadcast_ip,my_port,shared_memory_data,shared_vars,shared_memory_data_i,n_shared_recs);
    externalThread = new QThread();
    externalpointer->doSetup(externalThread);
    externalpointer->moveToThread(externalThread);
    externalThread->start();
    externalThread->setPriority(QThread::NormalPriority);

}


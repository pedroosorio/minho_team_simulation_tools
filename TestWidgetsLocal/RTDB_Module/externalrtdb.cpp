#include "externalrtdb.h"

externalRTDB::externalRTDB(int self_agent, int *cicle_iterator_agents, QByteArray *buffer, QHostAddress broadcast_ip, quint16 my_port, QChar *shared_memory_data, var **shared_vars, int *shared_memory_data_i, int *n_shared_recs, QObject *parent)
{
    Q_UNUSED(parent);
    this->shared_memory_data=shared_memory_data;
    this->shared_vars=shared_vars;
    this->shared_memory_data_i=shared_memory_data_i;
    this->n_shared_recs=n_shared_recs;
    this->my_port=my_port;
    this->broadcast_ip=broadcast_ip;
    this->socket= new QUdpSocket(this);
    this->socket->bind(QHostAddress::AnyIPv4,this->my_port,QUdpSocket::ShareAddress);
    this->buffer=buffer;
    socket->joinMulticastGroup(broadcast_ip);
    this->cicle_iterator_agents=cicle_iterator_agents;
    this->self_agent=self_agent;
    connect(socket,SIGNAL(readyRead()),this,SLOT(readyREAD()));
}

void externalRTDB::set_new_var_shared(var aux, QChar data)
{
    static int offset=0;
    if((aux.id) > ((this->n_shared_recs[aux.agent])-1))
       {
        qDebug()<< "RTDBClient:set_new_var_shared:id varible error" ;
        return;
    }


    shared_vars[aux.agent][aux.id].agent=aux.agent;
    shared_vars[aux.agent][aux.id].id=aux.id;
    shared_vars[aux.agent][aux.id].period=aux.period;
    shared_vars[aux.agent][aux.id].offset=offset;
    shared_vars[aux.agent][aux.id].size=aux.size;
    shared_vars[aux.agent][aux.id].timecreated=aux.timecreated;
    shared_vars[aux.agent][aux.id].timereceved=aux.timereceved;

    shared_memory_data[offset]=data;
    offset++;

    /*while(i<shared_vars[aux.agent][aux.id].size)
    {
    shared_memory_data[offset]=(data.data())[i];
    //qDebug()<< "RTDBClient:set_new_var_shared:data:"<<shared_memory_data[offset];
    offset++;
    i++;
    }

    */
    shared_memory_data_i[aux.agent]++;

}

void externalRTDB::set_update_var_shared(var aux, QChar data)
{
    if(aux.size!=shared_vars[aux.agent][aux.id].size)
    {
        //qDebug() << "RTDBClient:set_update_var_shared:ERROR!!";
        return;
    }
    else
    {
        shared_vars[aux.agent][aux.id].timecreated=aux.timecreated;
        shared_vars[aux.agent][aux.id].timereceved=aux.timereceved;
        shared_memory_data[shared_vars[aux.agent][aux.id].offset]=data;

       /* while(i!=shared_vars[aux.agent][aux.id].size)
        {
        shared_memory_data[shared_vars[aux.agent][aux.id].offset + i]=(data.data())[i];
        //qDebug()<< "RTDBClient:set_update_var_shared:data:"<<shared_memory_data[shared_vars[aux.agent][aux.id].offset + i];
        i++;

        }
        */
    }
}
void externalRTDB::set_var_shared(QString a)
{
    QTextStream sstr(&a);
    var aux;
    QString time;
    QChar data;
    QChar lixo;
    char auxi[4];
    sstr >> auxi[0] >> lixo>> auxi[1] >> lixo >> auxi[2] >> lixo >> auxi[3]  >> lixo >> time >> lixo >> data;
    aux.agent=int(auxi[0]);
    aux.id=int(auxi[1]);
    aux.period=int(auxi[2]);
    aux.size=int(auxi[3]);
    aux.timecreated=QTime::fromString(time,"hh:mm:ss:zzzz");
    aux.timereceved=QTime::currentTime();


    if ((-1<aux.agent) && (aux.agent<6))
    {

        if(aux.agent>-1 && aux.agent<7 && aux.period>0 && aux.period<11 && aux.size>-1){

            if(checkvarshared(aux))
            {
                set_new_var_shared(aux,data);
            }
            else
            {
                set_update_var_shared(aux,data);
                //qDebug() << aux.agent << "Update" << aux.id << data;
            }
        }
        else {
            qDebug()<< "RTDBClient:set_var_shared:ERROR";
            return;
        }


    }
    else {
        qDebug()<< "RTDBClient:set_var_shared:ERROR";
        return;

    }
}

bool externalRTDB::checkvarshared(var a)
{
    if(shared_vars[a.agent][a.id].agent==a.agent)
    {
        if(shared_vars[a.agent][a.id].id==a.id)
        {
            return false;
        }
    }

    return true;
}

void externalRTDB::set_var_on(int id)
{
    var aux;
    aux.agent=id;
    aux.id=75;
    aux.period=1;
    aux.size=1;
    aux.timecreated=QTime::currentTime();
    aux.timereceved=QTime::currentTime();

    QChar a=1;

    if(checkvarshared(aux))
    {
        set_new_var_shared(aux,a);
    }
    else
    {
        set_update_var_shared(aux,a);
    }
}

void externalRTDB::set_var_off(int id)
{
    var aux;
    aux.agent=id;
    aux.id=75;
    aux.period=1;
    aux.size=1;
    aux.timecreated=QTime::currentTime();
    aux.timereceved=QTime::currentTime();

    QChar a=99;

    if(checkvarshared(aux))
    {
        set_new_var_shared(aux,a);
    }
    else
    {
        set_update_var_shared(aux,a);
    }
}


void externalRTDB::readyREAD()
{
    QMutex z;
    if(z.tryLock(30))
    {
        QHostAddress sender;
        quint16 sender_port;
        QTime timer;
        timer.start();
        buffer->resize(socket->pendingDatagramSize());
        socket->readDatagram((buffer->data()),buffer->size(),&sender,&sender_port);

        int a = int(buffer->data()[0]);
        static int i = 0,on_ids[5]={0};


        if(0==(int(buffer->data()[2])))
        {
            if(0==(int(buffer->data()[4])))
            {

                if(a==0)
                {
                    if(i==5)
                    {
                        int loop=0;
                       while(loop<5)
                        {
                           var aux;
                           aux.agent=loop+1;
                           aux.id=61+loop;
                           aux.period=1;
                           aux.size=1;
                           aux.timecreated=QTime::currentTime();
                           aux.timereceved=QTime::currentTime();

                           if(on_ids[loop]==1){
                               if(checkvarshared(aux))
                               {

                                   set_new_var_shared(aux,1);
                               }
                               else
                               {
                                   set_update_var_shared(aux,1);
                               }
                           }
                           else
                           {
                               if(checkvarshared(aux))
                               {
                                   set_new_var_shared(aux,0);
                               }
                               else
                               {
                                   set_update_var_shared(aux,0);
                               }
                           }

                           loop++;
                        }
                        on_ids[0]=0;
                        on_ids[1]=0;
                        on_ids[2]=0;
                        on_ids[3]=0;
                        on_ids[4]=0;
                        i=-1;
                    }
                    i++;
                }
            }
        }



        if(a!=self_agent && (a<6) && (a>-1))
        {

            if(0==(int(buffer->data()[2])))
            {
                if(0==(int(buffer->data()[4])))
                {

                      (*cicle_iterator_agents)=(int(buffer->data()[0])+1);
                       on_ids[a-1]=1;
                 }
            }
            else
            {
                ((*buffer)[0])=((*buffer)[0])+1;
                                QString info(*buffer);
                                (info.data()[0])=(info.data()[0].cell())-1;
                                set_var_shared(info);
            }
            emit ready_signal();
        }
        z.unlock();
    }
}

void externalRTDB::doWork()
{
    while(1)
    {
        QEventLoop loop;
        connect(socket, SIGNAL(readyRead()), &loop, SLOT(quit()));
        loop.exec();
    }
}

void externalRTDB::doSetup(QThread *t)
{
    connect(t,SIGNAL(started()),this,SLOT(doWork()));
}

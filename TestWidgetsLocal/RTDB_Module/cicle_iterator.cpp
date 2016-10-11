#include "cicle_iterator.h"

cicle_iterator::cicle_iterator(var *local_vars,int *cicle_iterator_agent, int *data_iterator, QString *data_send, int self_agent, int max_agent, QChar *local_memory_data, QLinkedList<int> *list, sender_bythread *pointer, QObject *parent)
{
    Q_UNUSED(parent);
    this->list=list;
    this->self_agent=self_agent;
    this->max_agent=max_agent;
    this->cicle_iterator_agents=0;
    this->local_memory_data=local_memory_data;
    this->local_vars=local_vars;
    this->pointer=pointer;
    this->data_iterator=data_iterator;
    this->data_send=data_send;
    this->cicle_iterator_agents=cicle_iterator_agent;
}

void cicle_iterator::send()
{
    QMutex z;
    if(z.tryLock(30))
    {
        static int i=1;

        send_end();

        foreach(int s, list[i]) send_var(s);

        if(i==10) i=1;
        else i++;
      }
    z.unlock();
    pointer->set_send_flag(true);

}

void cicle_iterator::send_var(int i)
{
    //qDebug()<< get_QString_var(i);
    (*data_iterator)++;
    data_send[(*data_iterator)]=get_QString_var(i);
}

void cicle_iterator::send_end()
{
    if((*data_iterator)<79)(*data_iterator)++;
    data_send[(*data_iterator)]=get_end_data();
}

QString cicle_iterator::get_end_data()
{
    QString stringreturn;
    QTextStream sstr(&stringreturn);
    var aux;
    aux.agent=self_agent;
    aux.id=0;
    aux.period=0;
    aux.size=0;
    aux.timecreated=QTime::currentTime();
    sstr << char(aux.agent) << " "<< char(aux.id) << " " << char(aux.period) << " " << char(aux.size) << " " << aux.timecreated.toString("hh:mm:ss:zzzz") << '\n';
    return stringreturn;
}

QString cicle_iterator::get_QString_var(int id)
{
    QString stringreturn;
    QTextStream sstr(&stringreturn);
    QString data;

   for(int i=0; i<this->local_vars[id].size; i++) data.append(local_memory_data[this->local_vars[id].offset+i]);

   sstr << char(this->local_vars[id].agent) << " "<< char(this->local_vars[id].id) << " " << char(this->local_vars[id].period) << " " << char(this->local_vars[id].size) << " " << this->local_vars[id].timecreated.toString("hh:mm:ss:zzzz") << " " << data << '\n';
   return stringreturn;
}

void cicle_iterator::doSetup(QThread *t)
{
    connect(t,SIGNAL(started()),this,SLOT(doWork()));
    thread=t;
}

void cicle_iterator::doWork()
{
    //qDebug() << "Thread-Work cicle-iterator";
    while(1){

        if ((*cicle_iterator_agents)==(self_agent)) this->send();// aqui////////

        if((*cicle_iterator_agents)==(max_agent)) (*cicle_iterator_agents)=-1;

        //qDebug() << (*cicle_iterator_agents);
        (*cicle_iterator_agents)++;
        this->thread->msleep(TIMEOUT);
        }

}



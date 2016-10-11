#include "rtdbclient.h"

void RTDBClient::set_new_var_local( var aux, QString data)
{
    int i=0;
    static int offset=0;
    if((aux.id) > ((this->n_local_recs)-1))
       {
        qDebug()<< "RTDBClient:set_new_var_local:id varible error" ;
        return;
    }

    if(data.length()==aux.size)
    {
        local_vars[aux.id].agent=aux.agent;
        local_vars[aux.id].id=aux.id;
        local_vars[aux.id].period=aux.period;
        local_vars[aux.id].offset=offset;
        local_vars[aux.id].size=aux.size;
        local_vars[aux.id].timecreated=aux.timecreated;
        local_vars[aux.id].timereceved=aux.timereceved;

        while(i<local_vars[aux.id].size)
        {
        local_memory_data[offset]=(data.data())[i];
        offset++;
        i++;
        }

        local_memory_data_i++;
    }
    else
    {
        qDebug()<< "RTDBClient:set_new_var_local:ERROR datasize";
    }
}

void RTDBClient::set_update_var_local(var aux, QString data)
{
    int i=0;
    if(aux.size!=local_vars[aux.id].size)
    {
        qDebug() << "RTDBClient:set_update_var_local:ERROR!!";
        return;
    }
    else
    {
        local_vars[aux.id].timecreated=aux.timecreated;
        local_vars[aux.id].timereceved=aux.timereceved;
        while(i!=local_vars[aux.id].size)
        {
        local_memory_data[local_vars[aux.id].offset + i]=(data.data())[i];
        i++;

        }
    }
    return;
}

bool RTDBClient::checkvarlocal(var a)
{
        if(local_vars[a.id].id==a.id)
        {
            return false;
        }

    return true;
}

RTDBClient::RTDBClient(QObject *parent) : QObject(parent)
{
    this->shared_vars=NULL;
    this->local_vars=NULL;
    this->shared_memory_data=NULL;
    this->local_memory_data=NULL;
    local_memory_data_i=0;
    for(int i=0;i<MAX_AGENTS;i++)
    {
      shared_memory_data_i[i]=0;
    }

    lista = new QLinkedList<int> [11];

    lista[0].clear();
    lista[1].clear();
    lista[2].clear();
    lista[3].clear();
    lista[4].clear();
    lista[5].clear();
    lista[6].clear();
    lista[7].clear();
    lista[8].clear();
    lista[9].clear();
    lista[10].clear();



}

 void RTDBClient::ResetRTDB()
{
     memset(shared_memory_data,0,this->sizeof_memoryshared_data*sizeof(QChar));
     memset(local_memory_data,0,local_mem_size*sizeof(QChar));
     memset(local_vars,0,n_local_recs*sizeof(var));
     memset(shared_vars,0,n_agents*n_shared_recs[1]*sizeof(var));

     this->local_memory_data_i=0;
     memset(this->shared_memory_data_i,0,sizeof(int)*MAX_AGENTS);
}

RTDBClient::~RTDBClient()
{
    free(local_vars);
    free(shared_vars);
    free(local_memory_data);
    free(shared_memory_data);
}

void RTDBClient::config_RTDBClient()
{
    QFile file(pathconfigrtdbclient);
    int sizeof_memoryshared_data=0;
    file.open(QIODevice::ReadOnly);
    if(file.isOpen()){
        QByteArray rawData = file.readAll();
        QJsonDocument Jfile(QJsonDocument::fromJson(rawData));
        QJsonObject json = Jfile.object();
        QJsonArray jsonarray;

        this->self_agent=json["self_agent"].toInt();
        this->n_agents=json["n_agents"].toInt();
        this->n_local_recs=json["n_local_recs"].toInt();
        this->local_mem_size=json["local_mem_size"].toInt();

        jsonarray=json["shared_mem_size"].toArray();

        sizeof_memoryshared_data+=this->shared_mem_size[0]=jsonarray[0].toInt();
        sizeof_memoryshared_data+=this->shared_mem_size[1]=jsonarray[1].toInt();
        sizeof_memoryshared_data+=this->shared_mem_size[2]=jsonarray[2].toInt();
        sizeof_memoryshared_data+=this->shared_mem_size[3]=jsonarray[3].toInt();
        sizeof_memoryshared_data+=this->shared_mem_size[4]=jsonarray[4].toInt();
        sizeof_memoryshared_data+=this->shared_mem_size[5]=jsonarray[5].toInt();

        jsonarray=json["n_shared_recs"].toArray();

        this->n_shared_recs[0]=jsonarray[0].toInt();
        this->n_shared_recs[1]=jsonarray[1].toInt();
        this->n_shared_recs[2]=jsonarray[2].toInt();
        this->n_shared_recs[3]=jsonarray[3].toInt();
        this->n_shared_recs[4]=jsonarray[4].toInt();
        this->n_shared_recs[5]=jsonarray[5].toInt();

        //Criar dinamicamente a memoria partilhada e local
        this->shared_memory_data = new QChar[sizeof_memoryshared_data];
        this->local_memory_data = new QChar[this->local_mem_size];

        //Criar dinamicamente memoria para as variaveis partilhadas e locais
        this->shared_vars= new var*[this->n_agents];
        this->local_vars=new var[this->n_local_recs];

        for(int i=0; i<n_agents;i++)
        {
           this->shared_vars[i] = new var[n_shared_recs[i]];
        }
        this->sizeof_memoryshared_data=sizeof_memoryshared_data;

    }
    else
    {
        qDebug() << "RTDBClient:config_RTDBCliente: File Not Open";
    }

}

void RTDBClient::set_n_agents(int n_agents)
{
    this->n_agents=n_agents;
}

void RTDBClient::set_var_local(int id,int period, int value)
{
    var aux;
    aux.agent=self_agent;
    aux.id=id;
    aux.period=period;
    aux.size=1;
    aux.timereceved=QTime::currentTime();
    aux.timecreated=QTime::currentTime();
    QString data = "";
    data.append(QChar((int)value));
       if(checkvarlocal(aux))
       {
           if(period<1)
           {
               qDebug()<< "RTDBClient:set_var_local:ERROR period";
           }
           else
           {
           set_new_var_local(aux,data);
           this->set_cicle_period(period,(id));
           }
       }
       else
       {
           set_update_var_local(aux,data);
       }
}

int RTDBClient::get_var(int id, int agent)
{
    int offset_aux=0;

    if(id<0 && (id > ((this->n_shared_recs[agent])-1)))
    {
       qDebug()<< "RTDBClient:get_var:id ERROR";
    }
    else
    {
        if(agent<0 && (agent > MAX_AGENTS))
        {
           qDebug()<< "RTDBClient:get_var:agent ERROR";
        }
        else
            {
                offset_aux=shared_vars[agent][id].offset;
                if(offset_aux<0)
                {
                   return 0;
                }
                else
                {
                    return convert(shared_memory_data[offset_aux].cell());
                    /*aux_string.append(shared_memory_data[offset_aux]);
                    while(aux_int!=shared_vars[agent][id].size)
                    {
                        aux_string.append(shared_memory_data[offset_aux+aux_int]);
                        aux_int++;
                    }*/

                }
            }
    }
    return 0;
}

QString RTDBClient::get_QString_var(int b)
{
    QString stringreturn;
    QTextStream sstr(&stringreturn);
    QString data;

   for(int i=0; i<this->local_vars[b].size; i++)
   {
       data.append(local_memory_data[this->local_vars[b].offset+i]);

   }

   sstr << char(this->local_vars[b].agent) << " "<< char(this->local_vars[b].id) << " " << char(this->local_vars[b].period) << " " << char(this->local_vars[b].size) << " " << this->local_vars[b].timecreated.toString("hh:mm:ss:zzzz") << " " << data << '\n';
   return stringreturn;
}


QString RTDBClient::get_end_data()
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

void RTDBClient::set_cicle_period(int period, int var)
{
    int j=0;

    if( period<11 && period>0)
    {

        while(j<N_Cicle)
        {
           lista[j].append(var);

            j=period+j;

        }
     }
    else
    {
        qDebug() << "RTDBClient:set_cicle_period:ERROR period error";
    }
}

void RTDBClient::clean_cicle()
{
    //
}

int RTDBClient::get_vars_cicle(int i, int j)
{
   if(i==j)return 1;
   return 0;
}

void RTDBClient::printcicle()
{

}

bool RTDBClient::checkend(QString a)
{
    var aux;

    if(aux.period==0 && aux.size==0)
    {
        return true;

    }
    return false;
}

int RTDBClient::convert(int number)
{
    int base = number&0x7F;
    if((number>>7)&0x01) {
        base -=128;
    }
    return base;
}


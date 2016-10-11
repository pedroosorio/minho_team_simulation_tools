#ifndef RTDBCLIENT_H
#define RTDBCLIENT_H
#define MAX_AGENTS 6
#define N_Cicle 10
#include <QObject>
#include "paths.h"
#include "structs.h"
#include <QJsonDocument>
#include <QJsonObject>
#include <qjsonarray.h>
#include <QFile>
#include <QDebug>
#include <QTimer>
#include <QLinkedList>

class RTDBClient : public QObject
{
    Q_OBJECT
protected:
        int self_agent;						// numero do agente onde esta a correr
        int n_agents;						// numero total de agentes registados
        int n_shared_recs[MAX_AGENTS];		// numero de 'variaveis' shared
        int n_local_recs;					// numero de 'variaveis' local
        int shared_mem_size[MAX_AGENTS];	// tamanho das areas shared
        int local_mem_size;					// tamanho da area local
        int sizeof_memoryshared_data;

        QChar * local_memory_data;
        QChar * shared_memory_data;
        var** shared_vars;
        var* local_vars;
        int local_memory_data_i;
        int shared_memory_data_i[MAX_AGENTS];

        void set_new_var_shared(var aux, QString data);
        void set_update_var_shared(var aux, QString data);
        void set_new_var_local(var aux, QString data);
        void set_update_var_local(var aux, QString data);
        bool checkvarshared(var a);
        bool checkvarlocal(var a);

        /*cicle of vars*/
        QLinkedList<int> *lista;
        void set_cicle_period(int period, int var);
        void clean_cicle();
        int get_vars_cicle(int, int j);
        void printcicle();
        bool checkend(QString a);

        int convert(int number);
public:
        //void set_var_shared(QString a);
        explicit RTDBClient(QObject *parent = 0);
        ~RTDBClient();
        void ResetRTDB();
        void config_RTDBClient();
        void set_n_agents(int n_agents);
        void set_var_local(int id , int period, int value); //escrever variaveis na RTDB
        int get_var(int id, int agent);
        QString get_QString_var(int );
        QString get_end_data();
signals:
        void endofsend();

public slots:

};

#endif // RTDBCLIENT_H

#include "teleopprocessmanager.h"

TeleopProcessManager::TeleopProcessManager(int number_of_processes, QObject *parent) : QObject(parent)
{
    minho_teleop_.resize(number_of_processes);
    for(unsigned int i = 0;i<(unsigned int)number_of_processes;i++) minho_teleop_[i] = NULL; // put them as NULL
    env = new QProcessEnvironment(QProcessEnvironment::systemEnvironment());
    QString ipv4_addr;
    foreach (const QHostAddress &address, QNetworkInterface::allAddresses()) {
        if (address.protocol() == QAbstractSocket::IPv4Protocol && address != QHostAddress(QHostAddress::LocalHost))
             ipv4_addr = address.toString();
    }
    env->insert("ROS_IP",ipv4_addr);
}

bool TeleopProcessManager::run_process(int process_id)
{
    if(process_id>=0 && process_id<(int)minho_teleop_.size()){
        if(!minho_teleop_[process_id]){
            minho_teleop_[process_id] = new QProcess();
            QStringList _args;
            _args.push_back(PACKAGE);
            _args.push_back(NODE);
            _args.push_back(ARGS);
            _args.push_back(QString::number(process_id+1));
            minho_teleop_[process_id]->setProcessEnvironment(*env);
            connect(minho_teleop_[process_id],
                    SIGNAL(finished(int,QProcess::ExitStatus)),
                    this,
                    SLOT(onClose()));
            minho_teleop_[process_id]->start(EXEC_,_args);
//            qDebug() << _args;
            if(!minho_teleop_[process_id]->waitForStarted(500)) return false;
            else return true;
        } else return false;
    }

    return false;
}

bool TeleopProcessManager::close_process(int process_id)
{
    if(process_id>=0 && process_id<(int)minho_teleop_.size()){
        if(minho_teleop_[process_id]) {
            minho_teleop_[process_id]->kill();
            minho_teleop_[process_id]->waitForFinished(-1);
            delete minho_teleop_[process_id];
            minho_teleop_[process_id] = NULL;
            return true;
        }
        else return false;
    } else return false;
}

QString TeleopProcessManager::getProcess(int process_id)
{
    if(process_id>=0 && process_id<(int)minho_teleop_.size()){
        if(minho_teleop_[process_id]) {
            return minho_teleop_[process_id]->arguments().join(" ");
        }
    }

    return "Invalid Process ID";
}

void TeleopProcessManager::onClose()
{
    QProcess *process = (QProcess *)this->sender();
    int process_id = (process->arguments().at(3).toInt())-1;
    minho_teleop_[process_id]->kill();
    minho_teleop_[process_id]->waitForFinished(-1);
    delete minho_teleop_[process_id];
    minho_teleop_[process_id] = NULL;
}

#ifndef TELEOPPROCESSMANAGER_H
#define TELEOPPROCESSMANAGER_H

#define PACKAGE "minho_team_tools"
#define NODE "minho_teleop"
#define ARGS "-s"
#include <QObject>
#include <QProcess>
#include <QProcessEnvironment>
#include <QDebug>
#include <QHostAddress>
#include <QNetworkInterface>
#define EXEC_ "rosrun"

class TeleopProcessManager : public QObject
{
    Q_OBJECT
public:
    explicit TeleopProcessManager(int number_of_processes,QObject *parent = 0);
public slots:
    bool run_process(int process_id);
    bool close_process(int process_id);
    QString getProcess(int process_id);
private:
    std::vector<QProcess *> minho_teleop_;
    QProcessEnvironment *env;
private slots:
    void onClose();
};

#endif // TELEOPPROCESSMANAGER_H

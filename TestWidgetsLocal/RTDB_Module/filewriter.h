#ifndef FILEWRITER_H
#define FILEWRITER_H
#include <QFile>
#include <QThread>
#include <QObject>
#include <QDebug>
#include <QByteArray>
#include <QMutex>

class fileWriter : public QObject
{
    Q_OBJECT
public:
    explicit fileWriter(QString path,QByteArray * ,QObject *parent = 0);
    void set_flag(bool flag);
private:
    QFile *file;
    bool write_flag;
    QByteArray *pointer;

signals:

public slots:
    void doSetup(QThread *t);
private slots:
    void doWork();


};

#endif // FILEWRITER_H

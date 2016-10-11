#include "filewriter.h"

fileWriter::fileWriter(QString path, QByteArray *pointer, QObject *parent)
{
    file= new QFile(path);
    file->open(QIODevice::WriteOnly);
    this->pointer=pointer;
    write_flag=false;
}

void fileWriter::set_flag(bool flag)
{
    write_flag=flag;
}

void fileWriter::doSetup(QThread *t)
{
    connect(t,SIGNAL(started()),this,SLOT(doWork()));
}

void fileWriter::doWork()
{
    while(1){
        if(write_flag)
        {
        QMutex b;
        if(b.tryLock(30))
        {
            file->write(pointer->data());
            write_flag=false;
            b.unlock();
        }

        }
    }
}

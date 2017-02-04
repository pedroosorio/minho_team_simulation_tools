#include <QApplication>
#include "action_dialog.h"
#include <QDebug>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QString filepath = "";
    ACTION ret = INVALID;
    ActionDialog action(&ret,&filepath);
    action.exec();

    switch(ret){
        case NEW:{
            break;
        }
        case OPEN:{
            break;
        }
        case REPLAY:{
            break;
        }
        default:{
            return 0;
            break;
        }
    }

    return a.exec();
}

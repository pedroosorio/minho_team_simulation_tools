#include <QApplication>
#include "action_dialog.h"
#include "replay_window.h"
#include <QDebug>


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QString filepath = "";
    ACTION ret = INVALID;

    while(ret!=EXIT){ // Loop unitl Exit is triggered
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
                ReplayWindow replay(filepath);
                replay.exec();
                ret = INVALID;
                break;
            }
            default:{
                return 0;
                break;
            }
        }
    }


    return a.exec();
}

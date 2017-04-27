#include <iostream>
#include <fstream>
#include <QDebug>
#include "rapidjson/document.h"

using namespace std;
using namespace rapidjson;

class Replay{
public:
    Replay(int *ret, QString filepath);
    ~Replay();
    /* Get functions */
    QString inline getName() {return replay_name;}
    QString inline getDate() {return replay_date;}
    float inline getTotalTime() {return (float)replay_total_samples*(1.0/(float)replay_frequency);}
    float inline getTimeStep() {return 1.0/(float)replay_frequency;}
    unsigned int inline getTotalSamples() {return replay_total_samples;}
private:
    /* Replay Settings data */
    QString replay_name;
    QString replay_date;
    unsigned int replay_frequency;
    unsigned int replay_total_samples;
};


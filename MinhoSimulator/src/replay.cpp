#include "replay.h"

Replay::Replay(int *ret, QString filepath)
{
    ifstream infile;
    infile.open(filepath.toStdString().c_str());
    if(!infile.is_open()) { (*ret) = 0; return; } // 0 for error failing

    string replay((istreambuf_iterator<char>(infile)), istreambuf_iterator<char>());
    infile.close();
    Document document;
    char *cstr = new char[replay.length() + 1];
    strcpy(cstr, replay.c_str());

    if (document.ParseInsitu(cstr).HasParseError() || !document.IsObject()){
        (*ret) = 0; return;
    }else {
        if(document.HasMember("Replay_Settings")){
            GenericValue<UTF8<> > settings;
            settings = document["Replay_Settings"].GetObject();
            // Go to Replay_Settings
            if(settings.HasMember("replay_name") && settings["replay_name"].IsString())
                replay_name = QString::fromStdString(settings["replay_name"].GetString());
            else {(*ret) = 0; return;}

            if(settings.HasMember("replay_name") && settings["replay_date"].IsString())
                replay_date = QString::fromStdString(settings["replay_date"].GetString());
            else {(*ret) = 0; return;}

            if(settings.HasMember("replay_name") && settings["replay_frequency"].IsNumber())
                replay_frequency = settings["replay_frequency"].GetInt64();
            else {(*ret) = 0; return;}

            if(settings.HasMember("replay_name") && settings["replay_total_samples"].IsNumber())
                replay_total_samples = settings["replay_total_samples"].GetInt64();
            else {(*ret) = 0; return;}

        } else {(*ret) = 0; return;}

    }

    (*ret) = 1;
}

Replay::~Replay()
{
}

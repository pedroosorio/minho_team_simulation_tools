#ifndef REPLAYWINDOW_H
#define REPLAYWINDOW_H

#include <QWidget>
#include <QDebug>
#include <QDialog>
#include <QMessageBox>
#include <QTimer>
#include "replay.h"

namespace Ui {
class Window;
}

class ReplayWindow : public QDialog
{
    Q_OBJECT

public:
    explicit ReplayWindow(QString file, QDialog *parent = 0);
    ~ReplayWindow();
private slots:

    void on_hbar_time_valueChanged(int value);

    void on_bt_startpause_clicked();

    void on_bt_restart_clicked();

    void on_bt_movback_clicked();

    void on_bt_movforw_clicked();

    void renderTrigger();
private:
    void setupUI();

    void renderReplayFrame(int frame_id);

private:
    Ui::Window *ui;
    QString home;
    bool replay_running;
    Replay *replay;
    QString total_seconds;
    long int current_rendered_frame;
    unsigned int total_frames;
    QTimer *render_timer;
};

#endif // WIDGET_H


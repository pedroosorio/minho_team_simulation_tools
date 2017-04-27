#ifndef ACTIONDIALOG_H
#define ACTIONDIALOG_H

#include <QWidget>
#include <QDialog>
#include <stdlib.h>
#include <QMessageBox>
#include <QDebug>
#include <QFileDialog>

namespace Ui {
class Form;
}

typedef enum ACTION {INVALID=0,NEW,OPEN,REPLAY,EXIT} ACTION;

class ActionDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ActionDialog(ACTION *act, QString *file, QDialog *parent = 0);
    ~ActionDialog();
private slots:
    void on_bt_newsim_clicked();
    void on_bt_opensim_clicked();
    void on_bt_replay_clicked();
    void on_bt_exit_clicked();
    void show_not_implemented();
private:
    void setupUI();

private:
    Ui::Form *ui;
    QString *filepath;
    QString home;
    ACTION *action;
};

#endif // WIDGET_H

